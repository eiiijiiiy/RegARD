import open3d as o3d
import numpy as np
import math
import time
import matplotlib.pyplot as plt

plt.axis('off')
plt.axis('equal')

MAX_EVAL = 2000		# Iteration budget for each algorithm
VERBOSE = False		# Turn to "True" for show obj_func covergence over time 

as_built = None
as_designed = None
kd_tree_designed = None
bound_built = None
bound_designed = None
center_built = None
center_designed = None
itrations = 0

mem_sq_err_v3 = np.array([0.0, 0.0, 0.0])

best_rmse_ever = 1E+8
time_start = time.time()
time_end = None
iterations = 0
finetune_ratio = 1
for_finetune_transformation = np.zeros(5, dtype=float)
curr_transformation = None
curr_RMSE = None
fixed_rotation_first = None


def squared_err(vec1, vec2):
	global mem_sq_err_v3
	mem_sq_err_v3[0] = vec2[0]-vec1[0]
	mem_sq_err_v3[1] = vec2[1]-vec1[1]
	mem_sq_err_v3[0] *= mem_sq_err_v3[0]
	mem_sq_err_v3[1] *= mem_sq_err_v3[1]
	return mem_sq_err_v3[0] + mem_sq_err_v3[1]	#note: Z === 0.0, is thus ommitted in our tests


def decode_transformation_variables(T, finetune):
	# @param: T := [0, 1] bounded variables standing for {trans_x, tranx_y, scale_x, scale_y, rotation_z}
	global bound_built, bound_designed, center_built, center_designed, scale_bound, for_finetune_transformation
	
	for i in range(5):
		if T[i] > 1:
			T[i] = 1
		elif T[i] < 0:
			T[i] = 0
	# 1 decode actual translations
	transformation = np.array([0,0,1,1,0], dtype=float)
	if finetune:
		transformation = for_finetune_transformation.copy()
		transformation[:2] += 0.025 * bound_designed[:2] * (np.asarray(T[:2]) * 2 - 1)
		
	else:
		transformation[:2] = center_designed[:2] - center_built[:2] # default trans_x if code == 0; (moving center to center)
		transformation[:2] += bound_designed[:2] * (np.asarray(T[:2]) * 2 - 1) # 0 = translate to the min bound; 1 = translate to the max bound
		transformation[2:4] *= np.power(scale_bound, np.asarray(T[2:4]) * 2 - 1) # [scale_bound^-1, scale_bound^1]
		
		if fixed_rotation_first:
			transformation[4] = 0
		else:
			transformation[4] = np.pi * (T[4] * 2 - 1) 		# [-Pi, Pi]

	return transformation


def compute_RMSE(T=[0.5, 0.5, 0.5, 0.5, 0.5]):
	# @param: T := [0, 1] bounded variables standing for {trans_x, tranx_y, scale_x, scale_y, rotation_z}
	global as_built, as_designed, kd_tree_designed, best_rmse_ever, iterations, \
		   finetune_ratio, MAX_EVAL, curr_RMSE, for_finetune_transformation, curr_transformation

	# decode [0, 1] variables to transformation
	finetune = iterations>=int(MAX_EVAL * finetune_ratio)
	transformation = decode_transformation_variables(T, finetune)
	
	after_transform = np.array([0.0,0.0,0.0])		# a temp variable for saving memory allocation
	rmse = 0.0;										# dummpy variable to sum RMSE
	# loop for each point in the as-built PCD
	for point in as_built.points:
		# 1 after scaling
		after_transform[:2] = point[:2] * transformation[2:4]
		# 2 after rotation
		radius = math.sqrt(after_transform[0]*after_transform[0] + after_transform[1]*after_transform[1])	#radius
		theta = math.atan2(after_transform[1], after_transform[0])
		after_transform[0] = radius * math.cos(theta + transformation[4])
		after_transform[1] = radius * math.sin(theta + transformation[4])
		# 3 after translation
		after_transform[:2] += transformation[:2]
		# 4 find nearest point in the target(as-designed) PCD
		[k, idx, _] = kd_tree_designed.search_knn_vector_3d(after_transform, 1)
		nearest = as_designed.points[idx[0]]
		# 5 add RMSE to the sum
		rmse += squared_err(after_transform, nearest)
	# Registration RMSE 
	rmse = math.sqrt(rmse / len(as_built.points))

	curr_transformation = transformation
	curr_RMSE = rmse
	if iterations == int(MAX_EVAL * finetune_ratio) - 1:
		for_finetune_transformation = curr_transformation.copy()

	global time_start
	iterations += 1
	global VERBOSE
	if VERBOSE and rmse < best_rmse_ever:
		best_rmse_ever = rmse
		print('[Info] new registration rmse = ', rmse, ' at Iteration = ', iterations, ', ', time.time() - time_start, 'seconds')
	return rmse


def reg_by_cmaes():
	global best_rmse_ever
	global iterations
	global time_start
	global time_end
	global dof
	import lcmaes
	# define a array of 5 constrained variables
	x = [0.5]*dof
	# parameters of CMA-ES
	lambda_ = 20 			# "lambda" is a reserved keyword in python, using lambda_ instead.
	seed = 2020 				# 0 for seed auto-generated within the lib.
	sigma = 0.01
	lbounds = [0]*dof		# lower bound value
	ubounds = [1]*dof		# upper bound value
	gp = lcmaes.make_genopheno_pwqb(lbounds, ubounds, dof)
	p = lcmaes.make_parameters_pwqb(x, sigma, gp, lambda_, seed)
	global MAX_EVAL
	
	# generate a function object
	def obj_func_1st_stage_for_cmaes(x,n):
		if dof == 3:
			return compute_RMSE([0.5, x[0], x[1], x[2], 0.5], False)
		elif dof == 4:
			return compute_RMSE([x[0], x[1], x[2], x[3], 0.5], False)
		else:
			return compute_RMSE([x[0], x[1], x[2], x[3], x[4]], False)
	
	def obj_func_2nd_stage_for_cmaes(x,n):

		return compute_RMSE([x[0], x[1], x[2], x[3], x[4]], True)

	best_rmse_ever = 1E+8
	iterations = 0
	if finetune_ratio < 1:
		# 1st stage
		
		p.set_max_fevals(int(MAX_EVAL * finetune_ratio))
		p.set_sep()
		objfunc = lcmaes.fitfunc_pbf.from_callable(obj_func_1st_stage_for_cmaes)

		time_start = time.time()
		cmasols = lcmaes.pcmaes_pwqb(objfunc, p)
		time_end = time.time()

		# collect and inspect results
		bcand = cmasols.best_candidate()
		x = lcmaes.get_candidate_x(bcand)

		# 2nd stage
		lbounds = [0] * 5		# lower bound value
		ubounds = [1] * 5		# upper bound value
		gp = lcmaes.make_genopheno_pwqb(lbounds, ubounds, 5)
		p = lcmaes.make_parameters_pwqb(x, sigma, gp, lambda_, seed)
		p.set_max_fevals(int(MAX_EVAL * (1 - finetune_ratio)))
		p.set_sep()
		objfunc = lcmaes.fitfunc_pbf.from_callable(obj_func_2nd_stage_for_cmaes)

		time_start = time.time()
		cmasols = lcmaes.pcmaes_pwqb(objfunc, p)
		time_end = time.time()

		# collect and inspect results
		bcand = cmasols.best_candidate()
		bx = lcmaes.get_candidate_x(bcand)
	else:
		p.set_max_fevals(MAX_EVAL)
		p.set_sep()
		objfunc = lcmaes.fitfunc_pbf.from_callable(obj_func_1st_stage_for_cmaes)
		
		# run optimization and collect solution object.
		time_start = time.time()
		cmasols = lcmaes.pcmaes_pwqb(objfunc, p)
		time_end = time.time()
		
		# collect and inspect results
		bcand = cmasols.best_candidate()
		bx = lcmaes.get_candidate_x(bcand)

	return bx


def reg_by_nlopt(alg):
	global best_rmse_ever, iterations, time_start, time_end
	# global dof

	import nlopt
	
	def obj_func_for_nlopt(x, grad):
		return compute_RMSE(x)

	def choose_algorithm(alg):
		if alg=='DIRECT':
			return nlopt.GN_DIRECT
		elif alg=='MLSL':
			return nlopt.GN_MLSL
		elif alg=='MMA':
			return nlopt.LD_MMA
		elif alg=='COBYLA':
			return nlopt.LN_COBYLA
		elif alg=='NEWUOA':
			return nlopt.LN_NEWUOA_BOUND
		elif alg=='NELDERMEAD':
			return nlopt.LN_NELDERMEAD
		elif alg=='SBPLX':
			return nlopt.LN_SBPLX
		elif alg=='AUGLAG':
			return nlopt.LN_AUGLAG
		elif alg=='BOBYQA':
			return nlopt.LN_BOBYQA
		return nlopt.GN_DIRECT
	# create optimizer
	opt = nlopt.opt(choose_algorithm(alg), 5)
	opt.set_lower_bounds(0.0)
	opt.set_upper_bounds(1.0)
	opt.set_xtol_rel(1e-6)
	
	# reset record
	best_rmse_ever = 1E+8
	iterations = 0

	global MAX_EVAL, finetune_ratio
	opt.set_maxeval(MAX_EVAL)
	opt.set_min_objective(obj_func_for_nlopt)

	# start solving
	time_start = time.time()
	x = opt.optimize([0.5] * 5)
	time_end = time.time()
	
	return x


def get_rotation_matrix(theta):
	R = [[np.cos(theta), -np.sin(theta), 0], [np.sin(theta), np.cos(theta), 0],[0,0,1]]

	return R

def compute_initial_RMSD():
	global as_built, as_designed
	from scipy import spatial

	as_designed_pts = np.asarray(as_designed.points)[:, :2]
	as_built_pts = np.asarray(as_built.points)[:, :2]
	as_built_num = as_built_pts.shape[0]

	tree = spatial.KDTree(as_designed_pts)
	dists, idxs = tree.query(as_built_pts)
	RMSD = np.sqrt(np.sum(dists ** 2) / as_built_num)
	return RMSD

def register(fp, pc, alg, with_asd, reg_fig, max_iter=1000, fp_asd_rho=None,
			fp_asd_theta=None, pc_asd_rho=None, pc_asd_theta=None, ard_fig = None, bs=1.2):
	
	global as_built, as_designed, kd_tree_designed, bound_built, \
			bound_designed, center_built, center_designed,\
			scale_bound, finetune_ratio, MAX_EVAL, best_rmse_ever, \
			iterations, time_start, time_end, final_RMSE, final_transformation, fixed_rotation_first
	
	# global dof
	
	scale_bound = bs
	# read as 3D points
	print("> reading", fp)
	as_designed = o3d.io.read_point_cloud(fp,  format='xyz')	
	kd_tree_designed = o3d.geometry.KDTreeFlann(as_designed)			# Kd-tree structure for searching
	bound_designed = as_designed.get_max_bound() - as_designed.get_min_bound()
	center_designed = (as_designed.get_max_bound() + as_designed.get_min_bound())/2
	as_designed_pts = np.asarray(as_designed.points)
	
	print("> reading",pc)
	as_built = o3d.io.read_point_cloud(pc, format='xyz')	
	bound_built = as_built.get_max_bound() - as_built.get_min_bound()
	center_built = (as_built.get_max_bound() + as_built.get_min_bound()) / 2

	iterations = 0
	init_RMSE = compute_initial_RMSD()
	print("> Initial RMSE =", init_RMSE,'without any transformation')

	MAX_EVAL = max_iter
	algorithms = ['CMAES', 'DIRECT', 'MLSL', 'MMA', 'COBYLA', 'NEWUOA', 'NELDERMEAD', 'SBPLX', 'AUGLAG', 'BOBYQA']
	assert alg in ['ALL'] + algorithms

	best_x = None
	best_RMSE = 1E+8
	iterations = 0 
	alg_RMSES = []
	alg_TIMES = []

	if with_asd:
		assert fp_asd_rho is not None and fp_asd_theta is not None
		assert pc_asd_rho is not None and pc_asd_theta is not None
		fixed_rotation_first = True

		delta_asd_theta = fp_asd_theta - pc_asd_theta
		delta_asd_rho = fp_asd_rho - pc_asd_rho

		delta_tsl = np.array([np.cos(fp_asd_theta), np.sin(fp_asd_theta), 0]) * delta_asd_rho
		R = as_built.get_rotation_matrix_from_xyz((0,0,delta_asd_theta))
		as_built.rotate(R, center=(0,0,0))
		as_built.translate(delta_tsl)

		bound_built = as_built.get_max_bound() - as_built.get_min_bound()
		center_built = (as_built.get_max_bound() + as_built.get_min_bound())/2
		
		if ard_fig is not None:
			plt.clf()
			plt.axis('off')
			plt.axis('equal')
			as_built_pts = np.asarray(as_built.points)
			plt.plot(as_designed_pts[:,0],as_designed_pts[:,1],'rx',ms=1)
			plt.plot(as_built_pts[:,0],as_built_pts[:,1],'b*',ms=1)
			plt.savefig(ard_fig, dpi=600, bbox_inches=0)
			plt.clf()

		# find the fittest rotation whose ASD RMSE is the smallest
		R = as_built.get_rotation_matrix_from_xyz((0, 0, -np.pi/2))
		as_built.rotate(R, center=(0, 0, 0))
		best_rotation_x = None
		best_rotation_i = 0
		best_asd_RMSE = 1E+8

		for i in range(0, 4):
			theta = np.pi / 2
			print ('> ROTATION: ', i, 'PI/2')
			R = as_built.get_rotation_matrix_from_xyz((0, 0, theta))
			as_built.rotate(R, center=(0, 0, 0))
			bound_built = as_built.get_max_bound() - as_built.get_min_bound()
			center_built = (as_built.get_max_bound() + as_built.get_min_bound()) / 2
			asd_RMSE = compute_RMSE()
			if asd_RMSE < best_asd_RMSE:
    				best_asd_RMSE = asd_RMSE
    				best_rotation_i = i
		print('best_asd_RMSE {} at rotation {} PI'.format(best_asd_RMSE, best_rotation_i/2))

		# rotate the as_built to the best rotation
		theta = np.pi * (best_rotation_i + 1) / 2 
		R = as_built.get_rotation_matrix_from_xyz((0, 0, theta))
		as_built.rotate(R, center=(0,0,0)) 
		bound_built = as_built.get_max_bound() - as_built.get_min_bound()
		center_built = (as_built.get_max_bound() + as_built.get_min_bound())/2

		print ('> Start 4-DoF optimal registration, each algorithm has a budget of', MAX_EVAL, 'iterations.')

		if alg == 'ALL':
			for a in algorithms:
				if a == 'CMAES':
					reg_by_cmaes()
				else:
					reg_by_nlopt(a)
				alg_RMSES.append(curr_RMSE)
				alg_TIMES.append(time_end - time_start)
				# print('>> Algorithm', a, 'found a solution', x,' with RMSE =', RMSE, ', in ', time_end - time_start, 's')
				if curr_RMSE < best_RMSE:
					best_RMSE = curr_RMSE
					best_transformation = curr_transformation

		elif alg == 'CMAES': 
			best_x = reg_by_cmaes()
			best_RMSE = curr_RMSE
			best_transformation = curr_transformation
			alg_RMSES.append(curr_RMSE)
			alg_TIMES.append(time_end - time_start)

		else:
			reg_by_nlopt(alg)
			best_RMSE = curr_RMSE
			best_transformation = curr_transformation
			alg_RMSES.append(curr_RMSE)
			alg_TIMES.append(time_end - time_start)
		
		best_transformation[4] += best_rotation_i / 2 * np.pi
		
	else:  
		print ('> Start 5-DoF optimal registration, each algorithm has a budget of', MAX_EVAL, 'iterations.')
		fixed_rotation_first = False
		if alg == 'ALL':
			for a in algorithms:
				if alg == 'CMAES':
					reg_by_cmaes()
				else:
					reg_by_nlopt(a)
				alg_RMSES.append(curr_RMSE)
				alg_TIMES.append(time_end - time_start)
				if curr_RMSE < best_RMSE:
					best_RMSE = curr_RMSE
					best_transformation = curr_transformation
		elif alg == 'CMAES':
			reg_by_cmaes()
			best_RMSE = curr_RMSE
			best_transformation = curr_transformation
			alg_RMSES.append(best_RMSE)
			alg_TIMES.append(time_end - time_start)
		else:
			reg_by_nlopt(alg)
			best_RMSE = curr_RMSE
			best_transformation = curr_transformation
			alg_RMSES.append(best_RMSE)
			alg_TIMES.append(time_end - time_start)
		
	print("> best RMSE =", best_RMSE)
	print('tsl_x \t tsl_y \t scl_x \t scl_y \t rot \n {} \t {} \t {} \t {} \t {}'.format(
			best_transformation[0], best_transformation[1], best_transformation[2], 
			best_transformation[3], best_transformation[4]))		

	plt.clf()
	plt.plot(as_designed_pts[:,0],as_designed_pts[:,1],'rx',ms=1)
	# 1 after scaling
	transformed = np.asarray(as_built.points)[:,:2] * best_transformation[2:4]
	# 2 after rotation
	if not with_asd:
		radius = np.sqrt(np.sum(transformed ** 2,1))
		theta = np.arctan2(transformed[:,1], transformed[:,0])
		transformed[:,0] = radius * np.cos(theta + best_transformation[4])
		transformed[:,1] = radius * np.sin(theta + best_transformation[4])
	# 3 after translation
	transformed = transformed + best_transformation[:2]
	plt.axis('off')
	plt.axis('equal')
	plt.plot(transformed[:,0],transformed[:,1],'bo',ms=1)
	plt.savefig(reg_fig, dpi=600, bbox_inches=0)
	
	return alg_RMSES, alg_TIMES, best_transformation

if __name__ == "__main__":
	import argparse

	parser = argparse.ArgumentParser(description='Registration based on Architectural Reflection Detection')
	parser.add_argument('--fp', type=str, help='input sampled points of floor plan', required=True)
	parser.add_argument('--pc', type=str, help='input sampled points of point cloud', required=True)
	parser.add_argument('--alg', type=str, help='DFO algorithm, options: ALL, CMAES, DIRECT, MLSL, MMA, COBYLA, NEWUOA, NELDERMEAD, SBPLX, AUGLAG, BOBYQA', required=False, default='NELDERMEAD')
	parser.add_argument('--ard', action='store_true', help='enable ARD', required=False)
	parser.add_argument('--reg_fig', type=str, help='the ouput registration figure', required=True)
	parser.add_argument('--ard_fig', type=str, help='the ouput registration figure', required=False)
	parser.add_argument('--max_iter', type=int, help='max iterations', required=False, default=100)
	parser.add_argument('--bs', type=float, help='the bound of scaling', required=False, default=1.2)
	parser.add_argument('--fp_r', type=float, help='the r of the fp symmetry axis', required=False)
	parser.add_argument('--fp_theta', type=float, help='the theta of the fp symmetry axis', required=False)
	parser.add_argument('--pc_r', type=float, help='the r of the pc symmetry axis', required=False)
	parser.add_argument('--pc_theta', type=float, help='the theta of the pc symmetry axis', required=False)
	args = parser.parse_args()
	if args.ard:
		register(args.fp, args.pc, args.alg, args.ard, args.reg_fig, args.max_iter, 
				args.fp_r, args.fp_theta, args.pc_r, args.pc_theta, args.ard_fig, args.bs)
	else:
		register(args.fp, args.pc, args.alg, args.ard, args.reg_fig, args.max_iter, args.bs)
	
