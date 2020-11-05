import numpy as np
import math
import time
import open3d as o3d

# MAX_EVAL = 2000		# Iteration budget for each algorithm
VERBOSE = False		# Turn to "True" for show obj_func covergence over time 

as_built = None
as_designed = None
kd_tree_designed = None
bound_built = None
bound_designed = None
center_built = None
center_designed = None

mem_sq_err_v3 = np.array([0.0, 0.0, 0.0])

best_rmse_ever = 1E+8
iterations = 0

time_start = time.time()
time_end = None

def transform_pcd(in_pcdf, rho, theta, out_pcdf=None):
    import open3d as o3d
    pcd = o3d.io.read_point_cloud(in_pcdf)
    R = o3d.geometry.get_rotation_matrix_from_xyz([0, 0, -theta])
    pcd.rotate(R, pcd.get_center())
    pcd.translate((-rho, 0, 0))
    
    if out_pcdf is not None:
        o3d.io.write_point_cloud(out_pcdf, pcd)

    return pcd


def squared_err(vec1, vec2):
	global mem_sq_err_v3
	mem_sq_err_v3[0] = vec2[0]-vec1[0]
	mem_sq_err_v3[1] = vec2[1]-vec1[1]
	mem_sq_err_v3[0] *= mem_sq_err_v3[0]
	mem_sq_err_v3[1] *= mem_sq_err_v3[1]
	return mem_sq_err_v3[0] + mem_sq_err_v3[1]	#note: Z === 0.0, is thus ommitted in our tests


def decode_transformation_variables(T):
	# @param: T := [0, 1] bounded variables standing for {trans_x, tranx_y, scale_x, scale_y, rotation_z}
	global bound_built
	global bound_designed
	global center_built
	global center_designed
	global scale_bound
	
	for i in range(5):
		if T[i] > 1:
			T[i] = 1
		elif T[i] < 0:
			T[i] = 0
	# 1 decode actual translations
	trans = np.array([0.0, 0.0, 0.0])
	trans[0] = center_designed[0] - center_built[0]	# default trans_x if code == 0; (moving center to center)
	trans[0] += bound_designed[0] * (T[0] * 2 - 1)	# 0 = translate to the min bound; 1 = translate to the max bound
	trans[1] = center_designed[1] - center_built[1]
	trans[1] += bound_designed[1] * (T[1] * 2 - 1)
	# 2 decode actual scalings; assuming [0.1, 10] scaling, i.e., [10^-1, 10^1]
	scale = np.array([1.0, 1.0, 1.0])
	# scale[0] = bound_designed[0]/bound_built[0] 	# default scale_x if code == 0
	scale[0] *= math.pow(scale_bound, T[2] * 2 - 1)			# scale_x belongs to [0.1, 10] times of the default value
	# scale[1] = bound_designed[1]/bound_built[1]
	scale[1] *= math.pow(scale_bound, T[3] * 2 - 1)
	# 3 decode actual rotation
	rotation_z = math.pi * (T[4] * 2 - 1) 		# [-Pi, Pi]
	return trans, scale, rotation_z


def compute_RMSE(T=[0.5, 0.5, 0.5, 0.5, 0.5]):
	# @param: T := [0, 1] bounded variables standing for {trans_x, tranx_y, scale_x, scale_y, rotation_z}
	global as_built
	global as_designed
	global kd_tree_designed
	global best_rmse_ever
	global iterations
	# decode [0, 1] variables to transformation
	trans, scale, rotation_z = decode_transformation_variables(T)
	
	after_transform = np.array([0.0,0.0,0.0])		# a temp variable for saving memory allocation
	rmse = 0.0;										# dummpy variable to sum RMSE
	# loop for each point in the as-built PCD
	for point in as_built.points:
		# 1 after scaling
		after_transform[0] = point[0] * scale[0]
		after_transform[1] = point[1] * scale[1]
		# 2 after rotation
		radius = math.sqrt(after_transform[0]*after_transform[0] + after_transform[1]*after_transform[1])	#radius
		theta = math.atan2(after_transform[1], after_transform[0])
		after_transform[0] = radius * math.cos(theta + rotation_z)
		after_transform[1] = radius * math.sin(theta + rotation_z)
		# 3 after translation
		after_transform[0] += trans[0]
		after_transform[1] += trans[1]
		# 4 find nearest point in the target(as-designed) PCD
		[k, idx, _] = kd_tree_designed.search_knn_vector_3d(after_transform, 1)
		nearest = as_designed.points[idx[0]]
		# 5 add RMSE to the sum
		rmse += squared_err(after_transform, nearest)
	# Registration RMSE 
	rmse = math.sqrt(rmse / len(as_built.points))
	
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
	gp = lcmaes.make_genopheno_pwqb(lbounds,ubounds,dof)
	p = lcmaes.make_parameters_pwqb(x,sigma,gp,lambda_,seed)
	global MAX_EVAL
	p.set_max_fevals(MAX_EVAL)
	#p.set_noisy()
	p.set_sep()
	#p.set_elitism(1);
	#p.set_algo(10)
	
	# generate a function object
	def obj_func_for_cmaes(x,n):
		if dof == 3:
			return compute_RMSE([0.5, x[0], x[1], x[2], 0.5])
		elif dof == 4:
			return compute_RMSE([x[0], x[1], x[2], x[3], 0.5])
		else:
			return compute_RMSE([x[0], x[1], x[2], x[3], x[4]])

	objfunc = lcmaes.fitfunc_pbf.from_callable(obj_func_for_cmaes);
	
	# reset record
	best_rmse_ever = 1E+8
	iterations = 0
	
	# run optimization and collect solution object.
	time_start = time.time()
	cmasols = lcmaes.pcmaes_pwqb(objfunc, p)
	time_end = time.time()
	
	# collect and inspect results
	bcand = cmasols.best_candidate()
	bx = lcmaes.get_candidate_x(bcand)
	#print ("distribution mean=",lcmaes.get_solution_xmean(cmasols))
	#cov = lcmaes.get_solution_cov(cmasols) 			# numpy array
	#print ("cov=", cov)
	#print ("elapsed time=",cmasols.elapsed_time(),"ms")
	return bx


def reg_by_nlopt(alg):
	global best_rmse_ever
	global iterations
	global time_start
	global time_end
	global dof
	import nlopt
	def obj_func_for_nlopt(x, grad):
		if dof == 3:
			return compute_RMSE([0.5, x[0], x[1], x[2], 0.5])
		elif dof == 4:
			return compute_RMSE([x[0], x[1], x[2], x[3], 0.5])
		else:  # dof == 5
			return compute_RMSE([x[0], x[1], x[2], x[3], x[4]])
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
	opt = nlopt.opt(choose_algorithm(alg), dof)
	opt.set_lower_bounds(0.0);
	opt.set_upper_bounds(1.0);
	global MAX_EVAL
	opt.set_maxeval(MAX_EVAL);
	opt.set_min_objective(obj_func_for_nlopt);
	opt.set_xtol_rel(1e-6)
	
	# reset record
	best_rmse_ever = 1E+8
	iterations = 0
	
	# start solving
	time_start = time.time()
	x = opt.optimize([0.5]*dof)
	time_end = time.time()
	return x


def get_eval(dof, x):
	if dof == 3:
		eval = compute_RMSE([0.5, x[0],x[1], x[2], 0.5])
	elif dof == 4:
		eval = compute_RMSE([x[0], x[1],x[2], x[3], 0.5])
	else: # dof == 5
		eval = compute_RMSE(x)

	return eval


def get_rotation_matrix(theta):
	R = [[np.cos(theta), -np.sin(theta), 0], [np.sin(theta), np.cos(theta), 0],[0,0,1]]

	return R


def register(fp, pc, alg, with_ard, reg_fig, max_iter=100,bs=1.2, 
			fp_rho=None, fp_theta=None,pc_rho=None,pc_theta=None):
	global as_built
	global as_designed
	global kd_tree_designed
	global bound_built
	global bound_designed
	global center_built
	global center_designed
	global scale_bound
	scale_bound = bs

	# read as 3D points

	print("> reading", fp)
	as_designed = o3d.io.read_point_cloud(fp)	
	kd_tree_designed = o3d.geometry.KDTreeFlann(as_designed)			# Kd-tree structure for searching
	bound_designed = as_designed.get_max_bound() - as_designed.get_min_bound()
	center_designed = (as_designed.get_max_bound() + as_designed.get_min_bound())/2
	
	print("> reading",pc)
	as_built = o3d.io.read_point_cloud(pc)
	bound_built = as_built.get_max_bound() - as_built.get_min_bound()
	center_built = (as_built.get_max_bound() + as_built.get_min_bound())/2

	init_RMSE = compute_RMSE()
	print("> Initial RMSE =", init_RMSE,'without any transformation')
	
	global MAX_EVAL
	global best_rmse_ever
	global iterations
	global time_start
	global time_end
	global dof

	MAX_EVAL = max_iter
	algorithms = ['CMAES', 'DIRECT', 'MLSL', 'MMA', 'COBYLA', 'NEWUOA', 'NELDERMEAD', 'SBPLX', 'AUGLAG', 'BOBYQA']
	assert alg in ['ALL'] + algorithms

	best_x = None
	best_RMSE = 1E+8
	iterations = 0 
	alg_RMSES = []
	alg_TIMES = []


	if with_ard: # dof = 4
		dof = 4
		assert fp_rho is not None and fp_theta is not None
		assert pc_rho is not None and pc_theta is not None
		# read data and transform with ard parameters
		print("> transforming with ard parameters", fp)
		as_designed = transform_pcd(fp, fp_rho, fp_theta)
		# as_designed = o3d.geometry.PointCloud()
		# as_designed.points = o3d.utility.Vector3dVector(fp_transformed)
		kd_tree_designed = o3d.geometry.KDTreeFlann(as_designed)			# Kd-tree structure for searching
		bound_designed = as_designed.get_max_bound() - as_designed.get_min_bound()
		center_designed = (as_designed.get_max_bound() + as_designed.get_min_bound())/2

		print("> transforming with ard parameters", pc)
		as_built = transform_pcd(pc, pc_rho, pc_theta)
		# as_built = o3d.geometry.PointCloud()
		# as_built.points = o3d.utility.Vector3dVector(pc_transformed)
		bound_built = as_built.get_max_bound() - as_built.get_min_bound()
		center_built = (as_built.get_max_bound() + as_built.get_min_bound())/2

		# find the fittest rotation whose ARD RMSE is the smallest
		as_built.rotate(get_rotation_matrix(-np.pi/2),center=(0,0,0))
		best_rotation_x = None
		best_rotation_i = 0
		best_ard_RMSE = 1E+8
		for i in range(0,4):
			theta = np.pi/2
			print ('> ROTATION: ', i, 'PI/2')
			as_built.rotate(get_rotation_matrix(theta),center=(0,0,0))
			bound_built = as_built.get_max_bound() - as_built.get_min_bound()
			center_built = (as_built.get_max_bound() + as_built.get_min_bound())/2
			ard_RMSE = compute_RMSE()
			if ard_RMSE < best_ard_RMSE:
    				best_ard_RMSE = ard_RMSE
    				best_rotation_i = i
		print('best_ard_RMSE {} at rotation {} PI'.format(best_ard_RMSE, best_rotation_i/2))

		# rotate the as_built to the best rotation
		theta = math.pi * (best_rotation_i + 1) / 2 
		as_built.rotate(get_rotation_matrix(theta), center=(0,0,0)) 
		bound_built = as_built.get_max_bound() - as_built.get_min_bound()
		center_built = (as_built.get_max_bound() + as_built.get_min_bound())/2

		print ('> Start 4-DoF optimal registration, each algorithm has a budget of', MAX_EVAL, 'iterations.')

		if alg == 'ALL':
			for a in algorithms:
				x = None
				if a == 'CMAES':
					x = reg_by_cmaes()
				else:
					x = reg_by_nlopt(a)
				RMSE = get_eval(dof, x)
				alg_RMSES.append(RMSE)
				alg_TIMES.append(time_end - time_start)
				if RMSE < best_RMSE:
					best_RMSE = RMSE
					best_x = x
		elif alg == 'CMAES': 
			best_x = reg_by_cmaes()
			best_RMSE = get_eval(dof, best_x)
			alg_RMSES.append(best_RMSE)
			alg_TIMES.append(time_end - time_start)
		else:
			best_x = reg_by_nlopt(alg)
			best_RMSE = get_eval(dof, best_x)
			alg_RMSES.append(best_RMSE)
			alg_TIMES.append(time_end - time_start)
		
		print('best RMSE', best_RMSE)
		
		if type(best_x) == list:
    			best_x = best_x + [0.5]
		else: 
			best_x = best_x.tolist()+[0.5]
		trans, scale, rotation_z = decode_transformation_variables(best_x)
		print('--r {} --sx {} --sy {} --tx {} --ty {}'.format(best_rotation_i / 2, scale[0], scale[1], trans[0], trans[1]))
		print("> Rotation =", best_rotation_i / 2, "PI rad")
		print("> Translation =", trans)
		print("> Sclaing =", scale)
		
	else:  #dof == 5
		dof = 5
		print ('> Start 5-DoF optimal registration, each algorithm has a budget of', MAX_EVAL, 'iterations.')
		if alg == 'ALL':
			# algorithms = ['CMAES', 'DIRECT', 'MLSL', 'MMA', 'COBYLA', 'NEWUOA', 'NELDERMEAD', 'SBPLX', 'AUGLAG', 'BOBYQA']
			for a in algorithms:
				x = None
				if alg == 'CMAES':
					x = reg_by_cmaes()
				else:
					x = reg_by_nlopt(a)
				RMSE = compute_RMSE(x)
				alg_RMSES.append(RMSE)
				alg_TIMES.append(time_end - time_start)
				# print("> Algorithm ", a, " found a solution with RMSE =", RMSE, ", in ", time_end - time_start, "s")
				if RMSE < best_RMSE:
					best_RMSE = RMSE
					best_x = x
		elif alg == 'CMAES':
			best_x = reg_by_cmaes()
			best_RMSE = compute_RMSE(best_x)
			alg_RMSES.append(best_RMSE)
			alg_TIMES.append(time_end - time_start)
		else:
			best_x = reg_by_nlopt(alg)
			best_RMSE = compute_RMSE(best_x)
			alg_RMSES.append(best_RMSE)
			alg_TIMES.append(time_end - time_start)
		
		print("> best RMSE =", best_RMSE)
		trans, scale, rotation_z = decode_transformation_variables(best_x)
		print("> Translation =", trans)
		print("> Sclaing =", scale)
		print("> Rotation_z =", rotation_z, "rad")

    			

	import matplotlib.pyplot as plt
	plt.clf()
	as_designed_pts = np.asarray(as_designed.points)
	plt.plot(as_designed_pts[:,0],as_designed_pts[:,1],'rx',ms=1)

	# 1 after scaling
	transformed = np.asarray(as_built.points)[:,:2] * scale[:2]
	# 2 after rotation
	if dof == 5:
		radius = np.sqrt(np.sum(transformed ** 2,1))
		theta = np.arctan2(transformed[:,1], transformed[:,0])
		transformed[:,0] = radius * np.cos(theta + rotation_z)
		transformed[:,1] = radius * np.sin(theta + rotation_z)
	# 3 after translation
	transformed = transformed + trans[:2]
	plt.axis('off')
	plt.plot(transformed[:,0],transformed[:,1],'bo',ms=1)
	plt.savefig(reg_fig, dpi=600, bbox_inches=0)
	
	return alg_RMSES, alg_TIMES


def main():
	import argparse
	import os
	parser = argparse.ArgumentParser(description='Registration based on Architectural Reflection Detection')
	parser.add_argument('--fp', type=str, help='input sampled points of floor plan', required=True)
	parser.add_argument('--pc', type=str, help='input sampled points of point cloud', required=True)
	parser.add_argument('--alg', type=str, help='DFO algorithm, options: ALL, CMAES, DIRECT, MLSL, MMA, COBYLA, NEWUOA, NELDERMEAD, SBPLX, AUGLAG, BOBYQA', required=False, default='NELDERMEAD')
	parser.add_argument('--without_ard', action='store_true', help='enable ARD', required=False)
	parser.add_argument('--reg_fig', type=str, help='the ouput registration figure', required=True)
	parser.add_argument('--max_iter', type=float, help='max iterations', required=False, default=100)
	parser.add_argument('--bs', type=float, help='the bound of scaling', required=False, default=1.2)
	parser.add_argument('--fp_rho', type=float, help='the rho of the fp symmetry axis', required=False)
	parser.add_argument('--fp_theta', type=float, help='the theta of the fp symmetry axis', required=False)
	parser.add_argument('--pc_rho', type=float, help='the rho of the pc symmetry axis', required=False)
	parser.add_argument('--pc_theta', type=float, help='the theta of the pc symmetry axis', required=False)
	args = parser.parse_args()

	if args.without_ard:
		register(args.fp, args.pc, args.alg, False, args.reg_fig, max_iter=args.max_iter, bs=args.bs)
	else:
		register(args.fp, args.pc, args.alg, True, args.reg_fig, max_iter=args.max_iter, bs=args.bs,
				fp_rho=args.fp_rho,fp_theta=args.fp_theta, pc_rho=args.pc_rho, pc_theta=args.pc_theta)
    	

main()
