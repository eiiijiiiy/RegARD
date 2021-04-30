import argparse
import time
import ctypes


parser = argparse.ArgumentParser(description='Architectural Reflection Detection')
parser.add_argument('--fp', type=str, help='input sampled points of floor plan (*. pcd is required)', required=True)
parser.add_argument('--pc', type=str, help='input sampled points of point cloud (*. pcd is required)', required=True)
parser.add_argument('--fitness', type=float, help='the rmse threshold', required=False, default=10.)
args = parser.parse_args()

lib = ctypes.cdll.LoadLibrary('./libodas.so')

print('==== ard of fp ====')
fp_rho = ctypes.c_float(0)
fp_theta = ctypes.c_float(0)
fp_rmse = ctypes.c_double(0)
fp_fitness = ctypes.c_double(0)
fp_path = ctypes.create_string_buffer(args.fp.encode('utf-8'))
begin_t = time.time()
lib.odas_DIRECT(fp_path, ctypes.byref(fp_rho), ctypes.byref(fp_theta), ctypes.byref(fp_fitness), ctypes.byref(fp_time))

print('==== ard of pc ====')
pc_rho = ctypes.c_float(0)
pc_theta = ctypes.c_float(0)
pc_fitness = ctypes.c_double(0)
pc_time = ctypes.c_double(0)
pc_path = ctypes.create_string_buffer(args.pc.encode('utf-8'))

lib.odas_DIRECT(pc_path, ctypes.byref(pc_rho), ctypes.byref(pc_theta), ctypes.byref(pc_fitness), ctypes.byref(pc_time))

print('fp ard fitness: {} {} threshold'.format(fp_rmse.value, '>' if float(fp_rmse.value) > args.rmse_t else '<='))
print('pc ard fitness: {} {} threshold'.format(pc_rmse.value, '>' if float(pc_rmse.value) > args.rmse_t else '<='))
print('--fp_r {} --fp_theta {} --pc_r {} --pc_theta {}'.format(fp_rho.value, fp_theta.value, pc_rho.value, pc_theta.value))

if fp_fitness.value > args.fitness or pc_fitness.value > args.fitness:
    print('Use the 5-DoF transformation optimization')
else:
    print('Use the 4-DoF transformation optimization')
	
