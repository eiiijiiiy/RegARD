import argparse
import time
import ctypes


parser = argparse.ArgumentParser(description='Architectural Reflection Detection')
parser.add_argument('--fp', type=str, help='input sampled points of floor plan (*. pcd is required)', required=True)
parser.add_argument('--pc', type=str, help='input sampled points of point cloud (*. pcd is required)', required=True)
args = parser.parse_args()

lib = ctypes.cdll.LoadLibrary('./libodas.so')

fp_rho = ctypes.c_float(0)
fp_theta = ctypes.c_float(0)
fp_rmse = ctypes.c_double(0)
fp_time = ctypes.c_double(0)
fp_path = ctypes.create_string_buffer(fp.encode('utf-8'))
begin_t = time.time()
lib.odas_DIRECT(fp_path, ctypes.byref(fp_rho), ctypes.byref(fp_theta), ctypes.byref(fp_rmse), ctypes.byref(fp_time))



pc_rho = ctypes.c_float(0)
pc_theta = ctypes.c_float(0)
pc_rmse = ctypes.c_double(0)
pc_time = ctypes.c_double(0)
pc_path = ctypes.create_string_buffer(pc.encode('utf-8'))
lib.odas_DIRECT(pc_path, ctypes.byref(pc_rho), ctypes.byref(pc_theta), ctypes.byref(pc_rmse), ctypes.byref(pc_time))

ard_t = time.time() - begin_t

print('odas time: {}'.format(ard_t))
print('--fp_rho {} --fp_theta {} --pc_rho {} --pc_theta {}'.format(fp_rho.value, fp_theta.value, pc_rho.value, pc_theta.value))
	
