# About RegARD

The **Reg**istration based on **A**rchitectural **R**eflection **D**etection (RegARD) is a free and fast library for building layout registration from point sets, i.e., points sampled from point clouds or floor plans.

# How to cite

[Wu, Y.](mailto:wuyijie@hku.hk?subject=[GitHub]RegARD), [Shang J.](mailto:jgshang@cug.edu.cn?subject=[GitHub]RegARD), and [Xue F.*](mailto:xuef@hku.hk?subject=[GitHub]RegARD). RegARD: Symmetry-based Coarse Registration of Smartphone’s Colorful Point Clouds with CAD Drawings for Low-cost Digital Twin Buildings. (under review)

# How does it work

First, RegARD detects the innate architectural reflection symmetries based on [ODAS](//github.com/ffxue/odas) for the globally optimal **rotation** constants and proposes an initial alignment based on the symmetry axes. 
Then, RegARD applies advanced derivative-free optimization (DFO) algorithms to optimize the other degrees of freedom (5-DoF), i.e., translation and scaling. 

# Why RegARD is fast for building layout registration
The 5-DoF registration for building layouts is challenging due to the omnipresent self-similarities inside buildings. RegARD separates the 5-DoF problem into two subproblems: i.e., one for determining the possible rotations and the other for optimizing the translation and scaling. This indicating we only optimize 4 DoF based on highly accurate rotation, which significantly improves the registration efficiency. 

# Dependencies

Codes have only been tested on Ubuntu 18.04 and 20.04. The following guidelines are mainly for Ubuntu. 
Welcome any other contributions or experiments on the other platform :)

- [nlopt](//nlopt.readthedocs.io/) 
Installation for both C++ and Python is required. 
- [libcmaes](//github.com/beniz/libcmaes)   
- [open3d](//pypi.org/project/open3d/)
- [numpy](//pypi.org/project/numpy/)
- [odas](//github.com/ffxue/odas)
We provided a simplified odas for RegARD in the "odas-RegARD" fold. The libodas.so can be built from the source of odas-RegARD and then be imported by the ard.py. Or if the local locations and versions of the dependencies of odas are same as those of odas-RegARD, you can use the complied libodas.so directly.
- Python version: 3.8

# Registration

Note that the two point sets for registration are sampled from the CAD drawing and the 2D projected point cloud respectively. So, the code and args named the data as 'fp' and 'pc' or 'as-designed' and 'as-built'.

- Detect the architectural reflection axis
```sh
python3 ard.py --fp <fp-pcd> --pc <pc-pcd> --rmse_t <threshold of RMSE>
```
Pass the pcd files of the two point sets. Then the command will return the r and theta of the symmetry axes. 

If both RMSEs of 'fp' and 'pc' are less then the threshold, then we suggest use the 4 DoF transformation optimization. You can copy the outputted parameters of the symmetric axes '--fp_r <fp_r> --fp_theta <fp_theta> --pc_r <pc_r> --pc_theta <pc_theta>' and pass to the reg.py.

Otherwise, we suggest using the 5 DoF transformation optimization. 

- Registration
  - 4-DoF registration 
```sh
python3 reg.py --fp <fp-pcd> --pc <pc-pcd> --reg_fig <reg-fig> --ard_fig <ard-fig> --fp_r <fp_r> --fp_theta <fp_theta> --pc_r <pc_r> --pc_theta <pc_theta>
```
Pass the pcd files of the two point sets, the path of registration figure, and the results of architectural symmetry detection. Then the command will output the graphic result of the registration in the <reg-fig>
  - 5 DoF registration
```sh
python3 reg.py --fp <fp-pcd> --pc <pc-pcd> --reg_fig <reg-fig>
```
- Graphic example
<img src="./before.png" alt="Before registration" width="400"/>
<img src="./after.png" alt="After registration" width="400"/>

# Authors' Contacts

- Wu, Y.: [wuyijie@hku.hk](mailto:wuyijie@hku.hk?subject=[GitHub]RegARD)
- Xue, F.: [xuef@hku.hk](mailto:xuef@hku.hk?subject=[GitHub]RegARD), [homepage](//frankxue.com/)

# License

LGPL-3.0

# Acknowledgements

This work was supported by the Research Grant Council (RGC) of Hong Kong SAR (Nos. 17200218, 27200520).
