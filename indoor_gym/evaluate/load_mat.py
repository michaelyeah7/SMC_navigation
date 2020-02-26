#!python
#!/usr/bin/env python
from scipy.io import loadmat
x = loadmat('THOR_dataset/Exp_1_run_1.mat')
# lon = x['Timestamp']
# print(x.shape())
# lat = x['lat']
# # one-liner to read a single variable
# lon = loadmat('test.mat')['lon']
# print(lon)
print x['Experiment_1_run_1_0050']['Trajectories'][0][0][0]['']
print x.keys()