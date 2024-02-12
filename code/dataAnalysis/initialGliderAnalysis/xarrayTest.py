import xarray as xr 
import pandas as pd 
import numpy as np 
import matplotlib.pyplot as plt 
import os 

data_path = "/Users/calebflaim/Documents/thesis/openFloat/code/dataAnalysisScripts/initialGliderAnalysis/glider_dives/"
files = os.listdir(data_path)
files.remove('.DS_Store')

for file in files:
    data_set = xr.open_dataset(data_path+file)

    # display(data_set)
    print("rho: ", data_set['log_RHO'].values)
    print("Mass: ", data_set['log_MASS'].values)
    print("VBD min: ", data_set['log_VBD_MIN'].values)
    print("VBD max: ", data_set['log_VBD_MAX'].values)
    print("C VBD: ", data_set['log_C_VBD'].values)
    print("sg_cal_volmax: ", data_set['sg_cal_volmax'].values)
    print("Max buoy: ", data_set['log_MAX_BUOY'].values)
    print()



#log_RHO
#log_MASS
#log_VBD_MIN
#log_VBD_MAX
#log_C_VBD
