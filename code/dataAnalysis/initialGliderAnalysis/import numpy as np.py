import numpy as np 
import matplotlib.pyplot as plt 
import xarray as xr
import os

files_path = "/Users/calebflaim/Documents/toAddToThesis/glider_dives"
dive_files = os.listdir(files_path)

print(dive_files)

for file in dive_files:
    data = xr.open_dataset(files_path + "/" + file)
    print(data.head())
    for i in range(10): print()