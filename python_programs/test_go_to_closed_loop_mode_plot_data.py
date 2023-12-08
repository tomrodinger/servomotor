#!/usr/bin/env python3

import glob
import pandas as pd
import matplotlib.pyplot as plt

# Get list of all relevant files
DIRECTORY = "logs"
GLOB_SEARCH_STRING = DIRECTORY + "/X_data_*"
files = sorted(glob.glob(GLOB_SEARCH_STRING), key=lambda x:int(x.split('_')[-1]))

# Loop through the files, read each one into a dataframe and plot it
for filename in files:
    data = pd.read_csv(filename, sep=' ', header=None)
    plt.plot(data[0], data[1])

plt.xlabel('Index')
plt.ylabel('Data value')
plt.title('All data')
plt.legend() # To show a legend with the file names
plt.show()
