#%%|
import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

df = pd.read_csv("/home/nothecats/Documents/project/ASDfR/src/Assignment2.1/ros2_timing_analysis/src/timing_log.csv").to_numpy()


# %%
timing = []
for index, element in enumerate(df):
    index+=1
    stripElement = "Iteration " + str(index) + " - Execution Time:"
    element[0] = (element[0].strip(stripElement))
    timing.append(element[0].strip("ms "))
# %%
plt.plot(timing)
# %%
