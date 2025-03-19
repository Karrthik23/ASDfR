import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import os

os.chdir(os.path.dirname(os.path.abspath(__file__)))
filename = "timing_log_with_load_linux_stress.csv"

# Read the CSV file into a DataFrame
df = pd.read_csv("timing_logs/"+filename).to_numpy()

# Extract timing data from the DataFrame
timing = []
for index, element in enumerate(df):
    index += 1
    stripElement = "Iteration " + str(index) + " - Execution Time:"
    element[0] = (element[0].strip(stripElement))
    timing.append(element[0].strip("ms "))

# Convert timing data to float for plotting
timing = list(map(float, timing))

# Plot the timing data as data points without connecting them
plt.scatter(range(1, len(timing) + 1), timing)
plt.xlabel('Iteration')
plt.ylabel('Execution Time (ms)')
plt.title('Execution Time per Iteration')

# Save the plot to a file
plt.savefig('timing_logs/'+ filename + '.png')

print("Plot saved as " + filename + ".png")