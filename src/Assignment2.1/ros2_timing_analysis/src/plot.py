import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import os

os.chdir(os.path.dirname(os.path.abspath(__file__)))
filenames = ['timing_log_no_load_evl.csv','timing_log_with__no_load_linux.csv', 'timing_log_with_load_evl_bash.csv',
            'timing_log_with_load_evl_stress.csv', 'timing_log_with_load_linux_cmd.csv', 'timing_log_with_load_linux_stress.csv']

for filename in filenames:
    # Read the CSV file into a DataFrame
    df = pd.read_csv('timing_logs/'+filename).to_numpy()

    # Extract timing data from the DataFrame
    timing = []
    for index, element in enumerate(df):
        index += 1
        stripElement = 'Iteration ' + str(index) + ' - Execution Time:'
        element[0] = (element[0].strip(stripElement))
        timing.append(element[0].strip('ms '))

    # Convert timing data to float for plotting
    timing = list(map(float, timing))

    timing = np.array(timing)
    mean_timing = np.mean(timing)
    jitter = np.abs(timing - mean_timing)

    mean_jitter = np.mean(jitter)
    jitter_stddev = np.std(jitter)

    mean_jitter_for_plot = [mean_jitter] * len(timing)

    # Extract base filename (remove .csv extension)
    filename = os.path.splitext(filename)[0]

    # Plot the timing data as data points without connecting them
    plt.scatter(range(1, len(timing) + 1), timing)
    plt.xlabel('Iteration')
    plt.ylabel('Execution Time (ms)')
    plt.title('Execution Time per Iteration')
    # Save the plot to a file
    plt.savefig('timing_logs/'+ filename + '.png')
    print('Plot saved as ' + filename + '.png')
    plt.clf()

    # Plot the jitter data as data points without connecting them
    plt.scatter(range(1, len(timing) + 1), jitter, color='blue', label='jitter data')
    plt.plot(range(1, len(mean_jitter_for_plot)+1), mean_jitter_for_plot, color='red', label='average jitter')
    plt.xlabel('Iteration')
    plt.ylabel('Jitter time (ms)')
    plt.title('Jitter per Iteration')
    plt.legend()
    # Save the plot to a file
    plt.savefig('timing_logs/jitter_'+ filename + '.png')
    
    print('Jitter plot saved as ' + filename + '.png')
    plt.clf()
    
    print('Average jitter for ' + filename + '= ' + str(mean_jitter))
    print('Standard deviation in jitter for ' + filename + '= ' + str(jitter_stddev))
    