import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import glob
from datetime import datetime
import re


def extract_number(filename):
    match = re.search(r'\d+', filename)
    return int(match.group()) if match else 0


# Suppose we want to show only the first and third plots
custom_colors = [
    (38 / 255, 70 / 255, 83 / 255),  # R:038 G:070 B:083
    (42 / 255, 157 / 255, 142 / 255),  # R:042 G:157 B:142
    (233 / 255, 196 / 255, 107 / 255),  # R:233 G:196 B:107
    (244 / 255, 162 / 255, 97 / 255),  # R:243 G:162 B:097
    (230 / 255, 111 / 255, 81 / 255)  # R:230 G:111 B:081
]
plots_to_show = [True, True, True, True]
custom_legend_names = ['Drive-fuzz', 'Origin', 'TM-fuzzer-0.4', 'TM-fuzzer-0.8', 'TM-fuzzer-1.2']

# Find all log files in the current directory matching the pattern 'record_x.log'
log_files = sorted(glob.glob('record_*.log'), key=extract_number)

# Check how many plots we need to show based on our plots_to_show list
num_plots_to_show = sum(plots_to_show)

# Initialize plot with only the subplots we want to show
fig, axs = plt.subplots(1, num_plots_to_show, figsize=(3 * num_plots_to_show, 4))

# If we have only one plot to show, axs is not a list but a single object, so we make it a list
if num_plots_to_show == 1:
    axs = [axs]

# Ensure the number of custom legend names matches the number of log files
assert len(custom_legend_names) == len(log_files), "Number of custom legends must match number of log files"

# Store the legend labels for use in a single legend
legend_labels = []
# plt.cm.viridis(np.linspace(0, 1, len(log_files)))
# Process each log file
for log_file, color, custom_legend_name in zip(log_files, custom_colors, custom_legend_names):
    with open(log_file, 'r') as file:
        log_data = file.read()

    log_lines = log_data.strip().split('\n')

    # Initialize variables to hold data for plotting
    timestamps = []
    crashed_counts = []
    valid_frames_totals = []
    num_frames_totals = []
    distance_totals = []

    # Initialize accumulators
    crashed_count = 0
    valid_frames_total = 0
    num_frames_total = 0
    distance_total = 0

    # Process each line in the log file
    for line in log_lines:
        # Extract the timestamp and data
        timestamp_str, data = line.split(' - ')
        timestamp = datetime.strptime(timestamp_str, '%Y-%m-%d %H:%M:%S,%f')

        # Extract and process data based on the key
        if 'crashed' in data:
            crashed = data.split(':')[-1].strip() == 'True'
            if crashed:
                crashed_count += 1
        elif 'valid_time' in data:
            match = re.search(r'valid_time/time:\s*([\d.]+)/([\d.]+)', data)
            valid_time, num_time = map(float, match.groups())
            valid_frames_total += valid_time / 3600
            num_frames_total += num_time / 3600
        elif 'distance' in data:
            distance = float(data.split(':')[-1].strip())
            distance_total += distance
        # Append data to lists for plotting
        timestamps.append(timestamp)
        crashed_counts.append(crashed_count)
        valid_frames_totals.append(valid_frames_total)
        num_frames_totals.append(num_frames_total)
        distance_totals.append(distance_total)
    if num_frames_total > 0:
        ratio = valid_frames_total / num_frames_total
        print(f"{custom_legend_name} - TET-COV/Simulate Time ratio: {ratio:.4f}")
    else:
        print(f"{custom_legend_name} - TET-COV/Simulate Time ratio: Undefined (Simulate Time is 0)")
    # Convert lists to a DataFrame for easier plotting
    df = pd.DataFrame({
        'Timestamp': timestamps,
        'Crashed': crashed_counts,
        'TET-COV(h)': valid_frames_totals,
        'Simulate(h)': num_frames_totals,
        'Distance(km)': distance_totals
    })

    # Calculate seconds from the start time and convert to hours
    start_time = df['Timestamp'].min()
    df['Hours'] = (df['Timestamp'] - start_time).dt.total_seconds() / 3600

    # Convert distance to kilometers
    df['Distance(km)'] = df['Distance(km)'] / 1000
    df_filtered = df[df['Hours'] <= 24]  # Make sure this line is after the conversion

    # Mapping of plot titles to their respective data
    plot_data = {
        'Crashed Count Over Time': ('Hours', 'Crashed'),
        'TET-COV Over Time': ('Hours', 'TET-COV(h)'),
        'Distance Over Time': ('Hours', 'Distance(km)'),
        'Simulate Time Over Time': ('Hours', 'Simulate(h)')
    }

    # Counter for the current plot index
    current_plot_index = 0

    # Loop over the plot_data items and plot only if the corresponding flag in plots_to_show is True
    for i, (title, (x, y)) in enumerate(plot_data.items()):
        if plots_to_show[i]:
            ax = axs[current_plot_index]
            ax.plot(df_filtered[x], df_filtered[y], label=f'{custom_legend_name}', color=color)  # Removed {y} from label
            ax.set_title(title)
            ax.set_xlabel('Time (h)')
            ax.set_ylabel(y.split(' ')[0])
            ax.set_xlim([0, 25])
            current_plot_index += 1

            # Add the label for the legend
            if custom_legend_name not in legend_labels:
                legend_labels.append(custom_legend_name)

# Create a single horizontal legend at the bottom
fig.legend(legend_labels, loc='lower center', ncol=len(legend_labels))

# Adjust layout and show only the desired plots
plt.tight_layout(rect=[0, 0.1, 1, 0.95])
plt.show()
