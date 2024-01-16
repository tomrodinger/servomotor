#!/usr/bin/env python3

import pandas as pd
import matplotlib.pyplot as plt
from tabulate import tabulate

# Read in the data and sort it
data = pd.read_csv('analysis_output/best_weights_from_all_trials', sep="\s+", header=None)
data = data.sort_values(by=0).reset_index(drop=True)

# Rename columns for clarity
columns_names = ['Magnetic Ring Number'] + [f'Weight {i}' for i in range(1, 10)]
data.columns = columns_names

# Calculate averages for the weights
weight_averages = data.iloc[:, 1:].mean()

# Create table for console output
table_data = [(i, weight_averages[f'Weight {i}']) for i in range(1, 10)]
print(tabulate(table_data, headers=['Weight Index', 'Average Weight'], tablefmt="presto"))

# Plotting
fig, axes = plt.subplots(nrows=3, ncols=3, figsize=(15, 10))
fig.tight_layout(pad=5.0)

# Iterate over each weight to create subplot
for i, col in enumerate(columns_names[1:], 1):
    row, col_idx = (i - 1) // 3, (i - 1) % 3
    ax = axes[row, col_idx]
    
    ax.plot(data['Magnetic Ring Number'], data[f'Weight {i}'], marker='o')
    ax.set_title(f'Weight {i}')
    
    # Set consistent y-axis limits
    ax.set_ylim(-0.5, 1.5)

    # Add thicker black line at y=0 or y=1 and blue line at the average
    hline_y = 1.0 if i in [1, 4, 7] else 0.0
    ax.axhline(y=hline_y, color='black', linewidth=1.5)
    ax.axhline(y=weight_averages[f'Weight {i}'], color='blue', linewidth=1.0, linestyle='--')
    
    # Add gridlines
    ax.grid(color='gray', linestyle='--', linewidth=0.5, which='both')

    # Add labels
    if row == 2:  # Only label x-axis on bottom row
        ax.set_xlabel('Magnetic Ring Number')
    if col_idx == 0:  # Only label y-axis on first column
        ax.set_ylabel('Weight')

plt.show()
