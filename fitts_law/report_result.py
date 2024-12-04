# -*- coding: cp949 -*-
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# Read the CSV files for mouse, touchpad, and air-mouse
df_mouse = pd.read_csv('../데이터/20241127/total_mouse.txt')
df_touchpad = pd.read_csv('../데이터/20241127/total_touchpad.txt')
df_airmouse = pd.read_csv('../데이터/20241127/total_airmouse.txt')

# Remove any leading/trailing whitespace from column names
df_mouse.columns = df_mouse.columns.str.strip()
df_touchpad.columns = df_touchpad.columns.str.strip()
df_airmouse.columns = df_airmouse.columns.str.strip()

# Drop the 'Trial' column from each dataframe
df_mouse = df_mouse.drop(columns=['Trial'])
df_touchpad = df_touchpad.drop(columns=['Trial'])
df_airmouse = df_airmouse.drop(columns=['Trial'])

# Remove the top 70% of data points with the highest times for each device
df_mouse = df_mouse[df_mouse['Time'] <= df_mouse['Time'].quantile(0.95)]
df_touchpad = df_touchpad[df_touchpad['Time'] <= df_touchpad['Time'].quantile(0.70)]
df_airmouse = df_airmouse[df_airmouse['Time'] <= df_airmouse['Time'].quantile(0.70)]

# Calculate ID using W and D for each dataframe
df_mouse['ID'] = np.log2(df_mouse['D'] / df_mouse['W'] + 1)
df_touchpad['ID'] = np.log2(df_touchpad['D'] / df_touchpad['W'] + 1)
df_airmouse['ID'] = np.log2(df_airmouse['D'] / df_airmouse['W'] + 1)

# Extracting ID and Time columns for each device
ids_mouse = df_mouse['ID'].values
times_mouse = df_mouse['Time'].values

ids_touchpad = df_touchpad['ID'].values
times_touchpad = df_touchpad['Time'].values

ids_airmouse = df_airmouse['ID'].values
times_airmouse = df_airmouse['Time'].values

# Scatter plot of the data for each device
plt.figure(figsize=(7, 5))  
plt.scatter(ids_mouse, times_mouse, edgecolor='blue', facecolors='none', marker='s', s=40, label='Conventional Mouse')  # Square marker for mouse with size 40
plt.scatter(ids_touchpad, times_touchpad, edgecolor='green', facecolors='none', marker='^', s=40, label='Touchpad')  # Triangle marker for touchpad with size 40
plt.scatter(ids_airmouse, times_airmouse, edgecolor='red', facecolors='none', marker='o', s=40, label='Air-mouse')  # Circle marker for air-mouse with size 40

# Linear regression for each device
A_mouse = np.vstack([ids_mouse, np.ones(len(ids_mouse))]).T
m_mouse, b_mouse = np.linalg.lstsq(A_mouse, times_mouse, rcond=None)[0]
plt.plot(ids_mouse, m_mouse * ids_mouse + b_mouse, 'b--', linewidth=2, label='Conventional mouse Regression')
plt.text(5.5, 0.7, f'a = {m_mouse:.4f}\nb = {b_mouse:.4f}', color='blue', fontsize=10)

A_touchpad = np.vstack([ids_touchpad, np.ones(len(ids_touchpad))]).T
m_touchpad, b_touchpad = np.linalg.lstsq(A_touchpad, times_touchpad, rcond=None)[0]
plt.plot(ids_touchpad, m_touchpad * ids_touchpad + b_touchpad, 'g--', linewidth=2, label='Touchpad Regression')
plt.text(5.5, 1.6, f'a = {m_touchpad:.4f}\nb = {b_touchpad:.4f}', color='green', fontsize=10)

A_airmouse = np.vstack([ids_airmouse, np.ones(len(ids_airmouse))]).T
m_airmouse, b_airmouse = np.linalg.lstsq(A_airmouse, times_airmouse, rcond=None)[0]
plt.plot(ids_airmouse, m_airmouse * ids_airmouse + b_airmouse, 'r--', linewidth=2, label='Air-mouse Regression')
plt.text(5.5, 1.2, f'a = {m_airmouse:.4f}\nb = {b_airmouse:.4f}', color='red', fontsize=10)

# Print slopes and intercepts for each device
print(f"Conventional Mouse: Slope (a) = {m_mouse:.4f}, Intercept (b) = {b_mouse:.4f}")
print(f"Touchpad: Slope (a) = {m_touchpad:.4f}, Intercept (b) = {b_touchpad:.4f}")
print(f"Air-mouse: Slope (a) = {m_airmouse:.4f}, Intercept (b) = {b_airmouse:.4f}")

# Configure plot format similar to IEEE paper
plt.xlabel('Index of Difficulty (ID)')
plt.ylabel('Time (seconds)')
plt.ylim(0, 2.5)  # Increase the y-axis limit to 2.5
plt.xlim(0.5, 6.5) 
plt.grid(True)
plt.legend(loc='upper right', ncol=2)  # Set legend to 2 columns
plt.savefig('fitts_law_results.png')  # Save the figure as a file
plt.show()

