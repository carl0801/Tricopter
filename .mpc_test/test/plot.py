# Description: Plot the data from csv file
import matplotlib.pyplot as plt
import numpy as np

# Read csv file into array [[x,y,z]]
data = np.genfromtxt('test/position.csv', delimiter=',')

# Find first index where z > 1.5, and remove all indices up to that point
targetHeight = 1.5
for i in range(len(data)):
    if data[i][2] > targetHeight:
        timeOffset = data[i][3]
        startCoordinate = [data[i][0], data[i][1], data[i][2]]
        break
data = data[i:]

# Normalize coordinates
data[:,0] -= startCoordinate[0]
data[:,1] -= startCoordinate[1]
data[:,2] -= startCoordinate[2]

# Normalize time
data[:,3] -= timeOffset

# Cut off data after time = 60
maxTime = 60
data = data[data[:,3] <= maxTime]

# Plot each axis against time
plt.figure(figsize=(10, 5))
plt.plot(data[:,3], data[:,0], label='x')
plt.plot(data[:,3], data[:,1], label='y')
plt.plot(data[:,3], data[:,2], label='z')
plt.axhline(y=0, color='r', linestyle='--', label='Target')
plt.xlabel('Time (s)')
plt.ylabel('Offset (mm)')
plt.legend()
# Save plot to file
plt.savefig('test/plot.png')

# Make boxplot of x, y, z
plt.figure(figsize=(6, 5))
plt.boxplot(data[:,0:3], labels=['x', 'y', 'z'], showmeans=True, meanline=True, meanprops={'label':'Mean'}, medianprops={'label':'Median'}, flierprops={'markersize':2, 'label':'Outliers'})
plt.axhline(y=0, color='r', linestyle='--', label='Target')
plt.ylabel('Offset (mm)')
# Remove duplicate legend entries
handles, labels = plt.gca().get_legend_handles_labels()
by_label = dict(zip(labels, handles))
plt.legend(by_label.values(), by_label.keys())
# Save plot to file
plt.savefig('test/boxplot.png')

