import numpy as np
import csv 
import random

# Make a random nx3 array
n = 100
data = np.random.rand(n, 4)

reachedHeight = False
targetHeight = 1.5
for i in range(n):
    if not reachedHeight:
        data[i][2] = data[i][0] + i*0.05
    else:
        data[i][2] = data[i][0] + (targetHeight-0.5)
    if data[i][2] > targetHeight and not reachedHeight:
        reachedHeight = True
        print(f"Reached height at index {i}")

for i in range(n):
    noise = random.random() * 0.04
    data[i][3] = i*1 + noise

print(data)

# Write to csv file
with open('test/test.csv', mode='w', newline="") as f:
    writer = csv.writer(f)
    writer.writerow(['x', 'y', 'z', 'time'])
    for row in data:
        writer.writerow(row)