import ctypes

import numpy as np

# Define variables similar to Vector3f in C++
x0 = np.array([0, 0, 0], dtype=float)  # Position vector
p0 = np.array([0, 0, 0], dtype=float)  # Momentum vector
v0 = np.array([0, 0, 0], dtype=float)  # Velocity vector
w0 = np.array([0, 0, 0], dtype=float)  # Angular velocity vector
pt = np.array([0, 0, 0], dtype=float)  # Target position vector

# Define a variable similar to Matrix3f in C++
R = np.identity(3, dtype=float)  # Identity matrix (3x3)



# Load the shared library
sample_lib = ctypes.CDLL('.mpc_test\cpp\libipmmpc.dll')
# Call the C++ function
sample_lib.IPMMPC(x0, pt, p0, v0, w0, R)

import ctypes

# Load the shared library
lib = ctypes.CDLL('path/to/your/library.dll')

# Create the IPMMPC object
ipmmpc = lib.create_ipmmpc()

# Call methods on IPMMPC
# ...

# Destroy the IPMMPC object when you're done with it
lib.destroy_ipmmpc(ipmmpc)