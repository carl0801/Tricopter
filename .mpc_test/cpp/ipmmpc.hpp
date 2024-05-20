#ifndef IPM_MPC_HPP
#define IPM_MPC_HPP

#include <Eigen/Dense>
using namespace Eigen;

// Function to calcualte and solve matrix equation for Newton Solver
VectorXf newtonSolver(Vector3f x, VectorXf s, VectorXf l,
                  Vector3f pt, Vector3f p0, Vector3f v0, Vector3f w0, Matrix3f R, 
                  float mu, float xMax, float xMin, float t);

// Function to run the Interior Point Method for model predictive control
Vector3f IPMMPC(Vector3f x0, Vector3f pt, 
             Vector3f p0, Vector3f v0, Vector3f w0, Matrix3f R, 
             float xMax=7, float xMin=0, float timestep=0.5,
             int max_iter=18, float tol=1e-3, bool debug=false);

#endif