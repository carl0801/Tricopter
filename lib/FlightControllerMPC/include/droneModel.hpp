
#ifndef DRONE_MODEL_HPP
#define DRONE_MODEL_HPP

#include <ArduinoEigen.h>

using namespace Eigen;

// Define a helper function to assign variables
void assignVariables(const Vector3f& f, const VectorXf& s, const VectorXf& l, 
                     const Vector3f& pt, const Vector3f& p0, const Vector3f& v0, const Vector3f& w0, const Matrix3f& R,
                     float& f1, float& f2, float& f3,  
                     float& s0, float& s1, float& s2, float& s3, float& s4, float& s5,
                     float& l0, float& l1, float& l2, float& l3, float& l4, float& l5,
                     float& ptx, float& pty, float& ptz, float& p0x, float& p0y, float& p0z, 
                     float& v0x, float& v0y, float& v0z, float& w0x, float& w0y, float& w0z,
                     float& r00, float& r01, float& r02, float& r10, float& r11, float& r12,
                     float& r20, float& r21, float& r22);

// Define the function to be optimized
float costFunction(Vector3f f, VectorXf s, VectorXf l,
                   Vector3f pt, Vector3f p0, Vector3f v0, Vector3f w0, Matrix3f R, 
                   float mu, float fMax, float fMin, float t);

// Define the derivative of the function to be optimized
Vector3f costDerivative(Vector3f f, VectorXf s, VectorXf l,
                   Vector3f pt, Vector3f p0, Vector3f v0, Vector3f w0, Matrix3f R, 
                   float mu, float fMax, float fMin, float t);

// Define the b vector function for IPM
VectorXf bFunction(Vector3f f, VectorXf s, VectorXf l,
                   Vector3f pt, Vector3f p0, Vector3f v0, Vector3f w0, Matrix3f R, 
                   float mu, float fMax, float fMin, float t);

// Define the A matrix function for IPM 
MatrixXf AFunction(Vector3f f, VectorXf s, VectorXf l,
                   Vector3f pt, Vector3f p0, Vector3f v0, Vector3f w0, Matrix3f R, 
                   float mu, float fMax, float fMin, float t);

#endif
