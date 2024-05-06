#include <ArduinoEigen.h>
#include "droneModel.hpp"

using namespace Eigen;

VectorXf newtonSolver(Vector3f x, VectorXf s, VectorXf l,
                  Vector3f pt, Vector3f p0, Vector3f v0, Vector3f w0, Matrix3f R, 
                  float mu, float xMax, float xMin, float t){
    const VectorXf b = bFunction(x, s, l, pt, p0, v0, w0, R, mu, xMax, xMin, t);
    const MatrixXf A = AFunction(x, s, l, pt, p0, v0, w0, R, mu, xMax, xMin, t);
    // solve Ax = b
    VectorXf result = A.fullPivLu().solve(-b);
    return result;
}


Vector3f IPMMPC(Vector3f x0, Vector3f pt, 
             Vector3f p0, Vector3f v0, Vector3f w0, Matrix3f R, 
             float xMax, float xMin, float timestep,
             int max_iter, float tol, bool debug){
    float mu = 1;
    int variables = 3;
    int constraints = 6;
    VectorXf s = 0.01 * VectorXf::Ones(constraints);
    VectorXf l = VectorXf::Ones(constraints);
    Vector3f x = x0;

    // Make sure p0-pt is not more than 1m
    if ((p0 - pt).norm() > 1) {
        // Normalize pt
        pt = pt / pt.norm();
    }

    for (int i = 0; i < max_iter; ++i) {
        // Run Newton Solver
        VectorXf step = newtonSolver(x, s, l, pt, p0, v0, w0, R, mu, xMax, xMin, timestep);
        float cost = costFunction(x, s, l, pt, p0, v0, w0, R, mu, xMax, xMin, timestep);
        
        // Find Max step size for design varaibles
        float aDesign = 1;
        for (int j = 0; j < constraints; ++j) {
            float a = (0.005-1)*s(j)/step(variables+constraints+j);
            if (a > 0) {
                aDesign = min(a, aDesign);
            }
        }

        // Line search
        float newCost = costFunction(x + aDesign*step.head(variables), s + aDesign*step.segment(variables, constraints), l + aDesign*step.tail(constraints), pt, p0, v0, w0, R, mu, xMax, xMin, timestep);
        while (newCost > cost + 1e-4*aDesign*step.head(variables).norm()) {
            aDesign = 0.9*aDesign;
            newCost = costFunction(x + aDesign*step.head(variables), s + aDesign*step.segment(variables, constraints), l + aDesign*step.tail(constraints), pt, p0, v0, w0, R, mu, xMax, xMin, timestep);
            if (aDesign < tol) {
                break;
            }
        }

        // Max step size for lagrange variables
        float aLagrange = 1;
        for (int j = 0; j < constraints; ++j) {
            float a = (0.005-1)*l(j)/step(variables+j);
            if (a > 0) {
                aLagrange = min(a, aLagrange);
            }
        }
    
        // Rescale step
        // Design variables
        for (int i = 0; i < variables; ++i) {
            step[i] *= aDesign;
        }
        // Lagrange multipliers
        for (int i = variables; i < variables + constraints; ++i) {
            step[i] *= aLagrange;
        }
        // Slack variables
        for (int i = variables + constraints; i < step.size(); ++i) {
            step[i] *= aDesign;
        }
        // Update x, s, l
        x += step.head(variables);
        s += step.segment(variables, constraints);
        l += step.tail(constraints);
    
        // Check if max step size is too small
        // aSlack
        if (aDesign < tol) {
            s = 0.01 * VectorXf::Ones(constraints);
        }
        // aLagrange
        if (aLagrange < tol) {
            l = 0.01 * VectorXf::Ones(constraints);
        }

        // Check convergence
        float tolCurrent = costDerivative(x, s, l, pt, p0, v0, w0, R, mu, xMax, xMin, timestep).norm();     
        if (tolCurrent < tol) {
            break;
        }
        // Update mu
        mu = mu * 0.7;
    }
    // Clip x
    x(0) = min(x(0), xMax);
    x(1) = min(x(1), xMax);
    x(2) = min(x(2), xMax);
    x(0) = max(x(0), xMin);
    x(1) = max(x(1), xMin);
    x(2) = max(x(2), xMin);

    float finalCost = costFunction(x, s, l, pt, p0, v0, w0, R, mu, xMax, xMin, timestep);
    return x;
}