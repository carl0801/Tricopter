#include <iostream>
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include "droneModel.hpp"
#include <chrono>

using namespace Eigen;
using namespace std;

VectorXf newtonSolver(Vector3f x, VectorXf s, VectorXf l,
                  Vector3f pt, Vector3f p0, Vector3f v0, Vector3f w0, Matrix3f R, 
                  float mu, float xMax, float xMin, float t){
    const VectorXf b = bFunction(x, s, l, pt, p0, v0, w0, R, mu, xMax, xMin, t);
    //cout << "b: " << b << endl;
    //cout << "b: " << b.size() << endl;
    const MatrixXf A = AFunction(x, s, l, pt, p0, v0, w0, R, mu, xMax, xMin, t);
    //cout << "A: " << A << endl;
    //cout << "A: " << A.rows() << "x" << A.cols() << endl;
    // solve Ax = b
    
    /*// Convert dense matrix A to sparse matrix
    SparseMatrix<float> sparseA = A.sparseView();

    // SimplicialLDLT solver
    SimplicialLDLT<SparseMatrix<float>> solverLDLT;
    solverLDLT.compute(sparseA);
    VectorXf result = solverLDLT.solve(b);*/

    VectorXf result = A.fullPivLu().solve(-b);
    return result;
}


Vector3f IPM(Vector3f x0, Vector3f pt, 
             Vector3f p0, Vector3f v0, Vector3f w0, Matrix3f R, 
             float xMax=5, float xMin=0, float timestep=0.2,
             int max_iter=15, float tol=1e-3){
    float mu = 1;
    int variables = 3;
    int constraints = 6;
    VectorXf s = 0.01 * VectorXf::Ones(constraints);
    VectorXf l = VectorXf::Ones(constraints);
    Vector3f x = x0;
    for (int i = 0; i < max_iter; ++i) {
        // Run Newton Solver
        VectorXf step = newtonSolver(x, s, l, pt, p0, v0, w0, R, mu, xMax, xMin, timestep);
        float cost = costFunction(x, s, l, pt, p0, v0, w0, R, mu, xMax, xMin, timestep);
        
        // Find Max step size for design varaibles
        float aMAx = 1;
        for (int j = 0; j < constraints; ++j) {
            float a = (0.005-1)*s(j)/step(variables+constraints+j);
            if (a > 0) {
                aMAx = min(a, aMAx);
            }
        }
        float aDesign = 0.99*aMAx;

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
            cout << "aDesign is too small" << endl;
            s = 0.01 * VectorXf::Ones(constraints);
        }
        // aLagrange
        if (aLagrange < tol) {
            cout << "aLagrange is too small" << endl;
            l = 0.01 * VectorXf::Ones(constraints);
        }

        // Check convergence
        float tolCurrent = costDerivative(x, s, l, pt, p0, v0, w0, R, mu, xMax, xMin, timestep).norm();
        cout << "Iteration: " << i << " Cost: " << cost << " Tolerance: " << tolCurrent << endl;      
        if (tolCurrent < tol) {
            cout << "Converged!" << endl;
            break;
        }
        // Update mu
        mu = mu * 0.7;
    }
    return x;
}



int main() {
    // Initial state
    Vector3f x0(0, 0, 0);
    Vector3f p0(0, 0, 0);
    Vector3f v0(0, 0, 0);
    Vector3f w0(0, 0, 0);
    Matrix3f R = Matrix3f::Identity();
    // Target position
    Vector3f pt(0, 0.2, 0.5);
    // Call Interior Point Method
    auto start = chrono::steady_clock::now();
    Vector3f x;
    for (int i = 0; i < 1; ++i) {
        x = IPM(x0, pt, p0, v0, w0, R);
    }
    auto end = chrono::steady_clock::now();
    cout << "Result: " << endl << x << endl;
    
    cout << "Elapsed time: " << chrono::duration_cast<chrono::milliseconds>(end - start).count() << " milliseconds" << endl;
    
    return 0;
}