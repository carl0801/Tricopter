#include <iostream>
#include <Eigen/Dense>
#include <chrono>
#include "ipmmpc.hpp"

using namespace Eigen;
using namespace std;


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
        // randomize pt
        //pt = Vector3f::Random();
        // randomize x0
        //x0 = Vector3f::Random();
        x = IPMMPC(x0, pt, p0, v0, w0, R);
    }
    auto end = chrono::steady_clock::now();
    cout << "Result: " << endl << x << endl;
    
    cout << "Elapsed time: " << chrono::duration_cast<chrono::milliseconds>(end - start).count() << " milliseconds" << endl;
    
    return 0;
}