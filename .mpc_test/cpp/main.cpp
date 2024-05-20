#include "ipmmpc.hpp"
#include <iostream>
#include <Eigen/Dense>
#include <chrono>

using namespace Eigen;
using namespace std;


int main() {
    //cout << "Enter drone state (p0,v0,w0,R,pt): " << endl;
    // Initial state
    Vector3f x0(0, 0, 0);
    Vector3f p0(0, 0, 0);
    Vector3f v0(0, 0, 0);
    Vector3f w0(0, 0, 0);
    Matrix3f R = Matrix3f::Identity();
    // Target position
    Vector3f pt(0, 0, 0);
    // Control loop 
    while (true) {
        // Recive drone state
        string message;
        getline(std::cin, message);
        if (message == "exit") {
            break;
        }
        // Create a stringstream from the input string
        std::stringstream ss(message);
        // Variables to store the separated float values
        float p00, p01, p02, v00, v01, v02, w00, w01, w02, r00, r01, r02, r10, r11, r12, r20, r21, r22, pt0, pt1, pt2;
        char comma; // To store the comma
        // Read the float values separated by commas
        ss >> p00 >> comma >> p01 >> comma >> p02 >> comma >> v00 >> comma >> v01 >> comma >> v02 >> comma >> w00 >> comma >> w01 >> comma >> w02 >> comma >> r00 >> comma >> r01 >> comma >> r02 >> comma >> r10 >> comma >> r11 >> comma >> r12 >> comma >> r20 >> comma >> r21 >> comma >> r22 >> comma >> pt0 >> comma >> pt1 >> comma >> pt2;
        // Update drone state
        p0 << p00, p01, p02;
        v0 << v00, v01, v02;
        w0 << w00, w01, w02;
        R << r00, r01, r02, r10, r11, r12, r20, r21, r22;
        pt << pt0, pt1, pt2;
        
        // Call Interior Point Method
        Vector3f x;
        for (int i = 0; i < 1; ++i) {
            x = IPMMPC(x0, pt, p0, v0, w0, R);
        }
        x0 = x;
        // Send output
        cout << x[0] << "," << x[1] << "," << x[2] << endl;
        //auto start = chrono::steady_clock::now();
        //cout << "Elapsed time: " << chrono::duration_cast<chrono::milliseconds>(end - start).count() << " milliseconds" << endl;
    }
    return 0;
}