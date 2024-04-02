#include <iostream>
#include <vector>
#include "Eigen/Dense"
#include "droneModel.h"
#include <chrono>

using namespace Eigen;
using namespace std;

const vector<float> time_array = {0.1, 0.2, 0.4, 0.8, 1.6, 2.4};

float f_lambda(const VectorXf& x, const Vector3f& goal_position, tuple<Vector3f, Vector3f, Quaternionf, Vector3f> state) {
    Eigen::Map<const Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> X(x.data(), time_array.size(), 3);
    Vector3f p = get<0>(state);
    Vector3f v = get<1>(state);
    Quaternionf q = get<2>(state);
    Vector3f w = get<3>(state);
    tuple<Vector3f, Vector3f, Quaternionf, Vector3f> next_state;

    for (size_t i = 0; i < time_array.size() - 1; ++i) {
        Vector3f x_current = X.row(i);
        next_state = droneModel(x_current, time_array[i], p, v, q, w);
        p = get<0>(next_state);
        v = get<1>(next_state);
        q = get<2>(next_state);
        w = get<3>(next_state);
    }
    Vector3f position = dronePosition(X.row(time_array.size() - 1), time_array[time_array.size() - 1], p, v, q);
    float error = (goal_position - position).norm();
    return error;
}

VectorXf fd_lambda(const VectorXf& x, const Vector3f& goal_position, tuple<Vector3f, Vector3f, Quaternionf, Vector3f> state) {
    VectorXf gradient(x.size());
    float f_lambda_value = f_lambda(x, goal_position, state);
    float epsilon = 1e-4; // Small perturbation for finite difference approximation
    for (int i = 0; i < x.size(); ++i) {
        VectorXf x_plus_delta = x;
        x_plus_delta[i] += epsilon;
        gradient[i] = -(f_lambda(x_plus_delta, goal_position, state) - f_lambda_value) / epsilon;
    }
    return gradient;
}

VectorXf gradientDecent(VectorXf& x, tuple<Vector3f, Vector3f, Quaternionf, Vector3f> state, Vector3f goal_position, 
                        float min_value=-INFINITY, float max_value=INFINITY, 
                        float tol=1e-3, int max_iter=25, float momentum=0.2, float learning_rate=0.1) {
    VectorXf velocity = VectorXf::Zero(x.size());
    float previous_tol = 1;
    int stale = 0;
    for (int i = 0; i < max_iter; ++i) {
        // Take random step
        x = x + VectorXf::Random(x.size()) * learning_rate * 0.1;

        // Compute gradient
        VectorXf gradient = fd_lambda(x, goal_position, state);
        
        // Update velocity
        velocity = (momentum * velocity + gradient) * learning_rate;
        
        // Line search
        float cost = f_lambda(x, goal_position, state);
        float newCost = f_lambda(x + velocity, goal_position, state);
        float a = 1;
        const int iterations = 3;
        const float a_multiplier = 0.5;
        VectorXf best_a = VectorXf::Ones(iterations) * 1e10;
        for (int j = 0; j < iterations; ++j) {
            best_a[j] = newCost;
            if (newCost > cost * 0.99) {
                a *= a_multiplier;
                newCost = f_lambda(x + velocity * a, goal_position, state);
                //cout << "Line search, a = " << round(a * 100) / 100 << ", cost = " << round(cost * 100) / 100 << ", newCost = " << round(newCost * 100) / 100 << endl;
            } else {
                break;
            }
        }
        // Find best a
        int best_index = 0;
        for (int j = 1; j < iterations; ++j) {
            if (best_a[j] < best_a[best_index]) {
                best_index = j;
            }
        }
        if (best_a[best_index] < cost * 0.99) {
            a = pow(a_multiplier, best_index) * a;
        } else {
            a = 0;
            stale++;
        }
        x += velocity * a;
        cout << "Best line cost: " << best_a[best_index] << ", a = " << a << ", X cost: " << f_lambda(x, goal_position, state) << endl;
        
        // Update learning rate
        learning_rate *= 0.7;
        
        // Update x
        x += velocity;
        // Clamp x to min and max values
        x = x.array().min(max_value).max(min_value);
        
        // Check if converged
        float current_tol = gradient.norm();
        
        cout << "Iteration " << i+1 << ", gradient = " << round(current_tol * 100) / 100 << ", velocity = " << round(velocity.norm() * 100) / 100 << ", cost = " << round(f_lambda(x, goal_position, state) * 100) / 100 << endl;
        
        if (round(previous_tol * 10000000) == round(current_tol * 10000000)) {
            stale++;
        }

        if (stale == 3) {
            cout << "Converged in " << i+1 << " iterations (Reached a plateau)" << endl;
            cout << "Final cost: " << f_lambda(x, goal_position, state) << endl;
            return x;
        }
        
        previous_tol = current_tol;
        
        if (current_tol < tol) {
            cout << "Converged in " << i+1 << " iterations" << endl;
            cout << "Final cost: " << f_lambda(x, goal_position, state) << endl;
            return x;
        }
    }
    cout << "Did not converge" << endl;
    cout << "Final cost: " << f_lambda(x, goal_position, state) << endl;
    return x;
}

int main() {
    // Define variables and constraints
    int variables = time_array.size()*3; // Example: Number of variables
    VectorXf x(variables);
    x.setOnes(); 
    x = x * 0.7; // Example: Initial guess
    float min_value = 0;
    float max_value = 1.5;
    Vector3f goal_position = Vector3f(0.3, 0, 1.5);
    Vector3f p0 = Vector3f(0, 0, 0);
    Vector3f v0 = Vector3f(0, 0, 0);
    Quaternionf q0 = Quaternionf(1, 0, 0, 0);
    Vector3f w0 = Vector3f(0, 0, 0);
    tuple<Vector3f, Vector3f, Quaternionf, Vector3f> state = make_tuple(p0, v0, q0, w0);

    
    // Call gradient descent function
    auto start = chrono::steady_clock::now();
    VectorXf result = gradientDecent(x, state, goal_position, min_value, max_value);
    auto end = chrono::steady_clock::now();
    cout << "Result: " << result.transpose() << endl;
    
    cout << "Elapsed time: " << chrono::duration_cast<chrono::milliseconds>(end - start).count() << " milliseconds" << endl;
    
    return 0;
}