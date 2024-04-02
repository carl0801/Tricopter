# Already defined cpp functions:
# vector<vector<float>> droneModel(vector<float> f, float t, vector<float> p0, vector<float> v0, vector<float> w0, vector<float> q){
# returns:
# vector<float> p(3);
# vector<float> v(3);
# vector<float> quaternion(4);
# vector<float> w(3);
# return {{p, v, quaternion, w}}; 

# vector<float> dronePosition(vector<float> f, float t, vector<float> p0, vector<float> v0, vector<float> q){
# returns: vector<float> p(3);

# The function i want to convert to cpp:
def f_lambda(f, goal_position, p0, v0, q0, w0):
    time_array = [0.02, 0.1, 0.2, 0.5, 1, 1.2]
    p = p0
    v = v0
    q = q0
    w = w0

    for i in range(len(time_array)-1):
        p, v, q, w = droneModel(f[i], time_array[i], p, v, w, q)
    position = dronePosition(f[-1], time_array[-1], p, v, q)

    error = np.linalg.norm(np.array(goal_position) - np.array(position))
    return error
    