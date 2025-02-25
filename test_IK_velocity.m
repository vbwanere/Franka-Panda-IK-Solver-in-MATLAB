function test_IK_velocity()
    % Test function for IK_velocity

    % Define tolerance for comparisons
    tolerance = 1e-6;

    % Test case 1: Simple straight motion
    q_in1 = [0, 0, 0, 0, 0, 0, 0];
    v_in1 = [0.1, 0, 0];
    omega_in1 = [0, 0, 0];
    dq1 = IK_velocity(q_in1, v_in1, omega_in1);
    % Expected result depends on robot's geometry and calcJacobian implementation.  
    % Without knowing these specifics, it's difficult to hardcode exact expected values.
    % However, we can check if dq1 is a row vector of length 7.
    assert(isequal(size(dq1), [1, 7]), 'Test Case 1 Failed: dq1 is not a 1x7 vector');
    disp('Test Case 1 Passed: dq1 is a 1x7 vector');
    % Further testing would require knowing the calcJacobian implementation
    % and computing the expected dq based on the robot's geometry.  This is not
    % generally possible without more information about calcJacobian.

    % Test case 2: Rotation only
    q_in2 = [0, pi/2, 0, pi/4, 0, 0, 0];
    v_in2 = [0, 0, 0];
    omega_in2 = [0, 0.1, 0];
    dq2 = IK_velocity(q_in2, v_in2, omega_in2);
    assert(isequal(size(dq2), [1, 7]), 'Test Case 2 Failed: dq2 is not a 1x7 vector');
    disp('Test Case 2 Passed: dq2 is a 1x7 vector');

    % Test case 3: Combination of linear and angular velocity
    q_in3 = [pi/4, 0, pi/2, 0, pi/4, 0, 0];
    v_in3 = [0.05, 0, 0.05];
    omega_in3 = [0, 0, 0.05];
    dq3 = IK_velocity(q_in3, v_in3, omega_in3);
    assert(isequal(size(dq3), [1, 7]), 'Test Case 3 Failed: dq3 is not a 1x7 vector');
    disp('Test Case 3 Passed: dq3 is a 1x7 vector');

    % Test case 4: NaN linear velocity - should still produce a valid dq
    q_in4 = [0, 0, 0, 0, 0, 0, 0];
    v_in4 = [NaN, 0, 0];
    omega_in4 = [0, 0, 0];
    dq4 = IK_velocity(q_in4, v_in4, omega_in4);
    assert(isequal(size(dq4), [1, 7]), 'Test Case 4 Failed: dq4 is not a 1x7 vector');
    disp('Test Case 4 Passed: dq4 is a 1x7 vector');
    
    % Test case 5: NaN angular velocity - should still produce a valid dq
    q_in5 = [0, 0, 0, 0, 0, 0, 0];
    v_in5 = [0, 0, 0];
    omega_in5 = [0, NaN, 0];
    dq5 = IK_velocity(q_in5, v_in5, omega_in5);
    assert(isequal(size(dq5), [1, 7]), 'Test Case 5 Failed: dq5 is not a 1x7 vector');
    disp('Test Case 5 Passed: dq5 is a 1x7 vector');
    
    % Test case 6: All NaNs in velocities
    q_in6 = [0, 0, 0, 0, 0, 0, 0];
    v_in6 = [NaN, NaN, NaN];
    omega_in6 = [NaN, NaN, NaN];
    dq6 = IK_velocity(q_in6, v_in6, omega_in6);
    assert(isequal(size(dq6), [1, 7]), 'Test Case 6 Failed: dq6 is not a 1x7 vector');
    disp('Test Case 6 Passed: dq6 is a 1x7 vector');

    disp('All basic dimension tests passed.');
    disp('Further testing requires knowledge of the calcJacobian implementation.');

end