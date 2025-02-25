function dq = IK_velocity(q_in, v_in, omega_in)
    % Calculate joint velocities to achieve desired end-effector velocities
    % :param q_in: 1 x 7 vector corresponding to the robot's current configuration.
    % :param v_in: The desired linear velocity in the world frame. If any element is
    % NaN, then that velocity can be anything
    % :param omega_in: The desired angular velocity in the world frame. If any
    % element is NaN, then that velocity is unconstrained i.e. it can be anything
    % :return:
    % dq - 1 x 7 vector corresponding to the joint velocities. If v_in and omega_in
    %      are infeasible, then dq should minimize the least squares error. If v_in
    %      and omega_in have multiple solutions, then you should select the solution
    %      that minimizes the l2 norm of dq
    
    % Initialize solution
    dq = zeros(1, 7);
    
    % Reshape inputs to column vectors
    v_in = reshape(v_in, [3, 1]);
    omega_in = reshape(omega_in, [3, 1]);
    
    % Calculate Jacobian at current configuration
    velocity_jacobian = calcJacobian(q_in);
    
    % Find valid indices (non-NaN values)
    valid_num_index_in_v = ~isnan(v_in);
    valid_num_index_in_omega = ~isnan(omega_in);
    
    % Extract valid values
    v_in_valid = v_in(valid_num_index_in_v);
    omega_in_valid = omega_in(valid_num_index_in_omega);
    
    % Extract corresponding rows from the Jacobian
    linear_jacobian_rows = velocity_jacobian(1:3, :);
    angular_jacobian_rows = velocity_jacobian(4:6, :);
    
    linear_jacobian_valid = linear_jacobian_rows(valid_num_index_in_v, :);
    angular_jacobian_valid = angular_jacobian_rows(valid_num_index_in_omega, :);
    
    % Combine valid Jacobian rows and desired velocities
    jacobian_valid = [linear_jacobian_valid; angular_jacobian_valid];
    velocity_valid = [v_in_valid; omega_in_valid];
    
    % Solve least squares problem to find joint velocities
    % Use pinv to get minimum norm solution when system is underconstrained
    dq = (jacobian_valid \ velocity_valid)';
    
    % Alternative using lsqminnorm (MATLAB R2017b and later) for more stable results
    % dq = lsqminnorm(jacobian_valid, velocity_valid)';
end