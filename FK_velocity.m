function velocity = FK_velocity(q_in, dq)
    % Calculate the end effector velocity given joint angles and joint velocities
    % :param q_in: 1 x 7 vector corresponding to the robot's current configuration.
    % :param dq: 1 x 7 vector corresponding to the joint velocities.
    % :return:
    % velocity - 6 x 1 vector corresponding to the end effector velocities.
    
    % STUDENT CODE GOES HERE
    
    % Calculate the Jacobian at the current configuration
    velocity = calcJacobian(q_in) * dq';
end