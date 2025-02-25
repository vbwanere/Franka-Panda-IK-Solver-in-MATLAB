function J = calcJacobian(q_in)
    % Calculate the full Jacobian of the end effector in a given configuration
    % :param q_in: 1 x 7 configuration vector (of joint angles) [q1,q2,q3,q4,q5,q6,q7]
    % :return: J - 6 x 7 matrix representing the Jacobian, where the first three
    % rows correspond to the linear velocity and the last three rows correspond to
    % the angular velocity, expressed in world frame coordinates
    
    J = zeros(6, 7);
    
    % STUDENT CODE GOES HERE
    fk = FK();
    joint_transformation_matrices = fk.get_joint_transformation_matrices(q_in);
    
    On = get_Oi(joint_transformation_matrices{end});
    jacobian_linear_velocity = [];
    jacobian_angular_velocity = [];
    
    % The last one is end-effector, the motion of which won't contribute
    % to the linear or angular velocity of itself
    for i = 1:(length(joint_transformation_matrices) - 1)
        current_transformation_matrix = joint_transformation_matrices{i};
        Ri = get_Ri(current_transformation_matrix);
        Oi = get_Oi(current_transformation_matrix);
        z_hat = [0; 0; 1];
        jacobian_linear_velocity(:,i) = get_skew_symmetric(Ri*z_hat) * (On - Oi);
        jacobian_angular_velocity(:,i) = Ri * z_hat;
    end
    
    J(1:3, :) = jacobian_linear_velocity;
    J(4:6, :) = jacobian_angular_velocity;
end

function Ri = get_Ri(Ti)
    % Extract rotation matrix from transformation matrix
    Ri = Ti(1:3, 1:3);
end

function Oi = get_Oi(Ti)
    % Extract position vector from transformation matrix
    Oi = Ti(1:3, 4);
end

function skew = get_skew_symmetric(vector)
    % Create skew-symmetric matrix from a 3D vector
    skew = [
        0, -vector(3), vector(2);
        vector(3), 0, -vector(1);
        -vector(2), vector(1), 0
    ];
end