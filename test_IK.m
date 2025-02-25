function test_IK()
    % Create an instance of the IK solver
    ik = IK();  % Assuming the IK class is defined in the same script or in your path
    
    % Define the target transformation (end-effector pose)
    % Example: Position [x, y, z] = [0.5, 0.2, 0.3] and Orientation (roll, pitch, yaw)
    target_position = [0.5, 0.2, 0.3];
    target_orientation = [pi/4, pi/6, pi/3];  % Roll, Pitch, Yaw angles
    target_transform = transform(target_position, target_orientation);
    
    % Define the initial joint angles (seed) for the optimization
    seed = zeros(1, 7);  % Starting guess for joint angles (assuming 7 DOF robot)
    
    % Call the inverse kinematics solver to compute the joint angles
    [q, success, rollout] = ik.inverse(target_transform, seed);
    
    % Display results
    if success
        disp('IK solution found successfully!');
        disp('Joint Angles:');
        disp(q);
    else
        disp('IK solution failed to converge.');
    end
    
    % Plot the joint angles at each iteration of the solver
    figure;
    hold on;
    for i = 1:length(rollout)
        plot(rollout{i}, 'o');
    end
    hold off;
    title('Joint Angles Evolution');
    xlabel('Iteration');
    ylabel('Joint Angles');
end

function result = transform(position, orientation)
    % Helper function to compute a homogeneous transformation matrix
    % from translation (position) and rotation (orientation in roll-pitch-yaw)
    d = position;
    rpy = orientation;
    result = trans(d) * roll(rpy(1)) * pitch(rpy(2)) * yaw(rpy(3));
end

function result = trans(d)
    % Translation matrix for position d = [x, y, z]
    result = [1, 0, 0, d(1); 0, 1, 0, d(2); 0, 0, 1, d(3); 0, 0, 0, 1];
end

function result = roll(a)
    % Rotation matrix for rotation around the x-axis by angle a (in radians)
    result = [1, 0, 0, 0; 0, cos(a), -sin(a), 0; 0, sin(a), cos(a), 0; 0, 0, 0, 1];
end

function result = pitch(a)
    % Rotation matrix for rotation around the y-axis by angle a (in radians)
    result = [cos(a), 0, sin(a), 0; 0, 1, 0, 0; -sin(a), 0, cos(a), 0; 0, 0, 0, 1];
end

function result = yaw(a)
    % Rotation matrix for rotation around the z-axis by angle a (in radians)
    result = [cos(a), -sin(a), 0, 0; sin(a), cos(a), 0, 0; 0, 0, 1, 0; 0, 0, 0, 1];
end
