% Test script for the FK class

% Create an instance of FK
fk = FK();

% Test configuration matching the figure in the handout
q = [0, 0, 0, -pi/2, 0, pi/2, pi/4];

% Compute forward kinematics
[joint_positions, T0e] = fk.forward(q);

% Display results
disp('Joint Positions:');
disp(joint_positions);

disp('End Effector Pose:');
disp(T0e);