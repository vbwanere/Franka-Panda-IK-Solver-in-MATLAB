% Test script for the Jacobian calculation

% Test configuration 1
q1 = [0, 0, 0, -pi/2, 0, pi/2, pi/4];
J1 = calcJacobian(q1);
disp('Jacobian for first configuration:');
disp(round(J1, 3));

% Test configuration 2
q2 = [0, 0, 0, 0, 0, 0, 0];
J2 = calcJacobian(q2);
disp('Jacobian for second configuration:');
disp(round(J2, 3));