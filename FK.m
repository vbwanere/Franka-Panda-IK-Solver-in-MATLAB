classdef FK
    properties
        % Properties can be defined here if needed
    end
    
    methods
        function obj = FK()
            % Constructor
            % You may want to define geometric parameters here that will be
            % useful in computing the forward kinematics.
        end
        
        function [jointPositions, T0e] = forward(obj, q)
            % INPUT:
            % q - 1x7 vector of joint angles [q0, q1, q2, q3, q4, q5, q6]
            %
            % OUTPUTS:
            % jointPositions - 8x3 matrix, where each row corresponds to a rotational joint
            %                 of the robot or end effector. Each row contains the [x,y,z]
            %                 coordinates in the world frame of the respective joint's center
            %                 in meters. The base of the robot is located at [0,0,0].
            % T0e           - a 4x4 homogeneous transformation matrix, representing the end
            %                 effector frame expressed in the world frame

            % Initialize jointPositions matrix
            jointPositions = zeros(8, 3);
            
            % Get joint transformation matrices
            joint_transformation_matrices = obj.get_joint_transformation_matrices(q);
            
            % Get the end effector transformation
            T0e = joint_transformation_matrices{end};
            
            % Extract joint positions from transformation matrices
            for i = 1:length(joint_transformation_matrices)
                jointPositions(i, :) = joint_transformation_matrices{i}(1:3, 4)';
            end
        end
        
        function Ti = get_joint_transformation_matrices(obj, q)
            % Get Ai matrices
            Ai = obj.compute_Ai(q);
            
            % Initialize cell array for transformation matrices
            Ti = cell(length(Ai), 1);
            
            % Calculate transformation matrices
            current_transformation_matrix = eye(4);
            for i = 1:length(Ai)
                ai = Ai{i};
                current_transformation_matrix = current_transformation_matrix * ai;
                
                if i == 3
                    joint_transformation_matrix = current_transformation_matrix * [
                        1, 0, 0, 0;
                        0, 1, 0, 0;
                        0, 0, 1, 0.195;
                        0, 0, 0, 1
                    ];
                elseif i == 5
                    joint_transformation_matrix = current_transformation_matrix * [
                        1, 0, 0, 0;
                        0, 1, 0, 0;
                        0, 0, 1, 0.125;
                        0, 0, 0, 1
                    ];
                elseif i == 6
                    joint_transformation_matrix = current_transformation_matrix * [
                        1, 0, 0, 0;
                        0, 1, 0, 0;
                        0, 0, 1, -0.015;
                        0, 0, 0, 1
                    ];
                elseif i == 7
                    joint_transformation_matrix = current_transformation_matrix * [
                        1, 0, 0, 0;
                        0, 1, 0, 0;
                        0, 0, 1, 0.051;
                        0, 0, 0, 1
                    ];
                else
                    joint_transformation_matrix = current_transformation_matrix;
                end
                Ti{i} = joint_transformation_matrix;
            end
        end
        
        function axis_of_rotation = get_axis_of_rotation(obj, q)
            % INPUT:
            % q - 1x7 vector of joint angles [q0, q1, q2, q3, q4, q5, q6]
            %
            % OUTPUTS:
            % axis_of_rotation: - 3x7 matrix of unit vectors describing the axis of rotation
            %                    for each joint in the world frame
            
            % This is a function needed by lab 2
            axis_of_rotation = [];
            % Implement the code here
        end
        
        function Ai = compute_Ai(obj, q)
            % INPUT:
            % q - 1x7 vector of joint angles [q0, q1, q2, q3, q4, q5, q6]
            %
            % OUTPUTS:
            % Ai: - 4x4 cell array of homogenous transformations describing the FK of the robot.
            %      Transformations are not necessarily located at the joint locations
            
            % Helper functions for transformations
            function R = rotate_z(angle)
                R = [
                    cos(angle), -sin(angle), 0, 0;
                    sin(angle), cos(angle), 0, 0;
                    0, 0, 1, 0;
                    0, 0, 0, 1
                ];
            end
            
            function R = rotate_x(angle)
                R = [
                    1, 0, 0, 0;
                    0, cos(angle), -sin(angle), 0;
                    0, sin(angle), cos(angle), 0;
                    0, 0, 0, 1
                ];
            end
            
            function T = trans_z(distance)
                T = [
                    1, 0, 0, 0;
                    0, 1, 0, 0;
                    0, 0, 1, distance;
                    0, 0, 0, 1
                ];
            end
            
            function T = trans_x(distance)
                T = [
                    1, 0, 0, distance;
                    0, 1, 0, 0;
                    0, 0, 1, 0;
                    0, 0, 0, 1
                ];
            end
            
            % DH parameters
            a = [0, 0, 0, 0.0825, 0.0825, 0, 0.088, 0];
            alpha = [0, -pi/2, pi/2, pi/2, pi/2, -pi/2, pi/2, 0];
            d = [0.141, 0.192, 0, 0.195+0.121, 0, 0.125+0.259, 0, 0.051+0.159];
            
            q1 = q(1);
            q2 = q(2);
            q3 = q(3);
            q4 = q(4);
            q5 = q(5);
            q6 = q(6);
            q7 = q(7);
            
            theta = [q1, 0, q2, q3, q4+pi, q5, q6+pi, q7-pi/4];
            
            % Compute Ai matrices
            Ai = cell(length(a), 1);
            for i = 1:length(a)
                Ai{i} = rotate_z(theta(i)) * trans_z(d(i)) * trans_x(a(i)) * rotate_x(alpha(i));
            end
        end
    end
end