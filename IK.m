classdef IK
    % JOINT LIMITS
    properties (Constant)
        lower = [-2.8973,-1.7628,-2.8973,-3.0718,-2.8973,-0.0175,-2.8973];
        upper = [2.8973,1.7628,2.8973,-0.0698,2.8973,3.7525,2.8973];
        center = (IK.lower + (IK.upper - IK.lower) / 2); % middle of range of motion
    end
    
    properties
        fk
        linear_tol = 1e-4;
        angular_tol = 1e-3;
        max_steps = 1500;
        min_step_size = 1e-5;
    end
    
    methods
        function obj = IK(linear_tol, angular_tol, max_steps, min_step_size)
            if nargin > 0
                obj.linear_tol = linear_tol;
                obj.angular_tol = angular_tol;
                obj.max_steps = max_steps;
                obj.min_step_size = min_step_size;
            end
            obj.fk = FK(); % FK class needs to be defined elsewhere
        end
        
        % Helper Functions
        function [displacement, axis] = displacement_and_axis(target, current)
            % Computes the displacement vector and axis of rotation
            displacement = zeros(1,3);
            axis = zeros(1,3);
            
            displacement = Rotation.get_pose_from_tranformation_matrix(target) - Rotation.get_pose_from_tranformation_matrix(current);
            rotation_from_current_to_target = Rotation.get_rotation_from_transformation_matrix(target) * Rotation.get_rotation_from_transformation_matrix(current)';
            skew_symmetric = (rotation_from_current_to_target - rotation_from_current_to_target') / 2;
            vector_coeff = Rotation.get_vector_coefficients_from_skew_symmetric(skew_symmetric);
            axis = vector_coeff;
        end
        
        function [distance, angle] = distance_and_angle(G, H)
            % Computes the distance and angle between two transforms
            distance = 0;
            angle = 0;
            
            displacement = Rotation.get_pose_from_tranformation_matrix(G) - Rotation.get_pose_from_tranformation_matrix(H);
            distance = norm(displacement);
            
            rotation_from_current_to_target = Rotation.get_rotation_from_transformation_matrix(G) * Rotation.get_rotation_from_transformation_matrix(H)';
            angle = acos(min(max((trace(rotation_from_current_to_target) - 1) / 2, -1), 1));
        end
        
        function success = is_valid_solution(obj, q, target)
            % Determines if the solution is valid
            success = obj.is_solution_in_joint_limits(q) && obj.is_candidate_similar_to_desired_transformation(q, target);
        end
        
        function success = is_solution_in_joint_limits(~, q)
            % Checks if the solution respects the joint limits
            success = all(q >= IK.lower) && all(q <= IK.upper);
        end
        
        function success = is_candidate_similar_to_desired_transformation(~, q, target)
            % Checks if the candidate solution is similar to the desired transformation
            [~, actual_transformation] = FK().forward(q);
            pose_difference = abs(Rotation.get_pose_from_tranformation_matrix(actual_transformation) - Rotation.get_pose_from_tranformation_matrix(target));
            orientation_difference = abs(Rotation.get_rotation_from_transformation_matrix(actual_transformation) - Rotation.get_rotation_from_transformation_matrix(target));
            success = all(pose_difference < obj.linear_tol) && all(orientation_difference < obj.angular_tol);
        end
        
        % Task Functions
        function dq = end_effector_task(~, q, target)
            % Primary task for IK solver
            dq = zeros(1, 7);
            [~, current_end_effactor_pose] = FK().forward(q);
            [displacement, axis] = IK.displacement_and_axis(target, current_end_effactor_pose);
            dq = IK_velocity(q, displacement, axis); % IK_velocity needs to be defined
        end
        
        function dq = joint_centering_task(~, q, rate)
            % Secondary task for IK solver
            if nargin < 3
                rate = 5e-1;
            end
            offset = 2 * (q - IK.center) ./ (IK.upper - IK.lower);
            dq = rate * -offset; % proportional term
        end
        
        % Inverse Kinematics Solver
        function [q, success, rollout] = inverse(obj, target, seed)
            % Solves the inverse kinematics using gradient descent
            q = seed;
            rollout = {};
            while true
                rollout{end+1} = q; % Store the current guess
                
                % Primary Task - Achieve End Effector Pose
                dq_ik = obj.end_effector_task(q, target);
                
                % Secondary Task - Center Joints
                dq_center = obj.joint_centering_task(q);
                
                % Task Prioritization
                jacobian = calcJacobian(q); % calcJacobian needs to be defined
                dq = dq_ik + (eye(size(jacobian, 2)) - pinv(jacobian) * jacobian) * dq_center;
                
                % Termination Conditions
                if length(rollout) > obj.max_steps || norm(dq) < obj.min_step_size
                    break;
                end
                
                q = q + dq; % Update the joint angles
            end
            success = obj.is_valid_solution(q, target);
        end
    end
end
