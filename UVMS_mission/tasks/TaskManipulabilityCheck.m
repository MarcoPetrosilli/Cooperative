% classdef TaskManipulabilityCheck < Task
%     properties
%         id = "Manipulability";
%         arm_length;
%         armBase_distance;
%         correction_step;
%         err;
%         lin;
%     end
% 
%     methods
%         function obj = TaskManipulabilityCheck(arm_reaach, armBase_vehicle_dist, correction_step)
%             obj.arm_length = arm_reaach;
%             obj.armBase_distance = armBase_vehicle_dist;
%             obj.correction_step = correction_step;
%             obj.xdotbar = zeros(2,1);  % Inizializzato come 2x1
%         end
% 
%         function updateReference(obj, robot)
%             w_Tool_goal_XY = robot.goalPosition(1:2); % nodule
%             w_Vehicle_goal_XY = robot.vehicleGoalPosition(1:2);
%             dist_goal_nodule = norm(w_Vehicle_goal_XY - w_Tool_goal_XY);
% 
%             % Calcolo della direzione (sempre colonna 2x1)
%             obj.lin = (w_Tool_goal_XY - w_Vehicle_goal_XY);
%             obj.lin = obj.lin(:);  % Assicura che sia colonna
% 
%             obj.err = dist_goal_nodule - (obj.arm_length + obj.armBase_distance);
%             disp(size(obj.xdotbar))
% 
%             if obj.err > 0 % If original goal is too far:
%                 fprintf("Manip adjust\n")
%                 obj.xdotbar = 0.8 * obj.lin;
%                 obj.xdotbar = Saturate(obj.xdotbar, 0.3);  % Saturate già restituisce 2x1
%             else
%                 obj.xdotbar = [0.1; 0.1];  % Valori di init come vettore colonna
%             end
% 
%             % DEBUG
%             fprintf('Size xdotbar: %d x %d\n', size(obj.xdotbar));
%             fprintf('Value xdotbar: [%.3f, %.3f]\n', obj.xdotbar(1), obj.xdotbar(2));
%             fprintf('Size w_Tool_goal_XY: %d x %d\n', size(w_Tool_goal_XY));
%             fprintf('Size w_Vehicle_goal_XY: %d x %d\n', size(w_Vehicle_goal_XY));
%         end
% 
%         function updateJacobian(obj, robot)
%             obj.J = [zeros(2,7) robot.wTv(1:2,1:3) zeros(2,3)];
%         end
% 
%         function updateActivation(obj, robot)
%             th = 0;
%             delta = 0.5;
%             obj.A = eye(2) .* IncreasingBellShapedFunction(th-delta, th, 0, 1, obj.err);
% 
%             % DEBUG
%             fprintf('Size A: %d x %d\n', size(obj.A));
%         end
%     end
% end

classdef TaskManipulabilityCheck < Task   
    properties
        id = "Manipulability";
        arm_length;
        armBase_distance;
        manip_threshold;
        err;
        lin;
        yaw
    end

    methods
        function obj = TaskManipulabilityCheck(arm_reaach, armBase_vehicle_dist, manip_threshold)
            obj.arm_length = arm_reaach;
            obj.armBase_distance = armBase_vehicle_dist;
            obj.manip_threshold = manip_threshold;
            obj.xdotbar = zeros(2,1);
        end
        function updateReference(obj, robot)
            w_Tool_goal_XY = robot.goalPosition(1:2); % nodule
            w_Vehicle_XY = robot.wTv(1:2,4);
            dist_to_nodule = norm(w_Vehicle_XY - w_Tool_goal_XY);
            
            obj.err = dist_to_nodule - (obj.arm_length + obj.armBase_distance);


            if obj.err > obj.manip_threshold % If original goal is too far:
                obj.lin = w_Tool_goal_XY - w_Vehicle_XY;
                obj.lin = obj.lin(:);  % column vector

                % compute yaw so it moves in direction of target (not
                % forward of vehicle
                obj.yaw = atan2(obj.lin(2), obj.lin(1)); % Calculate yaw angle
                R_yaw = [cos(obj.yaw) -sin(obj.yaw); sin(obj.yaw) cos(obj.yaw)];

                obj.xdotbar = 0.8 * R_yaw * obj.lin;
                obj.xdotbar(1:2) = Saturate(obj.xdotbar(1:2), 0.2);
            end

        end
        function updateJacobian(obj, robot)
            obj.J = [zeros(2,7) robot.wTv(1:2,1:3) zeros(2,3)];
        end

        function updateActivation(obj, robot)
            th = 0;
            delta = 0.5;
            % act_yaw_err = DecreasingBellShapedFunction(th, th+delta, 0, 1, obj.yaw);
            act_manip_err = IncreasingBellShapedFunction(th-delta, th, 0, 1, obj.err);
            obj.A = eye(2) * act_manip_err %* act_yaw_err;
        end
    end
end
