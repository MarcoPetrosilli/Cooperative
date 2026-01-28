% classdef TaskManipulabilityCheck < Task   
%     properties
%         id = "Manipulability";
%         arm_length;
%         armBase_distance;
%         err;
%         lin;
%         yaw
%     end
% 
%     methods
%         function obj = TaskManipulabilityCheck(arm_reaach, armBase_vehicle_dist)
%             obj.arm_length = arm_reaach;
%             obj.armBase_distance = armBase_vehicle_dist;
%             obj.xdotbar = zeros(2,1);
%         end
%         function updateReference(obj, robot)
%             obj.err = inf; % init value
%             obj.yaw = 0.0; % init value
% 
%             manip_threshold = 1e-1;
%             % w_Tool_goal_XY = robot.goalPosition(1:2);   % nodule
%             % w_Vehicle_XY = robot.wTv(1:2,4);            % vehicle
%             % v_Vehicle_XY = [0;0];
%             % v_Tool_goal_XY = robot.vTg(1:2,4);
%             dist_to_nodule = norm(v_Vehicle_XY - v_Tool_goal_XY);
% 
% 
% 
%             obj.err = dist_to_nodule - (obj.arm_length + obj.armBase_distance);
% 
% 
%             if obj.err < manip_threshold % If original goal is too far:
%                 % obj.lin = w_Tool_goal_XY - (w_Vehicle_XY + [obj.armBase_distance;0]);
%                 obj.lin = v_Vehicle_XY - v_Tool_goal_XY;
%                 obj.lin = obj.lin(:);  % column vector
% 
%                 % compute yaw so it moves in direction of target (not forward of vehicle)
%                 % obj.yaw = atan2(obj.lin(2), obj.lin(1)); % Calculate yaw angle
%                 % R_yaw_t = [cos(obj.yaw) -sin(obj.yaw); sin(obj.yaw) cos(obj.yaw)];
%                 obj.xdotbar = 1 * obj.lin;
%                 obj.xdotbar(1:2) = Saturate(obj.xdotbar(1:2), 0.3);
%             end
% 
%         end
%         function updateJacobian(obj, robot)
%             obj.J = [zeros(2,7) robot.wTv(1:2,1:3) zeros(2,3)];
%             % obj.J = [zeros(2,7) [1 0 0; 0 1 0] zeros(2,3)];
%         end
% 
%         function updateActivation(obj, robot)
%             wPv = robot.wTv(1:3,4);
%             R_target = [cos(obj.yaw) -sin(obj.yaw) 0;
%                         sin(obj.yaw) cos(obj.yaw) 0;
%                         0 0 1];
%             wT_target = eye(4);
%             wT_target(1:3, 1:3) = R_target;
%             wT_target(1:3, 4) = wPv;
% 
%             [ang, ~] = CartError(wT_target, robot.wTv);
%             ang_err = norm(ang);
%             th_ang = 0.5;
%             th_manip = 0;
%             delta_ang = 3*(pi/5); 
%             delta_manip = 2e-1;
%             % We prefer the robot moving when it is aligned with the
%             % target, so we perform moltiplication with a second activation
%             % value related to the angular error
%             act_ang_err = DecreasingBellShapedFunction(th_ang, th_ang + delta_ang, 0, 1, ang_err);
%             act_manip_err = IncreasingBellShapedFunction(th_manip, th_manip + delta_manip, 0, 1, abs(obj.err));
%             obj.A = eye(2) * (act_manip_err * act_ang_err);
%         end 
%     end
% end

classdef TaskManipulabilityCheck < Task   
    properties
        id = "Manipulability";
        arm_length;
        armBase_distance;
        err;
        lin;
        yaw
    end

    methods
        function obj = TaskManipulabilityCheck(arm_reaach, armBase_vehicle_dist)
            obj.arm_length = arm_reaach;
            obj.armBase_distance = armBase_vehicle_dist;
            obj.xdotbar = zeros(2,1);
        end
        function updateReference(obj, robot)
            obj.err = inf; % init value
            obj.yaw = 0.0; % init value

            manip_threshold = 1e-1;
            w_Tool_goal_XY = robot.wTg(1:2,4);   % nodule
            w_Vehicle_XY = robot.wTv(1:2,4);            % vehicle
            dist_to_nodule = norm(w_Vehicle_XY - w_Tool_goal_XY);



            obj.err = dist_to_nodule - (obj.arm_length + obj.armBase_distance);


            if obj.err < manip_threshold % If original goal is too far:
                obj.lin = w_Tool_goal_XY - w_Vehicle_XY;
                obj.lin = obj.lin(:);  % column vector
                obj.yaw = atan2(obj.lin(2), obj.lin(1)); % Calculate yaw angle for activation
                obj.xdotbar = 0.4 * obj.lin/norm(obj.lin);
                obj.xdotbar(1:2) = Saturate(obj.xdotbar(1:2), 0.2);
            end

        end
        function updateJacobian(obj, robot)
            obj.J = [zeros(2,7) robot.wTv(1:2,1:3) zeros(2,3)];
            % obj.J = [zeros(2,7) [1 0 0; 0 1 0] zeros(2,3)];
        end

        function updateActivation(obj, robot)
            wPv = robot.wTv(1:3,4);
            R_target = [cos(obj.yaw) -sin(obj.yaw) 0;
                        sin(obj.yaw) cos(obj.yaw) 0;
                        0 0 1];
            wT_target = eye(4);
            wT_target(1:3, 1:3) = R_target;
            wT_target(1:3, 4) = wPv;
            
            [ang, ~] = CartError(wT_target, robot.wTv);
            ang_err = norm(ang)
            th_ang = 0; %0.5
            th_manip = 0;
            delta_ang = 4*pi/5; 
            delta_manip = 2e-1;
            % We prefer the robot moving when it is aligned with the
            % target, so we perform moltiplication with a second activation
            % value related to the angular error
            act_ang_err = DecreasingBellShapedFunction(th_ang, th_ang + delta_ang, 0, 1, ang_err);
            act_manip_err = IncreasingBellShapedFunction(th_manip, th_manip + delta_manip, 0, 1, abs(obj.err));
            obj.A = eye(2) * (act_manip_err * act_ang_err);
            
        end 
    end
end
