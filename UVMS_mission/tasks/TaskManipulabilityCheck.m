classdef TaskManipulabilityCheck < Task   
    properties
        id = "Manipulability";
        arm_length;
        armBase_distance;
        err;
        lin;
        yaw;
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
           
            [~ , lin_err] = CartError(robot.wTg, robot.wTv);

            obj.yaw = atan2(lin_err(2),lin_err(1));

            dist_to_nodule = norm(lin_err(1:2));
            obj.err = dist_to_nodule - (obj.arm_length + obj.armBase_distance);
            obj.xdotbar = 15 * lin_err(1:2);
            obj.xdotbar(1:2) = Saturate(obj.xdotbar(1:2), 0.2);
            
        

        end
        function updateJacobian(obj, robot)
            obj.J = [zeros(2,7) robot.wTv(1:2,1:3) zeros(2,3)];
        end

        function updateActivation(obj, robot)
            wPg = robot.wTg(1:3,4);
            R_target = [cos(obj.yaw) -sin(obj.yaw) 0;
                        sin(obj.yaw) cos(obj.yaw) 0;
                        0 0 1];
            wT_target = eye(4);
            wT_target(1:3, 1:3) = R_target;
            wT_target(1:3, 4) = wPg;
            [ang, ~] = CartError(wT_target, robot.wTv);
            ang_err = norm(ang);
            th_ang = 0;
            th_manip = 0;
            delta_ang = 3*pi/5; 
            delta_manip = 1.0; % 0.1
            % We prefer the robot moving when it is aligned with the
            % target, so we perform moltiplication with a second activation
            % value related to the angular error
            act_ang_err = DecreasingBellShapedFunction(th_ang, th_ang + delta_ang, 0, 1, ang_err);
            act_manip_err = IncreasingBellShapedFunction(th_manip, th_manip + delta_manip, 0, 1, abs(obj.err));
            obj.A = eye(2) * (act_manip_err * act_ang_err);
            
        end 
    end
end
