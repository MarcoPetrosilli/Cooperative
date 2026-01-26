classdef ee_altitude_task < Task
    %EE_ALTITUDE_TASK Summary of this class goes here
    %   Detailed explanation goes here
    
    properties
        desired_altitude;
        altitude
        constrained = false
    end
    
    methods
        function obj=ee_altitude_task(robot_ID,taskID,desired_altitude)
            obj.ID=robot_ID;
            obj.task_name=taskID;
            obj.desired_altitude = desired_altitude;
        end
        
        function updateReference(obj, robot_system,grasped)

            if(obj.ID=='L')
                robot=robot_system.left_arm;
            elseif(obj.ID=='R')
                robot=robot_system.right_arm;    
            end


         obj.altitude = robot.wTt(3,4);

         obj.xdotbar = 1.0 * obj.altitude;
         obj.xdotbar = Saturate(obj.xdotbar, 0.3);

        end
        function updateJacobian(obj,robot_system)
            if(obj.ID=='L')
                robot=robot_system.left_arm;
            elseif(obj.ID=='R')
                robot=robot_system.right_arm;    
            end
            tool_jacobian=robot.wJt;
            
            if obj.ID=='L'
                obj.J=[tool_jacobian(6,:), zeros(1, 7)];
            elseif obj.ID=='R'
                obj.J=[zeros(1, 7), tool_jacobian(6,:)];
            end
        end

        function updateActivation(obj, robot_system)
            obj.A = eye(6);
            
            th = obj.desired_altitude;
            delta = 0.1;

            obj.A = DecreasingBellShapedFunction(th,th+delta,0,1,obj.altitude);
        end
    end
end

