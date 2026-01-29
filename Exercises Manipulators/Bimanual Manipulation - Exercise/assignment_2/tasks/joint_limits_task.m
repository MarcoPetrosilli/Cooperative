classdef joint_limits_task < Task   
    
    properties
        threshold = 0.2;
        lambda = 0.5
        constrained = false;
    end

    methods
        function obj=joint_limits_task(robot_ID,taskID)
            obj.ID=robot_ID;
            obj.task_name=taskID;
        end

        function updateReference(obj, robot_system, StateMachine)
            if(obj.ID=='L')
                robot=robot_system.left_arm;
            elseif(obj.ID=='R')
                robot=robot_system.right_arm;    
            end
            
            
            obj.xdotbar = zeros(7,1);

            
            for i = 1:7
                dist_min = -obj.lambda * (robot.jlmin(i) - robot.q(i));
                dist_max = obj.lambda * (robot.jlmax(i) - robot.q(i));
        
                if abs(dist_min) < obj.threshold
                    obj.xdotbar(i) = dist_min;
                elseif abs(dist_max) < obj.threshold
                    obj.xdotbar(i) = dist_max;
                end
            end

            obj.xdotbar(1:7) = Saturate(obj.xdotbar(1:7), 0.3);

        end
        
        function updateJacobian(obj, robot_system)

            if obj.ID=='L'
                obj.J= [eye(7), zeros(7,7)];
            elseif obj.ID=='R'
                obj.J= [zeros(7,7), eye(7)];
            end
        end

        function updateActivation(obj, robot_system)
            if(obj.ID=='L')
                robot=robot_system.left_arm;
            elseif(obj.ID=='R')
                robot=robot_system.right_arm;    
            end

            obj.A = eye(7);

            threshold_activation = 0.2;

            for i = 1:7
                obj.A(i,i) = DecreasingBellShapedFunction(robot.jlmin(i), robot.jlmin(i)+threshold_activation, 0, 1, robot.q(i)) ...
                + IncreasingBellShapedFunction(robot.jlmax(i)-threshold_activation, robot.jlmax(i), 0, 1, robot.q(i));
            end
            
        end
    end
end