classdef coop_rigid_const_task < Task   

    properties
        constrained = true
    end

    methods
        function obj=coop_rigid_const_task(robot_ID,taskID)
            obj.ID=robot_ID;
            obj.task_name=taskID;

        end
        function updateReference(obj, robot_system, StateMachine)
            
            if(obj.ID=='L')
                obj.xdotbar = robot_system.left_arm.Xo_12;
            elseif(obj.ID=='R')
                obj.xdotbar = robot_system.right_arm.Xo_12;    
            end
            

        end


        function updateJacobian(obj,robot_system)
            if(obj.ID=='L')
                robot=robot_system.left_arm;
            elseif(obj.ID=='R')
                robot=robot_system.right_arm;    
            end
            
            obj.J=robot.wJo;
        end

        function updateActivation(obj, robot_system)
            obj.A = eye(6);
        end
    end
end