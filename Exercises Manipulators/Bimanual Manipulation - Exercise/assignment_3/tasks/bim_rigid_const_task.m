classdef bim_rigid_const_task < Task   
    %Tool position control for a single arm
    properties
        constrained = true
    end

    methods
        function obj=bim_rigid_const_task(robot_ID,taskID)
            obj.ID=robot_ID;
            obj.task_name=taskID;

        end
        function updateReference(obj, robot_system, StateMachine)

            obj.xdotbar = zeros(6, 1);
        end


        function updateJacobian(obj,robot_system)
            if(obj.ID=='L')
                robot=robot_system.left_arm;
            elseif(obj.ID=='R')
                robot=robot_system.right_arm;    
            end
            
            obj.J=robot.wJt;
        end

        function updateActivation(obj, robot_system)
            obj.A = eye(6);
        end
    end
end