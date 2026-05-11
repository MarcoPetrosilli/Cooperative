classdef bim_rigid_const_task < Task   
    %Tool position control for a single arm
    properties
        constrained = true;
    end

    methods
        function obj=bim_rigid_const_task(taskID)
            obj.task_name=taskID;

        end
    function updateReference(obj, robot_system, StateMachine)

            obj.xdotbar = zeros(6,1);
            
    end


        function updateJacobian(obj, robot_system, StateMachine)
            
            robot_left=robot_system.left_arm;
            robot_right=robot_system.right_arm;
            obj.J = [robot_left.wJo -robot_right.wJo];
        end

        function updateActivation(obj, robot_system)
            obj.A = eye(6);
        end
    end
end