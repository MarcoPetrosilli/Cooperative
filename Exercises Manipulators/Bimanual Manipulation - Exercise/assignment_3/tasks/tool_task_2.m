classdef tool_task_2 < Task   
    %Tool position control for a single arm
    properties
        constrained = false
    end

    methods
        function obj=tool_task_2(robot_ID,taskID)
            obj.ID=robot_ID;
            obj.task_name=taskID;

        end
        function updateReference(obj, robot_system, StateMachine)
            if(obj.ID=='L')
                robot=robot_system.left_arm;
            elseif(obj.ID=='R')
                robot=robot_system.right_arm;    
            end

            [v_ang, v_lin] = CartError(robot.wTog , robot.wTo);
           
            if StateMachine.isGrasped()
                robot.dist_to_goal=v_lin;
                robot.rot_to_goal=v_ang;
            end
         
         
         obj.xdotbar = 1.0 * [v_ang; v_lin];

         obj.xdotbar(1:3) = Saturate(obj.xdotbar(1:3), 0.3);
         obj.xdotbar(4:6) = Saturate(obj.xdotbar(4:6), 0.3);
        end


        function updateJacobian(obj,robot_system)
            if(obj.ID=='L')
                robot=robot_system.left_arm;
            elseif(obj.ID=='R')
                robot=robot_system.right_arm;    
            end
            
            r_skew = skew(robot.tTo(1:3,4));
            r_skew = robot.wTt(1:3,1:3)*r_skew;
            w_tSo = [eye(3) zeros(3,3);r_skew' eye(3)];

            robot.wJo=w_tSo*robot.wJt;
            
            obj.J=robot.wJo;
        end

        function updateActivation(obj, robot_system)
            obj.A = eye(6);
        end
    end
end