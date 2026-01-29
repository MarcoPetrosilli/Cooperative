% classdef tool_task < Task   
%     %Tool position control for a single arm
%     properties
%         constrained = false;
%     end
% 
%     methods
%         function obj=tool_task(robot_ID,taskID)
%             obj.ID=robot_ID;
%             obj.task_name=taskID;
%         end
%         function updateReference(obj, robot_system, StateMachine)
%             if(obj.ID=='L')
%                 robot=robot_system.left_arm;
%             elseif(obj.ID=='R')
%                 robot=robot_system.right_arm;    
%             end
% 
%          if StateMachine.isApproaching()
%              [v_ang, v_lin] = CartError(robot.wTg , robot.wTt);
% 
%          elseif StateMachine.isGrasped()
%              [v_ang, v_lin] = CartError(robot.wTog , robot.wTo);
%          else
%              v_ang = zeros(3,1);
%              v_lin = zeros(3,1);
%          end
% 
%          robot.dist_to_goal=v_lin;
%          robot.rot_to_goal=v_ang;
% 
%          obj.xdotbar = 1.0 * [v_ang; v_lin];
%          % limit the requested velocities...
%          obj.xdotbar(1:3) = Saturate(obj.xdotbar(1:3), 0.3);
%          obj.xdotbar(4:6) = Saturate(obj.xdotbar(4:6), 0.3);
%         end
%         function updateJacobian(obj,robot_system, StateMachine)
%             if(obj.ID=='L')
%                 robot=robot_system.left_arm;
%             elseif(obj.ID=='R')
%                 robot=robot_system.right_arm;    
%             end
% 
%             if ~StateMachine.isGrasped()
% 
%                 final_jacobian=robot.wJt;
% 
%             else
%                 r = robot.wTt(1:3,1:3)*robot.tTo(1:3,4);
%                 r_skew = skew(r);
%                 w_tSo = [eye(3) zeros(3,3);r_skew' eye(3)];
% 
%                 robot.wJo = w_tSo*robot.wJt;
%                 final_jacobian = robot.wJo;
%             end
% 
%             if obj.ID=='L'
%                 obj.J=[final_jacobian, zeros(6, 7)];
%             elseif obj.ID=='R'
%                 obj.J=[zeros(6, 7), final_jacobian];
%             end
%         end
% 
%         function updateActivation(obj, robot_system)
%             obj.A = eye(6);
%         end
%     end
% end



classdef tool_task < Task   
    %Tool position control for a single arm
    properties
        constrained = false
    end

    methods
        function obj=tool_task(robot_ID,taskID)
            obj.ID=robot_ID;
            obj.task_name=taskID;
        end
        function updateReference(obj, robot_system, StateMachine)
            if(obj.ID=='L')
                robot=robot_system.left_arm;
            elseif(obj.ID=='R')
                robot=robot_system.right_arm;    
            end
         
            if (obj.task_name == "LT" || obj.task_name == "RT")
                [v_ang, v_lin] = CartError(robot.wTg , robot.wTt);
                if ~StateMachine.isGrasped()
                   robot.dist_to_goal=v_lin;
                   robot.rot_to_goal=v_ang;
                end
            else 
                [v_ang, v_lin] = CartError(robot.wTog , robot.wTo);
                if StateMachine.isGrasped()
                    robot.dist_to_goal=v_lin;
                    robot.rot_to_goal=v_ang;
                end
            end
    
    
            obj.xdotbar = 1.0 * [v_ang; v_lin];
            % limit the requested velocities...
            obj.xdotbar(1:3) = Saturate(obj.xdotbar(1:3), 0.3);
            obj.xdotbar(4:6) = Saturate(obj.xdotbar(4:6), 0.3);
        end

        function updateJacobian(obj,robot_system, StateMachine)
            if(obj.ID=='L')
                robot=robot_system.left_arm;
            elseif(obj.ID=='R')
                robot=robot_system.right_arm;    
            end
            
            if (obj.task_name == "LT" || obj.task_name == "RT")
                if obj.ID=='L'
                    obj.J=[robot.wJt, zeros(6, 7)];
                elseif obj.ID=='R'
                    obj.J=[zeros(6, 7), robot.wJt];
                end
            else 
                r_skew = skew(robot.tTo(1:3,4));
                r_skew = robot.wTt(1:3,1:3)*r_skew;
                w_tSo = [eye(3) zeros(3,3);r_skew' eye(3)];
    
                robot.wJo=w_tSo*robot.wJt;
                
                if obj.ID=='L'
                    obj.J=[robot.wJo, zeros(6, 7)];
                elseif obj.ID=='R'
                    obj.J=[zeros(6, 7), robot.wJo];
                end
            end
            
        end

        function updateActivation(obj, robot_system)
            obj.A = eye(6);
        end
    end
end