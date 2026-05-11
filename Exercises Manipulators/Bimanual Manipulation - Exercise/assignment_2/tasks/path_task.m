classdef path_task < Task   
    %Tool position control for a single arm
    properties
        constrained = false;
        delta = 0.01;
        V_const = 0.05;
    end
    methods
        function obj=path_task(robot_ID,taskID)
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
                
                obj.xdotbar = 1.0 * [v_ang; v_lin];
            else 
                [v_ang, ~] = CartError(robot.wTog , robot.wTo);
                
                p_obj = robot.wTo(1:3, 4);
                
                waypoints = robot_system.lawnmower_path;
                idx = StateMachine.path_idx;
                
                if idx < size(waypoints, 2)
                    p_A = waypoints(:, idx);
                    p_B = waypoints(:, idx + 1);
                    
                    u = (p_B - p_A) / norm(p_B - p_A);
                    v = p_obj - p_A;
                    s = dot(v, u);
                    
                    % Controllo cambio segmento
                    % if (s + obj.delta) >= norm(p_B - p_A)
                    %     StateMachine.path_idx = idx + 1;
                    %     p_LOS = p_B;
                    % else
                    %     p_closest = p_A + s * u;
                    %     p_LOS = p_closest + obj.delta * u;
                    % end

                    if (s + obj.delta) >= norm(p_B - p_A)
                        if idx < size(waypoints, 2) - 1
                            % Non siamo ancora all'ultimo segmento: passa al prossimo
                            StateMachine.path_idx = idx + 1; 
                            p_LOS = p_B;
                        else
                            % SIAMO SULL'ULTIMO SEGMENTO!
                            % Non avanzare l'indice, fissa semplicemente il bersaglio all'ultimo waypoint
                            p_LOS = p_B; 
                            
                            % FONDAMENTALE: Rallentamento proporzionale
                            % Se manteniamo V_costante fino alla fine, il robot tremerà attorno al traguardo
                            % perché non riuscirà mai a fermarsi di colpo a 0.
                            distanza_rimanente = norm(p_LOS - p_obj);
                            obj.V_const = min(obj.V_const, 1.0 * distanza_rimanente); % Frena dolcemente
                        end
                    else
                        p_closest = p_A + s * u;
                        p_LOS = p_closest + obj.delta * u;
                    end

                    direction = (p_LOS - p_obj);
                    if norm(direction) > 1e-4
                        v_lin = obj.V_const * (direction / norm(direction));
                    else
                        v_lin = [0; 0; 0];
                    end
                else
                    v_lin = [0; 0; 0]; 
                end

                if StateMachine.isGrasped()
                    robot.dist_to_goal = p_obj - waypoints(:,end); 
                    robot.rot_to_goal = v_ang;
                end
                
                obj.xdotbar = 1.0 * [v_ang; v_lin];
            end
    
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