% classdef ActionManager < handle
%     properties
%         actions = {}      % cell array of actions (each action = stack of tasks)
%         currentAction = 1 % index of currently active action
%         mu_0
%         action_names = []     % names of actions
%         unified_action = {}   % all tasks (union)
%         lastAction = []       % previous action index
%         actionSwitchTime = 0  % timer handle
%         transitionDuration = 2.0 % [s] blending duration
%     end
% 
%     methods
% 
%         function obj = ActionManager()
%             obj.mu_0 = 0;
%         end
% 
%         function addAction(obj, taskStack, action_name)
%             % Append actions in a structure
%             obj.actions{end+1} = taskStack;
%             obj.action_names{end+1} = action_name;
%         end
% 
%         function cleanAction(obj, toRemove)
%             % Append actions in a structure
%              obj.actions(toRemove) = [];
%         end
% 
%         function addUnifiedAction(obj, unified_set)
%             % Define the global set of all tasks
%             obj.unified_action = unified_set;
%         end
% 
%         function setCurrentAction(obj, actionName)
%             % Switch to another action by name
%             % idx = find(strcmp(obj.action_names{:}, "safe_navigation"), 1);
%             % idx = find(strcmp(obj.action_names, actionName), 1);
%             idx = find(string(obj.action_names) == actionName, 1);
%             if isempty(idx)
%                 error('Action "%s" not found.', actionName);
%             end
%             obj.lastAction = obj.currentAction;
%             obj.currentAction = idx;
%             obj.actionSwitchTime = tic; % start blending timer
%         end
% 
%         function [ydotbar] = computeICAT(obj,bm_system, arm1, arm2, grasped, final)
% 
%             tasks = obj.unified_action;
%             tasks_id = [];
%             for i = 1:length(tasks)
%                 tasks_id{i} = tasks{i}.task_name;
%             end
% 
%             % compute blending ratio
%             if obj.actionSwitchTime ~= 0
%                 t = toc(obj.actionSwitchTime);
%                 alpha = min(t / obj.transitionDuration, 1);
%             else
%                 alpha = 1;
%             end
% 
%             %check discontinuità -> sostituisci con brll shape
% 
%             % current/previous task sets
%             current_tasks = obj.actions{obj.currentAction};
%             current_tasks_id = [];
%             for i = 1:length(current_tasks)
%                 current_tasks_id{i} = current_tasks{i}.task_name;
%             end
% 
%             prev_tasks_id = [];
%             if ~isempty(obj.lastAction)
%                 prev_tasks = obj.actions{obj.lastAction};
%                 for i = 1:length(prev_tasks)
%                     prev_tasks_id{i} = prev_tasks{i}.task_name;
%                 end
%             else
%                 prev_tasks = {};
%             end
% 
%             inCurrent = ismember(string(tasks_id), string(current_tasks_id));
%             inPrev    = ismember(string(tasks_id), string(prev_tasks_id));
% 
%             %% 1. Update references, Jacobians, activations
%             for i = 1:length(tasks)
%                 task = tasks{i};
% 
%                 task.updateReference(bm_system, grasped);
%                 task.updateJacobian(bm_system);
% 
%                 task.updateActivation(bm_system);
% 
% 
%                 if inCurrent(i) && ~inPrev(i) && ~task.constrained
%                     % entering → fade in
%                     task.A = task.A * alpha;
%                 elseif ~inCurrent(i) && inPrev(i) && ~task.constrained
%                     % leaving → fade out
%                     task.A = task.A * (1 - alpha);
%                 elseif ~inCurrent(i) && ~inPrev(i)
%                     task.A = task.A * 0;
%                 % else
%                     % steady → normal activation
%                 end
%             end
% 
%             %% 2. Perform ICAT (task-priority inverse kinematics) for the current Action
%             ydotbar = zeros(14,1);
%             Qp = eye(14);
%             for i = 1:length(tasks)
%                  %% TRANSITION FROM PREVIOUS ACTION SET TO NEXT ACTION SET
%                 [Qp, ydotbar] = iCAT_task(tasks{i}.A, tasks{i}.J, ...
%                                            Qp, ydotbar, tasks{i}.xdotbar, ...
%                                            1e-4, 0.01, 10);
%             end
%             %% 3. Last task: residual damping
%             [~, ydotbar] = iCAT_task(eye(14), eye(14), Qp, ydotbar, zeros(14,1), 1e-4, 0.01, 10);
% 
%             if grasped && ~final
% 
%                 X_o1 = arm1.wJo*ydotbar(1:7);
%                 X_o2 = arm2.wJo*ydotbar(8:14);
% 
%                 H_1 = arm1.wJo*pinv(arm1.wJo);
%                 H_2 = arm2.wJo*pinv(arm2.wJo);
%                 H_12 = [H_1 zeros(6,6);zeros(6,6) H_2];
% 
%                 [v_ang, v_lin] = CartError(arm1.wTog , arm1.wTo);
%                 xdotbar = 1.0 * [v_ang; v_lin];
%                 xdotbar(1:3) = Saturate(xdotbar(1:3), 0.3);
%                 xdotbar(4:6) = Saturate(xdotbar(4:6), 0.3);
% 
%                 mu_1 = obj.mu_0 + norm(xdotbar-X_o1);
%                 mu_2 = obj.mu_0 + norm(xdotbar-X_o2);
% 
%                 xdot_t = (mu_1*X_o1+mu_2*X_o2)/(mu_1+mu_2);
% 
%                 Xdot_t = [xdot_t;xdot_t];
% 
%                 C = [H_1 -H_2];
% 
%                 Xo_1_2 = H_12*(eye(12)-pinv(C)*C)*Xdot_t;
% 
%                 % Cerca nelle celle l'oggetto x che ha x.taskID uguale a "LB"
%                 lb_ID = find(cellfun(@(x) x.task_name == "LB", tasks), 1);
%                 rb_ID = find(cellfun(@(x) x.task_name == "RB", tasks), 1);
% 
%                 tasks{lb_ID}.xdotbar = Xo_1_2(1:6,:);
%                 tasks{rb_ID}.xdotbar = Xo_1_2(7:12,:);
% 
%                 tasks_lr = [tasks(rb_ID), tasks(1:rb_ID-1), tasks(rb_ID+1:end)];
%                 tasks_lr = [tasks(lb_ID), tasks(1:lb_ID-1), tasks(lb_ID+1:end)];
% 
% 
%                 for i = 1:length(tasks_lr)
%                      %% TRANSITION FROM PREVIOUS ACTION SET TO NEXT ACTION SET
%                     [Qp, ydotbar] = iCAT_task(tasks_lr{i}.A, tasks_lr{i}.J, ...
%                                                Qp, ydotbar, tasks_lr{i}.xdotbar, ...
%                                                1e-4, 0.01, 10);
%                 end
%                 %% 3. Last task: residual damping
%                 [~, ydotbar] = iCAT_task(eye(14), eye(14), Qp, ydotbar, zeros(14,1), 1e-4, 0.01, 10);
%             end
%         end
%     end
% end


classdef ActionManager < handle
    properties
        actions = {}      % cell array of actions (each action = stack of tasks)
        currentAction = 1 % index of currently active action
        mu_0
        action_names = []     % names of actions
        unified_action = {}   % all tasks (union)
        lastAction = []       % previous action index
        actionSwitchTime = 0  % timer handle
        transitionDuration = 2.0 % [s] blending duration
    end

    methods

        function obj = ActionManager()
            obj.mu_0 = 0;
        end

        function addAction(obj, taskStack, action_name)
            % Append actions in a structure
            obj.actions{end+1} = taskStack;
            obj.action_names{end+1} = action_name;
        end

        function cleanAction(obj, toRemove)
            % Append actions in a structure
             obj.actions(toRemove) = [];
        end

        function addUnifiedAction(obj, unified_set)
            % Define the global set of all tasks
            obj.unified_action = unified_set;
        end

        function setCurrentAction(obj, actionName)
            % Switch to another action by name
            % idx = find(strcmp(obj.action_names{:}, "safe_navigation"), 1);
            % idx = find(strcmp(obj.action_names, actionName), 1);
            idx = find(string(obj.action_names) == actionName, 1);
            if isempty(idx)
                error('Action "%s" not found.', actionName);
            end
            obj.lastAction = obj.currentAction;
            obj.currentAction = idx;
            obj.actionSwitchTime = tic; % start blending timer
        end

        function [ydotbar] = computeICAT(obj,bm_system, actual_arm, other_arm, grasped, final, arm)

            tasks = obj.unified_action;
            tasks_id = [];
            for i = 1:length(tasks)
                tasks_id{i} = tasks{i}.task_name;
            end

            % compute blending ratio
            if obj.actionSwitchTime ~= 0
                t = toc(obj.actionSwitchTime);
                alpha = min(t / obj.transitionDuration, 1);
            else
                alpha = 1;
            end

            %check discontinuità -> sostituisci con brll shape

            % current/previous task sets
            current_tasks = obj.actions{obj.currentAction};
            current_tasks_id = [];
            for i = 1:length(current_tasks)
                current_tasks_id{i} = current_tasks{i}.task_name;
            end

            prev_tasks_id = [];
            if ~isempty(obj.lastAction)
                prev_tasks = obj.actions{obj.lastAction};
                for i = 1:length(prev_tasks)
                    prev_tasks_id{i} = prev_tasks{i}.task_name;
                end
            else
                prev_tasks = {};
            end

            inCurrent = ismember(string(tasks_id), string(current_tasks_id));
            inPrev    = ismember(string(tasks_id), string(prev_tasks_id));

            %% 1. Update references, Jacobians, activations
            for i = 1:length(tasks)
                task = tasks{i};

                task.updateReference(bm_system, grasped);
                task.updateJacobian(bm_system);
                task.updateActivation(bm_system);


                if inCurrent(i) && ~inPrev(i) && ~task.constrained
                    % entering → fade in
                    task.A = task.A * alpha;
                elseif ~inCurrent(i) && inPrev(i) && ~task.constrained
                    % leaving → fade out
                    task.A = task.A * (1 - alpha);
                elseif ~inCurrent(i) && ~inPrev(i)
                    task.A = task.A * 0;
                % else
                    % steady → normal activation
                end
            end

            %% 2. Perform ICAT (task-priority inverse kinematics) for the current Action
            ydotbar = zeros(14,1);
            Qp = eye(14);
            for i = 1:length(tasks)
                 %% TRANSITION FROM PREVIOUS ACTION SET TO NEXT ACTION SET
                [Qp, ydotbar] = iCAT_task(tasks{i}.A, tasks{i}.J, ...
                                           Qp, ydotbar, tasks{i}.xdotbar, ...
                                           1e-4, 0.01, 10);
            end
            %% 3. Last task: residual damping
            [~, ydotbar] = iCAT_task(eye(14), eye(14), Qp, ydotbar, zeros(14,1), 1e-4, 0.01, 10);

            if arm == "L"
                actual_arm.X_o = actual_arm.wJo*ydotbar(1:7);
            else
                actual_arm.X_o = actual_arm.wJo*ydotbar(8:14);
            end

            if grasped && ~final

                H_actual = actual_arm.wJo*pinv(actual_arm.wJo);
                H_other = other_arm.wJo*pinv(other_arm.wJo);
                H_12 = [H_actual zeros(6,6);zeros(6,6) H_other];

                [v_ang, v_lin] = CartError(actual_arm.wTog , actual_arm.wTo);
                xdotbar = 1.0 * [v_ang; v_lin];
                xdotbar(1:3) = Saturate(xdotbar(1:3), 0.3);
                xdotbar(4:6) = Saturate(xdotbar(4:6), 0.3);

                mu_1 = obj.mu_0 + norm(xdotbar-actual_arm.X_o);
                mu_2 = obj.mu_0 + norm(xdotbar-other_arm.X_o);

                xdot_t = (mu_1*actual_arm.X_o+mu_2*other_arm.X_o)/(mu_1+mu_2);

                Xdot_t = [xdot_t;xdot_t];

                C = [H_actual -H_other];

                actual_arm.Xo_12 = H_12*(eye(12)-pinv(C)*C)*Xdot_t;
       
                if arm == "L"
                    bim_task_ID = find(cellfun(@(x) x.task_name == "LB", tasks), 1);
                elseif arm == "R"
                    bim_task_ID = find(cellfun(@(x) x.task_name == "RB", tasks), 1);
                end

                tasks{bim_task_ID}.xdotbar = actual_arm.Xo_12(1:6,:);


                tasks_lr = [tasks(bim_task_ID), tasks(1:bim_task_ID-1), tasks(bim_task_ID+1:end)];


                for i = 1:length(tasks_lr)
                     %% TRANSITION FROM PREVIOUS ACTION SET TO NEXT ACTION SET
                    [Qp, ydotbar] = iCAT_task(tasks_lr{i}.A, tasks_lr{i}.J, ...
                                               Qp, ydotbar, tasks_lr{i}.xdotbar, ...
                                               1e-4, 0.01, 10);
                end
                %% 3. Last task: residual damping
                [~, ydotbar] = iCAT_task(eye(14), eye(14), Qp, ydotbar, zeros(14,1), 1e-4, 0.01, 10);
            end
        end
    end
end