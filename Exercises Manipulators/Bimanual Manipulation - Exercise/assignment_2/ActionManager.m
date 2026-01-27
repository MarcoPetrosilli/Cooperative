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

        function [ydotbar] = computeICAT(obj, bm_system, StateMachine)

            tasks = obj.unified_action;
            tasks_id = [];

            for i = 1:length(tasks)
                tasks_id{i} = tasks{i}.task_name;
            end

            % compute blending ratio
            if obj.actionSwitchTime ~= 0
                t = toc(obj.actionSwitchTime);
                duration = obj.transitionDuration;
                
                % Chi entra deve andare da 0 a 1 (Increasing)
                alpha_in = IncreasingBellShapedFunction(0, duration, 0, 1, t);
                
                % Chi esce deve andare da 1 a 0 (Decreasing)
                alpha_out = DecreasingBellShapedFunction(0, duration, 0, 1, t);
            else
                % Steady state (nessuna transizione in corso)
                alpha_in = 1;
                alpha_out = 0;
            end

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

            obj.updateTasks(bm_system, tasks, StateMachine, inCurrent, inPrev, alpha_in, alpha_out)

            %% 2. Perform ICAT and residual dumping (task-priority inverse kinematics) for the current Action
            
            ydotbar = obj.perform_ICAT(tasks);
            
            if StateMachine.isGrasped()       
                bm_system.left_arm.X_o = bm_system.left_arm.wJo*ydotbar(1:7);
                bm_system.right_arm.X_o = bm_system.right_arm.wJo*ydotbar(8:14);
            end

        end

        function ydotbar = perform_ICAT(obj, tasks)

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
        end

        function updateTasks(obj, bm_system, tasks, StateMachine, inCurrent, inPrev, alpha_in, alpha_out)

            for i = 1:length(tasks)
                task = tasks{i};

                task.updateReference(bm_system, StateMachine);
                task.updateJacobian(bm_system, StateMachine);
                task.updateActivation(bm_system);


                if inCurrent(i) && ~inPrev(i) && ~task.constrained
                    % entering → fade in
                    task.A = task.A * alpha_in;
                elseif ~inCurrent(i) && inPrev(i) && ~task.constrained
                    % leaving → fade out
                    task.A = task.A * alpha_out;
                elseif ~inCurrent(i) && ~inPrev(i)
                    task.A = task.A * 0;
                % else
                    % steady → normal activation
                end
            end
        end

    end
end