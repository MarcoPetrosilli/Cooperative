classdef ActionManager < handle
    properties
        actions = {}          % cell array of actions (each = cell array of tasks)
        action_names = {}     % names of actions
        unified_action = {}   % all tasks (union)
        currentAction = 1     % index of active action
        lastAction = []       % previous action index
        actionSwitchTime = 0  % timer handle
        transitionDuration = 5.0 % [s] blending duration
    end

    methods
        function addAction(obj, taskStack, action_name)
            % Add a named action composed of several tasks
            obj.actions{end+1} = taskStack;
            obj.action_names{end+1} = char(action_name);
        end

        function addUnifiedAction(obj, unified_set)
            % Define the global set of all tasks
            obj.unified_action = unified_set;
        end

        function setCurrentAction(obj, actionName)
            idx = find(strcmp(obj.action_names, char(actionName)), 1);
            if isempty(idx)
                error('Action "%s" not found.', actionName);
            end
            obj.lastAction = obj.currentAction;
            obj.currentAction = idx;
            obj.actionSwitchTime = tic; % start blending timer
        end

        function [v_nu, qdot] = computeICAT(obj, robot)
            tasks = obj.unified_action;
            tasks_id = [];
            for i = 1:length(tasks)
                tasks_id{i} = tasks{i}.id;
            end

            % compute blending ratio
            if obj.actionSwitchTime ~= 0
                t = toc(obj.actionSwitchTime);
            else
                t = obj.transitionDuration;
            end

            % current/previous task sets
            current_tasks = obj.actions{obj.currentAction};
            current_tasks_id = [];
            for i = 1:length(current_tasks)
                current_tasks_id{i} = current_tasks{i}.id;
            end

            prev_tasks_id = [];
            if ~isempty(obj.lastAction)
                prev_tasks = obj.actions{obj.lastAction};
                for i = 1:length(prev_tasks)
                    prev_tasks_id{i} = prev_tasks{i}.id;
                end
            else
                prev_tasks = {};
            end

            inCurrent = ismember(string(tasks_id), string(current_tasks_id));
            inPrev    = ismember(string(tasks_id), string(prev_tasks_id));

            % update all tasks
            for i = 1:length(tasks)
                task = tasks{i};
                task.updateReference(robot);
                task.updateJacobian(robot);
                if inCurrent(i) && inPrev(i)
                    task.updateActivation(robot)
                elseif inCurrent(i) && ~inPrev(i) % entering → fade in
                    task.updateActivation(robot);
                    task.A = task.A * IncreasingBellShapedFunction(0,obj.transitionDuration,0,1,t);
                    
                elseif ~inCurrent(i) && inPrev(i) % leaving → fade out
                    task.updateActivation(robot);
                    task.A = task.A * DecreasingBellShapedFunction(0,obj.transitionDuration,0,1,t);
                else % task not considered
                    task.updateActivation(robot);
                    task.A = task.A * 0;
                end
            end

            % ICAT computation
            ydotbar = zeros(13,1);
            Qp = eye(13);
            for i = 1:length(tasks)
                i
                [Qp, ydotbar] = iCAT_task(tasks{i}.A, tasks{i}.J, Qp, ydotbar, tasks{i}.xdotbar, 1e-4, 0.01, 10);
            end

            [~, ydotbar] = iCAT_task(eye(13), eye(13), Qp, ydotbar, zeros(13,1), 1e-4, 0.01, 10);

            qdot = ydotbar(1:7);
            v_nu = ydotbar(8:13);
        end
    end
end