classdef ActionManager < handle
    properties
        actions = {}
        currentAction = 1
        action_names = []
        unified_action = {}
        lastAction = []
        actionSwitchTime = 0
        transitionDuration = 1.0 
        activation_history = struct(); 
    end
    methods
        function addAction(obj, taskStack, action_name)
            obj.actions{end+1} = taskStack;
            obj.action_names{end+1} = action_name;
        end
        function cleanAction(obj, toRemove)
             obj.actions(toRemove) = [];
        end
        function addUnifiedAction(obj, unified_set)
            obj.unified_action = unified_set;
            
            
            for i = 1:length(unified_set)
                t_name = unified_set{i}.task_name;
                obj.activation_history.(t_name) = [];
            end
        end
        function setCurrentAction(obj, actionName)
            idx = find(string(obj.action_names) == actionName, 1);
            if isempty(idx)
                error('Action "%s" not found.', actionName);
            end
            obj.lastAction = obj.currentAction;
            obj.currentAction = idx;
            obj.actionSwitchTime = tic;
        end
        
        function [ydotbar] = computeICAT(obj, bm_system, actual_arm, actual_StateMachine)
            tasks = obj.unified_action;
            tasks_id = cell(1, length(tasks));
            for i = 1:length(tasks)
                tasks_id{i} = tasks{i}.task_name;
            end
            

            if obj.actionSwitchTime ~= 0
                t = toc(obj.actionSwitchTime);
                duration = obj.transitionDuration;
                alpha_in = IncreasingBellShapedFunction(0, duration, 0, 1, t);
                alpha_out = DecreasingBellShapedFunction(0, duration, 0, 1, t);
            else
                alpha_in = 1;
                alpha_out = 0;
            end
            
            current_tasks = obj.actions{obj.currentAction};
            current_tasks_id = cell(1, length(current_tasks));
            for i = 1:length(current_tasks)
                current_tasks_id{i} = current_tasks{i}.task_name;
            end
            prev_tasks_id = {};
            if ~isempty(obj.lastAction)
                prev_tasks = obj.actions{obj.lastAction};
                prev_tasks_id = cell(1, length(prev_tasks));
                for i = 1:length(prev_tasks)
                    prev_tasks_id{i} = prev_tasks{i}.task_name;
                end
            end
            
            inCurrent = ismember(string(tasks_id), string(current_tasks_id));
            inPrev    = ismember(string(tasks_id), string(prev_tasks_id));
            
            for i = 1:length(tasks)
                task = tasks{i};
                task.updateReference(bm_system, actual_StateMachine);
                task.updateJacobian(bm_system);
                task.updateActivation(bm_system);
                
               
                if inCurrent(i) && ~inPrev(i) && ~task.constrained
                    task.A = task.A * alpha_in;
                elseif ~inCurrent(i) && inPrev(i) && ~task.constrained
                    task.A = task.A * alpha_out;
                elseif ~inCurrent(i) && ~inPrev(i) || task.constrained
                    task.A = task.A * 0;
                end
                
                current_act = diag(task.A)';
                obj.activation_history.(task.task_name)(end+1, :) = current_act;
            end
            
  
            if actual_arm.robot_ID == "L"
                bim_task_ID = find(cellfun(@(x) x.task_name == "LC", tasks), 1);        
            elseif actual_arm.robot_ID == "R"
                bim_task_ID = find(cellfun(@(x) x.task_name == "RC", tasks), 1);
            end
            
            ydotbar = obj.perform_ICAT(tasks);
            actual_arm.X_o = actual_arm.wJo*ydotbar;
            
            tasks{bim_task_ID}.A = eye(6);


            if actual_arm.robot_ID == "L"
                tool_task_ID = find(cellfun(@(x) x.task_name == "LT2", tasks), 1);        
            elseif actual_arm.robot_ID == "R"
                tool_task_ID = find(cellfun(@(x) x.task_name == "RT2", tasks), 1);
            end
         
            tasks{tool_task_ID}.A = zeros(6);
         
        end
        

        function ydotbar = perform_ICAT(obj, tasks)
            ydotbar = zeros(7,1);
            Qp = eye(7);
            for i = 1:length(tasks)
                [Qp, ydotbar] = iCAT_task(tasks{i}.A, tasks{i}.J, ...
                                           Qp, ydotbar, tasks{i}.xdotbar, ...
                                           1e-4, 0.01, 10);
            end
            [~, ydotbar] = iCAT_task(eye(7), eye(7), Qp, ydotbar, zeros(7,1), 1e-4, 0.01, 10);
        end

       
        function plotActivations(obj, dt, arm)

            task_names = fieldnames(obj.activation_history);
        
            task_names = setdiff(task_names, {'LC','RC'}, 'stable');
        
            num_tasks = length(task_names);
        
            if num_tasks == 0
                warning('No task activations to plot (LC and RC excluded).');
                return;
            end
        
            figure('Name', 'Task Activations over Time', 'Color', 'w');
        
            cols = ceil(sqrt(num_tasks));
            rows = ceil(num_tasks / cols);
        
            for i = 1:num_tasks
                name = task_names{i};
                data = obj.activation_history.(name);
        
                if isempty(data)
                    continue;
                end
        
                steps = size(data, 1);
                time_vector = (0:steps-1) * dt;
        
                subplot(rows, cols, i);
                plot(time_vector, data, 'LineWidth', 1.5);
                xline(arm.tg)
                xline(arm.tf)
        
                title(name, 'Interpreter', 'none', 'FontWeight', 'bold');
                xlabel('Time [s]');
                ylabel('Activation (A)');
                ylim([-0.1, 1.1]);
                grid on;
            end
        end
    end
end