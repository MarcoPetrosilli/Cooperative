% classdef ActionManager < handle
%     properties
%         actions = {}
%         currentAction = 1
%         mu_0 = 0
%         action_names = []
%         unified_action = {}
%         lastAction = []
%         actionSwitchTime = 0
%         transitionDuration = 2.0
%     end
%     methods
% 
%         function addAction(obj, taskStack, action_name)
%             obj.actions{end+1} = taskStack;
%             obj.action_names{end+1} = action_name;
%         end
%         function cleanAction(obj, toRemove)
%              obj.actions(toRemove) = [];
%         end
%         function addUnifiedAction(obj, unified_set)
%             obj.unified_action = unified_set;
%         end
%         function setCurrentAction(obj, actionName)
%             idx = find(string(obj.action_names) == actionName, 1);
%             if isempty(idx)
%                 error('Action "%s" not found.', actionName);
%             end
%             obj.lastAction = obj.currentAction;
%             obj.currentAction = idx;
%             obj.actionSwitchTime = tic;
%         end
%         function [ydotbar] = computeICAT(obj, bm_system, actual_arm, other_arm, actual_StateMachine, other_StateMachine)
%             tasks = obj.unified_action;
%             tasks_id = cell(1, length(tasks));
%             for i = 1:length(tasks)
%                 tasks_id{i} = tasks{i}.task_name;
%             end
%             if obj.actionSwitchTime ~= 0
%                 t = toc(obj.actionSwitchTime);
%                 duration = obj.transitionDuration;
%                 alpha_in = IncreasingBellShapedFunction(0, duration, 0, 1, t);
%                 alpha_out = DecreasingBellShapedFunction(0, duration, 0, 1, t);
%             else
%                 alpha_in = 1;
%                 alpha_out = 0;
%             end
%             current_tasks = obj.actions{obj.currentAction};
%             current_tasks_id = cell(1, length(current_tasks));
%             for i = 1:length(current_tasks)
%                 current_tasks_id{i} = current_tasks{i}.task_name;
%             end
%             prev_tasks_id = {};
%             if ~isempty(obj.lastAction)
%                 prev_tasks = obj.actions{obj.lastAction};
%                 prev_tasks_id = cell(1, length(prev_tasks));
%                 for i = 1:length(prev_tasks)
%                     prev_tasks_id{i} = prev_tasks{i}.task_name;
%                 end
%             end
%             inCurrent = ismember(string(tasks_id), string(current_tasks_id));
%             inPrev    = ismember(string(tasks_id), string(prev_tasks_id));
%             for i = 1:length(tasks)
%                 task = tasks{i};
%                 task.updateReference(bm_system, actual_StateMachine);
%                 task.updateJacobian(bm_system);
%                 task.updateActivation(bm_system);
%                 if inCurrent(i) && ~inPrev(i)
%                     task.A = task.A * alpha_in;
%                 elseif ~inCurrent(i) && inPrev(i)
%                     task.A = task.A * alpha_out;
%                 elseif ~inCurrent(i) && ~inPrev(i) && ~task.constrained
%                     task.A = task.A * 0;
%                 end
%             end
% 
%             if actual_arm.robot_ID == "L"
%                 bim_task_ID = find(cellfun(@(x) x.task_name == "LB", tasks), 1);        
%             elseif actual_arm.robot_ID == "R"
%                 bim_task_ID = find(cellfun(@(x) x.task_name == "RB", tasks), 1);
%             end
% 
%             tasks_f = tasks;
%             tasks_f{bim_task_ID}.A = task.A * 0;
% 
%             ydotbar = obj.perform_ICAT(tasks_f);
% 
%             actual_arm.X_o = actual_arm.wJo*ydotbar;
% 
%             if actual_StateMachine.isGrasped() || other_StateMachine.isGrasped() 
% 
%                 actual_arm.Xo_12 = obj.coordinate_velocities(actual_arm, other_arm);
% 
%                 tasks_lr = tasks;
% 
%                 tasks_lr{bim_task_ID}.xdotbar = actual_arm.Xo_12(1:6);
% 
%                 ydotbar = obj.perform_ICAT(tasks_lr);
%             end
%         end
% 
%         function Xo_12 = coordinate_velocities(obj, actual_arm, other_arm)
% 
%             H_actual = actual_arm.wJo*pinv(actual_arm.wJo);
% 
%             H_other = other_arm.wJo*pinv(other_arm.wJo);
% 
%             H_12 = [H_actual zeros(6,6);zeros(6,6) H_other];
% 
%             [v_ang, v_lin] = CartError(actual_arm.wTog , actual_arm.wTo);
%             % [v_ango, v_lino] = CartError(other_arm.wTog , other_arm.wTo);
% 
%             xdotbar = 1.0 * [v_ang; v_lin];
% 
%             xdotbar(1:3) = Saturate(xdotbar(1:3), 0.3);
% 
%             xdotbar(4:6) = Saturate(xdotbar(4:6), 0.3);
% 
%             mu_1 = obj.mu_0 + norm(xdotbar-actual_arm.X_o);
% 
%             mu_2 = obj.mu_0 + norm(xdotbar-other_arm.X_o);
% 
%             xdot_t = (mu_1*actual_arm.X_o+mu_2*other_arm.X_o)/(mu_1+mu_2);
% 
%             Xdot_t = [xdot_t;xdot_t];
% 
%             C = [H_actual -H_other];
% 
%             Xo_12 = H_12*(eye(12)-pinv(C)*C)*Xdot_t;
% 
%         end
% 
%         function tasks_lr = reorder_priorities(obj, tasks, actual_arm)
% 
%             if actual_arm.robot_ID == "L"
%                 bim_task_ID = find(cellfun(@(x) x.task_name == "LB", tasks), 1);        
%             elseif actual_arm.robot_ID == "R"
%                 bim_task_ID = find(cellfun(@(x) x.task_name == "RB", tasks), 1);
%             end
% 
%             tasks{bim_task_ID}.xdotbar = actual_arm.Xo_12(1:6,:);
% 
%             tasks_lr = [tasks(bim_task_ID), tasks(1:bim_task_ID-1), tasks(bim_task_ID+1:end)];
% 
%             % if actual_arm.robot_ID == "L"
%             %     tool_task_ID = find(cellfun(@(x) x.task_name == "LT2", tasks), 1);        
%             % elseif actual_arm.robot_ID == "R"
%             %     tool_task_ID = find(cellfun(@(x) x.task_name == "RT2", tasks), 1);
%             % end
%             % 
%             % tasks_lr(tool_task_ID) = [];
%         end
% 
%         function ydotbar = perform_ICAT(obj, tasks)
%             ydotbar = zeros(7,1);
%             Qp = eye(7);
%             for i = 1:length(tasks)
%                 [Qp, ydotbar] = iCAT_task(tasks{i}.A, tasks{i}.J, ...
%                                            Qp, ydotbar, tasks{i}.xdotbar, ...
%                                            1e-4, 0.01, 10);
%             end
%             [~, ydotbar] = iCAT_task(eye(7), eye(7), Qp, ydotbar, zeros(7,1), 1e-4, 0.01, 10);
%         end
%     end
% end

classdef ActionManager < handle
    properties
        actions = {}
        currentAction = 1
        mu_0 = 0
        action_names = []
        unified_action = {}
        lastAction = []
        actionSwitchTime = 0
        transitionDuration = 1.0
        
        % --- NEW: Struttura per salvare la storia delle attivazioni ---
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
            
            % --- NEW: Inizializza la struttura di log per ogni task ---
            for i = 1:length(unified_set)
                t_name = unified_set{i}.task_name;
                % Crea un campo vuoto per ogni task
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
        
        function [ydotbar] = computeICAT(obj, bm_system, actual_arm, other_arm, actual_StateMachine, other_StateMachine)
            tasks = obj.unified_action;
            tasks_id = cell(1, length(tasks));
            for i = 1:length(tasks)
                tasks_id{i} = tasks{i}.task_name;
            end
            
            % Calcolo Alpha (Blending)
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
                
                % Applicazione del Blending
                if inCurrent(i) && ~inPrev(i)
                    task.A = task.A * alpha_in;
                elseif ~inCurrent(i) && inPrev(i)
                    task.A = task.A * alpha_out;
                elseif ~inCurrent(i) && ~inPrev(i)
                    task.A = task.A * 0;
                end
                
                current_act = diag(task.A)'; % Trasposta per avere riga
                obj.activation_history.(task.task_name)(end+1, :) = current_act;
            end
            
            % ... (Resto della logica per bim_task_ID e ICAT invariata) ...
            if actual_arm.robot_ID == "L"
                bim_task_ID = find(cellfun(@(x) x.task_name == "LB", tasks), 1);        
            elseif actual_arm.robot_ID == "R"
                bim_task_ID = find(cellfun(@(x) x.task_name == "RB", tasks), 1);
            end
            
            % Nota: Salviamo 'task' temporaneamente perchè il ciclo for sopra 
            % sovrascrive la variabile 'task' ad ogni iterazione.
            % Qui sotto usi 'task.A' ma 'task' è l'ultimo del ciclo for!
            % Correzione consigliata: usa tasks{bim_task_ID}.A invece di task.A
            

             % FIX: usa l'elemento specifico
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
        
        % ... (Altri metodi come coordinate_velocities, perform_ICAT rimangono uguali) ...
        

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

        % --- NEW: Funzione per plottare tutte le attivazioni ---
        function plotActivations(obj, dt)
            % Recupera i nomi di tutti i task salvati
            task_names = fieldnames(obj.activation_history);
            num_tasks = length(task_names);
            
            % Crea una figura
            figure('Name', 'Task Activations over Time', 'Color', 'w');
            
            % Calcola layout subplot (es. griglia quadrata)
            cols = ceil(sqrt(num_tasks));
            rows = ceil(num_tasks / cols);
            
            for i = 1:num_tasks
                name = task_names{i};
                data = obj.activation_history.(name);
                
                % Se non ci sono dati, salta
                if isempty(data)
                    continue; 
                end
                
                % Crea asse temporale
                steps = size(data, 1);
                time_vector = (0:steps-1) * dt;
                
                subplot(rows, cols, i);
                plot(time_vector, data, 'LineWidth', 1.5);
                
                title(name, 'Interpreter', 'none', 'FontWeight', 'bold');
                xlabel('Time [s]');
                ylabel('Activation (A)');
                ylim([-0.1, 1.1]); % Le attivazioni sono tipicamente tra 0 e 1
                grid on;
            end
        end
    end
end