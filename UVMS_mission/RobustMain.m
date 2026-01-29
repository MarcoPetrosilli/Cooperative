%% 
% Add paths
addpath('./simulation_scripts');
addpath('./tools');
addpath('./icat');
addpath('./robust_robot');
addpath('./tasks/');
clc; clear; close all;

% Simulation parameters
dt = 0.01;
endTime = 70;
% Initialize robot model and simulator
robotModel = UvmsModel();          
sim = UvmsSim(dt, robotModel, endTime);
% Initialize Unity interface
unity = UnityInterface("127.0.0.1");



%% Define desired positions and orientations (world frame)
w_arm_goal_position = [12.2025, 37.3748, -39.8860]';
w_arm_goal_orientation = [0, pi, pi/2];
w_vehicle_goal_position = [10.5, 37.5, -38]';
% w_vehicle_goal_position = [9.5, 38.5, -38]'; TEST
w_vehicle_goal_orientation = [0, -0.06, 0.5];

% Set goals in the robot model
robotModel.setGoal(...
    w_arm_goal_position,...
    w_arm_goal_orientation,...
    w_vehicle_goal_position,...
    w_vehicle_goal_orientation);

%% Define tasks and actions
% Altitude parameters
landing_altitude = 0.1;
safe_altitude = 2.0;
% Transitions trasholds (tuning)
dist_threshold = 1e-1;                     % to wp [m]
manip_threshold = 1e-1;
ang_threshold = 0.1;
alt_threshold = landing_altitude + 1e-2;   % landing [m]
tool_threshold = 1e-2;                     % manipulation [m]
% Parameters for manipulator
arm_reach = 0.6;
armBase_vehicle_dist = 1.0;
q_home = 2*(robotModel.jlmax + robotModel.jlmin)/3;
% q_home = [0.0 0.0 0.0 -pi/2 0.0 -pi/2 0.0]';
mu_min = 0.015;
% Task list
task_target_attitude = TaskTargetAlignment();
task_to_altitude = TaskAltitudeControl(landing_altitude, "to_altitude"); 
task_min_safe_altitude = TaskAltitudeControl(safe_altitude, "safe_mode"); 
task_pose = TaskPoseControl();                             
task_horizontal_attitude = TaskHorizontalAttitude();             
task_tool = TaskToolControl();                                              
task_still_manip = TaskJointsPosition(q_home);
task_manipulability_check = TaskManipulabilityCheck(arm_reach, armBase_vehicle_dist);
task_look_ahead = TaskLookAhead();
task_dexterity = TaskDexterity(mu_min, robotModel);
task_mantain_xy = TaskMantainxy();
%---
safe_wp_nav_set  = {
    task_min_safe_altitude,...
    task_still_manip, ...
    task_horizontal_attitude,...
    task_look_ahead, ...
    task_pose, ...
    };

manipulability_check = {
    task_min_safe_altitude, ...
    task_still_manip, ...
    task_horizontal_attitude, ...
    task_target_attitude,...
    task_manipulability_check, ... 
};

landing_set = {
    task_still_manip, ...
    task_horizontal_attitude, ...
    task_target_attitude,...
    task_manipulability_check, ...
    task_mantain_xy, ...
    task_to_altitude, ...
    };

manip_set = {
    task_horizontal_attitude, ...
    task_target_attitude,...
    task_manipulability_check,...
    task_mantain_xy, ...
    task_to_altitude,...
    task_dexterity,...
    task_tool, ...
    };

unified_set = {
    task_min_safe_altitude, ...
    task_still_manip, ...
    task_horizontal_attitude, ...
    task_target_attitude, ...
    task_look_ahead, ...
    task_manipulability_check, ...
    task_mantain_xy,...
    task_dexterity,...
    task_pose, ...
    task_to_altitude, ...
    task_tool, ...
    };

%% Define actions and add to ActionManager
actionManager = ActionManager();

actionManager.addAction(safe_wp_nav_set,"Safe Navigation");
actionManager.addAction(manipulability_check, "Manipulability Check")
actionManager.addAction(landing_set,"Landing");
actionManager.addAction(manip_set,"Manipulation");
actionManager.addUnifiedAction(unified_set);

currentState = "Safe Navigation";

actionManager.setCurrentAction(currentState);

%% Initialize the logger
logger = SimulationLogger(...
    ceil(endTime/dt)+1,...
    robotModel,...
    unified_set);

%% Main simulation loop

for step = 1:sim.maxSteps
    logging_disp = mod(sim.loopCounter, round(1 / sim.dt));
    % 1. Receive altitude from Unity
    robotModel.altitude = unity.receiveAltitude(robotModel);
    
    % === FSM ===
    switch currentState

        case "Safe Navigation"
            if logging_disp == 0
                fprintf("WP distance: %.2f m\n", task_pose.err);
            end
            if task_pose.err < dist_threshold % ma se nella simulazione non ci arriva mai, va bene così o deve essere nullo perchè equality
                currentState = "Manipulability Check";
                actionManager.setCurrentAction(currentState);
                fprintf('t = %.2f s: Waypoint reached. Start Manipulability Check \n', sim.time);
            end
        case "Manipulability Check" % manipulability and alignment
            if logging_disp == 0
                fprintf("Distance to reachability: %.2f m\n", task_manipulability_check.err);
                fprintf("Angular error to target: %.2f rad\n ", task_target_attitude.err);
            end
            if (abs(task_manipulability_check.err) < manip_threshold)  && (abs(task_target_attitude.err) < ang_threshold)
                currentState = "Landing";
                actionManager.setCurrentAction(currentState);
                fprintf('t = %.2f s: Target in manipulation range. Start Landing \n', sim.time);
                robotModel.stablePos = robotModel.wTv(:,:);
            end
        case "Landing"
            if logging_disp == 0
                fprintf("XY error = [%.2f %.2f] m\n", task_mantain_xy.err(1), task_mantain_xy.err(2));
            end
            if robotModel.altitude <= alt_threshold
                currentState = "Manipulation";
                actionManager.setCurrentAction(currentState);
                fprintf('t = %.2f s: Landing complete. Start Manipulation \n', sim.time);
            end
            
        case "Manipulation"
            % veichle is still - only manipulator movement. 
            [~, lin] = CartError(robotModel.vTg , robotModel.vTt);
            tool_dist = norm(lin);
            if logging_disp == 0
                fprintf("Target distance: %.3f \n", tool_dist);
                fprintf("Dexterity: mu = %.4f \n", task_dexterity.current_mu)
                fprintf("XY error = [%.2f %.2f] m\n", task_mantain_xy.err(1), task_mantain_xy.err(2));
                if tool_dist < tool_threshold
                    fprintf("Target reached - Mission complete \n")
                end
            end
            
    end % === FSM ===
    


    % 2. Compute control commands for current action
    [v_nu, q_dot] = actionManager.computeICAT(robotModel);

    % 3. Step the simulator (integrate velocities)
    sim.step(v_nu, q_dot);

    % 4. Send updated state to Unity
    unity.send(robotModel);

    % 5. Logging
    logger.update(sim.time, sim.loopCounter);

    % 6. Optional debug prints
    if logging_disp == 0
        fprintf('t = %.2f s\n', sim.time);
        fprintf('Alt = %.2f m\n', robotModel.altitude);
        fprintf('Pose : \n \t x = %.2f m\n \t y = %.2f m\n \t z = %.2f m\n]\n', robotModel.wTv(1, 4), robotModel.wTv(2, 4), robotModel.wTv(3, 4))
    end

    % 7. Optional real-time slowdown
    SlowdownToRealtime(dt);
end

% Display plots
logger.plotAll();

% t = 0:dt:endTime;
% figure;
% plot(theta, t);

% Clean up Unity interface
delete(unity);