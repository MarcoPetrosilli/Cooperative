function main()
    % Add path
    addpath('./simulation_scripts');
    addpath('./tools')
    addpath('./icat')
    addpath('./tasks')
    clc; clear; close all; 

    % --- Simulation Parameters ---
    dt = 0.005;
    end_time = 15;

    xl = [];
    xr = [];
    dist = [];
    X_o1 = [];
    X_o2 = [];
    Xo_12_arm1 = [];
    Xo_12_arm2 = [];

    % Initialize Franka Emika Panda Model
    model = load("panda.mat");

    % --- Simulation Setup ---
    real_robot = false;

    % Initialize panda_arm() Class
    arm1 = panda_arm(model, eye(4), "L");
    
    % Transformation Matrix from World to Right Arm Base
    wTb2 = [-1 0 0 1.06; 0 -1 0 -0.01; 0 0 1 0; 0 0 0 1];
    arm2 = panda_arm(model, wTb2,  "R");

    % Initialize Bimanual Simulator Class
    bm_sim = bimanual_sim(dt, arm1, arm2, end_time);

    % --- Define Object ---
    obj_length = 0.6;
    w_obj_pos = [0.5 0 0.59]';
    w_obj_ori = rotation(0, 0, 0);

    % Set goal frames based on object frame
    % arm1.setGoal(w_obj_pos, w_obj_ori, w_obj_pos + [0 0 -0.05]', rotation(pi, -deg2rad(20), 0));
    % arm2.setGoal(w_obj_pos, w_obj_ori, w_obj_pos + [+0.05 0 +0.05]', rotation(0, pi+deg2rad(20), 0));
    
    arm1.setGoal(w_obj_pos, w_obj_ori, w_obj_pos + [-0.05 0 0]', rotation(pi, -deg2rad(20), 0));
    arm2.setGoal(w_obj_pos, w_obj_ori, w_obj_pos + [+0.05 0 0]', rotation(0, pi+deg2rad(20), 0));

    % Define Object goal frame (Cooperative Motion)
    % wTog = [rotation(0, +pi/4, 0) [0.65, -0.35, 0.3]'; 0 0 0 1];

    wTog = [rotation(0, 0, 0) [0.6, 0.4, 0.3]'; 0 0 0 1];
 
    arm1.set_obj_goal(wTog);
    arm2.set_obj_goal(wTog);

    % --- Define Tasks ---
    left_tool_task = tool_task("L", "LT");
    right_tool_task = tool_task("R", "RT");
    left_tool_task_2 = tool_task("L", "LT2");
    right_tool_task_2 = tool_task("R", "RT2");
    left_min_altitude = ee_altitude_task("L", "LA", 0.3);
    right_min_altitude = ee_altitude_task("R", "RA", 0.15);
    left_joint_limits_task = joint_limits_task("L", "LL");
    right_joint_limits_task = joint_limits_task("R", "RL");
    left_bim_constraint_task = bim_rigid_const_task("L", "LB");
    right_bim_constraint_task = bim_rigid_const_task("R", "RB");

    % --- Define Action Sets (LEFT) ---
    l_go_to_grasp_set = {left_joint_limits_task, left_min_altitude, left_tool_task};
    l_move_grasped_obj_set = {left_joint_limits_task, left_min_altitude, left_tool_task_2};
    l_final_set = {left_min_altitude};
    l_unified_set = {left_bim_constraint_task, left_joint_limits_task, left_min_altitude, left_tool_task, left_tool_task_2};

    % --- Define Action Sets (RIGHT) ---
    r_go_to_grasp_set = {right_joint_limits_task, right_min_altitude, right_tool_task};
    r_move_grasped_obj_set = {right_joint_limits_task, right_min_altitude, right_tool_task_2};
    r_final_set = {right_min_altitude};
    r_unified_set = {right_bim_constraint_task, right_joint_limits_task, right_min_altitude, right_tool_task, right_tool_task_2};

    % --- Initialize LEFT Action Manager ---
    l_actionManager = ActionManager();
    l_actionManager.addAction(l_go_to_grasp_set, "l_go_to_grasp");
    l_actionManager.addAction(l_move_grasped_obj_set, "l_move_grasped_obj");
    l_actionManager.addAction(l_final_set, "l_final");
    l_actionManager.addUnifiedAction(l_unified_set);
    l_actionManager.setCurrentAction("l_go_to_grasp");

    % --- Initialize RIGHT Action Manager ---
    r_actionManager = ActionManager();
    r_actionManager.addAction(r_go_to_grasp_set, "r_go_to_grasp");
    r_actionManager.addAction(r_move_grasped_obj_set, "r_move_grasped_obj");
    r_actionManager.addAction(r_final_set, "r_final");
    r_actionManager.addUnifiedAction(r_unified_set);
    r_actionManager.setCurrentAction("r_go_to_grasp");

    % --- Initialize State Machines ---
    
    l_StateMachine = DualArmStateMachine(l_actionManager, arm1); 
    r_StateMachine = DualArmStateMachine(r_actionManager, arm2);

    % Initialize Robot Interface & Logger   
    robot_udp = UDP_interface(real_robot);

    logger = SimulationLogger(ceil(end_time/dt)+1, bm_sim, l_actionManager);

    % --- Main Simulation Loop ---
    for t = 0:dt:end_time
        
        % 1. Receive UDP packets
        [ql, qr] = robot_udp.udp_receive(t);
        
        if real_robot
            bm_sim.left_arm.q = ql;
            bm_sim.right_arm.q = qr;
        end

        % 2. Update Full Kinematics
        bm_sim.update_full_kinematics(l_StateMachine, r_StateMachine);
        
        % 3. Compute Control Commands
        [q_dot_l] = l_actionManager.computeICAT(bm_sim, arm1, arm2, l_StateMachine, r_StateMachine);
        [q_dot_r] = r_actionManager.computeICAT(bm_sim, arm2, arm1, r_StateMachine, l_StateMachine);
        
        if l_StateMachine.isGrasped() || r_StateMachine.isGrasped() 
               coordinate_velocities(arm1, arm2);
               q_dot_l = l_actionManager.perform_ICAT(l_actionManager.unified_action);
               q_dot_r = r_actionManager.perform_ICAT(r_actionManager.unified_action);
        end

        Xl = arm1.wJo*q_dot_l;
        Xr = arm2.wJo*q_dot_r;

        q_dot = [q_dot_l; q_dot_r];

        % 4. Step Simulator
        bm_sim.sim(q_dot);
        
        % 5. Send UDP
        robot_udp.send(t, bm_sim);

        % 6. Logging
        logger.update(bm_sim.time, bm_sim.loopCounter);
        bm_sim.time

        % 7. Real-time Slowdown
        SlowdownToRealtime(dt);
        
        % 8. Update State Machines
        l_StateMachine.update(arm1, l_actionManager, r_StateMachine, t);
        r_StateMachine.update(arm2, r_actionManager, l_StateMachine, t);

        [a, v] = CartError(arm1.wTt , arm2.wTt);
        dist = [dist norm(v)];
    
        xl = [xl Xl];
        xr = [xr Xr];
        X_o1 = [X_o1 arm1.X_o];
        X_o2 = [X_o2 arm2.X_o];
        Xo_12_arm1 = [Xo_12_arm1 arm1.Xo_12];
        Xo_12_arm2 = [Xo_12_arm2 arm2.Xo_12];
    end

    fprintf('Plotting Left Arm Activations...\n');
    l_actionManager.plotActivations(dt);
    
    % Plot delle attivazioni per il braccio destro
    fprintf('Plotting Right Arm Activations...\n');
    r_actionManager.plotActivations(dt);

    % % Plotting
    % action = 1;
    % tasks = [1];
    % logger.plotAll(action, tasks);
    % 
    t = 0:dt:end_time;
    d = timeseries(dist, t);
    figure;
    plot(d);
    hold on
    xline(arm1.tg)
    % xline(arm1.tf)
    hold off

    tg_pos = find(t == arm1.tg);
    % tf_pos = find(t == arm1.tf);
    t = t(tg_pos:end);
    X_o1_ = X_o1(:, tg_pos:end);
    X_o2_ = X_o2(:, tg_pos:end);
    xl = xl(:, tg_pos:end);
    xr = xr(:, tg_pos:end);
    Xo_12_arm1 = Xo_12_arm1(:, tg_pos:end);
    Xo_12_arm2 = Xo_12_arm2(:, tg_pos:end);

    X_o1_1 = timeseries(X_o1_(1,:), t);
    X_o2_1 = timeseries(X_o2_(1,:), t);
    xl_1 = timeseries(xl(1,:), t);
    xr_1 = timeseries(xr(1,:), t);
    Xo_12_arm1_1 = timeseries(Xo_12_arm1(1,:), t);
    figure;
    subplot(2,3,1);
    plot(X_o1_1);
    hold on;
    plot(X_o2_1);
    plot(Xo_12_arm1_1);
    hold off
    legend('Xo1', 'Xo2', 'Xo12arm1');
    title('X_ang');

    X_o1_2 = timeseries(X_o1_(2,:), t);
    X_o2_2 = timeseries(X_o2_(2,:), t);
    xl_2 = timeseries(xl(2,:), t);
    xr_2 = timeseries(xr(2,:), t);
    Xo_12_arm1_2 = timeseries(Xo_12_arm1(2,:), t);
    subplot(2,3,2);
    plot(X_o1_2);
    hold on;
    plot(X_o2_2);
    plot(Xo_12_arm1_2);
    hold off
    legend('Xo1', 'Xo2', 'Xo12arm1');
    title('Y_ang');

    X_o1_3 = timeseries(X_o1_(3,:), t);
    X_o2_3 = timeseries(X_o2_(3,:), t);
    xl_3 = timeseries(xl(3,:), t);
    xr_3 = timeseries(xr(3,:), t);
    Xo_12_arm1_3 = timeseries(Xo_12_arm1(3,:), t);
    subplot(2,3,3);
    plot(X_o1_3);
    hold on;
    plot(X_o2_3);
    plot(Xo_12_arm1_3);
    hold off
    legend('Xo1', 'Xo2', 'Xo12arm1');
    title('Z_ang');

    X_o1_4 = timeseries(X_o1_(4,:), t);
    X_o2_4 = timeseries(X_o2_(4,:), t);
    xl_4 = timeseries(xl(4,:), t);
    xr_4 = timeseries(xr(4,:), t);
    Xo_12_arm1_4 = timeseries(Xo_12_arm1(4,:), t);
    subplot(2,3,4);
    plot(X_o1_4);
    hold on;
    plot(X_o2_4);
    plot(Xo_12_arm1_4);
    hold off
    legend('Xo1', 'Xo2', 'Xo12arm1');
    title('X_lin');

    X_o1_5 = timeseries(X_o1_(5,:), t);
    X_o2_5 = timeseries(X_o2_(5,:), t);
    xl_5 = timeseries(xl(5,:), t);
    xr_5 = timeseries(xr(5,:), t);
    Xo_12_arm1_5 = timeseries(Xo_12_arm1(5,:), t);
    subplot(2,3,5);
    plot(X_o1_5);
    hold on;
    plot(X_o2_5);
    plot(Xo_12_arm1_5);
    hold off
    legend('Xo1', 'Xo2', 'Xo12arm1');
    title('Y_lin');

    X_o1_6 = timeseries(X_o1_(6,:), t);
    X_o2_6 = timeseries(X_o2_(6,:), t);
    xl_6 = timeseries(xl(6,:), t);
    xr_6 = timeseries(xr(6,:), t);
    Xo_12_arm1_6 = timeseries(Xo_12_arm1(6,:), t);
    Xo_12_arm2_6 = timeseries(Xo_12_arm2(6,:), t);
    subplot(2,3,6);
    plot(X_o1_6);
    hold on;
    plot(X_o2_6);
    plot(Xo_12_arm1_6);
    hold off
    legend('Xo1', 'Xo2', 'Xo12arm1');
    title('Z_lin');

    figure;
    subplot(2,3,1);
    plot(Xo_12_arm1_1);
    hold on;
    plot(xl_1);
    plot(xr_1);
    hold off
    legend('Xo12arm1', 'xl', 'xr');
    title('X_ang');

    subplot(2,3,2);
    plot(Xo_12_arm1_2);
    hold on;
    plot(xl_2);
    plot(xr_2);
    hold off
    legend('Xo12arm1', 'xl', 'xr');
    title('Y_ang');

    subplot(2,3,3);
    plot(Xo_12_arm1_3);
    hold on;
    plot(xl_3);
    plot(xr_3);
    hold off
    legend('Xo12arm1', 'xl', 'xr');
    title('Z_ang');

    subplot(2,3,4);
    plot(Xo_12_arm1_4);
    hold on;
    plot(xl_4);
    plot(xr_4);
    hold off
    legend('Xo12arm1', 'xl', 'xr');
    title('X_lin');

    subplot(2,3,5);
    plot(Xo_12_arm1_5);
    hold on;
    plot(xl_5);
    plot(xr_5);
    hold off
    legend('Xo12arm1', 'xl', 'xr');
    title('Y_lin');

    subplot(2,3,6);
    plot(Xo_12_arm1_6);
    hold on;
    plot(xl_6);
    plot(xr_6);
    plot(Xo_12_arm2_6);
    hold off
    legend('Xo12arm1', 'xl', 'xr', 'Xo12arm2');
    title('Z_lin');
end


function coordinate_velocities(left_arm, right_arm)

        mu_0 = 0;

        H_actual = left_arm.wJo*pinv(left_arm.wJo);
        H_other = right_arm.wJo*pinv(right_arm.wJo);
        H_12 = [H_actual zeros(6,6);zeros(6,6) H_other];
        
        [v_ang, v_lin] = CartError(left_arm.wTog , left_arm.wTo);
        xdotbar = 1.0 * [v_ang; v_lin];
        xdotbar(1:3) = Saturate(xdotbar(1:3), 0.3);
        xdotbar(4:6) = Saturate(xdotbar(4:6), 0.3);
        mu_1 = mu_0 + norm(xdotbar-left_arm.X_o);
        mu_2 = mu_0 + norm(xdotbar-right_arm.X_o);
        xdot_t = (mu_1*left_arm.X_o+mu_2*right_arm.X_o)/(mu_1+mu_2);
        Xdot_t = [xdot_t;xdot_t];
        C = [H_actual -H_other];
        Xo_12 = H_12*(eye(12)-pinv(C)*C)*Xdot_t;
        left_arm.Xo_12 = Xo_12(1:6);
        right_arm.Xo_12 = Xo_12(7:12);
        end