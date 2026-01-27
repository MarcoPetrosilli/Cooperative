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

    dist = [];
    X_o1 = [];
    X_o2 = [];
    Xo_12 = [];

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
    obj_length = 0.12;
    w_obj_pos = [0.5 0 0.5]';
    w_obj_ori = rotation(0, 0, 0);

    % Set goal frames based on object frame
    arm1.setGoal(w_obj_pos, w_obj_ori, w_obj_pos + [-0.1 0 0]', rotation(pi, -pi/6, 0));
    arm2.setGoal(w_obj_pos, w_obj_ori, w_obj_pos + [+0.1 0 0]', rotation(0, pi+pi/6, 0));

    % Define Object goal frame (Cooperative Motion)
    wTog = [rotation(0, 0, 0) [0.65, -0.35, 0.25]'; 0 0 0 1];
    arm1.set_obj_goal(wTog);
    arm2.set_obj_goal(wTog);

    % --- Define Tasks ---
    left_tool_task = tool_task("L", "LT");
    right_tool_task = tool_task("R", "RT");
    left_tool_task_2 = tool_task_2("L", "LT2");
    right_tool_task_2 = tool_task_2("R", "RT2");
    left_min_altitude = ee_altitude_task("L", "LA", 0.15);
    right_min_altitude = ee_altitude_task("R", "RA", 0.15);
    left_joint_limits_task = joint_limits_task("L", "LL");
    right_joint_limits_task = joint_limits_task("R", "RL");
    left_bim_constraint_task = bim_rigid_const_task("L", "LB");
    right_bim_constraint_task = bim_rigid_const_task("R", "RB");

    % --- Define Action Sets (LEFT) ---
    l_go_to_grasp_set = {left_joint_limits_task, left_min_altitude, left_tool_task};
    l_move_grasped_obj_set = {left_joint_limits_task, left_min_altitude, left_tool_task_2, left_bim_constraint_task};
    l_final_set = {left_min_altitude};
    l_unified_set = {left_joint_limits_task, left_min_altitude, left_tool_task, left_tool_task_2, left_bim_constraint_task};

    % --- Define Action Sets (RIGHT) ---
    r_go_to_grasp_set = {right_joint_limits_task, right_min_altitude, right_tool_task};
    r_move_grasped_obj_set = {right_joint_limits_task, right_min_altitude, right_tool_task_2, right_bim_constraint_task};
    r_final_set = {right_min_altitude};
    r_unified_set = {right_joint_limits_task, right_min_altitude, right_tool_task, right_tool_task_2, right_bim_constraint_task};

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
    
        X_o1 = [X_o1 arm1.X_o];
        X_o2 = [X_o2 arm2.X_o];
        Xo_12 = [Xo_12 arm1.Xo_12];
    end

    % Plotting
    action = 1;
    tasks = [1];
    logger.plotAll(action, tasks);

    t = 0:dt:end_time;
    d = timeseries(dist, t);
    figure;
    plot(d);
    hold on
    xline(arm1.tg)
    xline(arm1.tf)
    hold off

    %tg_pos = find(t == arm1.tg);
    %t = t(tg_pos:end);
    X_o1_1 = timeseries(X_o1(1,:), t);
    X_o2_1 = timeseries(X_o2(1,:), t);
    %Xo_12_1 = timeseries(Xo_12(1,:), t);
    figure;
    subplot(2,3,1);
    plot(X_o1_1);
    hold on;
    plot(X_o2_1);
    %plot(Xo_12_1);
    hold off
    legend('Xo1', 'Xo2');
    %legend('Xo1', 'Xo2', 'Xo12');
    title('X_ang');

    X_o1_2 = timeseries(X_o1(2,:), t);
    X_o2_2 = timeseries(X_o2(2,:), t);
    %Xo_12_2 = timeseries(Xo_12(2,:), t);
    subplot(2,3,2);
    plot(X_o1_2);
    hold on;
    plot(X_o2_2);
    %plot(Xo_12_2);
    hold off
    legend('Xo1', 'Xo2');
    %legend('Xo1', 'Xo2', 'Xo12');
    title('Y_ang');

    X_o1_3 = timeseries(X_o1(3,:), t);
    X_o2_3 = timeseries(X_o2(3,:), t);
    %Xo_12_3 = timeseries(Xo_12(3,:), t);
    subplot(2,3,3);
    plot(X_o1_3);
    hold on;
    plot(X_o2_3);
    %plot(Xo_12_3);
    hold off
    legend('Xo1', 'Xo2');
    %legend('Xo1', 'Xo2', 'Xo12');
    title('Z_ang');

    X_o1_4 = timeseries(X_o1(4,:), t);
    X_o2_4 = timeseries(X_o2(4,:), t);
    %Xo_12_4 = timeseries(Xo_12(4,:), t);
    subplot(2,3,4);
    plot(X_o1_4);
    hold on;
    plot(X_o2_4);
    %plot(Xo_12_4);
    hold off
    legend('Xo1', 'Xo2');
    %legend('Xo1', 'Xo2', 'Xo12');
    title('X_lin');

    X_o1_5 = timeseries(X_o1(5,:), t);
    X_o2_5 = timeseries(X_o2(5,:), t);
    %Xo_12_5 = timeseries(Xo_12(5,:), t);
    subplot(2,3,5);
    plot(X_o1_5);
    hold on;
    plot(X_o2_5);
    %plot(Xo_12_5);
    hold off
    legend('Xo1', 'Xo2');
    %legend('Xo1', 'Xo2', 'Xo12');
    title('Y_lin');

    X_o1_6 = timeseries(X_o1(6,:), t);
    X_o2_6 = timeseries(X_o2(6,:), t);
    %Xo_12_6 = timeseries(Xo_12(6,:), t);
    subplot(2,3,6);
    plot(X_o1_6);
    hold on;
    plot(X_o2_6);
    %plot(Xo_12_6);
    hold off
    legend('Xo1', 'Xo2');
    %legend('Xo1', 'Xo2', 'Xo12');
    title('Z_lin');
end