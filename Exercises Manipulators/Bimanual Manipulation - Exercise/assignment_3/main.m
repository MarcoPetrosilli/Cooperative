function main()
    % Add path
    addpath('./simulation_scripts');
    addpath('./tools')
    addpath('./icat')
    addpath('./tasks')
    clc; clear; close all; 

    % --- Simulation Parameters ---
    dt = 0.005;
    end_time = 20;

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
    wTog = [rotation(0, 0, 0) [0.65, -0.35, 4.3]'; 0 0 0 1];
    arm1.set_obj_goal(wTog);
    arm2.set_obj_goal(wTog);

    % --- Define Tasks ---
    left_tool_task = tool_task("L", "LT");
    right_tool_task = tool_task("R", "RT");
    left_min_altitude = ee_altitude_task("L", "LA", 0.15);
    right_min_altitude = ee_altitude_task("R", "RA", 0.15);
    left_joint_limits_task = joint_limits_task("L", "LL");
    right_joint_limits_task = joint_limits_task("R", "RL");
    left_bim_constraint_task = bim_rigid_const_task("L", "LB");
    right_bim_constraint_task = bim_rigid_const_task("R", "RB");

    % --- Define Action Sets (LEFT) ---
    l_go_to_grasp_set = {left_joint_limits_task, left_min_altitude, left_tool_task};
    l_move_grasped_obj_set = {left_joint_limits_task, left_min_altitude, left_bim_constraint_task};
    l_final_set = {left_min_altitude};
    l_unified_set = {left_joint_limits_task, left_min_altitude, left_tool_task, left_bim_constraint_task};

    % --- Define Action Sets (RIGHT) ---
    r_go_to_grasp_set = {right_joint_limits_task, right_min_altitude, right_tool_task};
    r_move_grasped_obj_set = {right_joint_limits_task, right_min_altitude, right_bim_constraint_task};
    r_final_set = {right_min_altitude};
    r_unified_set = {right_joint_limits_task, right_min_altitude, right_tool_task, right_bim_constraint_task};

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
        
        q_dot = [q_dot_l(1:7); q_dot_r(8:14)];

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
        l_StateMachine.update(arm1, l_actionManager, r_StateMachine);
        r_StateMachine.update(arm2, r_actionManager, l_StateMachine);
    end

    % Plotting
    action = 1;
    tasks = [1];
    logger.plotAll(action, tasks);
end