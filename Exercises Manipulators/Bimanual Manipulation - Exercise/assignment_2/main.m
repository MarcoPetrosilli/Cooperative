function main()
    %Add path
    addpath('./simulation_scripts');
    addpath('./tools')
    addpath('./icat')
    addpath('./tasks')
    clc;clear;close all; 
    %Simulation Parameters
    dt = 0.005;
    end_time = 15;
    
    xl = [];
    xr = [];
    dist = [];
    X_o1 = [];
    X_o2 = [];
    
    % Initialize Franka Emika Panda Model
    model = load("panda.mat");
    
    
    %Simulation Setup
    real_robot = false;
    
    
    %Initiliaze panda_arm() Class, specifying the base offset w.r.t World Frame
    arm1=panda_arm(model,eye(4));
    
    
    %TO DO: TRANSFORMATION MATRIX FROM WORLD FRAME TO RIGHT ARM BASE FRAME
    wTb2 =[-1 0 0 1.06;0 -1 0 -0.01;0 0 1 0;0 0 0 1];
    arm2=panda_arm(model,wTb2);
    
    
    %Initialize Bimanual Simulator Class
    bm_sim=bimanual_sim(dt,arm1,arm2,end_time);
    
    
    %Define Object Shape and origin Frame
    obj_length = 0.6;
    w_obj_pos = [0.5 0 0.59]';
    w_obj_ori = rotation(0,0,0);
    
    
    %Set goal frames for left and right arm, based on object frame
    %TO DO: Set arm goal frame based on object frame.
    arm1.setGoal(w_obj_pos,w_obj_ori,w_obj_pos+[-0.1 0 0]',rotation(pi, -pi/6, 0));
    arm2.setGoal(w_obj_pos,w_obj_ori,w_obj_pos+[+0.1 0 0]',rotation(0, pi+pi/6, 0));
    
    
    %Define Object goal frame (Cooperative Motion)
    wTog=[rotation(0,0,0) [0.65, -0.35, 0.25]'; 0 0 0 1];
    arm1.set_obj_goal(wTog)
    arm2.set_obj_goal(wTog)
    
    
    %Define Tasks, input values(Robot type(L,R,BM), Task Name)
    left_tool_task=tool_task("L","LT");
    right_tool_task=tool_task("R","RT");
    left_tool_task_2=tool_task("L","LT2");
    right_tool_task_2=tool_task("R","RT2");
    left_min_altitude=ee_altitude_task("L","LA",0.15);
    right_min_altitude=ee_altitude_task("R","RA",0.15);
    left_joint_limits_task=joint_limits_task("L","LL");
    right_joint_limits_task=joint_limits_task("R","RL");

    bim_constraint_task=bim_rigid_const_task("BIM_CONST");

    
    
    %Actions for each phase: go to phase, coop_motion phase, end_motion phase
    go_to_grasp_set={left_joint_limits_task, right_joint_limits_task, left_min_altitude,right_min_altitude,left_tool_task,right_tool_task};
    move_grasped_obj_set={bim_constraint_task, left_joint_limits_task, right_joint_limits_task, left_min_altitude,right_min_altitude,left_tool_task_2, right_tool_task_2};
    final_set = {left_min_altitude, right_min_altitude};
    
    
    %Unified task set
    unified_set = {bim_constraint_task, left_joint_limits_task, right_joint_limits_task, left_min_altitude,right_min_altitude,left_tool_task,right_tool_task, left_tool_task_2, right_tool_task_2};
    
    
    %Load Action Manager Class and load actions
    actionManager = ActionManager();
    actionManager.addAction(go_to_grasp_set,"go_to_grasp");
    actionManager.addAction(move_grasped_obj_set,"move_grasped_obj");
    actionManager.addAction(final_set,"final");
    actionManager.addUnifiedAction(unified_set);
    
    StateMachine = DualArmStateMachine(actionManager);
    
    %Initiliaze robot interface
    robot_udp=UDP_interface(real_robot);
    
    %Initialize logger
    logger=SimulationLogger(ceil(end_time/dt)+1,bm_sim,actionManager);
    
    %Main simulation Loop
    for t = 0:dt:end_time
    
        % 1. Receive UDP packets - DO NOT EDIT
        [ql,qr]=robot_udp.udp_receive(t);
        if real_robot==true %Only in real setup, assign current robot configuration as initial configuratio
            bm_sim.left_arm.q=ql;
            bm_sim.right_arm.q=qr;
        end
    
        % 2. Update Full kinematics of the bimanual system
        bm_sim.update_full_kinematics(StateMachine);
        
        % 3. Compute control commands for current action
        [q_dot]=actionManager.computeICAT(bm_sim, StateMachine);
        
        Xl = arm1.wJo*q_dot(1:7);
        Xr = arm2.wJo*q_dot(8:14);

        % 4. Step the simulator (integrate velocities)
        bm_sim.sim(q_dot);
        
        % 5. Send updated state to Pybullet
        robot_udp.send(t,bm_sim)
    
        % 6. Logging
        logger.update(bm_sim.time,bm_sim.loopCounter)
        bm_sim.time
    
        % 7. Optional real-time slowdown
        SlowdownToRealtime(dt);
    
        StateMachine.update(arm1, arm2, actionManager, t);

        [a, v] = CartError(arm1.wTt , arm2.wTt);
        dist = [dist norm(v)];
    
        xl = [xl Xl];
        xr = [xr Xr];
        X_o1 = [X_o1 arm1.X_o];
        X_o2 = [X_o2 arm2.X_o];
    end

    action=1;
    tasks=[1];
    logger.plotAll(action,tasks);

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

    X_o1_1 = timeseries(X_o1_(1,:), t);
    X_o2_1 = timeseries(X_o2_(1,:), t);
    xl_1 = timeseries(xl(1,:), t);
    xr_1 = timeseries(xr(1,:), t);
    figure;
    subplot(2,3,1);
    plot(X_o1_1);
    hold on;
    plot(X_o2_1);
    hold off
    legend('Xo1', 'Xo2');
    title('X_ang');

    X_o1_2 = timeseries(X_o1_(2,:), t);
    X_o2_2 = timeseries(X_o2_(2,:), t);
    xl_2 = timeseries(xl(2,:), t);
    xr_2 = timeseries(xr(2,:), t);
    subplot(2,3,2);
    plot(X_o1_2);
    hold on;
    plot(X_o2_2);
    hold off
    legend('Xo1', 'Xo2');
    title('Y_ang');

    X_o1_3 = timeseries(X_o1_(3,:), t);
    X_o2_3 = timeseries(X_o2_(3,:), t);
    xl_3 = timeseries(xl(3,:), t);
    xr_3 = timeseries(xr(3,:), t);
    subplot(2,3,3);
    plot(X_o1_3);
    hold on;
    plot(X_o2_3);
    hold off
    legend('Xo1', 'Xo2');
    title('Z_ang');

    X_o1_4 = timeseries(X_o1_(4,:), t);
    X_o2_4 = timeseries(X_o2_(4,:), t);
    xl_4 = timeseries(xl(4,:), t);
    xr_4 = timeseries(xr(4,:), t);
    subplot(2,3,4);
    plot(X_o1_4);
    hold on;
    plot(X_o2_4);
    hold off
    legend('Xo1', 'Xo2');
    title('X_lin');

    X_o1_5 = timeseries(X_o1_(5,:), t);
    X_o2_5 = timeseries(X_o2_(5,:), t);
    xl_5 = timeseries(xl(5,:), t);
    xr_5 = timeseries(xr(5,:), t);
    subplot(2,3,5);
    plot(X_o1_5);
    hold on;
    plot(X_o2_5);
    hold off
    legend('Xo1', 'Xo2');
    title('Y_lin');

    X_o1_6 = timeseries(X_o1_(6,:), t);
    X_o2_6 = timeseries(X_o2_(6,:), t);
    xl_6 = timeseries(xl(6,:), t);
    xr_6 = timeseries(xr(6,:), t);
    subplot(2,3,6);
    plot(X_o1_6);
    hold on;
    plot(X_o2_6);
    hold off
    legend('Xo1', 'Xo2');
    title('Z_lin');

    figure;
    subplot(2,3,1);
    plot(xl_1);
    hold on;
    plot(xr_1);
    hold off
    legend('xl', 'xr');
    title('X_ang');

    subplot(2,3,2);
    plot(xl_2);
    hold on;
    plot(xr_2);
    hold off
    legend('xl', 'xr');
    title('Y_ang');

    subplot(2,3,3);
    plot(xl_3);
    hold on;
    plot(xr_3);
    hold off
    legend('xl', 'xr');
    title('Z_ang');

    subplot(2,3,4);
    plot(xl_4);
    hold on;
    plot(xr_4);
    hold off
    legend('xl', 'xr');
    title('X_lin');

    subplot(2,3,5);
    plot(xl_5);
    hold on;
    plot(xr_5);
    hold off
    legend('xl', 'xr');
    title('Y_lin');

    subplot(2,3,6);
    plot(xl_6);
    hold on;
    plot(xr_6);
    hold off
    legend('xl', 'xr');
    title('Z_lin');
end