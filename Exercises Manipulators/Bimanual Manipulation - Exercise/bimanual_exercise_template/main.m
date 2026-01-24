function main()
%Add path
addpath('./simulation_scripts');
addpath('./tools')
addpath('./icat')
addpath('./tasks')
clc;clear;close all; 
%Simulation Parameters
dt = 0.005;
end_time = 20;

mu_0 = 1;

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
obj_length = 0.12;
w_obj_pos = [0.5 0 0.5]';
w_obj_ori = rotation(0,0,0);
wTog=[w_obj_ori w_obj_pos; 0 0 0 1];

%Set goal frames for left and right arm, based on object frame
%TO DO: Set arm goal frame based on object frame.
arm1.setGoal(w_obj_pos,w_obj_ori,w_obj_pos+[-0.1 0 0]',rotation(pi, -pi/6, 0));
arm2.setGoal(w_obj_pos,w_obj_ori,w_obj_pos+[+0.1 0 0]',rotation(0, pi+pi/6, 0));

%Define Object goal frame (Cooperative Motion)
wTog=[rotation(0,0,0) [0.65, -0.35, 0.28]'; 0 0 0 1];
arm1.set_obj_goal(wTog)
arm2.set_obj_goal(wTog)

%Define Tasks, input values(Robot type(L,R,BM), Task Name)

left_tool_task=tool_task("L","LT");
right_tool_task=tool_task("R","RT");
left_min_altitude=ee_altitude_task("L","LA",0.15);
right_min_altitude=ee_altitude_task("R","RA",0.15);
left_joint_limits_task=joint_limits_task("L","LL");
right_joint_limits_task=joint_limits_task("R","RL");
left_bim_constraint_task=bim_rigid_const_task("L","LB");
right_bim_constraint_task=bim_rigid_const_task("R","RB");

%Actions for each phase: go to phase, coop_motion phase, end_motion phase

go_to_grasp_set={left_joint_limits_task, right_joint_limits_task, left_min_altitude,right_min_altitude,left_tool_task,right_tool_task};
move_grasped_obj_set={left_joint_limits_task, right_joint_limits_task, left_min_altitude,right_min_altitude, left_bim_constraint_task, right_bim_constraint_task};
unified_set = {left_joint_limits_task, right_joint_limits_task, left_min_altitude,right_min_altitude,left_tool_task,right_tool_task,left_bim_constraint_task,right_bim_constraint_task};
final_set = {left_min_altitude, right_min_altitude};

grasped = false;
final = false;

%Load Action Manager Class and load actions
actionManager = ActionManager();

actionManager.addAction(go_to_grasp_set,"go_to_grasp");
actionManager.addAction(move_grasped_obj_set,"move_grasped_obj");
actionManager.addAction(final_set,"final");

actionManager.addUnifiedAction(unified_set);

actionManager.setCurrentAction("go_to_grasp");

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
    bm_sim.update_full_kinematics(grasped);
    
    % 3. Compute control commands for current action
    [q_dot]=actionManager.computeICAT(bm_sim, arm1, arm2, grasped, final);
    
    
    % 4. Step the simulator (integrate velocities)
    bm_sim.sim(q_dot);
    
    % 5. Send updated state to Pybullet
    robot_udp.send(t,bm_sim)

    % 6. Lggging
    logger.update(bm_sim.time,bm_sim.loopCounter)
    bm_sim.time
    % 7. Optional real-time slowdown
    SlowdownToRealtime(dt);
    
    if (norm(arm1.dist_to_goal)<1.0e-03) && norm(arm2.dist_to_goal)<1.0e-03 && ~grasped
        actionManager.setCurrentAction("move_grasped_obj");
        grasped = true;
    elseif (norm(arm1.dist_to_goal)<1.0e-03) && norm(arm2.dist_to_goal)<1.0e-03 && ~final && grasped
        actionManager.setCurrentAction("final");
        final = true;
    end
end
%Display joint position and velocity, Display for a given action, a number
%of tasks
action=1;
tasks=[1];
logger.plotAll(action,tasks);
end
