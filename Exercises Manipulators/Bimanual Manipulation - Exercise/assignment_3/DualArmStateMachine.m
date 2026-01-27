classdef DualArmStateMachine < handle
    
    properties (SetAccess = private)
        State       
        Tolerance
        transition = false
        isReached
    end

    properties (SetAccess = public)
        robot_ID
    end
    
    properties (Constant)

        STATE_APPROACHING = "APPROACHING" 
        STATE_GRASPED     = "GRASPED"     
        STATE_FINAL       = "FINAL"       
    end
    
    methods


        function obj = DualArmStateMachine(actionManager, arm, tolerance)
            if nargin < 4
                obj.Tolerance = 1.0e-02;
            else
                obj.Tolerance = tolerance;
            end

            obj.State = obj.STATE_APPROACHING;

            if arm.robot_ID == "L"
                actionManager.setCurrentAction("l_go_to_grasp");
            else
                actionManager.setCurrentAction("r_go_to_grasp");
            end
        end
        
        function update(obj, arm, actionManager, other_StateMachine, t)
            
            obj.checkTargetReached(arm);

            switch obj.State
                
                case obj.STATE_APPROACHING
                    if obj.isReached
                        disp(['Rot Error at Grasp: ', num2str(norm(arm.rot_to_goal))]);
                        obj.State = obj.STATE_GRASPED;
                        obj.transition = true;
                        fprintf('State Transition: APPROACHING -> GRASPED\n');
                    end
                    
                case obj.STATE_GRASPED

                    if obj.isGrasped && other_StateMachine.isGrasped && obj.transition
                        if arm.robot_ID == "L"
                            actionManager.setCurrentAction("l_move_grasped_obj");
                        else
                            actionManager.setCurrentAction("r_move_grasped_obj");
                        end
                        obj.transition = false;
                        arm.tg = t;
                    end

                    if obj.isReached  
                      
                        obj.State = obj.STATE_FINAL;
                        fprintf('State Transition: GRASPED -> FINAL\n');
                    end
                    
                case obj.STATE_FINAL
                   if obj.isFinished() && other_StateMachine.isFinished() && ~obj.transition
                        if arm.robot_ID == "L"
                            actionManager.setCurrentAction("l_final");
                        else
                            actionManager.setCurrentAction("r_final");
                        end
                        obj.transition = true;
                        arm.tf = t;
                    end
            end
        end
        
        function checkTargetReached(obj, arm)
            dist = norm(arm.dist_to_goal) < obj.Tolerance;
            rot  = norm(arm.rot_to_goal)  < obj.Tolerance;
            
            obj.isReached = dist && rot;
        end
        
        function isFinished = isFinished(obj)
            isFinished = (obj.State == obj.STATE_FINAL);
        end

        function isGrasped = isGrasped(obj)
            isGrasped = (obj.State == obj.STATE_GRASPED);
        end

        function isReached = IsReached(obj)
            isReached = obj.isReached;
        end
    end
end