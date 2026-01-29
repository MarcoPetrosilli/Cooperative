classdef DualArmStateMachine < handle
    
    properties (SetAccess = private)
        State       
        Tolerance 
        transition = false;
    end
    
    properties (Constant)

        STATE_APPROACHING = "APPROACHING" 
        STATE_GRASPED     = "GRASPED"     
        STATE_FINAL       = "FINAL"       
    end
    
    methods


        function obj = DualArmStateMachine(actionManager, tolerance)
            if nargin < 2
                obj.Tolerance = 1.0e-02;
            else
                obj.Tolerance = tolerance;
            end

            obj.State = obj.STATE_APPROACHING;
            actionManager.setCurrentAction("go_to_grasp");
        end
        
        function update(obj, arm1, arm2, actionManager, t)
            
            targetReached = obj.checkTargetReached(arm1, arm2);

            switch obj.State
                
                case obj.STATE_APPROACHING
                    if targetReached
                        disp(['Rot Error at Grasp: ', num2str(norm(arm1.rot_to_goal))]);
                       
                        obj.State = obj.STATE_GRASPED;
                        fprintf('State Transition: APPROACHING -> GRASPED\n');
                    end
                    
                case obj.STATE_GRASPED

                    if ~obj.transition
                        actionManager.setCurrentAction("move_grasped_obj");
                        arm1.tg = t;
                        arm2.tg = t;
                        obj.transition = true;
                    end

                    if targetReached
                        
                        obj.State = obj.STATE_FINAL;
                        
                        fprintf('State Transition: GRASPED -> FINAL\n');
                    end
                    
                case obj.STATE_FINAL

                    if obj.transition
                        actionManager.setCurrentAction("final");
                        arm1.tf = t;
                        arm2.tf = t;
                        obj.transition = false;
                    end
                     
            end
        end
        
        function isReached = checkTargetReached(obj, arm1, arm2)

            
            dist1 = norm(arm1.dist_to_goal) < obj.Tolerance;
            dist2 = norm(arm2.dist_to_goal) < obj.Tolerance;
            rot1  = norm(arm1.rot_to_goal)  < obj.Tolerance;
            rot2  = norm(arm2.rot_to_goal)  < obj.Tolerance;
            
            isReached = dist1 && dist2 && rot1 && rot2;

            disp(norm(arm1.dist_to_goal));
            disp(norm(arm1.rot_to_goal));
        end
        
        function isFinished = isFinished(obj)
            isFinished = (obj.State == obj.STATE_FINAL);
        end

        function isApproaching = isApproaching(obj)
            isApproaching = (obj.State == obj.STATE_APPROACHING);
        end

        function isGrasped = isGrasped(obj)
            isGrasped = (obj.State == obj.STATE_GRASPED);
        end
    end
end