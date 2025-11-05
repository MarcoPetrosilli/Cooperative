classdef TaskAlignment < Task   
    properties

    end


    methods
        function updateReference(obj, robot)
            persistent theta;

            wRv = robot.wTv(1:3,1:3);
            zR = wRv(:,3);
            
            ang =  skew(zR)*[0;0;1];

            % ref = [1;0;0];                       
            % sign_theta = sign(dot(ref, ang));
            
            theta_val = atan2(norm(ang), dot(wRv(:,3),[0;0;1]));

            if(theta_val<0)
                theta_val = 2*pi+theta_val;
            end

            obj.xdotbar = -0.2 * (0.1-theta_val);

            obj.xdotbar(1:1) = Saturate(obj.xdotbar(1:1), 0.2);
        
            theta = [theta; theta_val];

            assignin('base', 'theta', theta);
        end


        function updateJacobian(obj,robot)
            % wRv = robot.wTv(1:3,1:3);
            % ang =  skew(wRv(:,3))*[0;0;1];
            % ang = ang / norm(ang);
            % ang = wRv'*ang;
            % 
            % obj.J = ang'*[zeros(3,7) zeros(3,3),eye(3,3)];

            wRv = robot.wTv(1:3,1:3);
            ang =  skew(wRv(:,3))*[0;0;1];
            ang = ang / norm(ang);

            obj.J = ang'*[zeros(3,7) zeros(3,3),wRv];
        end
        
        function updateActivation(obj, robot)
            th = 0.2;
            delta = 0.1;

            
            wRv = robot.wTv(1:3,1:3);
            zR = wRv(:,3);

            ang =  skew(zR)*[0;0;1];

            % ref = [1;0;0];                       
            % sign_theta = sign(dot(ref, ang));
            
            theta_val = atan2(norm(ang), dot(wRv(:,3),[0;0;1]));

            if(theta_val<0)
                theta_val = 2*pi+theta_val;
            end

            obj.A = IncreasingBellShapedFunction(th-delta,th,0,1,theta_val);
        end
    end
end