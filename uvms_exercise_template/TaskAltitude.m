classdef TaskAltitude < Task   
    properties
        desired_altitude = 2.0;
    end


    methods
        function updateReference(obj, robot)

            wRv = robot.wTv(1:3,1:3);
            zR = wRv(:,3);
            
            ang =  skew(zR)*[0;0;1];

            % ref = [1;0;0];                       
            % sign_theta = sign(dot(ref, ang));

            theta_val = atan2(norm(ang), dot(wRv(:,3),[0;0;1]));

            if(theta_val<0)
                theta_val = 2*pi+theta_val;
            end

            e = robot.altitude * cos(theta_val) - obj.desired_altitude;
            
            obj.xdotbar = 0.2 * (0.5 - e);
            obj.xdotbar(1:1) = Saturate(obj.xdotbar(1:1), 0.2);
        end
        function updateJacobian(obj, robot)
            obj.J = [0;0;1]'*[zeros(3,7) robot.wTv(1:3,1:3), zeros(3,3)];
        end
        
        function updateActivation(obj, robot)
            th = 0;
            delta = 0.5;

            wRv = robot.wTv(1:3,1:3);
            zR = wRv(:,3);
            
            ang =  skew(zR)*[0;0;1];

            % ref = [1;0;0];                       
            % sign_theta = sign(dot(ref, ang));

            theta_val = atan2(norm(ang), dot(wRv(:,3),[0;0;1]));

            if(theta_val<0)
                theta_val = 2*pi+theta_val;
            end

            e = robot.altitude * cos(theta_val) - obj.desired_altitude;

            obj.A = DecreasingBellShapedFunction(th,th+delta,0,1,e);
        end
    end
end