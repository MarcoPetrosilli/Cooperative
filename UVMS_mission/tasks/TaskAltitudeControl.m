classdef TaskAltitudeControl < Task   
    properties
        desired_altitude;
        equality;
        e;
        id;
    end

    
    methods

        function obj = TaskAltitudeControl(desired_altitude, mode)
            obj.desired_altitude = desired_altitude;
            switch mode
                case "safe_mode"
                    obj.equality = false;
                    obj.id = "Distance Safe Mode";

                case "to_altitude"
                    obj.equality = true;
                    obj.id = "Distance to Altitude";

                otherwise
                    obj.equality = false;
            end
        end

        function updateReference(obj, robot)

            wRv = robot.wTv(1:3,1:3);
            zR = wRv(:,3);
            
            ang =  skew(zR)*[0;0;1];

            theta_val = atan2(norm(ang), dot(zR,[0;0;1]));

            if(theta_val<0)
                theta_val = 2*pi+theta_val;
            end
            
            if isempty(robot.altitude)
                obj.e = 0;
            else
                obj.e = robot.altitude * cos(theta_val) - obj.desired_altitude;
            end

            obj.xdotbar = -0.8 * obj.e;
            obj.xdotbar = Saturate(obj.xdotbar, 0.2);
        end
        function updateJacobian(obj, robot)
            obj.J = [0;0;1]'*[zeros(3,7) robot.wTv(1:3,1:3), zeros(3,3)];
        end
        
        function updateActivation(obj, robot)

            if(obj.equality)
                obj.A = 1;
            else
                th = 0;
                delta = 0.5;
    
                obj.A = DecreasingBellShapedFunction(th,th+delta,0,1,obj.e);
                
            end   
        end
    end
end