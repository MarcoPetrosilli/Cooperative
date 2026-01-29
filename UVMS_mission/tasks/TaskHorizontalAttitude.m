classdef TaskHorizontalAttitude < Task   
    properties
        theta
        id = "World Alignment";
    end
    methods
        function updateReference(obj, robot)
            persistent theta;
            wRv = robot.wTv(1:3,1:3);
            zR = wRv(:,3);
            ang =  skew(zR)*[0;0;1];
            obj.theta = atan2(norm(ang), dot(zR,[0;0;1]));
            if(obj.theta<0)
                obj.theta = 2*pi+obj.theta;
            end
            obj.xdotbar = -0.9 * (0.1-obj.theta);
            obj.xdotbar = Saturate(obj.xdotbar, 0.3);
            theta = [theta; obj.theta];
            assignin('base', 'theta', theta);
        end
        function updateJacobian(obj,robot)
            wRv = robot.wTv(1:3,1:3);
            ang =  skew(wRv(:,3))*[0;0;1];
            ang = ang / norm(ang);
            obj.J = ang'*[zeros(3,7) zeros(3,3) robot.wTv(1:3,1:3)];
        end
        function updateActivation(obj, robot)
            th = 0.2;
            delta = 0.1;
            obj.A = IncreasingBellShapedFunction(th-delta,th,0,1,obj.theta);
        end
    end
end