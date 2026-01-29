classdef TaskDexterity < Task   
    properties
        id = "Dexterity Objective";
        mu_min;
        current_mu;
        q_center = [];
    end
    
    methods
        function obj = TaskDexterity(mu_min)
            obj.mu_min = mu_min;
            obj.q_center = (robot.jlmax + jlmin) / 2;
        end
        
        function updateReference(obj, robot)
            J_a = RobustJacobian(robot.q);
            obj.current_mu = sqrt(det(J_a * J_a'));
            obj.err = obj.mu_min - obj.current_mu;
            
            q_error = obj.q_center - robot.q;
            % obj.xdotbar = 0.1 * 
            % Se siamo in zona critica, generiamo una velocità di allontanamento.
            % Poiché non sappiamo a priori "in che direzione" muoverci per 
            % aumentare la mu, usiamo una strategia di massimizzazione del gradiente
            % o, più semplicemente, spingiamo il drone verso il target.

            if obj.err > 0
                % Direzione: dal veicolo verso il target (per accorciare la distanza del braccio)
                dir_to_target = robot.goalPosition(1:2) - robot.wTv(1:2,4);
                obj.xdotbar = 0.5 * [dir_to_target; 0];
                obj.xdotbar(1:2) = Saturate(obj.xdotbar(1:2), 0.1);
            end
        end
        
        function updateJacobian(obj, robot)
            obj.J = [zeros(2,7), robot.wTv(1:2,1:3), zeros(2,3)];
        end
        
        function updateActivation(obj, robot)
            delta = 1e-2;
            act = DecreasingBellShapedFunction(0, delta, 0, 1, obj.err);
            
            obj.A = eye(2) * act;
        end
    end
end