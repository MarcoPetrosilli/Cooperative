classdef TaskJointsPosition < Task
    properties
        id = "Joints Home Position";
        q_home;
        e
    end
    methods
        function obj = TaskJointsPosition(q_home)
            obj.q_home = q_home;
        end
        function updateReference(obj, robot)
            % Errore tra posizione giunti attuale e quella di riposo
            obj.e = robot.q - obj.q_home;
            obj.xdotbar = -0.5 * obj.e; % Guadagno per riportarlo in posizione
        end
        function updateJacobian(obj, robot)
            % Lo Jacobiano per i giunti è una matrice identità per la parte braccio
            % [I(7x7) 0(7x6)]
            obj.J = [eye(7), zeros(7,6)]; 
        end
        function updateActivation(obj, robot)
            obj.A = eye(7);
        end
    end
end


% Il tuo sistema ha molti gradi di libertà (6 del veicolo + 7 del braccio = 13 totali). 
% Quando esegui [un'azione come safe_nav, stai usando solo i task del veicolo.
% L'iCAT calcola le velocità per tutti i 13 giunti. Poiché il braccio non 
% ha un task che gli impone di stare fermo, i suoi giunti sono "liberi".
% Durante i calcoli matriciali (specialmente con la pseudo-inversa dello Jacobiano), 
% piccole frazioni di velocità possono "sgocciolare" nei giunti del braccio 
% se questo aiuta a minimizzare l'energia totale o se ci sono residui numerici.

% Nota sulla priorità: Metti task_hold_arm come primo elemento del cell array 
% (priorità più bassa). In questo modo, se per qualche motivo il movimento del 
% veicolo avesse assoluto bisogno di muovere un po' il braccio per stabilizzarsi,
% potrebbe farlo, ma in generale il braccio rimarrà immobile.
% 
% Riassunto
% Il braccio si muoveva perché era "abbandonato a se stesso" dalla matematica del controllo. 
% Aggiungendo un task di controllo giunti a bassa priorità, gli dai un'ancora 
% che lo terrà fermo durante la navigazione.