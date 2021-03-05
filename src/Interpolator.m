classdef Interpolator
    properties
        time_movement
        type_movement
    end
    
    methods
        function obj = Interpolator(type,time)
            obj.time_movement = time;
            obj.type_movement = type;
        end
        
        function scaler = getScaler(obj, delta_time)
           if strcmp(obj.type_movement, 'linear') == 1
               scaler = delta_time/time;
           elseif strcmp(obj.type_movement, 'cubic') == 1
               % pass in 0 for q0 and 1 for qf since scalar goes from 0 to
               % 1
               a = double(obj.cubic_traj(0,obj.time_movement,0,0,0,1));
               scaler = a(1) * a(2)*delta_time + a(3) * delta_time^2 + a(4) * delta_time^3;
           elseif strcmp(obj.type_movement, 'quintic') == 1
                a = double(obj.quintic_traj(0,obj.time_movement,0,0,0,1,0,0));
               scaler = a(1) * a(2) * delta_time + a(3) * delta_time^2 + a(4) * delta_time^3 + a(5) * delta_time^4 + a(6) * delta_time^5;
           end
        end   
    end
    methods(Static)
      function coeffs = cubic_traj(t0,tf,v0,vf,q0,qf)
        %q0 and qf are joint angles.
        %v0 and vf are joint velocities.
        constraints = [q0; v0; qf; vf];
        eq1 = [1 t0 t0^2 t0^3]; %q0
        eq2 = [0 1 2*t0 3*t0^2]; %v0
        eq3 = [1 tf tf^2 tf^3]; %qf
        eq4 = [0 1 2*tf 3*tf^2]; %vf
        eqns = [eq1; eq2; eq3; eq4];
        coeffs = transpose(inv(eqns) * constraints);     
       end  
       function coeffs = quintic_traj(t0,tf,v0,vf,q0,qf,ac0,acf)
         %q0 and qf are joint angles.
         %v0 and vf are joint velocities.
         %ac0 and acf are joint accelerations
        constraints = [q0; v0; ac0; qf; vf; ac0];
        eq1 = [1 t0 t0^2 t0^3 t0^4 t0^5]; %q0
        eq2 = [0 1 2*t0 3*t0^2 4*t0^3 5*t0^4]; %v0
        eq3 = [0 0 2 6*t0 12*t0^2 20*t0^3]; %a0
        eq4 = [1 tf tf^2 tf^3 tf^4 tf^5]; %q0
        eq5 = [0 1 2*tf 3*tf^2 4*tf^3 5*tf^4]; %v0
        eq6 = [0 0 2 6*tf 12*tf^2 20*tf^3]; %a0
        eqns = [eq1; eq2; eq3; eq4; eq5; eq6];
        coeffs = transpose(inv(eqns) * constraints);  
        end  
    end
end