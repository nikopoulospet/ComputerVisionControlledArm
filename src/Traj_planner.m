classdef Traj_planner
    properties
        path
        %path is an Nx3 array of positions to interpolate between
        quinic_coeff = [ 0, 0, 0, 10, -15, 6];
        cubic_coeff = [ 0, 0, 3, -2]
    end    
    methods
        function scalar = createScalar(Traj_planner,time,type)
           if type == "cubic"
               scalar = Traj_planner.cubic_coeff(3) * (time^2) + Traj_planner.cubic_coeff(4) * (time^3)
           end
           if type == "quintic"
               scalar = Traj_planner.quinic_coeff(4) * (time^3) + Traj_planner.quinic_coeff(5) * (time^4) + Traj_planner.quinic_coeff(6) * (time^5) 
           end
            
        end
        function coeffs = cubic_traj(Traj_planner,t0,tf,v0,vf,q0,qf)
            %q0 and qf are joint angles.
            %v0 and vf are joint velocities.
            syms a0 a1 a2 a3;
            eq1 = a0 + a1*t0 + a2 * (t0^2) + a3 * (t0^3) == q0;
            eq2 = a1 + 2 * a2 * t0 + 3 * a3 * (t0^2) == v0;
            eq3 = a0 + a1*tf + a2 * (tf^2) + a3 * (tf^3) == qf;
            eq4 = a1 + 2 * a2 * tf + 3 * a3 * (tf^2) == vf;
            eqns = [eq1,eq2,eq3,eq4];
            c = solve(eqns,[a0 a1 a2 a3]);
            coeffs = [c.a0,c.a1,c.a2,c.a3];
            disp(coeffs);
        end
        
        function coeffs = quintic_traj(Traj_planner,t0,tf,v0,vf,q0,qf,ac0,acf)
            %q0 and qf are joint angles.
            %v0 and vf are joint velocities.
            syms a0 a1 a2 a3 a4 a5;
            eq1 = a0 + a1*t0 + a2 * (t0^2) + a3 * (t0^3) + a4 * (t0^4) + a5 * (t0^5) == q0;
            eq2 = a1 + 2 * a2 * t0 + 3 * a3 * (t0^2) + 4 * a4 * (t0^3) + 5 * a5 * (t0^4) == v0;
            eq3 = 2 * a2 + 6 * a3 * (t0) + 12 * a4 * (t0^2) + 20 * a5 * (t0^3) == ac0;
            eq4 = a0 + a1*tf + a2 * (tf^2) + a3 * (tf^3) + a4 * (tf^4) + a5 * (tf^5) == qf;
            eq5 = a1 + 2 * a2 * tf + 3 * a3 * (tf^2) + 4 * a4 * (tf^3) + 5 * a5 * (tf^4)== vf;
            eq6 = 2 * a2 + 6 * a3 * (tf) + 12 * a4 * (tf^2) + 20 * a5 * (tf^3) == acf;
            eqns = [eq1,eq2,eq3,eq4,eq5,eq6];
            c = solve(eqns,[a0 a1 a2 a3 a4 a5]);
            coeffs = [c.a0,c.a1,c.a2,c.a3,c.a4,c.a5];
        end
        
        function trajectory = linear_traj(Traj_planner,start_pos,end_pos,t0,tf)
            % takes in start and ending positions of the end-effector
            % returns planned trajectory in the task space
            
            %generate the points between the start and end (via points)
            between_pos = Traj_planner.interpolate(start_pos,end_pos);

            tipmatrix = between_pos;
            writematrix(tipmatrix, 'Part5_2_a.csv', "WriteMode", "append");
            
            for i = 1:1:length(between_pos)-1
                
                %for 
                for j = 1:1:3
                    %get the two points to go between
                    x0 = between_pos(i,j);
                    x1 = between_pos(i + 1,j);

                    %to solve for the linear trajectoy
                    syms a0 a1;

                    %solve for their linear coeficents
                    eq1 = a0 + a1*t0 == x0;
                    eq2 = a0 + a1*tf == x1;
                    eqns = [eq1,eq2];

                    %get the coefficants
                    c = solve(eqns,[a0 a1]);
                    
                    k = j*2;
                    %save the coefficants for the planned traj
                    trajectory(i,k -1) = c.a0;
                    trajectory(i,k) = c.a1;
                end

            end
        end
    end
    methods(Static)
        function points = interpolate(vertex1, vertex2)
            % interpolates 10 points between the specified verteces and
            % returns a 3x10 matrix with evenly spaced points
            P1 = vertex1;
            P2 = vertex2;
            n = 10;
            t = linspace(0,1,n)';
            points = (1-t)*P1 + t*P2;
        end
    end
    
end
