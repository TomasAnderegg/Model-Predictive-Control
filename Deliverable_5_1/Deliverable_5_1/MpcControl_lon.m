classdef MpcControl_lon < MpcControlBase
    
    methods
        % Design a YALMIP optimizer object that takes a steady-state state
        % and input (xs, us) and returns a control input
        function ctrl_opti = setup_controller(mpc)
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % INPUTS
            %   x0           - initial state (estimate)
            %   V_ref, u_ref - reference state/input
            %   d_est        - disturbance estimate
            %   x0other      - initial state of other car
            % OUTPUTS
            %   u0           - input to apply to the system
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            N_segs = ceil(mpc.H/mpc.Ts); % Horizon steps
            N = N_segs + 1;              % Last index in 1-based Matlab indexing

            [nx, nu] = size(mpc.B);
            
            % Targets
            V_ref = sdpvar(1);
            u_ref = sdpvar(1);

            % Disturbance estimate (Ignore this before Todo 4.1)
            d_est = sdpvar(1);

            % Initial states
            x0 = sdpvar(nx, 1);
            x0other = sdpvar(nx, 1); % (Ignore this before Todo 5.1)

            % Input to apply to the system
            u0 = sdpvar(nu, 1);
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            
            % NOTE: The matrices mpc.A, mpc.B, mpc.C and mpc.D
            %       are the DISCRETE-TIME MODEL of your system.
            %       You can find the linearization steady-state
            %       in mpc.xs and mpc.us.
            
            % SET THE PROBLEM CONSTRAINTS con AND THE OBJECTIVE obj HERE
            
            % Define optimization variables
            x = sdpvar(nx,N,'full');
            z = sdpvar(nx,N,'full');
            u = sdpvar(nu,N-1,'full');
            
            load('tube mpc data.mat', 'X_tilde', 'U_tilde', 'Xf', 'P', 'E', 'x_safe', 'Q', 'R', 'K');
            
            con = [];
            obj = 0;
            
           x(:,1) = x0other - x0 - [x_safe;0];
           con = [con, E.A*(x(:,1)-z(:,1)) <= E.b];

            for i = 1:N-1                
                con = [con, z(:,i+1) == mpc.A*z(:,i) - mpc.B*u(i)];
                con = [con, U_tilde.A*u(i) <= U_tilde.b];
                con = [con, X_tilde.A*z(:,i)<= X_tilde.b];
                obj = obj + (z(:,i))'*Q*(z(:,i)) + (u(i))'*R*(u(i));

            end
            obj = obj + (z(:,N))'*P*(z(:,N));
            con = [con, Xf.A*z(:,N) <= Xf.b];
            con = con + ( u0 == K*(x(:,1) - z(:,1)) + u(1));
            
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            % Return YALMIP optimizer object
            ctrl_opti = optimizer(con, obj, sdpsettings('solver','gurobi'), ...
                {x0, V_ref, u_ref, d_est, x0other}, {u0});
        end
        
        % Computes the steady state target which is passed to the
        % controller
        function [Vs_ref, us_ref] = compute_steady_state_target(mpc, ref, d_est)

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % INPUTS
            %   ref    - reference to track
            %   d_est  - disturbance estimate (Ignore before Todo 4.1)
            % OUTPUTS
            %   Vs_ref, us_ref - steady-state target
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            % Steady-state subsystem
            A = mpc.A(2, 2);
            B = mpc.B(2, 1);
            

            % Subsystem linearization steady-state
            % remember x = [y, theta, v]
            xs = mpc.xs(2); % correspond a la deuxieme valeur du vecteur [0, theta_s, V_s]
            us = mpc.us;

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE

            ref_augm = [-A*xs - B*us + xs; ref];

            Matrix = [eye - A, -B; 1, zeros];
            Matrix_inv = pinv(Matrix);
            solution = Matrix_inv*ref_augm; %delta_Xs and delta_Us

            % % Extraire xs et us
            Vs_ref = solution(1); 
            us_ref = solution(2); 

            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        end
    end
end
