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
            
            % SET THE PROBLEM CONSTRAINTS con AND THE OBJECTIVE obj HERE
            
            % Define optimization variables
            x = sdpvar(nx,N,'full');% x = [x0, x1, x2, ..., xN]
            u = sdpvar(nu,N-1,'full');% u = [u0, u1, u2, ..., uN]
            
            % Matrice Input constraints: M*u <= m avec u = u_t
            M = [1; -1]; 
            m = [1; 1];
            
            Q = eye(1)*1;
            R = 1;

            con = [];
            obj = 0;

            con = x(:,1) == x0;
            con = con + ( u0 == u(1));

            [~, Qf, ~] = dlqr(mpc.A(2,2),mpc.B(2,1),Q,R);
         
            for i = 1:N-1
                con = [con, (x(2,i+1)) == mpc.xs(2) + mpc.A(2,2)*(x(2,i)-mpc.xs(2)) + mpc.B(2,1)*(u(i)-mpc.us) + mpc.B(2,1)*d_est];
                con = [con, M*u(i) <= m];
                obj = obj + (x(2,i)-V_ref)'*Q*(x(2,i)-V_ref) + (u(i)-u_ref)*R*(u(i)-u_ref);
            end
            obj = obj + (V_ref-x(2,N))'*Qf*(V_ref-x(2,N));

            u(N) = u(N-1);

            debugVars = {x, u};
            
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

            ref_augm = [-A*xs - B*(us - d_est) + xs; ref];
            Matrix = [eye - A, -B; 1, zeros];
            Matrix_inv = pinv(Matrix);
            solution = Matrix_inv*ref_augm; %delta_Xs and delta_Us
          
            Vs_ref = solution(1);
            us_ref = solution(2);

            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        end
    end
end
