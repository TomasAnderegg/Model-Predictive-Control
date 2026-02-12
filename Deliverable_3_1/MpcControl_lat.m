classdef MpcControl_lat < MpcControlBase
    
    methods
        % Design a YALMIP optimizer object that takes a steady-state state
        % and input (xs, us) and returns a control input
        function ctrl_opti = setup_controller(mpc)
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % INPUTS
            %   x0           - initial state (estimate)
            %   x_ref, u_ref - reference state/input
            % OUTPUTS
            %   u0           - input to apply to the system
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            N_segs = ceil(mpc.H/mpc.Ts); % Horizon steps
            N = N_segs + 1;              % Last index in 1-based Matlab indexing

            [nx, nu] = size(mpc.B);

            % % Define optimization variables
            x = sdpvar(2,N,'full');
            u = sdpvar(1,N-1,'full');

            % Targets
            x_ref = sdpvar(nx, 1); 
            u_ref = sdpvar(nu, 1);

            % Initial states
            x0 = sdpvar(nx, 1); 
            x0other = sdpvar(nx, 1); % (Ignore this, not used)

            % Input to apply to the system
            u0 = sdpvar(nu, 1);
            
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            
            % NOTE: The matrices mpc.A, mpc.B, mpc.C and mpc.D
            %       are the DISCRETE-TIME MODEL of your system
            %       You can find the linearization steady-state
            %       in mpc.xs and mpc.us.
            
            % SET THE PROBLEM CONSTRAINTS con AND THE OBJECTIVE obj HERE
            con = [];
            obj = 0;

            % Matrice Input constraints: M*u <= m avec u = delta
            M = [1;-1]; 
            m = [deg2rad(30); deg2rad(30)];
            U = polytope(M,m);
            
            % Matrice state constraint F*x<=f
            F = [1, 0; -1, 0; 0, 1; 0, -1];
            f = [3.5; 0.5; deg2rad(5); deg2rad(5)];
            X = polytope(F,f);

            Q = diag([1,100]); 
            R = 1;
            [K, Qf, ~] = dlqr(mpc.A,mpc.B,Q,R);
            K = -K;
            
            % POLYTOPE
            figure(1); clf;
            h1=plot(polytope(F,f), 'b');
            hold on;
           
            Ak = mpc.A + mpc.B*K; % Closed-loop dynamics
            
            HH = [F;M*K]; hh = [f;m]; % State and input constraints
            
            h4=plot(polytope(HH,hh), 'c');
            hold on;
            
            % Compute the maximal invariant set
            i = 1;
            O = polytope(HH,hh);
            while 1
                Oprev = O;
                [H,h] = double(O);	
                % Compute the pre-set
                O = polytope([H;H*Ak],[h;h]);
                if O == Oprev, break; end
                
                h2=plot(O, 'y');
                fprintf('Iteration %i... not yet equal\n', i)
            
                i = i + 1;
            end
            fprintf('Maximal invariant set computed after %i iterations\n\n', i);
            h3=plot(O,'g');
            legend([h3;h1;h2;h4],{'Invariant set';'State constraints';'Iterations';'State and input constraints'});
            title('State and input constraints (lat)')
            [Ff, ff] = double(O);           
            
            con = x(:,1) == x0;
            con = con + ( u0 == u(1));


            for i = 1:N-1
                con = [con, x(:,i+1) == mpc.xs + mpc.A*(x(:,i)-mpc.xs)  + mpc.B*(u(i)-mpc.us)];
                con = [con, M*u(i) <= m];
                con = [con,F*x(:,i) <= f];
                obj = obj + (x(:,i)-x_ref)'*Q*(x(:,i)-x_ref) + (u(i)-u_ref)'*R*(u(i)-u_ref);
            end
            
            con = [con, Ff*x(:,N) <= ff];
            obj = obj + (x_ref-x(:,N))'*Qf*(x_ref-x(:,N));
            u(N) = u(N-1);

            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            
            % Return YALMIP optimizer object
            ctrl_opti = optimizer(con, obj, sdpsettings('solver','gurobi'), ...
                {x0, x_ref, u_ref, x0other}, {u0});
        end
        
        % Computes the steady state target which is passed to the
        % controller
        function [xs_ref, us_ref] = compute_steady_state_target(mpc, ref)

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % INPUTS
            %   ref    - reference to track
            % OUTPUTS
            %   xs_ref, us_ref - steady-state target
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

            % Steady-state system
            A = mpc.A;
            B = mpc.B;

            % Linearization steady-state
            xs = mpc.xs;
            us = mpc.us;

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            C = [1, 0];
            ref_augm = [-A*xs - B*us + xs; ref];
            
            Matrix = [eye(2) - A, -B; C, zeros];
            Matrix_inv = pinv(Matrix);
            solution = Matrix_inv*ref_augm; % (ys, theta_s, delta)

            xs_ref = solution(1:2);
            us_ref = solution(3);
          
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        end
    end
end
