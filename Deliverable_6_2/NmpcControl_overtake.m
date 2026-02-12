classdef NmpcControl_overtake < handle

    properties
        % The NMPC problem
        opti

        % Problem parameters
        x0, ref, x0other

        % Most recent problem solution
        sol

        % The input that you want to apply to the system
        u0

        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        % Add any variables you would like to read to debug here
        % and then store them in the NmpcControl function below.
        % e.g., you could place X here and then add obj.X = X
        % in the NmpcControl function below.
        % 
        % After solving the problem, you can then read these variables 
        % to debug via
        %   nmpc.sol.value(nmpc.X)
        % 
        X, U
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
    end

    methods
        function obj = NmpcControl_overtake(car, H)

            import casadi.*

            N_segs = ceil(H/car.Ts); % Horizon steps
            N = N_segs + 1;          % Last index in 1-based Matlab indexing

            nx = 4;
            nu = 2;

            % Define the NMPC optimization problem
            opti = casadi.Opti();
            
            % Parameters (symbolic)
            obj.x0 = opti.parameter(nx, 1);       % initial state
            obj.ref = opti.parameter(2, 1);       % target y, velocity
            obj.x0other = opti.parameter(nx, 1);  % initial state of other car

            % SET THIS VALUE TO BE YOUR CONTROL INPUT
            obj.u0 = opti.variable(nu, 1);

            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE

            % Set variables
            
            X = opti.variable(nx,N); 
            U = opti.variable(nu,N-1);

            obj.X = X;
            obj.U = U;

            cost = 0;

            f_car_discrete_RK4 = @(x,u) RK4_me(x,u,car.Ts,@car.f);  % to discretize the system at each state

            Q = diag([0,1,100,10]);
            R = diag([1,1]);

            X_ref = [0; obj.ref(1); 0; obj.ref(2)]; % X_ref = [0, Y_ref, 0, V_ref]

            % 1) Create Matrix H and lead car variables
            x_safe = 12;
            y_safe = 2.1;
            a = car.length/2 + x_safe;
            b = car.width/2 + y_safe;
            H = [1/a^2, 0; 0, 1/b^2];

            X_other = obj.x0other(1);
            V_other = obj.x0other(4); % Lead car is going at constant velocity
            
            for k = 1:N-1
                opti.subject_to(X(:,k+1) == f_car_discrete_RK4(X(:,k), U(:,k)));
                opti.subject_to(-deg2rad(30) <= U(1,k) <= deg2rad(30));  % delta is limited
                opti.subject_to(-1 <= U(2,k) <= 1);  % u_T is limited
                opti.subject_to(-0.5 <= X(2,k) <= 3.5);  % y limited
                opti.subject_to(-deg2rad(5) <= X(3,k) <= deg2rad(5));  % theta is limited

                % 2) Elipsoidal constraint
                opti.subject_to(1 <= [X(1,k) - X_other; X(2,k)]'*H*[X(1,k) - X_other; X(2,k)]);

                % 3) Update x_other
                X_other = X_other + V_other*car.Ts;  

                cost = cost + (X(:,k) - X_ref)'* Q * (X(:,k) - X_ref) + U(:,k)' * R * U(:,k);
            end

            % ---- boundary conditions --------
            opti.subject_to(X(:,1) == obj.x0);   % use initial state

            % change this line accordingly
            opti.subject_to(obj.u0 == U(:,1));
            
            opti.minimize(cost);

            % YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE YOUR CODE HERE
            %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

            % Store the defined problem to solve in get_u
            obj.opti = opti;

            % Setup solver
            options = struct;
            options.ipopt.print_level = 0;
            options.print_time = 0;
            options.expand = true;
            obj.opti.solver('ipopt', options);
        end

        function u = get_u(obj, x0, ref, x0other)

            if nargin < 4
                x0other = zeros(4, 1);
            end

            % Compute solution from x0
            obj.solve(x0(1:4), ref, x0other(1:4));

            u = obj.sol.value(obj.u0);
        end

        function solve(obj, x0, ref, x0other)

            % Pass parameter values
            obj.opti.set_value(obj.x0, x0);
            obj.opti.set_value(obj.ref, ref);
            obj.opti.set_value(obj.x0other, x0other);

            obj.sol = obj.opti.solve();   % actual solve
            
            % Set warm start for next solve
            obj.opti.set_initial(obj.sol.value_variables());
            obj.opti.set_initial(obj.opti.lam_g, obj.sol.value(obj.opti.lam_g));
        end
    end
end