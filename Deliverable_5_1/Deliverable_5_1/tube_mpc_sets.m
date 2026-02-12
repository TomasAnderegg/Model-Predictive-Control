% Tube MPC Offline Calculation
close all 
clear all

Ts = 1/10;
car = Car(Ts);
Vs = 120/3.6; % 120 km/h
[xs, us] = car.steady_state(Vs); % Compute steady−state for which f s(xs,us) = 0
sys = car.linearize(xs, us); % Linearize the nonlinear model around xs, us
[sys_lon, sys_lat] = car.decompose(sys);
[f_xs_us, Ad, Bd, Cd, Dd] = Car.c2d_with_offset(sys_lon, Ts);

% Tuning parameters
Q = eye(2);
R = 1;
x_safe = 10;
u_ts = us(2);

[K, Qf, ~] = dlqr(Ad,-Bd,Q,R);
K = -K;

Acl = Ad - Bd*K;

% Compute maximal invariant set
M = [1; -1]; m = [u_ts + 0.5; -u_ts + 0.5];

% Calculate the epsilon minimal invariant set
W = Polyhedron(M, m);
W_d = Bd*W;

E_prev = Polyhedron.emptySet(2);

i = 0;
while 1

     E = E_prev + (Acl^i)*W_d;
     E = minHRep(E);
     if norm(Acl^i) < 1e-2, break; end
        
     E_prev = E;
     i = i+1;
 
end

figure;
hold on;
h = E.plot('color', 'green', 'alpha', 1);
legend(h, '${\mathcal{E}}$', 'Interpreter', 'latex', 'Location', 'best');

% Contrainte X_tilde
F = [-1 0]; f = x_safe - 6;
X_full = Polyhedron(F,f);  % Position relative
X_tilde = X_full - E;

figure;
hold on;
p1 = X_full.plot('color', 'red', 'alpha', 1);
p2 = X_tilde.plot('color', 'blue', 'alpha', 1);

% Contrainte U_tilde
U = [1; -1]; u = [1; 1];
U_full = Polyhedron(U,u);
U_tilde = U_full - K*E;

figure;
hold on;
h1 = U_full.plot('color', 'red', 'alpha', 1);
h2 = U_tilde.plot('color', 'blue', 'alpha', 1);
legend([h1 h2], '$\mathcal{U}$', '$\tilde{\mathcal{U}}$', 'Interpreter', 'latex', 'Location', 'best');


% Maximal invariant Set
HH = [X_tilde.A;U_tilde.A*K]; hh = [X_tilde.b;U_tilde.b];
Xf = Polyhedron(HH,hh);
while 1

    Xf_prev = Xf;
    Xf = Polyhedron([Xf.A;Xf.A*Acl],[Xf.b;Xf.b]);
    if Xf == Xf_prev, break; end

end

figure;
hold on;
h = Xf.plot('color', 'orange', 'alpha', 1);
legend(h, '${X_f}$', 'Interpreter', 'latex', 'Location', 'best');


% Terminal weight and constraints
P = Qf;

save('tube mpc data.mat', 'X_tilde', 'U_tilde', 'Xf', 'x_safe', 'P','E', 'x_safe', 'Q', 'R', 'K');