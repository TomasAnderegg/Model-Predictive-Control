% deliverable_4_1.m

%Louis Gilles Canoen, Matas Antanas Jones, Tomas Joaquin Garate Anderegg
%Group BM

% Nettoyage
clear;
clc;
close all;

% Charger les paramètres ou modèles nécessaires
Ts = 0.1; % Exemple : Temps d'échantillonnage
car = Car(Ts); % Instanciation de la classe Car

[xs, us] = car.steady_state(120 / 3.6);
sys = car.linearize(xs, us);
[sys_lon, sys_lat] = car.decompose(sys);

% Design MPC controller
H = 10; % Horizon length in seconds

mpc_lon = MpcControl_lon(sys_lon, Ts, H);
mpc_lat = MpcControl_lat(sys_lat, Ts, H);
mpc = car.merge_lin_controllers(mpc_lon, mpc_lat);

estimator = LonEstimator(sys_lon, Ts);
x0 = [0 0 0 80/3.6]';
ref1 = [0 80/3.6]'; 
ref2 = [3 50/3.6]';
params = {};
params.Tf = 15;
params.myCar.model = car;
params.myCar.x0 = x0;
params.myCar.est_fcn = @estimator.estimate;
params.myCar.est_dist0 = 0;
params.myCar.u = @mpc.get_u;
params.myCar.ref = car.ref_step(ref1, ref2, 2); % delay reference step by 2s; result = simulate(params);
result = simulate(params);
visualization(car, result);

