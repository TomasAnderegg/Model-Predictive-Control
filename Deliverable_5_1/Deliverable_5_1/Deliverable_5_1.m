% deliverable_5_1.m

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

%partie 5.1
ref = [0 120/3.6]';
otherRef = 100 / 3.6;

% Case 1:
params = {};
params.Tf = 25;
params.myCar.model = car;
params.myCar.x0 = [0 0 0 100/3.6]';
params.myCar.u = @mpc.get_u;
params.myCar.ref = ref;
params.otherCar.model = car;
params.otherCar.x0 = [15 0 0 otherRef]';
params.otherCar.u = car.u_const(otherRef);
result = simulate(params);
visualization(car, result);

% % Case 2:
% params = {};
% params.Tf = 25;
% params.myCar.model = car;
% params.myCar.x0 = [0 0 0 115/3.6]';
% params.myCar.u = @mpc.get_u;
% params.myCar.ref = ref;
% params.otherCar.model = car;
% params.otherCar.x0 = [8 0 0 120/3.6]';
% params.otherCar.u = car.u_fwd_ref();
% params.otherCar.ref = car.ref_robust();
% result = simulate(params); 
% visualization(car, result);




