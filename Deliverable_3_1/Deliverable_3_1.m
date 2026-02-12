% deliverable_3_1.m

%Louis Gilles Canoen, Matas Antanas Jones, Tomas Joaquin Garate Anderegg
%Group BM

clear;
clc;
close all;

% Charger les paramètres ou modèles nécessaires
Ts = 0.1; 
car = Car(Ts); 

[xs, us] = car.steady_state(120 / 3.6);
sys = car.linearize(xs, us);
[sys_lon, sys_lat] = car.decompose(sys);

H = 10; % Horizon length in seconds

mpc_lon = MpcControl_lon(sys_lon, Ts, H);
mpc_lat = MpcControl_lat(sys_lat, Ts, H);

mpc = car.merge_lin_controllers(mpc_lon, mpc_lat);

x0 = [0 0 0 80/3.6]'; % (x, y, theta, V)
ref1 = [0 80/3.6]'; % (y ref, V ref)
ref2 = [3 120/3.6]'; % (y ref, V ref)
params = {};
params.Tf = 15;
params.myCar.model = car;
params.myCar.x0 = x0;
params.myCar.u = @mpc.get_u;
params.myCar.ref = car.ref_step(ref1, ref2, 5); % delay reference step by 5s
result = simulate(params);
visualization(car, result);
