% deliverable_6_2.m

%Louis Gilles Canoen, Matas Antanas Jones, Tomas Joaquin Garate Anderegg
%Group BM

%Nettoyage
clear;
clc;
close all;

% Charger les paramètres ou modèles nécessaires
Ts = 0.1; % Exemple : Temps d'échantillonnage
car = Car(Ts); % Instanciation de la classe Car

% Design MPC controller
H = 5; % Horizon length in seconds
mpc = NmpcControl_overtake(car, H);
x0_ego = [0 0 0 80/3.6]';
x0_other = [20 0 0 80/3.6]'; ref1 = [0 80/3.6]';
ref2 = [0 100/3.6]';
params = {};
params.Tf = 15;
params.myCar.model = car;
params.myCar.x0 = x0_ego;
params.myCar.u = @mpc.get_u;
params.myCar.ref = car.ref_step(ref1, ref2, 1);
params.otherCar.model = car; params.otherCar.x0 = x0_other; params.otherCar.u = car.u_const(80/3.6);
result = simulate(params);
visualization(car, result);