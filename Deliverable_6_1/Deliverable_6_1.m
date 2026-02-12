% deliverable_6_1.m

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
x0 = [0 0 0 80/3.6]';
ref = [3 100/3.6]';
mpc = NmpcControl(car, H);

u = mpc.get_u(x0, ref);

params = {};
params.Tf = 15;
params.myCar.model = car;
params.myCar.x0 = [0 0 0 80/3.6]';
params.myCar.u = @mpc.get_u;
ref1 = [0 80/3.6]';
ref2 = [3 100/3.6]';
params.myCar.ref = car.ref_step(ref1, ref2, 2);
result = simulate(params);
visualization(car, result);