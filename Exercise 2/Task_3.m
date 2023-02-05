%% Name                   -  A.M.
%% Eirini Maria Georganta - 02121201
%% Georgios Kassavetakis - 02121203
%% Georgios Krommydas - 02121208
%% Frantzieska Michail - 02121216
%% Task 3 of Series 2

clc
clear
close all

%% System and LQR Control

%Starting System
A = [1 3; 
     4 8];
B = [1; 
     0.5];
C = [1, 0];
D = 0;

%Number of States,Inputs,Outputs
n = size(A,2);
m = size(B,2);
r = size(C,1);

%Augmented system
A_aug = [zeros(r,r), -C;
         zeros(n,r), A;];
B_aug = [zeros(r,m);
         B];
C_aug = [eye(r), zeros(r,n)];

%LQR Creation
Q = eye(r+n);
p = [0.1, 1, 10];
% R = p*eye(m)

%% Simulink Simulation
r = 1.0;
y = cell(3,1);
time = cell(3,1);
open('Task3.mdl')
for i = 1:3
    R = p(i)*eye(m);
    K = lqr(A_aug,B_aug,Q,R);
    out = sim('Task3.mdl');
    y(i) = {out.y_out};
    time(i) = {out.tout};
end
close_system('Task3.mdl')

% plot
figure(1)
clf
for i=1:3
    str = 'Output y';
    plot(time{i},y{i})
    hold on
end
plot(time{1},r*ones(size(time{1})))
grid minor
title(str)
ylabel('y')
xlabel('t[s]')
legend('Output for p1','Output for p2','Output for p3','Reference',...
    'location','southeast')
