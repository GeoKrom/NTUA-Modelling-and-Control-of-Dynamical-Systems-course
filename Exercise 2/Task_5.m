%% Name                   -  A.M.
%% Eirini Maria Georganta - 02121201
%% Georgios Kassavetakis - 02121203
%% Georgios Krommydas - 02121208
%% Frantzieska Michail - 02121216

%% Task 5 of Series 2

clc
clear
close all

%% System Parameters

%Starting system
A=[-1.460,0, 2.4276;
    0.0575, -0.4, -0.1326;
    0.3107, 0, -2.23];
B=[0.4182, 5.2026;
    0.1365, -0.0436;
    0.5186, 0.0236];
C=[1, 0, 0;
    0, 1, 0];
D=[0, 0;
    0, 0];

%System Creation
sys_original = ss(A,B,C,D);

%Number of States,Inputs,Outputs
n = size(A,2);
m = size(B,2);
r = size(C,1);

%Augmented system
A_aug = [zeros(r,r), C;
         zeros(n,r), A;];
B_aug = [zeros(r,m);
         B];
C_aug = [eye(r),zeros(r,n)];

%% LQR Creation 

%Using the Q creation Algorithm
M_L = -inv(C*inv(A)*B);
M_H = inv(B'*B)*B';
Q = [M_L, M_H]'*[M_L, M_H];                      % Compute Q
p = 0.102^2
R = p*eye(m)                                % Compute R

Kr = lqr(A_aug,B_aug,Q,R);            % LQR Controller
%Ku = [Kr(:,1:2)]                            % K = [Ku Ky]
%Ky = [-Kr(:,3:5)]

s=tf('s');
L_LQ=Kr*inv(s*eye(n+r)-A_aug)*B_aug;        % Open Loop Transfer Function

%% Singular Value analysis

w = logspace(-3,3,200);
%[s,w] = sigma(sys_original,omega);
figure(1)
clf
sigma(L_LQ,sys_original,w)
hold on
xline(1,'r--',{'s_{min}','Lower Limit 20dB'});
yline(20,'r--');
xline(100,'r--',{'s_{max}','Upper Limit -20dB'});
yline(-20,'r--');
xline(10,'r--',{'s_{min} & s_{max}','0dB'});
yline(0,'r--');
grid;
points=[1,20;100,-20;10,0];
scatter(points(:,1),points(:,2),'rx')
legend('LQR system','Original system','location','southwest')
%% Simulink Simulation

%Using simulink to simulate the system
r = [1,0;0,1;1,1;1,-1];
y1 = cell(4,1);
y2 = cell(4,1);
time = cell(4,1);
open('LQR_integration.mdl')
for i = 1:4
    r1 = r(i,1);
    r2 = r(i,2);
    sim('LQR_integration.mdl')
    y1(i) = {LQR_reg(:,1)};
    y2(i) = {LQR_reg(:,2)};
    time(i) = {tout};
end
close_system('LQR_integration.mdl')
clc

%% Simulation Results

%Plot of systems output for each compined input r1 and r2
for i = 1:4
    figure(1+i)
    clf
    str1=['Output y_1 for r_1=',num2str(r(i,1)),' and r_2=',num2str(r(i,2))];
    str2=['Output y_2 for r_1=',num2str(r(i,1)),' and r_2=',num2str(r(i,2))];
    subplot(2,1,1)
    plot(time{i},y1{i})
    hold on
    plot(time{i},r(i,1)*ones(size(time{i})),'r--')
    grid minor
    title(str1)
    ylabel('y_1')
    xlabel('t[s]')
    legend('Output','Reference','location','east')
    subplot(2,1,2)
    plot(time{i},y2{i})
    hold on
    plot(time{i},r(i,2)*ones(size(time{i})),'r--')
    grid minor
    title(str2)
    ylabel('y_2')
    xlabel('t[s]')
    legend('Output','Reference','location','east')
end
