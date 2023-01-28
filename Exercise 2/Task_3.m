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
    str='Output y';
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
%% System Simulation without Simulink
% %LQR gain Calculation
% p1 = 0.1;
% p2 = 1;
% p3 = 10;
% R1 = p1*eye(m);
% R2 = p2*eye(m);
% R3 = p3*eye(m);
% K(1,:) = lqr(A_aug,B_aug,Q,R1);
% K(2,:) = lqr(A_aug,B_aug,Q,R2);
% K(3,:) = lqr(A_aug,B_aug,Q,R3);
% %Alternative to Simulink 
% A_s1 = A_aug - B_aug*K(1,:);
% disp('The closed loop system after the LQR control (1) is has eigenvalues:')
% disp(eig(A_s1))
% 
% A_s2 = A_aug - B_aug*K(2,:);
% disp('The closed loop system after the LQR control (2) is has eigenvalues:')
% disp(eig(A_s2))
% 
% A_s3 = A_aug - B_aug*K(3,:);
% disp('The closed loop system after the LQR control (3) is has eigenvalues:')
% disp(eig(A_s3))
% 
% B_s = zeros(r+n,m);
% C_s = [1,0,0];
% sys1 = ss(A_s1,B_s,C_s,0);
% sys2 = ss(A_s2,B_s,C_s,0);
% sys3 = ss(A_s3,B_s,C_s,0);
% %Simulation
% ref = 1;
% tspan = 0:0.01:10;
% x0 = [ref,0,0];
% u= 0*tspan;
% [e1,~] = lsim(sys1,u,tspan,x0);
% [e2,~] = lsim(sys2,u,tspan,x0);
% [e3,~] = lsim(sys3,u,tspan,x0);
% 
% %Plot
% figure
% clf
% plot(tspan,ref*ones(size(e1))-e1)
% hold on
% plot(tspan,ref*ones(size(e2))-e2)
% plot(tspan,ref*ones(size(e3))-e3)
% plot(tspan,ref*ones(size(tspan)),'r--')
% grid minor
% title('Output step reference tracking')
% xlabel('Time[s]')
% ylabel('Amplitude')
% legend('y1','y2','y3','reference', 'Location','southeast');
