%% Prootyp Matlab 
clc
clear
close all
%% Ex 3 Simulation


s = tf('s');
t_start = 0;
t_step = 0.001;
t_final = 10;

% Transfer Function Init
time_delay = 0.9672;
% First Way
G1 = exp(-time_delay*s)/(10*s+1);
% Second Way
G = 1/(10*s+1);
G.InputDelay = time_delay;

% Input
slope = 1;
u = slope*s^(-2); %Input in Laplace

% Simulation
tspan = t_start:t_step:t_final;

% [y,t] = step(G*u*s, tspan);
% OR
[y,t] = impulse(G*u, tspan);

% [u,~] = step(u*s, tspan);
% OR 
[u,~] = impulse(u, tspan);

% Alternative Function
% u = slope*tspan % Input in time
% [y,t] = lsim(G,u,tspan)

figure(1)
plot(t, u)
hold on 
plot(t, y)
title('Ex 3:Simulation Result for Ramp Response')
xlabel('Time[s]')
ylabel('Amplitude')
grid minor
legend('Input','Output')
%% Ex 4 Simulation

s = tf('s');
t_start = 0;
t_step = 0.001;
t_final = 1;

% Transfer Function Init
T1 = 2;
T2 = 10;
K_G = 1;
tc = [0.1, 0.5, 1, 10, 20, 100];
y = cell(1,length(tc));
t = cell(1,length(tc));
for i = 1:length(tc)
    % First Way
    Num = K_G;
    Den = [T1*T2,T1+T2,1];
    G = tf(Num,Den);
    % G = K/(((T1*s+1)*(T2*s+1));
    Kp = (T1+T2)/(K_G*tc(i));
    Ti = T1+T2;
    Td = T1*T2/(T1+T2);
    K = Kp*(1+1/(Ti*s)+Td*s);
    L = K*G;
    G_cl = feedback(L,1); % G_cl = L/(1+L);
    [out, time] = step(G_cl);
    y{i} = out;
    t{i} = time;
end

figure(2)
plot(t{end}, ones(size(t{end})))
Names = cell(1,length(tc)+1);
Names(1) = {'Input'};

for i = 1:length(tc)
    hold on 
    plot(t{i}, y{i})
    Names(i+1) = {['Output with tc=', num2str(tc(i))]};
end

title('Ex 4: Simulation Result for Step Response')
xlabel('Time[s]')
ylabel('Amplitude')
grid minor
xlim([0, 120])
ylim([0, 1.1])
legend(Names, 'Location', 'southeast')