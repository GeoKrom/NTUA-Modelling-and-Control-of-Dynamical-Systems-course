%% Name                   -  A.M.
%% Eirini Maria Georganta - 02121201
%% Georgios Kassavetakis - 02121203
%% Georgios Krommydas - 02121208
%% Frantzieska Michail - 02121216
%% Task 1 of Series 2
 
clc
clear
close all

%% System Parameters

G = tf(3,[2,1]);
f = tf(1,[0.1,1])^2;
% Alternative: f = (1/(0.1*s+1)^2);
I = tf(1,[1, 0]); %Integrator
GP = G*f; %Real Plant
wi = minreal((GP-G)/G); %Uncertainty filter
% Alternative: wi = f-1;

%% Norm Calculation for a variety of K

% Defining the K to test RS
K = linspace(0,2,1000);
ninf = ones(size(K));
fpeak = ones(size(K));
for i = 1:length(K)
    % Open Loop transfer function
    K_tf = K(i)*I;
    % Open Loop transfer function
    L = G*K_tf;
    % Complementary Sensitivity transfer function
    T = L/(1+L);
    [ninf(i), fpeak(i)] = norm(minreal(wi*T), Inf);
end

%% Finding the K barrier

% ind_RS = find(ninf<1,1,'last');
ind = find(ninf>=1,1,'first');
str = ['The System is RS stable for K < ',num2str(K(ind))];
disp(str)
[GM, PM] = margin(GP*I);
str = ['The Plant(added integrator) has Gain Margin GM = ',...
    num2str(GM)];
disp(str)
% Extra Barrier:
% Asking T = GK/(1+GK) stable we get:
% T = 3K/(2s^2+s+3k)
% For this function to be stable we need k>0
str = ['Asking for stability of the T function, ',...
      'the K(s) gain is a K such that 0 < K < ',...
      num2str(K(ind))];
disp(str)

%% Figure of Norm - Κ

figure(1)
clf
plot(K,ninf,'LineWidth',1.5)
grid minor
hold on
yline(1,'r--',{'RS Threshold'},'LineWidth',1.5,...
    'LabelHorizontalAlignment','left')
xline(K(ind),'-.',{'R.S. Stability Barrier'},'LineWidth',1.5,'Color','r',...
    'LabelHorizontalAlignment','center',...
    'LabelVerticalAlignment','middle')
ylabel('$\| w_iT\| _{\infty}$','fontsize',15,'interpreter','latex')
xlabel('$K$','fontsize',15,'interpreter','latex')
title('$Robust$ $Stability$ $(RS)$','fontsize',15,'interpreter','latex')
%% Figure of Norm - w

K_tf = K(ind)*I;
% Open Loop transfer function
L_RS = G*K_tf;
% Complementary Sensitivity transfer function
T_RS = L_RS/(1+L_RS);
w = logspace(-1,2,1000);
[Amp,~,]=bode(wi*T_RS,w);
Amp = squeeze(Amp);
Amp = Amp(:,1);
figure(2)
clf
plot(w,Amp)
grid minor
hold on
ylabel('$\| w_iT\| _{\infty}$','fontsize',15,'interpreter','latex')
xlabel('$w$','fontsize',15,'interpreter','latex')
title('$Norm$ $of$ $the$ $first$ $NON$ $RS$ $System$',...
    'fontsize',15,'interpreter','latex')