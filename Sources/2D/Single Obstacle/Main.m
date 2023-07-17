%*************************************************************************
%
% Filename:				Main.m
%
% Authors:				Johannes Autenrieb, johannes.autenrieb@outlook.com
% Created:				27-Nov-2022
%
%*************************************************************************
%
% Description:
%		Simple first order control exampl for a study on the application of 
%       CBF in the control of linear systems based on the paper of 
%
% Input parameter:
%		- none
%		- none
%
% Output parameter:
%		- none
%		- none
%
%% #######################    SCRIPT START   ############################

%% Pre-steps
clear workspace
clear all; close all;
addpath(genpath('./../../include'))

UNIT_RAD2DEG = 180/pi;
UNIT_DEG2RAD = 1 / UNIT_RAD2DEG ; 

%% init all relevant simulation settings and simulation cases
sim.dt = 0.01;
sim.sim_t = 10;


%% init all relevant parameter of the regarded case and model
params.A = [0,0,1,0;
             0,0,0,1;
             0,0,0,0;
             0,0,0,0];
         
params.B = [0,0;
             0,0;
             1,0;
             0,1];
         
params.k_fb = [-3,0,-3,0; 
               0,-3,0,-3];
           
params.x_d  = [0; 
               0;
               0;
               0] ;
           
params.k_r = [1, 0; 
              0, 1];

params.gamma = 5;
params.gamma2 = 100;
            
params.orderOfAlpha =1;

params.u_0 = [0.8;0.8];

params.x_obs = [-2; -2.25];

params.r_obs = 1.5;

th = [0:1:360]*pi/180;

params.x_circ = params.r_obs * cos(th) + params.x_obs(1);
params.y_circ = params.r_obs * sin(th) + params.x_obs(2);

%% init state of system
init.xp_0 = [-5;-5;0;0];

%% initialize and pre-allocate run time variables.
total_k = ceil(sim.sim_t / sim.dt);
x0 = [init.xp_0];
xs(1, :)=x0;
x =x0;
t = 0; 
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
% Simulation with Saftey filter constrained control input (QP)
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

for k = 1:total_k
    
    % Computing needed control input to get desired cruise behaviour
    u_des = params.k_fb*(x-params.x_d);
    
    % saftey filter on computed control input
    [u,h] = SafetyFilter(x,u_des, params);
    
    % Run one time step propagation.
    [ts_temp, xs_temp] = ode45(@(t, s) Dynamics(x,u,params), [t t+sim.dt], x);
 
    % saving the state of current time step
    x = xs_temp(end, :)';
    xs(k+1, :) = x';
    ts(k+1) = ts_temp(end);
    
    % Saving control relevant information of current time step
    us(k, :) = u';
    hs(k, :) = h';
    
    % Setting time for next time step
    t = t + sim.dt;
    
end

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%
%% Plot results
%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

state_p = xs; 
innput = us;


figure('Color', 'w','units','normalized','outerposition',[0 0 1 1]);
p = plot(state_p(:,1), state_p(:,2),'b', 'LineWidth',1.5);
hold on;
p2 = plot(params.x_circ,params.y_circ,'r', 'LineWidth',1.5);
legend([p p2], 'System w/ CBF-QP', 'Obstacle', 'Location', 'Southeast');
set(gca,'FontSize',14);
ylabel("y - Direction");
xlabel("x - Direction");
hold off;

set(gcf,'Units','inches');
screenposition = get(gcf,'Position');
set(gcf,...
'PaperPosition',[0 0 screenposition(3:4)],...
'PaperSize',[screenposition(3:4)]); 

% export_fig('results_1', '-png')

figure('Color', 'w','units','normalized','outerposition',[0 0 1 1]);
subplot(3,2,1);
p = plot(ts, state_p(:,1),'b', 'LineWidth',1.5);
hold on;
p2 = plot(ts, params.x_d(1)*ones(size(ts,2,1)),'--k', 'LineWidth',1.5);
set(gca,'FontSize',14);
legend([p p2], 'System w/ CBF-QP','Desired state x', 'Location', 'Southeast');
ylabel("State x");
xlabel("Time t");
hold off;

subplot(3,2,2);
p = plot(ts, state_p(:,2),'b', 'LineWidth',1.5);
hold on;
p2 = plot(ts, params.x_d(2)*ones(size(ts,2,1)),'--k', 'LineWidth',1.5);
set(gca,'FontSize',14);
legend([p p2], 'System w/ CBF-QP','Desired state y', 'Location', 'Southeast');
ylabel("State y");
xlabel("Time t");
hold off;

subplot(3,2,3);
p = plot(ts(1:end-1), us(:,1),'b', 'LineWidth',1.5);
hold on;
set(gca,'FontSize',14);
ylabel("Control input u_1");
xlabel("Time t");
hold off;

subplot(3,2,4);
p = plot(ts(1:end-1), us(:,2),'b', 'LineWidth',1.5);
hold on;
set(gca,'FontSize',14);
ylabel("Control input u_2");
xlabel("Time t");
hold off;

subplot(3,2,[5 6]);
p = plot(ts(1:end-1), hs,'b', 'LineWidth',1.5);
hold on;
set(gca,'FontSize',14);
ylabel("Barrier value h");
xlabel("Time t");
hold off;


set(gcf,'Units','inches');
screenposition = get(gcf,'Position');
set(gcf,...
'PaperPosition',[0 0 screenposition(3:4)],...
'PaperSize',[screenposition(3:4)]); 

% export_fig('results_2', '-png')

   

% #######################     SCRIPT END    ############################