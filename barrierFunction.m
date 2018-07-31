clc; clear; close all; pause on;

% Loading drone flight and reference trajectory variables
load stateHistory.mat
load refTrajectory.mat

% Defining constraint barrier function
mu = 7;
x  = stateHistory(:,1)';
y  = stateHistory(:,2)';
z  = stateHistory(:,3)';
vx = stateHistory(:,7)';
vy = stateHistory(:,8)';
vz = stateHistory(:,9)';

x_term = (1/0.16)*x.^2;
y_term = (1/0.16)*y.^2;
z_term = (1/0.36)*(z+0.8).^2;

vx_term = (1/0.25)*vx.^2;
vy_term = (1/0.25)*vy.^2;
vz_term = mu*vz.^2;

h_mu = 1 - x_term - y_term - z_term - vx_term - vy_term - vz_term;

