clear; close all; clc
clear pin
restoredefaultpath;
addpath(fullfile(pwd, 'build'));

pin('exit')
urdf = strcat(pwd, '/urdf/rrbot.urdf');
n = 2;
q = 0.5*ones(n, 1);
v = 0.5*ones(n, 1);
a = 0.5*ones(n, 1);

pin('load', urdf)
D = pin('crba', q)
Y = pin('computeJointTorqueRegressor', q, v, a)

pin('exit')
