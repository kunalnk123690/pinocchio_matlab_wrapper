clear; close all; clc
clear pin
restoredefaultpath;
addpath(fullfile('../build'));
load('Data.mat')

pin('exit')
urdf = '../urdf/rrbot.urdf';

pin('load', urdf)

PI_actual = [pin('getDynamicParameters', 1); pin('getDynamicParameters', 2)];

Y = [];
tau = [];
for i = 1:size(Data.q,2)
    q = Data.q(:,i);
    v = Data.v(:,i);
    a = Data.a(:,i);
    tau = [tau; Data.u(:,i)];
    Y = [Y; pin('computeJointTorqueRegressor', q, v, a)];
end


PI = Y\tau;

PI_actual'
PI'

pin('exit')