clear; close all; clc
clear pin
restoredefaultpath;
addpath(fullfile('../build'));

pin('quit')
q = [pi/3; pi/4];
dq = [0.5; 0.5];
ddq = [0.5; 0.5];
urdf = '../urdf/rrbot.urdf';

pin('load', urdf)
pin('forwardKinematics', q)
a = pin('getBodyId', 'link2')
b = pin('getJointId', 'joint2')
d = pin('getFrameId', 'ee_link')
D = pin('crba', q)
J = pin('computeFrameJacobian', q, 'ee_link')
[pose, R] = pin('data.oMf.pose', 'ee_link')
Jdot = pin('getJointJacobianTimeVariation', q, dq, 'joint2')
Y = pin('computeJointTorqueRegressor', q, dq, ddq)
PI = pin('getDynamicParameters', 1)

pin('exit')