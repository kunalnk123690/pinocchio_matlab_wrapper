clear; close all; clc
clear pin
restoredefaultpath;
addpath(fullfile('../build'));


pin('exit')
urdf = '../urdf/rrbot.urdf';
pin('load', urdf)

pin('getFrameId', 'ee_link')

q = [0.5; 0.5];
v = [0.5; 0.5];
pin('computeFrameJacobian', q, 'ee_link')
pin('getJointJacobianTimeVariation', q, v, 'joint2')

pin('forwardKinematics', q)
[p, R] = pin('data.oMf.pose', 'ee_link')