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
pin('forwardKinematics', q);
a = pin('getBodyId', 'link2');
b = pin('getJointId', 'joint2');
d = pin('getFrameId', 'ee_link');
D = pin('crba', q);
J = pin('computeFrameJacobian', q, 'ee_link');
[pose, R] = pin('data.oMf.pose', 'ee_link');
Jdot = pin('getJointJacobianTimeVariation', q, dq, 'joint2');
Y = pin('computeJointTorqueRegressor', q, dq, ddq);
PI = pin('getDynamicParameters', 2);

[m, pc, Irot] = getParams(PI);
S = hat(pc);
Ic = Irot - m*S'*S

pin('exit')



%% Functions:
function ahat = hat(a)
ahat = [0    -a(3)    a(2);
        a(3)    0    -a(1);
       -a(2)  a(1)    0];
end
        
function [m, pc, I] = getParams(Phi)
m = Phi(1);
pc = Phi(2:4)/Phi(1);
I = [Phi(5)   Phi(6)   Phi(8);
     Phi(6)   Phi(7)   Phi(9);
     Phi(8)   Phi(9)   Phi(10)];
end