function [calibrated_output,M,G,e_alpha,e_beta]=estimate_M_G(O,R)
%下面的算法 基于六维力传感器的工业机器人末端负载受力感知研究
%世界坐标系到基坐标系的旋转矩阵，先绕世界坐标系x轴转alpha度,然后绕世界坐标系y轴转beta度，再绕，世界坐标系z轴转gamma度
%output R is the Output-bias,several columns; M is [I;skew(r)]; G is a 3*1 vector which equals m*g 
%e_alpha is estimated alpha
%input O is output of sensor 6 by N and R is rotation matrix from base to
%ati_frame 3 by 3*N

g0 = [0; 0; -9.7940]; % acceleration of gravity

O_F = O(1:3,:);
O_M = O(4:6,:);
vector_F = reshape(O_F,[],1);
vector_M = reshape(O_M,[],1);
[rows, columns] = size(O);
matrix_F_left_half = zeros(3*columns,3);
for i=1:columns
    matrix_F_left_half((3*i-2):(3*i),:) = -skew(O_F(:,i));
end
matrix_F = [matrix_F_left_half kron(ones(columns,1),eye(3))];
p = (matrix_F'*matrix_F)\matrix_F'*vector_M;
e_r = p(1:3) %estimated r
k = p(4:6); % k = M0 - cross(r,F0)

matrix_R = [R' kron(ones(columns,1),eye(3))];
l = (matrix_R'*matrix_R)\matrix_R'*vector_F;
F0 = l(4:6);
M0 = k + cross(e_r,F0);
e_S0 = [F0;M0];  %estimated sensor reading bias
L = l(1:3);
e_G = sqrt(L'*L);
e_m = e_G/abs(sum(g0)) %estimated mass
e_beta = asin(L(1)/e_G)*180/pi; %convert to degree
e_alpha = atan(L(2)/L(3))*180/pi; %convert to degree
% calibrated_output = [O1 O2 O3 O4] - repmat(e_S0,1,4);
M = [eye(3);skew(e_r)];
e_Rw2b = rotz(0)*roty(e_beta)*rotx(e_alpha);  
e_g = [e_Rw2b*t2r(T1) e_Rw2b*t2r(T2) e_Rw2b*t2r(T3) e_Rw2b*t2r(T4)]'*g0;
G = reshape(e_g,3,4)*e_m;

