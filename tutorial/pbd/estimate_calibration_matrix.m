function [e_C,test_error] = estimate_calibration_matrix(wrench_matrix,rotation_matrix)
% wrench_matrix is [bl_force ba_force br_force] which is 6 by N*3, rotation
% matrix is 3*3*N  where N is the number of pose

% remove empty columns
wrench_matrix(:,all(wrench_matrix==0,1))=[];
rotation_matrix(:,all(rotation_matrix==0,1))=[];
%below left
bl_vec=[68;75;21.1]/1000;
%below ahead
ba_vec=[0;143;21.1]/1000;
%below right
br_vec=[-68;75;21.1]/1000;
Ml = [eye(3);skew(bl_vec)];
Ma = [eye(3);skew(ba_vec)];
Mr = [eye(3);skew(br_vec)];
m = 1.501;
g0 = [0; 0; -9.7940];
G = reshape(rotation_matrix'*g0,3,[]);   % 3 by N, every column is g0 expressed in sensor frame
right_equation = m*[Ml*G Ma*G Mr*G];
vec_right_equ = reshape(right_equation,[],1);    %left_equ*vec(C) = vec_right_equ
left_equ = kron(wrench_matrix',eye(6));
vec_C = ((left_equ'*left_equ)\left_equ')*vec_right_equ;
e_C = reshape(vec_C,6,6);
test_error = right_equation - e_C*wrench_matrix;
end