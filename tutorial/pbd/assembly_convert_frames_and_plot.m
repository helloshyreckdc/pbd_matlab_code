clear all
clc
close('tool_frame')
close('compliance_frame')

% load demo_assembly20211201.mat
load demo_assembly202112012125.mat

tool_force = demo_tool_force_seq;
pose_seq = demo_pose_seq;
length = size(demo_pose_seq,3)



% % load exe_assembly202112301444.mat
% % load exe_assembly202112301453linear_offset.mat
% load exe_assembly202112301503angular_offset.mat
% tool_force = exe_tool_force_seq;
% pose_seq = exe_pose_seq;
% % delete zero matrix
% length = 0;
% for i=1:size(pose_seq, 3)
%     if(sum(sum(pose_seq(:,:,i)))~=0)
%         length = length + 1;
%     end
% end
% pose_seq = pose_seq(:,:,1:length);



compliance_force = zeros(6,length);

base_T_init_tool = pose_seq(:,:,1);
% init_tool_T_compliance = [rotz(-pi/6)*eye(3) [0 0 0.4]';0 0 0 1];  % choose main x axis
init_tool_T_compliance = [eye(3) [0 0 0.4]';0 0 0 1];  
base_T_compliance = base_T_init_tool*init_tool_T_compliance;
Lg_seq = zeros(1,length);

for i=1:length
    base_T_tool = pose_seq(:,:,i);
    tool_T_compliance = inv(base_T_tool)*base_T_compliance;
    compliance_force(:,i) = Adjoint(tool_T_compliance)'*tool_force(:,i);
    angle(i) = atan(compliance_force(5,i)/compliance_force(4,i));
    Lg_seq(i) = (base_T_init_tool(3,4) - base_T_tool(3,4))*1000;
end

% % for execution
% start_t = 170;
% end_t = 650;

% for demo
start_t = 170;
end_t = 300;

if(end_t > length)
    end_t = length
end
time_range = start_t:end_t;
plot_row = 2;
plot_col = 3;

fg1 = figure('numbertitle','off','name','tool_frame'); 
for i = 1:plot_row*plot_col
subplot(plot_row,plot_col,i);
if i <= 3
    y = tool_force(i,start_t:end_t)*1000;
else
    y = tool_force(i,start_t:end_t);
end
plot(time_range,y);
xlim([start_t end_t+20])
xlabel('Time\_seq')
switch i
    case 1
        ylabel('T_x (N.mm)')
    case 2
        ylabel('T_y (N.mm)')
    case 3
        ylabel('T_z (N.mm)')
%          ylabel('F_x (N)')
    case 4
        ylabel('F_x (N)')
%         ylabel('F_y (N)')
    case 5
        ylabel('F_y (N)')
    case 6
        ylabel('F_z (N)')
    otherwise
        disp('error loop number')
end
end

fg2 = figure('numbertitle','off','name','compliance_frame'); 
for i = 1:plot_row*plot_col
subplot(plot_row,plot_col,i);
if i <= 3
    y = compliance_force(i,start_t:end_t)*1000;
else
    y = compliance_force(i,start_t:end_t);
end
plot(time_range,y);
xlim([start_t end_t+20])
xlabel('Time\_seq')
switch i
    case 1
        ylabel('T_x (N.mm)')
    case 2
        ylabel('T_y (N.mm)')
    case 3
        ylabel('T_z (N.mm)')
%          ylabel('F_x (N)')
    case 4
        ylabel('F_x (N)')
%         ylabel('F_y (N)')
    case 5
        ylabel('F_y (N)')
    case 6
        ylabel('F_z (N)')
    otherwise
        disp('error loop number')
end
end

sorted_compliance_force = abs(compliance_force(4,start_t:end_t));
sorted_compliance_force = sort(sorted_compliance_force);
% figure
% plot(1:size(sorted_compliance_force,2),sorted_compliance_force)
round(0.9*(end_t-start_t))
Fx10per = sorted_compliance_force(round(0.9*(end_t-start_t)))


% compute kx, ktheta
R = 16.008;
r = 15.990;
D = 2*R;
d = 2*r;
l = 30;
% Lg = l;
c = (R-r)/R;
theta0 = (D-d)/sqrt(D^2-d^2)/2;
epsilon0 = 0.5/2 + c*R


start_index = 150;
end_index = 260;
Fx = compliance_force(4,start_index:end_index)';
Lg = Lg_seq(start_index:end_index)';
B = Fx;
A = (Lg.*(theta0-c*D/l)+epsilon0);
Kx = inv(A'*A)*A'*B

Ty = compliance_force(2,start_index:end_index)'*1000;
C = Ty - Kx*Lg.*(Lg*theta0-c*D+epsilon0);
D = theta0-c*D./Lg;
Ktheta = -inv(D'*D)*D'*C


