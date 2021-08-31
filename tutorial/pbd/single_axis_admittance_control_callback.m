function single_axis_admittance_control_callback(~, ~, handles)

global loop_rate_hz
global ref_pose_data
global current_pose_data
global pre_pose_data
global gravity_compensated_force
global current_vel_data
persistent force_before_contact previous_contact_force;
persistent pos_seq n;

vel_msg = handles.vel_msg;
vel_pub = handles.vel_pub;

% define m,b,k along z-axis
m = 5;
b = 200;
k = 5;



if isempty(force_before_contact)
    force_before_contact = gravity_compensated_force;
    previous_contact_force = force_before_contact;
    % force_before_contact = [0.36;0.18;3.66;0.21;-0.01;-0.02];
end

if isempty(pos_seq)
    pos_seq = zeros(100000,2);
    n = 1;
end

% gravity_compensated_force = [-0.01;0.4;-7.73;0.14;0.138;0.01];

% get value
% ref_trans = ref_pose_data(1:3);
% ref_trans = [0.109;0.487;0.4687];
ref_trans = [-0.056;0.418;0.33];
% refRotm = quat2rotm(ref_pose_data(4:7)');   % quaternion value;  w x y z
cur_trans = current_pose_data(1:3);
curRotm = quat2rotm(current_pose_data(4:7)');
pre_trans = pre_pose_data(1:3);
preRotm = quat2rotm(pre_pose_data(4:7)');

contact_force = gravity_compensated_force - force_before_contact

% design velocity along z according to the contact force
zdd = (-gravity_compensated_force(3)-b*current_vel_data(3)-k*(cur_trans(3)-ref_trans(3)))/m;
%computed vel
zd = zdd/100 + current_vel_data(3)
vel_msg.Linear.Z = zd;

% design velocity along x and y according to the contact position
% r x F = M; F x r = -M; S(F)r = -M

if abs(contact_force(3)) > 1
    
%     use constant rz
        Mx = contact_force(4);
        My = contact_force(5);
        Fx = contact_force(1);
        Fy = contact_force(2);
        Fz = contact_force(3);
        rz = 0.23; %contact point to the origin of sensor frame
    %     choose = 0; % Fx and Fy are considered noise during exploring right position
        choose = 1; % compute according formula
        rx = (choose*Fx*rz - My)/Fz
        ry = (Mx + choose*Fy*rz)/Fz
        pos_seq(n,1)=rx;
        pos_seq(n,2)=ry;
        n = n+1;
    %     save('pos_seq.mat','pos_seq');
    
        vel_msg.Linear.X = rx/8;
        vel_msg.Linear.Y = -ry/8;
    
    
%     % calculate r by two contact force
%     
%     F1 = previous_contact_force(1:3);
%     F2 = contact_force(1:3);
%     F_matrix = [skew(F1);skew(F2)];
%     M_vector = -[previous_contact_force(4:6);contact_force(4:6)];
%     
%     r = F_matrix \ M_vector;
%     rx = r(1)
%     ry = r(2)
%     rz = r(3)
%     
%     vel_msg.Linear.X = rx/5;
%     vel_msg.Linear.Y = -ry/5;
    
else
    vel_msg.Linear.X = 0;
    vel_msg.Linear.Y = 0;
end

vel_msg.Angular.X = 0;
vel_msg.Angular.Y = 0;
vel_msg.Angular.Z = 0;


% speed limit
vel_msg.Linear.X = limit_speed(vel_msg.Linear.X,0.0005,0.1);
vel_msg.Linear.Y = limit_speed(vel_msg.Linear.Y,0.0005,0.1);
vel_msg.Linear.Z = limit_speed(vel_msg.Linear.Z,0.0005,0.1);
vel_msg.Angular.X = limit_speed(vel_msg.Angular.X,0.0001,0.1);
vel_msg.Angular.Y = limit_speed(vel_msg.Angular.Y,0.0001,0.1);
vel_msg.Angular.Z = limit_speed(vel_msg.Angular.Z,0.0001,0.1);


send(vel_pub,vel_msg);

% update previous contact force
previous_contact_force = contact_force;

end