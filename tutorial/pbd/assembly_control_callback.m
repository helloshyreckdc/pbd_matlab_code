function assembly_control_callback(~, ~, handles)

global ati_pose_data
global gravity_compensated_force
global current_vel_data

global force_before_contact;
global demo_force_seq;
global force_seq;
global pos_seq exe_count vel_seq energy_seq;
global demo_ati_pose_seq demo_count demo_vel_seq demo_energy_seq;

size_of_seq = 10000;
persistent contact_force pre_contact_force demo_contact_force ...
    pre_demo_contact_force static_count pre_trans trans ...
    pre_demo_trans demo_trans

if isempty(force_before_contact)
    force_before_contact = gravity_compensated_force;
    demo_force_seq = zeros(6,size_of_seq);
    force_seq = zeros(6,size_of_seq);
    contact_force = zeros(6,1);
    pre_contact_force = zeros(6,1);
    demo_contact_force = zeros(6,1);
    pre_demo_contact_force = zeros(6,1);
    
    static_count = 0;
end

if isempty(pos_seq)
    pos_seq = zeros(4,4,size_of_seq);
    vel_seq = zeros(6,size_of_seq);
    energy_seq = zeros(1,size_of_seq);
    exe_count = 1;
    pre_trans = zeros(3,1);
    trans = zeros(3,1);
    
end

if isempty(demo_ati_pose_seq)
    demo_ati_pose_seq = zeros(4,4,size_of_seq);
    demo_vel_seq = zeros(6,size_of_seq);
    demo_energy_seq = zeros(1,size_of_seq);
    demo_count = 1;
    pre_demo_trans = zeros(3,1);
    demo_trans = zeros(3,1);
end
demo_energy_seq(1,1:500);

% init params
vel_msg = handles.vel_msg;
vel_pub = handles.vel_pub;

% define m,b,k along z-axis
m = 5;
b = 400;
k = 50;

% for assembly
assembly_demo = rosparam('get','/assembly_demo');
assembly_exe = rosparam('get','/assembly_exe');
assembly_learning = rosparam('get','/assembly_learning');

% record demonstration data
if assembly_demo
    demo_trans = ati_pose_data(1:3);
    demo_rotm = quat2rotm(ati_pose_data(4:7)');
    
    demo_contact_force = gravity_compensated_force - force_before_contact;
    
    
    % judge if two nearest poses are the same
    if norm(demo_trans - pre_demo_trans) > 0.0005 || norm(demo_contact_force - pre_demo_contact_force) > 0.01
        demo_ati_pose_seq(:,:,demo_count) = RpToTrans(demo_rotm, demo_trans);
        
        
        % exchange the first and last three rows to express the force in
        % the form of [torque force]'
        demo_contact_force([1:3 4:6]) = demo_contact_force([4:6 1:3]);
        demo_force_seq(:,demo_count) = demo_contact_force;
        
        if demo_count > 1
            T_diff = demo_ati_pose_seq(:,:,demo_count-1)\demo_ati_pose_seq(:,:,demo_count);
            se3_diff = MatrixLog6(T_diff);
            S_theta = se3ToVec(se3_diff); % the axis S is multiplied by the angle theta
            demo_vel_seq(:,demo_count-1) = S_theta; 
            demo_energy_seq(demo_count-1) = pre_demo_contact_force'*S_theta;
        end
        
        demo_count = demo_count + 1;
        
        pre_demo_trans = demo_trans;
        pre_demo_contact_force = demo_contact_force;
        
        static_count = 1;
    else % if static for a long time, exit demo mode
        static_count = static_count + 1;
    end
    
    if static_count > 500 || demo_count > size_of_seq
            % rate is 50hz, if static for ten seconds, stop
            rosparam('set','/assembly_demo',false);
            static_count = 0;
    end
end
% end record demonstration data



% processing data and generate strategy
if assembly_learning
    
    
end
% end learning

% start execution
if assembly_exe
    
    
end
% end execution




% % gravity_compensated_force = [-0.01;0.4;-7.73;0.14;0.138;0.01];
% 
% % get value
% % demo_trans = demo_pose_data(1:3);
% % demo_trans = [0.109;0.487;0.4687];
% demo_trans = [-0.056;0.418;0.33];
% % demoRotm = quat2rotm(demo_pose_data(4:7)');   % quaternion value;  w x y z
% cur_trans = ati_pose_data(1:3);
% curRotm = quat2rotm(ati_pose_data(4:7)');
% pre_trans = pre_ati_pose_data(1:3);
% preRotm = quat2rotm(pre_ati_pose_data(4:7)');
% 
% contact_force = gravity_compensated_force - force_before_contact
% 
% % design velocity along z according to the contact force
% zdd = (-gravity_compensated_force(3)-b*current_vel_data(3)-k*(cur_trans(3)-demo_trans(3)))/m;
% %computed vel
% zd = zdd/100 + current_vel_data(3)
% vel_msg.Linear.Z = zd;
% 
% % design velocity along x and y according to the contact position
% % r x F = M; F x r = -M; S(F)r = -M
% 
% if abs(contact_force(3)) > 1
%     
%     %     use constant rz
%     Mx = contact_force(4);
%     My = contact_force(5);
%     Fx = contact_force(1);
%     Fy = contact_force(2);
%     Fz = contact_force(3);
%     rz = 0.23; %contact point to the origin of sensor frame
%     %     choose = 0; % Fx and Fy are considered noise during exploring right position
%     choose = 1; % compute according formula
%     rx = (choose*Fx*rz - My)/Fz
%     ry = (Mx + choose*Fy*rz)/Fz
%     pos_seq(n,1)=rx;
%     pos_seq(n,2)=ry;
%     n = n+1;
%     %     save('pos_seq.mat','pos_seq');
%     
%     vel_msg.Linear.X = rx/8;
%     vel_msg.Linear.Y = -ry/8;
%     
%     
%     %     % calculate r by two contact force
%     %
%     %     F1 = previous_contact_force(1:3);
%     %     F2 = contact_force(1:3);
%     %     F_matrix = [skew(F1);skew(F2)];
%     %     M_vector = -[previous_contact_force(4:6);contact_force(4:6)];
%     %
%     %     r = F_matrix \ M_vector;
%     %     rx = r(1)
%     %     ry = r(2)
%     %     rz = r(3)
%     %
%     %     vel_msg.Linear.X = rx/5;
%     %     vel_msg.Linear.Y = -ry/5;
%     
% else
%     vel_msg.Linear.X = 0;
%     vel_msg.Linear.Y = 0;
% end
% 
% vel_msg.Angular.X = 0;
% vel_msg.Angular.Y = 0;
% vel_msg.Angular.Z = 0;
% 
% 
% % speed limit
% vel_msg.Linear.X = limit_speed(vel_msg.Linear.X,0.0005,0.1);
% vel_msg.Linear.Y = limit_speed(vel_msg.Linear.Y,0.0005,0.1);
% vel_msg.Linear.Z = limit_speed(vel_msg.Linear.Z,0.0005,0.1);
% vel_msg.Angular.X = limit_speed(vel_msg.Angular.X,0.0001,0.1);
% vel_msg.Angular.Y = limit_speed(vel_msg.Angular.Y,0.0001,0.1);
% vel_msg.Angular.Z = limit_speed(vel_msg.Angular.Z,0.0001,0.1);
% 
% 
% send(vel_pub,vel_msg);
% 
% % update previous contact force
% previous_contact_force = contact_force;

end