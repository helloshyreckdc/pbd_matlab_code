function single_axis_admittance_control_callback(~, ~, handles)

global loop_rate_hz
global current_pose_data
global ati_pose_data
global pre_pose_data
global gravity_compensated_force
global current_vel_data
persistent force_before_contact previous_contact_force;
persistent pos_seq;
persistent n previous_updated_user_V_user

% the default control frame of ur is formed by the trans of tool frame w.r.t. the
% base frame. The pose of the control frame is identical to the base
% frame.
% The control frame can be changed in the installation setting of the control panel
persistent control_frame; 
persistent user_frame; 

vel_msg = handles.vel_msg;
vel_pub = handles.vel_pub;

% define M B K matrix in user-defined frame, for example, the frame fixed
% on the tip of the peg
M = [2*eye(3) zeros(3,3);zeros(3,3) 5*eye(3)];
B = [5*eye(3) zeros(3,3);zeros(3,3) 5*eye(3)];
K = 0*[eye(3) zeros(3,3);zeros(3,3) eye(3)];


% M = 1*eye(6);
% B = 0*eye(6);
% K = 0*eye(6);

% adjust the param along z-axis
% B(6,6) = 50;
% K(6,6) = 500;



if isempty(force_before_contact)
    force_before_contact = gravity_compensated_force;
    previous_contact_force = force_before_contact;
    % force_before_contact = [0.36;0.18;3.66;0.21;-0.01;-0.02];
end

if isempty(pos_seq)
    pos_seq = zeros(4,4,10000);
    n = 1;
end

if isempty(previous_updated_user_V_user)
    previous_updated_user_V_user = zeros(6,1);
end



% if isempty(control_frame)
base_trans_tool = current_pose_data(1:3);
base_rot_base = eye(3);
control_frame = RpToTrans(base_rot_base, base_trans_tool); % update the control frame in real time
% end





if ~isempty(pre_pose_data)
    % get value
    %[refRotm, ref_trans] = TransToRp(static_pose);
    
%     base_trans_tool = current_pose_data(1:3);
%     base_rot_tool = quat2rotm(current_pose_data(4:7)');
%     base_T_tool = RpToTrans(base_rot_tool, base_trans_tool); % end effector pose
%     base_rot6_tool = [base_rot_tool zeros(3,3); zeros(3,3) base_rot_tool];
%     
%     pre_base_trans_tool = pre_pose_data(1:3);
%     pre_base_rot_tool = quat2rotm(pre_pose_data(4:7)');
%     pre_base_T_tool = RpToTrans(pre_base_rot_tool, pre_base_trans_tool); % end effector pose
    
    base_trans_sensor = ati_pose_data(1:3);
    base_rot_sensor = quat2rotm(ati_pose_data(4:7)');
    base_T_sensor = RpToTrans(base_rot_sensor, base_trans_sensor); % sensor pose
    
    sensor_T_control = base_T_sensor \ control_frame;
    
    contact_force = gravity_compensated_force - force_before_contact;
    % omit disturbance
    if max(abs(contact_force)) < 2
        contact_force = zeros(6,1);
    end
    
    % exchange the first and last three rows to express the force in
    % the form of [torque force]'
    
    contact_force([1:3 4:6]) = contact_force([4:6 1:3]);
    
    
    
%     %     static_T_sensor = static_pose \ base_T_sensor;
%     static_se3_sensor = MatrixLog6(static_T_sensor);
%     static_S_theta = se3ToVec(static_se3_sensor); % the axis S is multiplied by the angle theta
%     sensor_S_theta = Adjoint(inv(static_T_sensor))*static_S_theta;
    
    % using system vel interface to get 
    control_V_control = current_vel_data;
    control_V_control([1:3 4:6]) = control_V_control([4:6 1:3]); % change to the form of [omega v]'
    
    
    %     % using own algorithm to get vel
    %     pre_tool_T_tool = pre_base_T_tool \ base_T_tool;
    %     pre_tool_se3_tool = MatrixLog6(pre_tool_T_tool);
    %     pre_tool_S_theta = se3ToVec(pre_tool_se3_tool); % the axis S is multiplied by the angle theta
    %     tool_V_tool =  pre_tool_S_theta*loop_rate_hz;
    
%     sensor_V_sensor = Adjoint(sensor_T_tool) * tool_V_tool;

% % user_frame = control_frame; % in drag mode, the two frame overlap
% control_T_user = eye(4);
% sensor_T_user = sensor_T_control*control_T_user;

% assembly mode
% sensor_T_user = [eye(3) [0 0 0.31]'; 0 0 0 1];
sensor_T_user = [eye(3) [0 0 0.41]'; 0 0 0 1];
control_T_user = sensor_T_control \ sensor_T_user;
   

    % compute spatial acceleration in user frame
    if norm(contact_force) > 0.001
        updated_user_V_user = zeros(6,1);
%         contact_force
        user_contact_force = Adjoint(sensor_T_user)'*contact_force;
        user_V_user = Adjoint(inv(control_T_user))*control_V_control;
        user_S_theta = user_V_user*(1/loop_rate_hz); %这个不对，不是当前速度的函数，应当是平衡（参考）位置和当前位置的偏差形成的旋转轴从而形成的回复力
        dV = M\(user_contact_force - K*user_S_theta - B*user_V_user);
        % compute spatial twist
        dV
        updated_user_V_user(1:3) = (4*dV(1:3) + 0.5*user_V_user(1:3))/(loop_rate_hz);
        updated_user_V_user(4:6) = (0.6*dV(4:6) + 0.1*user_V_user(4:6))/(loop_rate_hz);
%         previous_updated_user_V_user = updated_user_V_user;
updated_user_V_user
    else
        updated_user_V_user = zeros(6,1);
%         previous_updated_user_V_user = updated_user_V_user;
    end
    
        control_V_control_feedback = Adjoint(control_T_user)*updated_user_V_user;
        
    
%     tool_V_tool = Adjoint(inv(sensor_T_tool)) * updated_user_V_user;
%     
%     base_V_tool = base_rot6_tool * tool_V_tool;
    
    vel_msg.Angular.X = control_V_control_feedback(1);
    vel_msg.Angular.Y = control_V_control_feedback(2);
    vel_msg.Angular.Z = control_V_control_feedback(3);
       vel_msg.Linear.X = control_V_control_feedback(4);
    vel_msg.Linear.Y = control_V_control_feedback(5);
%     vel_msg.Linear.Z = control_V_control_feedback(6);
     vel_msg.Linear.Z =-0.001;
    % vel_msg.Linear.Z = 0.02;
    
    % speed limit
    vel_msg.Linear.X = limit_speed(vel_msg.Linear.X,0.001,0.25);
    vel_msg.Linear.Y = limit_speed(vel_msg.Linear.Y,0.001,0.25);
    vel_msg.Linear.Z = limit_speed(vel_msg.Linear.Z,0.000,0.25);
    %         vel_msg.Linear.X = limit_speed(vel_msg.Linear.X,0.001,0);
    %     vel_msg.Linear.Y = limit_speed(vel_msg.Linear.Y,0.001,0);
    %     vel_msg.Linear.Z = limit_speed(vel_msg.Linear.Z,0.001,0);
    vel_msg.Angular.X = limit_speed(vel_msg.Angular.X,0.001,1);
    vel_msg.Angular.Y = limit_speed(vel_msg.Angular.Y,0.001,1);
    vel_msg.Angular.Z = limit_speed(vel_msg.Angular.Z,0.001,1);
    
    
    send(vel_pub,vel_msg);
    n = n+1;
    % update previous contact force
    previous_contact_force = contact_force;
end


end