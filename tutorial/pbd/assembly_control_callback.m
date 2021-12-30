function assembly_control_callback(~, ~, handles)

%%%%%%%%%%%%%%%%%%%%% system variables %%%%%%%%%%%%%%%%%%%%%
global loop_rate_hz
global current_pose_data  % tool0 pose
global ati_pose_data
global gravity_compensated_force
global current_vel_data % tool0 vel in control frame

global base_T_initial_compliance_pose

% variables in demo
global demo_seq_length demo_count demo_tool_vel_seq demo_energy_seq
global demo_tool_force_seq demo_pose_seq

% variables used in assembly execution
global exe_seq_length exe_count exe_tool_vel_seq exe_energy_seq
global exe_pose_seq exe_tool_force_seq

% the default control frame of ur is formed by the trans of tool frame w.r.t. the
% base frame. The pose of the control frame is identical to the base
% frame.
% The control frame can be changed in the installation setting of the control panel
% The control frame is the reference frame of the obtained vel from ur
% script

persistent pre_base_T_tool;
persistent force_before_contact;
persistent pos_seq;
persistent init_user_pose base_T_final_target_user
persistent tool_T_user sensor_T_user
persistent final_target_user_S_theta_per_second
persistent M B K;
persistent assembly_time;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



%%%%%%%%%%%%%%%%%%% decide task phase and operation mode %%%%%%%%%%%%%
% choose proper task phase, 1 for demo, 2 for learning, 3 for execution
% 0 for waiting
% In demonstration mode, the exe_matlab_vel node should not be started.
% default task phase is 0, defined in rosnodes.m
task_phase = rosparam('get','/task_phase');

%%%%%%%%%%%%%%%%%%% assign values to common variables %%%%%%%%%%%%%
% vel handle
vel_msg = handles.vel_msg;
vel_pub = handles.vel_pub;


% assign default value
if isempty(force_before_contact)
    force_before_contact = gravity_compensated_force;
end

if isempty(exe_count)
    %pos_seq = zeros(4,4,10000);
    exe_count = 1;
    exe_seq_length = loop_rate_hz * 20; % record 20 seconds
    exe_tool_vel_seq = zeros(6, exe_seq_length);
    exe_energy_seq = zeros(6, exe_seq_length);
    exe_tool_force_seq = zeros(6, exe_seq_length);
    exe_pose_seq = zeros(4, 4, exe_seq_length);
end

% get control frame
base_trans_tool = current_pose_data(1:3);
base_T_control = RpToTrans(eye(3), base_trans_tool); % update the control frame in real time

% get tool frame
base_rot_tool = quat2rotm(current_pose_data(4:7)');
base_T_tool = RpToTrans(base_rot_tool, base_trans_tool); % update the tool frame in real time

% get tool_T_control
tool_T_control = base_T_tool \ base_T_control;

% get sensor frame
base_trans_sensor = ati_pose_data(1:3);
base_rot_sensor = quat2rotm(ati_pose_data(4:7)');
base_T_sensor = RpToTrans(base_rot_sensor, base_trans_sensor); % sensor pose

% get sensor_T_control
sensor_T_control = base_T_sensor \ base_T_control;

% get sensor_T_tool
sensor_T_tool = base_T_sensor \ base_T_tool;

contact_force = gravity_compensated_force - force_before_contact;
% contact_force = gravity_compensated_force;

% omit disturbance
if max(abs(contact_force)) < 2
    contact_force = zeros(6,1);
end

% exchange the first and last three rows to express the force in
% the form of [torque force]'
contact_force([1:3 4:6]) = contact_force([4:6 1:3]);

% convert to tool frame
tool_contact_force = Adjoint(sensor_T_tool)'*contact_force;

% get vel using own algorithm
if isempty(pre_base_T_tool)
    pre_base_T_tool = base_T_tool;
end
derived_pose_diff = pre_base_T_tool \ base_T_tool;
derived_tool_se3 = MatrixLog6(derived_pose_diff);
derived_tool_S_theta = se3ToVec(derived_tool_se3);
derived_tool_V_tool = derived_tool_S_theta*loop_rate_hz;
pre_base_T_tool = base_T_tool;

% using system vel interface to get velocity
control_V_control = current_vel_data;
control_V_control([1:3 4:6]) = control_V_control([4:6 1:3]); % change to the form of [omega v]'
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

if isempty(base_T_final_target_user)
    
    %%%%%%%%%%%%%% user-defined frame, nominal value %%%%%%%%%%%
    
    % custom user frame
    sensor_T_user = [eye(3) [0 0 0.31]'; 0 0 0 1];
%     base_T_final_target_user = RpToTrans(roty(pi),[0.109, 0.487,
%     -0.018]'); % 短粗杆 common
    base_T_final_target_user = RpToTrans(roty(pi),[0.109, 0.487, -0.04]'); % 短粗杆 test on goal
    %     base_T_final_target_user = RpToTrans(roty(pi),[0.109, 0.487, 0.025]'); % 短粗杆
    % sensor_T_user = [eye(3) [0 0 0.392]'; 0 0 0 1];  % 长杆
    %     sensor_T_user = [eye(3) [0 0 0.277]'; 0 0 0 1];  % 短粗杆
    %     sensor_T_user = [eye(3) [0 0 0.377]'; 0 0 0 1];  % 短粗杆
    tool_T_user = sensor_T_tool \ sensor_T_user;
    control_T_user = sensor_T_control \ sensor_T_user;
    % base_T_final_target_user = RpToTrans(roty(pi),[0.109, 0.487, -0.10]'); % 长杆
    %     base_T_final_target_user = RpToTrans(roty(pi),[0.109, 0.487, -0.032]'); % 短粗杆
    %      base_T_final_target_user = RpToTrans(roty(pi),[0.109, 0.487, -0.131]'); % 短粗杆
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
end

base_T_user = base_T_sensor*sensor_T_user;


%%%%%%%%%%%%%%%%%%%%% different task phase %%%%%%%%%%%%%%%%%%%%%%
if task_phase == 0
    % waiting, do nothing
    vel_msg.Angular.X = 0;
    vel_msg.Angular.Y = 0;
    vel_msg.Angular.Z = 0;
    vel_msg.Linear.X = 0;
    vel_msg.Linear.Y = 0;
    vel_msg.Linear.Z = 0;
    send(vel_pub,vel_msg);
    
elseif task_phase == 1
    % demonstration mode
    if isempty(demo_seq_length)
        demo_seq_length = loop_rate_hz * 20; % record 20 seconds
        demo_count = 1;
        demo_tool_vel_seq = zeros(6, demo_seq_length);
        demo_energy_seq = zeros(6, demo_seq_length);
        demo_tool_force_seq = zeros(6, demo_seq_length);
        demo_pose_seq = zeros(4, 4, demo_seq_length);
    end
    
    if demo_count < 3
        disp("start demonstration")
    end

    
    % using system vel in demonstration mode, using derived vel in execution
    % The sys vel has a delay, and the derived vel has some small vibration due
    % to the error of the tool pose from ros interface when the robot stops
    
    % compute spatial acceleration in tool frame
    tool_V_tool = Adjoint(tool_T_control)*control_V_control;
    
    
    if demo_count <= demo_seq_length
        demo_tool_vel_seq(:,demo_count) = tool_V_tool;
        demo_tool_force_seq(:,demo_count) = tool_contact_force;
        demo_energy_seq(:,demo_count) = tool_V_tool.*tool_contact_force;
        demo_pose_seq(:,:,demo_count) = base_T_tool;
        demo_count = demo_count + 1;
    else
        rosparam('set','/task_phase',0); % wait after learning
        disp("end demonstration")
    end
    
    
    
elseif task_phase == 2
    % learning mode
    
    disp("start learning")
    
    % obtain the time when the robot moves
    demo_sum_vel = vecnorm(demo_tool_vel_seq,2);
    nonzero_vel_loc = find(demo_sum_vel>0.01);
    assembly_start_time = nonzero_vel_loc(1);
    assembly_end_time = nonzero_vel_loc(end);
    % obtain the assembly time in demonstration. Sometimes the demonstraion
    % finishes in a second, the execution phase can't execute in such a high
    % speed to make sure safety. Therefore, the assembly time in execution is
    % three times that in demonstration.
    time_scaler = 3;
    assembly_time = (assembly_end_time - assembly_start_time) / loop_rate_hz * time_scaler;
    
    %     obtain base_T_final_target_user
    base_T_final_target_user = demo_pose_seq(:,:,assembly_end_time)*tool_T_user;
    % to overcome the friction force, the real target frame is set below the
    % theoretical target frame
    base_T_final_target_user = base_T_final_target_user - 0.006
    %     base_T_final_target_user = base_T_final_target_user + 0.01
    
    % obtain assembly direction
    % at present,1 for rx, 2 for ry, 3 for rz, 4 for x, 5 for y, 6 for z in tool frame
    % it is better to choose the vel in stiffness diretion when stiffness
    % should be maintained in multi directions
    [~, direction_index] = max(sum(demo_energy_seq<-0.2,2));
    
    %     M = [0.15*eye(3) zeros(3,3);zeros(3,3) 15*eye(3)]; % omega in the first three rows
    %     B = [0.3*eye(3) zeros(3,3);zeros(3,3) 60*eye(3)];
    %     K = [0*eye(3) zeros(3,3);zeros(3,3) 0*eye(3)];
    % % adjust the param along z-axis
    %     M(direction_index,direction_index) = 25
    %     B(direction_index,direction_index) = 130
    %     %     K(6,6) = 170;
    %     K(direction_index,direction_index) = 160
    
    M = [0.5*eye(3) zeros(3,3);zeros(3,3) 10*eye(3)]; % omega in the first three rows
    B = [0.8*eye(3) zeros(3,3);zeros(3,3) 60*eye(3)];
    K = [0*eye(3) zeros(3,3);zeros(3,3) 0*eye(3)];
    M(direction_index,direction_index) = 25
    B(direction_index,direction_index) = 100
    %     K(6,6) = 170;
    K(direction_index,direction_index) = 100
    
    
    rosparam('set','/task_phase',0); % wait after learning
    disp("end learning")
    
elseif task_phase == 3
    % execution mode
    if exe_count < 3
        disp("start execution")
    end
    if isempty(assembly_time)
        % should be obtained from demonstration, if not, specify a value
        assembly_time = 10;
    end
    
    % dynamic balance pose
    if isempty(init_user_pose)
        init_user_pose = base_T_user;
        final_target_user_T_init_user = base_T_final_target_user \ base_T_user;
        final_target_user_se3 = MatrixLog6(final_target_user_T_init_user);
        final_target_user_S_theta = se3ToVec(final_target_user_se3); % the angle theta has been included
        final_target_user_S_theta_per_second = final_target_user_S_theta / assembly_time;
    end
    
    
    % use derived vel
    tool_V_tool = derived_tool_V_tool;
    
    
    % The acceleration should be changed with rosparam /speedl_acceleration
    % 0.1 for keep static, 0.5 for drag, 0.1 for assembly
    rosparam('set','/speedl_acceleration',0.1);
    
    if isempty(M)
        %         M = [0.15*eye(3) zeros(3,3);zeros(3,3) 15*eye(3)]; % omega in the first three rows
        %         B = [0.3*eye(3) zeros(3,3);zeros(3,3) 60*eye(3)];
        %         K = [0*eye(3) zeros(3,3);zeros(3,3) 0*eye(3)];
        %         % % adjust the param along z-axis
        %         %     M(6,6) = 25;
        %         %     B(6,6) = 130;
        %         %     K(6,6) = 250;
        %         M(6,6) = 25;
        %         B(6,6) = 130;
        %         K(6,6) = 160;
        
%         M = [1*eye(3) zeros(3,3);zeros(3,3) 15*eye(3)]; % omega in the first three rows
%         B = [2*eye(3) zeros(3,3);zeros(3,3) 60*eye(3)];
%         K = [0*eye(3) zeros(3,3);zeros(3,3) 0*eye(3)];
%         % % adjust the param along z-axis
%         %     M(6,6) = 25;
%         %     B(6,6) = 130;
%         %     K(6,6) = 250;
%         M(6,6) = 25;
%         B(6,6) = 100;
%         K(6,6) = 100;
        
        % test
        M = [4*eye(3) zeros(3,3);zeros(3,3) 15*eye(3)]; % omega in the first three rows
        B = [40*eye(3) zeros(3,3);zeros(3,3) 300*eye(3)];
        K = [50*eye(3) zeros(3,3);zeros(3,3) 2000*eye(3)];
        % % adjust the param along z-axis
            M(6,6) = 25;
            B(6,6) = 100;
            K(6,6) = 100;


%         M = [2*eye(3) zeros(3,3);zeros(3,3) 10*eye(3)]; % omega in the first three rows
%         B = [4*eye(3) zeros(3,3);zeros(3,3) 50*eye(3)];
%         K = [0*eye(3) zeros(3,3);zeros(3,3) 0*eye(3)];
%         % % adjust the param along z-axis
%         %     M(6,6) = 25;
%         %     B(6,6) = 130;
%         %     K(6,6) = 250;
%         M(6,6) = 10;
%         B(6,6) = 20;
%         K(6,6) = 80;
    end
    
    
    if (assembly_time - exe_count/loop_rate_hz) > 0
        base_T_target_user = base_T_final_target_user * MatrixExp6(VecTose3(final_target_user_S_theta_per_second * (assembly_time - exe_count/loop_rate_hz)));
    else
        base_T_target_user = base_T_final_target_user;
    end

    if isempty(base_T_initial_compliance_pose)
        base_T_initial_compliance_pose = base_T_user;
    end

    
    % impedance system in z-axis
    target_user_T_user = base_T_target_user \ base_T_user;
    target_user_se3 = MatrixLog6(target_user_T_user);
    target_user_S_theta = se3ToVec(target_user_se3); % the angle theta has been included
    user_S_theta = Adjoint(inv(target_user_T_user))*target_user_S_theta;
    tool_S_theta = Adjoint(tool_T_user)*user_S_theta;
    
    % impedence system in other directions
    tool_T_compliance_pose = base_T_tool \ base_T_initial_compliance_pose;
    compliance_pose_T_user = base_T_initial_compliance_pose \ base_T_user;
    compliance_pose_se3 = MatrixLog6(compliance_pose_T_user);
    compliance_pose_S_theta = se3ToVec(compliance_pose_se3); % the angle theta has been included
    tool_T_compliance_pose_S_theta = Adjoint(tool_T_compliance_pose)*compliance_pose_S_theta;

    %%%%%%%%%%%%%%%%%%
    
    % contact_force;
    % user_contact_force = Adjoint(sensor_T_user)'*contact_force;
    % user_V_user = Adjoint(inv(control_T_user))*control_V_control;
    % tool_dV_from_user = M\(user_contact_force - K*user_S_theta - B*user_V_user)
    
    % % compute spatial twist
    % % updated_user_V_user(1:3) = (4*tool_dV_from_user(1:3) + 0.5*user_V_user(1:3))/(loop_rate_hz);
    % % updated_user_V_user(4:6) = (0.6*tool_dV_from_user(4:6) + 0.1*user_V_user(4:6))/(loop_rate_hz);
    % updated_user_V_user(1:3) = (tool_dV_from_user(1:3)/(loop_rate_hz) + 0.2*user_V_user(1:3));
    % updated_user_V_user(4:6) = (tool_dV_from_user(4:6)/(loop_rate_hz) + 0.2*user_V_user(4:6));
    % %         previous_updated_user_V_user = updated_user_V_user;
    % updated_user_V_user;
    
    % % base_T_user-base_T_target_user
    % control_V_control_feedback = Adjoint(control_T_user)*updated_user_V_user;
    %%%%%%%%%%%%%%%%%%%
    
    
    % M B K is defined in user frame
    Ad_user_T_tool = Adjoint(inv(tool_T_user));
    M_tool = Ad_user_T_tool'*M*Ad_user_T_tool;
    B_tool = Ad_user_T_tool'*B*Ad_user_T_tool;
    K_tool = Ad_user_T_tool'*K*Ad_user_T_tool;
    %     tool_contact_force(1:3) = tool_contact_force(1:3)/2;
    
    % selection matrix, P_z control the force in z-axis
    P_z = zeros(6,6);
    P_z(6,6) = 1;
    % selection matrix, P_other control wrench in other five directions
    P_other = eye(6) - P_z;
    
    tool_dV_from_user = M_tool\(tool_contact_force - P_z*K_tool*tool_S_theta - P_other*K_tool*tool_T_compliance_pose_S_theta - B_tool*tool_V_tool);
    f_other = K_tool*tool_T_compliance_pose_S_theta
    f_z = K_tool*tool_S_theta
    % tool_dV_from_user = M\(tool_contact_force - K*tool_S_theta - B*tool_V_tool);
    
    
    % M B K is defined in sensor frame
    Ad_sensor_T_tool = Adjoint(sensor_T_tool);
    M_tool_from_sensor = Ad_sensor_T_tool'*M*Ad_sensor_T_tool;
    B_tool_from_sensor = Ad_sensor_T_tool'*B*Ad_sensor_T_tool;
    K_tool_from_sensor = Ad_sensor_T_tool'*K*Ad_sensor_T_tool;
    %     tool_contact_force(1:3) = tool_contact_force(1:3)/2;
    tool_dV_from_sensor = M_tool_from_sensor\(tool_contact_force - K_tool_from_sensor*tool_S_theta - B_tool_from_sensor*tool_V_tool);
    % tool_dV_from_user = M\(tool_contact_force - K*tool_S_theta - B*tool_V_tool);
    
    rotation_same_direction = (tool_dV_from_sensor(4)*tool_dV_from_user(4)+tool_dV_from_sensor(5)*tool_dV_from_user(5))>-0.01;
    
    %     if rotation_same_direction == 0
    %         tool_dV_from_user = tool_dV_from_sensor;
    %     end
    
    
    if exe_count <= exe_seq_length
        exe_tool_vel_seq(:,exe_count) = tool_V_tool;
        exe_tool_force_seq(:,exe_count) = tool_contact_force;
        exe_energy_seq(:,exe_count) = tool_V_tool.*tool_contact_force;
        exe_pose_seq(:,:,exe_count) = base_T_tool;
        exe_count = exe_count + 1;
    else
        rosparam('set','/task_phase',0); % wait after execution done
        disp("end execution")
    end
    
    
    updated_tool_V_tool = zeros(6,1);
    % compute spatial twist
    
    updated_tool_V_tool(1:3) = (1*tool_dV_from_user(1:3)/(loop_rate_hz) + 0.2*tool_V_tool(1:3));
    updated_tool_V_tool(4:6) = (1*tool_dV_from_user(4:6)/(loop_rate_hz) + 0.2*tool_V_tool(4:6));

%     % for test
%     updated_tool_V_tool(1:3) = (1*tool_dV_from_user(1:3)/(loop_rate_hz) + 1*tool_V_tool(1:3));
%     updated_tool_V_tool(4:6) = (1*tool_dV_from_user(4:6)/(loop_rate_hz) + 1*tool_V_tool(4:6));
    
    control_V_control_feedback = Adjoint(inv(tool_T_control))*updated_tool_V_tool;
    
    %     tool_V_tool = Adjoint(inv(sensor_T_tool)) * updated_user_V_user;
    %
    %     base_V_tool = base_rot6_tool * tool_V_tool;
    
    % emergency stop
    if max(abs(contact_force)) > 50
        control_V_control_feedback = zeros(6,1);
        % set task phase, 0 for waiting
        rosparam('set','/task_phase',0);        
    end
    
    % on goal
    if max(abs(contact_force)) > 10 && abs(base_trans_tool(3)-0.377) < 0.003
        control_V_control_feedback = zeros(6,1);
        % set task phase, 0 for waiting
        rosparam('set','/task_phase',0);
    end
    
    vel_msg.Angular.X = control_V_control_feedback(1);
    vel_msg.Angular.Y = control_V_control_feedback(2);
    vel_msg.Angular.Z = control_V_control_feedback(3);
    vel_msg.Linear.X = control_V_control_feedback(4);
    vel_msg.Linear.Y = control_V_control_feedback(5);
    vel_msg.Linear.Z = control_V_control_feedback(6);
    % vel_msg.Linear.Z =-0.001;
    % vel_msg.Linear.Z = 0.02;
    
    % % speed limit
    vel_msg.Linear.X = limit_speed(vel_msg.Linear.X,0.001,0.25);
    vel_msg.Linear.Y = limit_speed(vel_msg.Linear.Y,0.001,0.25);
    vel_msg.Linear.Z = limit_speed(vel_msg.Linear.Z,0.001,0.25);
    % vel_msg.Angular.X = 0;
    % vel_msg.Angular.Y = 0;
    % vel_msg.Angular.Z = 0;
    % vel_msg.Linear.X = 0;
    % vel_msg.Linear.Y = 0;
    % vel_msg.Linear.Z = 0;
    vel_msg.Angular.X = limit_speed(vel_msg.Angular.X,0.001,1);
    vel_msg.Angular.Y = limit_speed(vel_msg.Angular.Y,0.001,1);
    vel_msg.Angular.Z = limit_speed(vel_msg.Angular.Z,0.001,1);
    
    
    send(vel_pub,vel_msg);
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

end