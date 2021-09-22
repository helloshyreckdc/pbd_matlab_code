function multi_axis_admittance_control_callback(~, ~, handles)

%%%%%%%%%%%%%%%%%%%%% system variables %%%%%%%%%%%%%%%%%%%%%
global loop_rate_hz
global current_pose_data  % tool0 pose
global ati_pose_data
global gravity_compensated_force
global current_vel_data % tool0 vel in control frame

% variables in demo
global demo_seq_length demo_count demo_tool_vel_seq demo_energy_seq
global demo_tool_force_seq demo_pose_seq



% the default control frame of ur is formed by the trans of tool frame w.r.t. the
% base frame. The pose of the control frame is identical to the base
% frame.
% The control frame can be changed in the installation setting of the control panel
% The control frame is the reference frame of the obtained vel from ur
% script
persistent control_frame;
persistent pre_base_T_tool;
persistent force_before_contact;
persistent pos_seq;
persistent init_user_pose base_T_final_target_user
persistent tool_T_user sensor_T_user
persistent count_number final_target_user_S_theta_per_second
persistent M B K;
persistent assembly_time;
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



%%%%%%%%%%%%%%%%%%% decide task phase and operation mode %%%%%%%%%%%%%
% choose proper task phase, 1 for demo, 2 for learning, 3 for execution
% 0 for waiting

task_phase = rosparam('get','/task_phase');
operation_mode = rosparam('get','/operation_mode');

% The operation mode should be chosen during execution
% 0 for keep static, 1 for drag, 2 for assembly
% In demonstration mode, the exe_matlab_vel node should not be started.
% The acceleration should be changed with rosparam /speedl_acceleration
% 0.1 for keep static, 0.5 for drag, 0.1 for assembly
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



%%%%%%%%%%%%%%%%%%% assign values to common variables %%%%%%%%%%%%%
% vel handle
vel_msg = handles.vel_msg;
vel_pub = handles.vel_pub;


% assign default value
if isempty(force_before_contact)
    force_before_contact = gravity_compensated_force;
end

if isempty(pos_seq)
    pos_seq = zeros(4,4,10000);
    count_number = 1;
end



% get control frame
base_trans_tool = current_pose_data(1:3);
control_frame = RpToTrans(eye(3), base_trans_tool); % update the control frame in real time

% get tool frame
base_rot_tool = quat2rotm(current_pose_data(4:7)');
base_T_tool = RpToTrans(base_rot_tool, base_trans_tool); % update the tool frame in real time

% get tool_T_control
tool_T_control = base_T_tool \ control_frame;

% get sensor frame
base_trans_sensor = ati_pose_data(1:3);
base_rot_sensor = quat2rotm(ati_pose_data(4:7)');
base_T_sensor = RpToTrans(base_rot_sensor, base_trans_sensor); % sensor pose

% get sensor_T_control
sensor_T_control = base_T_sensor \ control_frame;

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
    
    % % when tool frame and user frame overlap
    % tool_T_user = eye(4);
    % sensor_T_user = sensor_T_tool*tool_T_user;
    % base_T_final_target_user = RpToTrans(rotx(pi)*rotz(pi/2),[0.111, 0.486, 0.453]');
    % control_T_user = sensor_T_control \ sensor_T_user;
    
    % custom user frame
    % sensor_T_user = [eye(3) [0 0 0.31]'; 0 0 0 1];
    % sensor_T_user = [eye(3) [0 0 0.392]'; 0 0 0 1];  % 长杆
    sensor_T_user = [eye(3) [0 0 0.277]'; 0 0 0 1];  % 短粗杆
    tool_T_user = sensor_T_tool \ sensor_T_user;
    control_T_user = sensor_T_control \ sensor_T_user;
    % base_T_target_user = RpToTrans(roty(pi),[0.109, 0.487, 0.05]');
    % base_T_final_target_user = RpToTrans(roty(pi),[0.109, 0.487, -0.10]'); % 长杆
    base_T_final_target_user = RpToTrans(roty(pi),[0.109, 0.487, -0.0286]'); % 短粗杆
    %     base_T_final_target_user = RpToTrans(roty(pi),[0.109, 0.487, -0.02]'); % 短粗杆
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
end

base_T_user = base_T_sensor*sensor_T_user;



%%%%%%%%%%%%%%%%%%%%% different task phase %%%%%%%%%%%%%%%%%%%%%%
if task_phase == 0
    % waiting, do nothing
    
elseif task_phase == 1
    % demonstration mode
    if isempty(demo_seq_length)
        demo_seq_length = loop_rate_hz * 30; % record 30 seconds
        demo_count = 1;
        demo_tool_vel_seq = zeros(6, demo_seq_length);
        demo_energy_seq = zeros(6, demo_seq_length);
        demo_tool_force_seq = zeros(6, demo_seq_length);
        demo_pose_seq = zeros(4, 4, demo_seq_length);
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
    end
    
    
    
elseif task_phase == 2
    % learning mode
    
    
    
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
    
    % obtain assembly direction
    % at present,1 for rx, 2 for ry, 3 for rz, 4 for x, 5 for y, 6 for z in tool frame
    % it is better to choose the vel in stiffness diretion when stiffness
    % should be maintained in multi directions
    [~, direction_index] = max(sum(demo_energy_seq<-0.2,2));
    M = [0.15*eye(3) zeros(3,3);zeros(3,3) 15*eye(3)]; % omega in the first three rows
    B = [0.3*eye(3) zeros(3,3);zeros(3,3) 60*eye(3)];
    K = [0*eye(3) zeros(3,3);zeros(3,3) 0*eye(3)];
    % % adjust the param along z-axis
    M(direction_index,direction_index) = 25;
    B(direction_index,direction_index) = 130;
    %     K(6,6) = 170;
    K(direction_index,direction_index) = 250;
    
    
    rosparam('set','/task_phase',0); % wait after learning
    
elseif task_phase == 3
    % execution mode
    
    if isempty(assembly_time)
        % should be obtained from demonstration, if not, specify a value
        assembly_time = 10;
    end
    
    % dynamic balance pose
    if isempty(init_user_pose)
        init_user_pose = base_T_user;
        final_target_user_T_init_user = inv(base_T_final_target_user)*base_T_user;
        final_target_user_se3 = MatrixLog6(final_target_user_T_init_user);
        final_target_user_S_theta = se3ToVec(final_target_user_se3); % the angle theta has been included
        final_target_user_S_theta_per_second = final_target_user_S_theta / assembly_time;
    end
    
    
    % use derived vel
    tool_V_tool = derived_tool_V_tool;
    
    
    if operation_mode == 0
        rosparam('set','/speedl_acceleration',0.1);
        base_T_final_target_user = RpToTrans(roty(pi),[0.109, 0.487, 0.0286]'); % 短粗杆
        % % maintain in a static pose in space
        %         M = [1*eye(3) zeros(3,3);zeros(3,3) 5*eye(3)]; % omega in the first three rows
        %         B = [20*eye(3) zeros(3,3);zeros(3,3) 40*eye(3)];
        %         K = [20*eye(3) zeros(3,3);zeros(3,3) 80*eye(3)];
        
        % define M B K matrix in user-defined frame, for example, the frame fixed
        % on the tip of the peg
        M = [1 0 0 0 0 0;
            0 1 0 0 0 0;
            0 0 1 0 0 0;
            0 0 0 5 0 0;
            0 0 0 0 5 0;
            0 0 0 0 0 5];
        
        B = [2 0 0 0 0 0;
            0 2 0 0 0 0;
            0 0 2 0 0 0;
            0 0 0 20 0 0;
            0 0 0 0 20 0;
            0 0 0 0 0 20];
        
        K = [10 0 0 0 0 0;
            0 10 0 0 0 0;
            0 0 10 0 0 0;
            0 0 0 100 0 0;
            0 0 0 0 100 0;
            0 0 0 0 0 100];
    elseif operation_mode == 1
        % drag mode
        rosparam('set','/speedl_acceleration',0.5);
        M = [0.1*eye(3) zeros(3,3);zeros(3,3) 0.1*eye(3)]; % omega in the first three rows
        B = [0.2*eye(3) zeros(3,3);zeros(3,3) 0.5*eye(3)];
        K = [0*eye(3) zeros(3,3);zeros(3,3) 0*eye(3)];
        
        
    elseif operation_mode == 2
        rosparam('set','/speedl_acceleration',0.1);
        base_T_final_target_user = RpToTrans(roty(pi),[0.109, 0.487, -0.0286]'); % 短粗杆
%         if isempty(M)
            M = [0.15*eye(3) zeros(3,3);zeros(3,3) 15*eye(3)]; % omega in the first three rows
            B = [0.3*eye(3) zeros(3,3);zeros(3,3) 60*eye(3)];
            K = [0*eye(3) zeros(3,3);zeros(3,3) 0*eye(3)];
            % % adjust the param along z-axis
            %     M(6,6) = 25;
            %     B(6,6) = 130;
            %     K(6,6) = 250;
            M(6,6) = 25;
            B(6,6) = 130;
            K(6,6) = 170;
%         end
        % assembly mode
        %     M = [0.1*eye(3) zeros(3,3);zeros(3,3) 5*eye(3)]; % omega in the first three rows
        %     B = [0.2*eye(3) zeros(3,3);zeros(3,3) 20*eye(3)];
        %     K = [0*eye(3) zeros(3,3);zeros(3,3) 0*eye(3)];
        %     % % adjust the param along z-axis
        %     M(6,6) = 25;
        %     B(6,6) = 130;
        %     K(6,6) = 160;
        %     M(6,6) = 25;
        %     B(6,6) = 100;
        %     K(6,6) = 100;
        
        % init M B K in other phase
        %         M = [0.15*eye(3) zeros(3,3);zeros(3,3) 15*eye(3)]; % omega in the first three rows
        %         B = [0.3*eye(3) zeros(3,3);zeros(3,3) 60*eye(3)];
        %         K = [0*eye(3) zeros(3,3);zeros(3,3) 0*eye(3)];
        %         % % adjust the param along z-axis
        %         M(6,6) = 25;
        %         B(6,6) = 130;
        %         %     K(6,6) = 170;
        %         K(6,6) = 250;
        
        
    end
    
    
    
    if (assembly_time - count_number/loop_rate_hz) > 0
        base_T_target_user = base_T_final_target_user * MatrixExp6(VecTose3(final_target_user_S_theta_per_second * (assembly_time - count_number/loop_rate_hz)));
    else
        base_T_target_user = base_T_final_target_user;
    end
    base_T_target_user
    
    target_user_T_user = inv(base_T_target_user)*base_T_user;
    target_user_se3 = MatrixLog6(target_user_T_user);
    target_user_S_theta = se3ToVec(target_user_se3); % the angle theta has been included
    user_S_theta = Adjoint(inv(target_user_T_user))*target_user_S_theta;
    tool_S_theta = Adjoint(tool_T_user)*user_S_theta;
    
    %%%%%%%%%%%%%%%%%%
    % % compute spatial acceleration in user frame
    % updated_user_V_user = zeros(6,1);
    % contact_force;
    % user_contact_force = Adjoint(sensor_T_user)'*contact_force;
    % user_V_user = Adjoint(inv(control_T_user))*control_V_control;
    % dV = M\(user_contact_force - K*user_S_theta - B*user_V_user)
    
    % % compute spatial twist
    % % updated_user_V_user(1:3) = (4*dV(1:3) + 0.5*user_V_user(1:3))/(loop_rate_hz);
    % % updated_user_V_user(4:6) = (0.6*dV(4:6) + 0.1*user_V_user(4:6))/(loop_rate_hz);
    % updated_user_V_user(1:3) = (dV(1:3)/(loop_rate_hz) + 0.2*user_V_user(1:3));
    % updated_user_V_user(4:6) = (dV(4:6)/(loop_rate_hz) + 0.2*user_V_user(4:6));
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
    
    dV = M_tool\(tool_contact_force - K_tool*tool_S_theta - B_tool*tool_V_tool);
    % dV = M\(tool_contact_force - K*tool_S_theta - B*tool_V_tool);
    
    
    
    
    
    updated_tool_V_tool = zeros(6,1);
    % compute spatial twist
    if operation_mode == 0
        updated_tool_V_tool(1:3) = (1*dV(1:3)/(loop_rate_hz) + 0.2*tool_V_tool(1:3));
        updated_tool_V_tool(4:6) = (1*dV(4:6)/(loop_rate_hz) + 0.2*tool_V_tool(4:6));
    elseif operation_mode == 1
        updated_tool_V_tool(1:3) = (1*dV(1:3)/(loop_rate_hz) + 0*tool_V_tool(1:3));
        updated_tool_V_tool(4:6) = (1*dV(4:6)/(loop_rate_hz) + 0*tool_V_tool(4:6));
    elseif operation_mode == 2
        updated_tool_V_tool(1:3) = (1*dV(1:3)/(loop_rate_hz) + 0.2*tool_V_tool(1:3));
        updated_tool_V_tool(4:6) = (1*dV(4:6)/(loop_rate_hz) + 0.2*tool_V_tool(4:6));
        
    end
    
    
    % if(abs(contact_force(6))>5)
    %     disp("seq")
    %     count_number
    %     tool_contact_force
    %     control_V_control
    %     tool_V_tool
    %     derived_tool_V_tool
    %     dV
    %     updated_tool_V_tool
    %     disp("end")
    % end
    
    control_V_control_feedback = Adjoint(inv(tool_T_control))*updated_tool_V_tool;
    
    %     tool_V_tool = Adjoint(inv(sensor_T_tool)) * updated_user_V_user;
    %
    %     base_V_tool = base_rot6_tool * tool_V_tool;
    
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
    
    
    
    
    
    % vel_msg.Linear.X = limit_speed(vel_msg.Linear.X,0.001,0.02);
    % vel_msg.Linear.Y = limit_speed(vel_msg.Linear.Y,0.001,0.02);
    % vel_msg.Linear.Z = limit_speed(vel_msg.Linear.Z,0.001,0.02);
    % vel_msg.Angular.X = limit_speed(vel_msg.Angular.X,0.001,0.1);
    % vel_msg.Angular.Y = limit_speed(vel_msg.Angular.Y,0.001,0.1);
    % vel_msg.Angular.Z = limit_speed(vel_msg.Angular.Z,0.001,0.1);
    
    
    send(vel_pub,vel_msg);
    count_number = count_number+1;
end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

end