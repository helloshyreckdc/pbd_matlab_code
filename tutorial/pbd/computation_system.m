% This node is for design control law,gravity compensation and force
% conversion between different coordinate frames



rosshutdown
rosinit('http://shyreckdc-16-PC:11311')
clear;clc;
format long

computation_system_node = robotics.ros.Node('/computation_system');


global current_pose_data
global ref_pose_data
global current_force_data
global ati_pose_data
current_pose_data = rosmessage('geometry_msgs/Transform');
ref_pose_data = rosmessage('geometry_msgs/Transform');
current_force_data = rosmessage('geometry_msgs/WrenchStamped');
ati_pose_data = rosmessage('geometry_msgs/Transform');
current_pose = rossubscriber('/current_pose','geometry_msgs/Transform',@currCB);
ref_pose = rossubscriber('/ref_traj','geometry_msgs/Transform',@refCB);
current_force = rossubscriber('/netft_data','geometry_msgs/WrenchStamped',@forceCB);
ati_pose = rossubscriber('/ati_current_pose','geometry_msgs/Transform',@ati_poseCB);


vel_pub = rospublisher('/ur/velocity', 'geometry_msgs/Twist');
vel_msg = rosmessage(vel_pub);
force_pub = rospublisher('/force_compensated', 'geometry_msgs/WrenchStamped');
force_msg = rosmessage(force_pub);

% wait for node initialize
pause(2);

% loop rate 50 hz
loop_rate_hz = 50;
rate = robotics.ros.Rate(computation_system_node,loop_rate_hz);

% pid for control law
p = 1.5;  % pid

% used to judge stable
robot_in_goal_count = 0;
robot_stable_count = 0;
preX = 0;
preY = 0;
preZ = 0;
preQ = current_pose_data.Rotation;
preRotm = quat2rotm([preQ.W preQ.X preQ.Y preQ.Z]);

% used to compensate gravity
gravity_count = 0;  % used to lower rate of recording gravity sequence
rosparam('set','/gravity_record_seq',false);
rosparam('set','/calculate_compensate',false);
max_columns = 10;
force_sensor_output = zeros(6,max_columns);
atiRotm_matrix = zeros(3,3*max_columns);
gravity_record_seq_index = 1; % index for output and atiRotm_matrix
while(1)
    
    % get current value
    refX = ref_pose_data.Translation.X;
    refY = ref_pose_data.Translation.Y;
    refZ = ref_pose_data.Translation.Z;
    refQ = ref_pose_data.Rotation;   % quaternion value;
    refRotm = quat2rotm([refQ.W refQ.X refQ.Y refQ.Z]);
    curX = current_pose_data.Translation.X;
    curY = current_pose_data.Translation.Y;
    curZ = current_pose_data.Translation.Z;
    curQ = current_pose_data.Rotation;
    curRotm = quat2rotm([curQ.W curQ.X curQ.Y curQ.Z]);
    atiX = ati_pose_data.Translation.X;
    atiY = ati_pose_data.Translation.Y;
    atiZ = ati_pose_data.Translation.Z;
    atiQ = ati_pose_data.Rotation;
    atiRotm = quat2rotm([atiQ.W atiQ.X atiQ.Y atiQ.Z]);
    Fx = current_force_data.Wrench.Force.X;
    Fy = current_force_data.Wrench.Force.Y;
    Fz = current_force_data.Wrench.Force.Z;
    Mx = current_force_data.Wrench.Torque.X;
    My = current_force_data.Wrench.Torque.Y;
    Mz = current_force_data.Wrench.Torque.Z;
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % This part for robot speed calculation
    
    % control law
    vel_msg.Linear.X = p*(refX - curX);
    vel_msg.Linear.Y = p*(refY - curY);
    vel_msg.Linear.Z = p*(refZ - curZ);
    vel_msg.Angular.X = 0;
    vel_msg.Angular.Y = 0;
    vel_msg.Angular.Z = 0;
    
    
    if(~isnan(det(curRotm)))
        diff_R = refRotm*pinv(curRotm);
        speed_w = p*rotm2axang(diff_R)/1; %here 1 is delta_t
        vel_msg.Angular.X = speed_w(4)*speed_w(1);
        vel_msg.Angular.Y = speed_w(4)*speed_w(2);
        vel_msg.Angular.Z = speed_w(4)*speed_w(3);
    end
    
    % speed limit
    vel_msg.Linear.X = limit_speed(vel_msg.Linear.X,0.1);
    vel_msg.Linear.Y = limit_speed(vel_msg.Linear.Y,0.1);
    vel_msg.Linear.Z = limit_speed(vel_msg.Linear.Z,0.1);
    vel_msg.Angular.X = limit_speed(vel_msg.Angular.X,0.1);
    vel_msg.Angular.Y = limit_speed(vel_msg.Angular.Y,0.1);
    vel_msg.Angular.Z = limit_speed(vel_msg.Angular.Z,0.1);
    
    
    send(vel_pub,vel_msg);
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    
    
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % This part for gravity compensation
    gravity_record_seq = rosparam('get','/gravity_record_seq');  %record sensor output and rotation matrix
    if gravity_record_seq
        gravity_count = gravity_count + 1;
        % lower recording rate in matrix array, 0.5s per record
        if gravity_count > 0.5*loop_rate_hz
            gravity_count = 0;
            force_sensor_output(:,gravity_record_seq_index) = [Fx;Fy;Fz;Mx;My;Mz]
            atiRotm_matrix(:,(gravity_record_seq_index*3-2):(3*gravity_record_seq_index)) = atiRotm
            gravity_record_seq_index = gravity_record_seq_index + 1;
            if gravity_record_seq_index > max_columns
                rosparam('set','/gravity_record_seq',false);
            end
        end
    end
    
    calculate_compensate = rosparam('get','/calculate_compensate');
    if calculate_compensate
        rosparam('set','/gravity_record_seq',false);
        % delete all zeros columns
        force_sensor_output(:,all(force_sensor_output==0,1))=[];
        atiRotm_matrix(:,all(atiRotm_matrix==0,1))=[];
        % calculate compensate
        [e_S0,calibrated_output,M,G,e_alpha,e_beta]=estimate_M_G(force_sensor_output,atiRotm_matrix)
        
        % clear cache 
        rosparam('set','/calculate_compensate',false);
%         force_sensor_output = zeros(6,max_columns);
%         atiRotm_matrix = zeros(3,3*max_columns);
%         gravity_count = 0;
%         gravity_record_seq_index = 1;
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    
    
    
    
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % This part for stable check
    
    % judge stable, if stable for 3 seconds set /robot_stable true
    linear_dis = sqrt((preX - curX)^2+(preY - curY)^2+(preZ - curZ)^2);
    angular_dis = max(abs(preRotm - curRotm),[],'all');
    if linear_dis < 0.001 && angular_dis<0.001
        robot_stable_count = robot_stable_count+1;
    else
        rosparam('set','/robot_stable',false);
        robot_stable_count = 0;
    end
    
    if robot_stable_count > 3*loop_rate_hz
        rosparam('set','/robot_stable',true);
    end
    
    % judge robot in goal, if stable for 3 seconds set /robot_in_goal true
    diff_largest_element = max(abs(refRotm - curRotm),[],'all')
    ref_distance = sqrt((refX - curX)^2+(refY - curY)^2+(refZ - curZ)^2)
    if ref_distance < 0.001 && diff_largest_element<0.001
        robot_in_goal_count = robot_in_goal_count+1;
    else
        rosparam('set','/robot_in_goal',false);
        robot_in_goal_count = 0;
    end
    
    if robot_in_goal_count > 3*loop_rate_hz
        rosparam('set','/robot_in_goal',true);
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    preX = curX;
    preY = curY;
    preZ = curZ;
    preQ = curQ;
    
    waitfor(rate);
end


function [] = currCB(~,message)
global current_pose_data
current_pose_data = message;
end

function [] = ati_poseCB(~,message)
global ati_pose_data
ati_pose_data = message;
end

function [] = refCB(~,message)
global ref_pose_data
ref_pose_data = message;
end

function [] = forceCB(~,message)
global current_force_data
current_force_data = message;
end

function limited_speed = limit_speed(original_speed,max_speed)
if original_speed > max_speed
    limited_speed = max_speed;
else
    limited_speed = original_speed;
end
end

