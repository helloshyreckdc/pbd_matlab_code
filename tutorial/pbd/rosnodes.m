%exampleHelperROSCreateSampleNetwork Create an example ROS network
%   This network is representative of a realistic ROS network and is used 
%   throughout the MATLAB ROS examples. It features three nodes,
%   two publishers, three subscribers, and two services. Additionally we use a
%   timer to control publishing of ROS messages over the network.
%
%   See also exampleHelperROSShutDownSampleNetwork, exampleHelperROSSimTimer,
%   exampleHelperROSLoadRanges

%   Copyright 2014-2016 The MathWorks, Inc.

rosshutdown
rosinit('http://shyreckdc-16-PC:11311')
clear;clc;

format long

% Initialize three nodes (in addition to the global MATLAB node that is 
% created through rosinit). Note that the nodes will try to connect to the
% ROS master at 'localhost'. If you are connecting to an external master,
% you will have to use its IP address or hostname.
masterHost = 'http://shyreckdc-16-PC:11311';
speed_calculation = robotics.ros.Node('speed_calculation', masterHost);
record_gravity_seq = robotics.ros.Node('record_gravity_seq', masterHost);

global current_joint_data % from joint state
global current_vel_data  % tool velocity in base frame
global pre_vel_data
global current_pose_data   % frame base to tool0
global pre_pose_data   % frame base to tool0
global ref_pose_data       % frame base to tool0
global sr300_pose_data     % frame base to sr300 camera frame
global averaged_raw_force_data  % force measured in frame ati_sensor 
global gravity_compensated_force  % force measured in frame ati_sensor 
global ati_pose_data       % frame base to frame ati_sensor 
global pre_ati_pose_data       % frame base to frame ati_sensor 
global sr300_data
global sr300_vel_data

current_joint_data = zeros(6,1);
current_pose_data = [0;0;0;1;0;0;0];
pre_pose_data = [0;0;0;1;0;0;0];
ref_pose_data = [0;0;0;1;0;0;0];
sr300_pose_data = [0;0;0;1;0;0;0];
averaged_raw_force_data = [0;0;0;0;0;0];
gravity_compensated_force = [0;0;0;0;0;0];
current_vel_data = [0;0;0;0;0;0];
pre_vel_data = [0;0;0;0;0;0];
averaged_calibrated_force_data = [0;0;0;0;0;0];
ati_pose_data = [0;0;0;1;0;0;0];
pre_ati_pose_data = [0;0;0;1;0;0;0];
sr300_data = zeros(480,640,3);
sr300_vel_data = zeros(6,1);

current_joint = rossubscriber('/joint_states','sensor_msgs/JointState',@curr_jointCB);
current_vel = rossubscriber('/tool_velocity','geometry_msgs/TwistStamped',@curr_velCB);
current_pose = rossubscriber('/current_pose','geometry_msgs/Transform',@currCB);
ref_pose = rossubscriber('/ref_traj','geometry_msgs/Transform',@refCB);
sr300_pose = rossubscriber('/sr300_pose','geometry_msgs/Transform',@sr300_poseCB);
averaged_calibrated_force = rossubscriber('/averaged_calibrated_force','geometry_msgs/WrenchStamped',@averaged_calibrated_forceCB);
averaged_raw_force = rossubscriber('/averaged_raw_force','geometry_msgs/WrenchStamped',@averaged_raw_forceCB);
gravity_compensated_force_sub = rossubscriber('/gravity_compensated_force','geometry_msgs/WrenchStamped',@gravity_compensated_forceCB);
ati_pose = rossubscriber('/ati_current_pose','geometry_msgs/Transform',@ati_poseCB);
% sr300 = rossubscriber('/xtion/rgb/image_raw',@sr300CB);
sr300 = rossubscriber('/sr300/color/image_rect_color','sensor_msgs/Image',@sr300CB);
sr300_vel = rossubscriber('sr300_vel','geometry_msgs/Twist',@sr300_velCB);

vel_pub = rospublisher('/ur/velocity', 'geometry_msgs/Twist');
vel_msg = rosmessage(vel_pub);

% topic for sending ur script
ur_script_pub = rospublisher('/ur_driver/URScript', 'std_msgs/String');
ur_script_msg = rosmessage(ur_script_pub);

% gravity_compensated_force_pub = rospublisher('/gravity_compensated_wrench', 'geometry_msgs/WrenchStamped');
% gravity_compensated_force_msg = rosmessage(gravity_compensated_force_pub);


% for visual servo
visual_servo_vel_pub = rospublisher('/visual_servo/ur/velocity', 'geometry_msgs/Twist');
visual_servo_vel_msg = rosmessage(visual_servo_vel_pub);
visual_servo_calculation_handle.visual_servo_vel_pub = visual_servo_vel_pub;
visual_servo_calculation_handle.visual_servo_vel_msg = visual_servo_vel_msg;

% % Create two service servers for the '/add' and '/reply' services
% srv1 = robotics.ros.ServiceServer(node_3,'/add', 'roscpp_tutorials/TwoInts');
% srv2 = robotics.ros.ServiceServer(node_3,'/reply', 'std_srvs/Empty', @exampleHelperROSEmptyCallback);



% for speed control
speed_handle.vel_pub = vel_pub;
speed_handle.vel_msg = vel_msg;

% for ur script
ur_script_handle.ur_script_pub = ur_script_pub;
ur_script_handle.ur_script_msg = ur_script_msg;


% record_gravity_seq_handle.scanPub = scanPub;
% record_gravity_seq_handle.scanPubmsg = scanPubmsg;

% 50hz
global loop_rate_hz
loop_rate_hz = 50;

% for visual servo
rosparam('set','/record_image_template',false);

% for gravity compensation
rosparam('set','/clear_gravity_seq',false);
rosparam('set','/gravity_record_seq',false);
rosparam('set','/calculate_compensate',false);
%for_calibrating_sensor, if the sensor is calibrated, calibrated data is
%used
rosparam('set','/choose_raw_force_data_in_gravity_record_seq',false);

% for assembly
rosparam('set','/assembly_demo',false);  % start demonstration
rosparam('set','/assembly_learning',false); % processing data and generate strategy
rosparam('set','/assembly_exe',false); % start execution



% move ur and record force sensor output
rosparam('set','/move_robot_and_record',false);


global raw_force_sensor_output force_sensor_output atiRotm_matrix max_gravity_seq_columns 
global mass_matrix sensor_bias
global force_sensor_gravity_compensated_output
sensor_bias = zeros(6,1);
mass_matrix = zeros(6,3);
% % for discrete record of force data
% max_gravity_seq_columns = 20;
% for contineous record of force data
max_gravity_seq_columns = 1200;
force_sensor_gravity_compensated_output = zeros(6,max_gravity_seq_columns);
force_sensor_output = zeros(6,max_gravity_seq_columns);
raw_force_sensor_output = zeros(6,max_gravity_seq_columns);
atiRotm_matrix = zeros(3,3*max_gravity_seq_columns);


pause(5)  % wait for initialize

% visual_servo_timer = ExampleHelperROSTimer(1/loop_rate_hz, {@visual_servo_callback});

% visual_servo_calculation_timer = ExampleHelperROSTimer(1/loop_rate_hz, {@visual_servo_calculation_callback,visual_servo_calculation_handle});

% speed_calculation_timer = ExampleHelperROSTimer(1/loop_rate_hz, {@speed_calculation_callback,speed_calculation_handle});

% stable_check_timer = ExampleHelperROSTimer(1/loop_rate_hz, {@stable_check_callback});

% gravity_record_seq_timer = ExampleHelperROSTimer(50/loop_rate_hz, {@gravity_record_seq_callback});
gravity_contineous_record_seq_timer = ExampleHelperROSTimer(5/loop_rate_hz, {@gravity_contineous_record_seq_callback,ur_script_handle});
% estimate_M_G_timer = ExampleHelperROSTimer(50/loop_rate_hz, {@estimate_M_G_callback});

% single_axis_admittance_control_timer = ExampleHelperROSTimer(1/loop_rate_hz, {@single_axis_admittance_control_callback,speed_handle});
% multi_axis_admittance_control_timer = ExampleHelperROSTimer(1/loop_rate_hz, {@multi_axis_admittance_control_callback,speed_handle});

% assembly_control_timer = ExampleHelperROSTimer(1/loop_rate_hz, {@assembly_control_callback,speed_handle});


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% find a bug for persistent variable in ExampleHelperROSTimer
% if the screen output of the persistent variable is suppressed and then
% output again, the value of the persistent variable will be recalculated

% The reason is that after a saving operation, all the function variables
% are re-initialized
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

global demo_ati_pose_seq demo_count demo_vel_seq demo_energy_seq demo_force_seq;
global t n;
t=1:500;
n=1;

function [] = currCB(~,message)
global current_pose_data
global pre_pose_data
pre_pose_data = current_pose_data;
current_pose_data = [message.Translation.X;message.Translation.Y;message.Translation.Z;...
    message.Rotation.W;message.Rotation.X;message.Rotation.Y;message.Rotation.Z];  %quaternion w,x,y,z
end

function [] = curr_velCB(~,message)
global current_vel_data
global pre_vel_data
pre_vel_data = current_vel_data;
current_vel_data = [message.Twist.Linear.X;message.Twist.Linear.Y;message.Twist.Linear.Z;...
    message.Twist.Angular.X;message.Twist.Angular.Y;message.Twist.Angular.Z];  %quaternion w,x,y,z
end

function [] = curr_jointCB(~,message)
global current_joint_data
current_joint_data = message.Position;
end

function [] = ati_poseCB(~,message)
global ati_pose_data
global pre_ati_pose_data
pre_ati_pose_data = ati_pose_data;
ati_pose_data = [message.Translation.X;message.Translation.Y;message.Translation.Z;...
    message.Rotation.W;message.Rotation.X;message.Rotation.Y;message.Rotation.Z];  %quaternion w,x,y,z
end

function [] = refCB(~,message)
global ref_pose_data
ref_pose_data = [message.Translation.X;message.Translation.Y;message.Translation.Z;...
    message.Rotation.W;message.Rotation.X;message.Rotation.Y;message.Rotation.Z];  %quaternion w,x,y,z
end

function [] = sr300_poseCB(~,message)
global sr300_pose_data
sr300_pose_data = [message.Translation.X;message.Translation.Y;message.Translation.Z;...
    message.Rotation.W;message.Rotation.X;message.Rotation.Y;message.Rotation.Z];  %quaternion w,x,y,z
end

function [] = sr300CB(~,message)
global sr300_data
sr300_data = readImage(message);
end

function [] = sr300_velCB(~,message)
global sr300_vel_data
sr300_vel_data = [message.Linear.X;message.Linear.Y;message.Linear.Z;...
    message.Angular.X;message.Angular.Y;message.Angular.Z];
end

function [] = averaged_raw_forceCB(~,message)
global averaged_raw_force_data
averaged_raw_force_data = [message.Wrench.Force.X;message.Wrench.Force.Y;message.Wrench.Force.Z;...
    message.Wrench.Torque.X;message.Wrench.Torque.Y;message.Wrench.Torque.Z];
end

function [] = averaged_calibrated_forceCB(~,message)
global averaged_calibrated_force_data
averaged_calibrated_force_data = [message.Wrench.Force.X;message.Wrench.Force.Y;message.Wrench.Force.Z;...
    message.Wrench.Torque.X;message.Wrench.Torque.Y;message.Wrench.Torque.Z];
end

function [] = gravity_compensated_forceCB(~,message)
global gravity_compensated_force
gravity_compensated_force = [message.Wrench.Force.X;message.Wrench.Force.Y;message.Wrench.Force.Z;...
    message.Wrench.Torque.X;message.Wrench.Torque.Y;message.Wrench.Torque.Z];
end
