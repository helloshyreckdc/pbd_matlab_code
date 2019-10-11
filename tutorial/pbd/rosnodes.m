%exampleHelperROSCreateSampleNetwork Create an example ROS network
%   This network is representative of a realistic ROS network and is used 
%   throughout the MATLAB ROS examples. It features three nodes,
%   two publishers, three subscribers, and two services. Additionally we use a
%   timer to control publishing of ROS messages over the network.
%
%   See also exampleHelperROSShutDownSampleNetwork, exampleHelperROSSimTimer,
%   exampleHelperROSLoadRanges

%   Copyright 2014-2016 The MathWorks, Inc.

% rosshutdown
% rosinit('http://shyreckdc-16-PC:11311')
% clear;clc;

format long

% Initialize three nodes (in addition to the global MATLAB node that is 
% created through rosinit). Note that the nodes will try to connect to the
% ROS master at 'localhost'. If you are connecting to an external master,
% you will have to use its IP address or hostname.
masterHost = 'http://shyreckdc-16-PC:11311';
speed_calculation = robotics.ros.Node('speed_calculation', masterHost);
record_gravity_seq = robotics.ros.Node('record_gravity_seq', masterHost);


global current_pose_data   % frame base to tool0
global pre_pose_data   % frame base to tool0
global ref_pose_data       % frame base to tool0
global averaged_raw_force_data  % force measured in frame ati_sensor 
global calibrated_force_data  % force measured in frame ati_sensor 
global ati_pose_data       % frame base to frame ati_sensor 
global sr300_data

current_pose_data = [0;0;0;1;0;0;0];
pre_pose_data = [0;0;0;1;0;0;0];
ref_pose_data = [0;0;0;1;0;0;0];
averaged_raw_force_data = [0;0;0;0;0;0];
averaged_calibrated_force_data = [0;0;0;0;0;0];
ati_pose_data = [0;0;0;1;0;0;0];
sr300_data = zeros(480,640,3);

current_pose = rossubscriber('/current_pose','geometry_msgs/Transform',@currCB);
ref_pose = rossubscriber('/ref_traj','geometry_msgs/Transform',@refCB);
averaged_calibrated_force = rossubscriber('/averaged_calibrated_force','geometry_msgs/WrenchStamped',@averaged_calibrated_forceCB);
averaged_raw_force = rossubscriber('/averaged_raw_force','geometry_msgs/WrenchStamped',@averaged_raw_forceCB);
ati_pose = rossubscriber('/ati_current_pose','geometry_msgs/Transform',@ati_poseCB);
% sr300 = rossubscriber('/xtion/rgb/image_raw',@sr300CB);
sr300 = rossubscriber('/sr300/color/image_rect_color',@sr300CB);

vel_pub = rospublisher('/ur/velocity', 'geometry_msgs/Twist');
vel_msg = rosmessage(vel_pub);
% gravity_compensated_force_pub = rospublisher('/gravity_compensated_wrench', 'geometry_msgs/WrenchStamped');
% gravity_compensated_force_msg = rosmessage(gravity_compensated_force_pub);

 
% % Create two service servers for the '/add' and '/reply' services
% srv1 = robotics.ros.ServiceServer(node_3,'/add', 'roscpp_tutorials/TwoInts');
% srv2 = robotics.ros.ServiceServer(node_3,'/reply', 'std_srvs/Empty', @exampleHelperROSEmptyCallback);


% for speed control
speed_calculation_handle.vel_pub = vel_pub;
speed_calculation_handle.vel_msg = vel_msg;


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
global force_sensor_output atiRotm_matrix max_gravity_seq_columns 
global mass_matrix sensor_bias
sensor_bias = zeros(6,1);
mass_matrix = zeros(6,3);
max_gravity_seq_columns = 10;
force_sensor_output = zeros(6,max_gravity_seq_columns);
atiRotm_matrix = zeros(3,3*max_gravity_seq_columns);


pause(5)  % wait for initialize

visual_servo_timer = ExampleHelperROSTimer(1/loop_rate_hz, {@visual_servo_callback});
speed_calculation_timer = ExampleHelperROSTimer(1/loop_rate_hz, {@speed_calculation_callback,speed_calculation_handle});
stable_check_timer = ExampleHelperROSTimer(1/loop_rate_hz, {@stable_check_callback});
gravity_record_seq_timer = ExampleHelperROSTimer(50/loop_rate_hz, {@gravity_record_seq_callback});
estimate_M_G_timer = ExampleHelperROSTimer(50/loop_rate_hz, {@estimate_M_G_callback});






function [] = currCB(~,message)
global current_pose_data
global pre_pose_data
pre_pose_data = current_pose_data;
current_pose_data = [message.Translation.X;message.Translation.Y;message.Translation.Z;...
    message.Rotation.W;message.Rotation.X;message.Rotation.Y;message.Rotation.Z];  %quaternion w,x,y,z
end

function [] = ati_poseCB(~,message)
global ati_pose_data
ati_pose_data = [message.Translation.X;message.Translation.Y;message.Translation.Z;...
    message.Rotation.W;message.Rotation.X;message.Rotation.Y;message.Rotation.Z];  %quaternion w,x,y,z
end

function [] = refCB(~,message)
global ref_pose_data
ref_pose_data = [message.Translation.X;message.Translation.Y;message.Translation.Z;...
    message.Rotation.W;message.Rotation.X;message.Rotation.Y;message.Rotation.Z];  %quaternion w,x,y,z
end

function [] = sr300CB(~,message)
global sr300_data
sr300_data = readImage(message);
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
