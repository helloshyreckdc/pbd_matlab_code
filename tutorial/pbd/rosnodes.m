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
global current_force_data  % force measured in frame ati_sensor 
global ati_pose_data       % frame base to frame ati_sensor 
current_pose_data = rosmessage('geometry_msgs/Transform');
pre_pose_data = rosmessage('geometry_msgs/Transform');
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


% % Create two service servers for the '/add' and '/reply' services
% srv1 = robotics.ros.ServiceServer(node_3,'/add', 'roscpp_tutorials/TwoInts');
% srv2 = robotics.ros.ServiceServer(node_3,'/reply', 'std_srvs/Empty', @exampleHelperROSEmptyCallback);

% % used to judge stable
% robot_in_goal_count = 0;
% robot_stable_count = 0;
% preX = 0;
% preY = 0;
% preZ = 0;
% preQ = current_pose_data.Rotation;
% preRotm = quat2rotm([preQ.W preQ.X preQ.Y preQ.Z]);


% Create a timer for publishing messages and assign appropriate handles
% The timer will call exampleHelperROSSimTimer at a rate of 10 Hz.
speed_calculation_handle.vel_pub = vel_pub;
speed_calculation_handle.vel_msg = vel_msg;

% record_gravity_seq_handle.scanPub = scanPub;
% record_gravity_seq_handle.scanPubmsg = scanPubmsg;

% 50hz
global loop_rate_hz
loop_rate_hz = 50;
speed_calculation_timer = ExampleHelperROSTimer(1/loop_rate_hz, {@speed_calculation_callback,speed_calculation_handle});
stable_check_timer = ExampleHelperROSTimer(1/loop_rate_hz, {@stable_check_callback});






function [] = currCB(~,message)
global current_pose_data
global pre_pose_data
pre_pose_data = current_pose_data;
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

