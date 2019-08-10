clear;clc;
% clear calculate_speed_node;
calculate_speed_node = robotics.ros.Node('/calculate_speed');


global current_pose_data
global ref_pose_data
current_pose_data = rosmessage('geometry_msgs/Transform');
ref_pose_data = rosmessage('geometry_msgs/Transform');
current_pose = rossubscriber('/current_pose','geometry_msgs/Transform',@currCB);
ref_pose = rossubscriber('/ref_traj','geometry_msgs/Transform',@refCB);



vel_pub = rospublisher('/ur/velocity', 'geometry_msgs/Twist');
vel_msg = rosmessage(vel_pub);

pause(2);

rate = robotics.ros.Rate(calculate_speed_node,50);
count = 0;

while(1)
    %     reset_mat_value = rosparam("get",'reset_mat');
    %     if reset_mat_value    % to solve the problem of matlab ros topic cache
    %         clear vel_pub;
    %         vel_pub = rospublisher('/ur/velocity', 'geometry_msgs/Twist');
    %     else
    
    %         current_pose_data = receive(current_pose);
    %         ref_pose_data = receive(ref_pose);
    
    
    
    count = count+1
    X = ref_pose_data.Translation.X - current_pose_data.Translation.X;
    Y = ref_pose_data.Translation.Y - current_pose_data.Translation.Y;
    Z = 2*(ref_pose_data.Translation.Z - current_pose_data.Translation.Z)
    refz = ref_pose_data.Translation.Z
    curz = current_pose_data.Translation.Z
    vel_msg.Linear.Z = Z;
    
    send(vel_pub,vel_msg);
    %     end
    waitfor(rate);
end


function [] = currCB(~,message)
global current_pose_data
current_pose_data = message;
end

function [] = refCB(~,message)
global ref_pose_data
ref_pose_data = message;
end

