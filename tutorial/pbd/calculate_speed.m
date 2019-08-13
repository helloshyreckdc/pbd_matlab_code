clear;clc;
% clear calculate_speed_node;
format long
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

p = 1.5;  % pid

% used to judge stable
preX = 0;
preY = 0;
preZ = 0;
preQ = current_pose_data.Rotation;
preRotm = quat2rotm([preQ.W preQ.X preQ.Y preQ.Z]);
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
    % control law
    vel_msg.Linear.X = p*(refX - curX);
    vel_msg.Linear.Y = p*(refY - curY);
    vel_msg.Linear.Z = p*(refZ - curZ);
    vel_msg.Angular.X = 0;
    vel_msg.Angular.Y = 0;
    vel_msg.Angular.Z = 0;
    
    %     % angular vel
    %     Oldquat = [current_pose_data.Rotation.W,current_pose_data.Rotation.X,current_pose_data.Rotation.Y,current_pose_data.Rotation.Z];
    %     Newquat = [ref_pose_data.Rotation.W,ref_pose_data.Rotation.X,ref_pose_data.Rotation.Y,ref_pose_data.Rotation.Z];
    %
    %     [wx,wy,wz] = Quat2EulerVel(Oldquat,Newquat);
    
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
    
    
    % judge stable with distance, if stable for 3 seconds
    linear_dis = sqrt((preX - curX)^2+(preY - curY)^2+(preZ - curZ)^2) 
    angular_dis = norm(preRotm-curRotm,'fro')
    ref_distance = sqrt((refX - curX)^2+(refY - curY)^2+(refZ - curZ)^2);
    if linear_dis < 0.001 && angular_dis<0.02
        count = count+1;  
    else
        rosparam('set','/robot_stable',false);
        count = 0;
    end
    
    if count > 150
        rosparam('set','/robot_stable',true);
    end
    
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

function [] = refCB(~,message)
global ref_pose_data
ref_pose_data = message;
end

function limited_speed = limit_speed(original_speed,max_speed)
if original_speed > max_speed
    limited_speed = max_speed;
else
    limited_speed = original_speed;
end
end

