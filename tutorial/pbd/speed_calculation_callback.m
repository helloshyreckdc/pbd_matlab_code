function speed_calculation_callback(~, ~, handles)

global ref_pose_data
global current_pose_data
vel_msg = handles.vel_msg;
vel_pub = handles.vel_pub;
% pid for control law
p = 1.5;  % pid


% get value
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


end