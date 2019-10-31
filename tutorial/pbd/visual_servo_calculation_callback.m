function visual_servo_calculation_callback(~, ~, handles)

global sr300_pose_data
global current_pose_data
global sr300_vel_data
visual_servo_vel_msg = handles.visual_servo_vel_msg;
visual_servo_vel_pub = handles.visual_servo_vel_pub;



% get value
sr300_trans = sr300_pose_data(1:3);
sr300Rotm = quat2rotm(sr300_pose_data(4:7)');
cur_trans = current_pose_data(1:3);
curRotm = quat2rotm(current_pose_data(4:7)');
sr300Rotm'*(sr300_trans-cur_trans);
vt = veltfcam2tl02nd(sr300_vel_data,sr300Rotm,(sr300_trans-cur_trans));
% vt = veltfcam2tool0(sr300_vel_data,curRotm,curRotm'*sr300Rotm,sr300Rotm'*(sr300_trans-cur_trans));
visual_servo_vel_msg.Linear.X = vt(1);
visual_servo_vel_msg.Linear.Y = vt(2);
visual_servo_vel_msg.Linear.Z = vt(3);
visual_servo_vel_msg.Angular.X = vt(4);
visual_servo_vel_msg.Angular.Y = vt(5);
visual_servo_vel_msg.Angular.Z = vt(6);

% speed limit
visual_servo_vel_msg.Linear.X = limit_speed(visual_servo_vel_msg.Linear.X,0.1);
visual_servo_vel_msg.Linear.Y = limit_speed(visual_servo_vel_msg.Linear.Y,0.1);
visual_servo_vel_msg.Linear.Z = limit_speed(visual_servo_vel_msg.Linear.Z,0.1);
visual_servo_vel_msg.Angular.X = limit_speed(visual_servo_vel_msg.Angular.X,0.1);
visual_servo_vel_msg.Angular.Y = limit_speed(visual_servo_vel_msg.Angular.Y,0.1);
visual_servo_vel_msg.Angular.Z = limit_speed(visual_servo_vel_msg.Angular.Z,0.1);


send(visual_servo_vel_pub,visual_servo_vel_msg);

end

function limited_speed = limit_speed(original_speed,max_speed)
if original_speed > max_speed
    limited_speed = max_speed;
elseif original_speed < -max_speed;
    limited_speed = -max_speed;
else
    limited_speed = original_speed;
end
end