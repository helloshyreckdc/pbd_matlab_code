function speed_calculation_callback(~, ~, handles)

global ref_pose_data
global current_pose_data
vel_msg = handles.vel_msg;
vel_pub = handles.vel_pub;
% pid for control law
p = 1.5;  % pid


% get value
ref_trans = ref_pose_data(1:3);
refRotm = quat2rotm(ref_pose_data(4:7)');
cur_trans = current_pose_data(1:3);
curRotm = quat2rotm(current_pose_data(4:7)');



% control law
diff_trans = ref_trans - cur_trans;
vel_msg.Linear.X = p*diff_trans(1);
vel_msg.Linear.Y = p*diff_trans(2);
vel_msg.Linear.Z = p*diff_trans(3);
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
vel_msg.Linear.X = limit_speed(vel_msg.Linear.X,0.0001,0.1);
vel_msg.Linear.Y = limit_speed(vel_msg.Linear.Y,0.0001,0.1);
vel_msg.Linear.Z = limit_speed(vel_msg.Linear.Z,0.0001,0.1);
vel_msg.Angular.X = limit_speed(vel_msg.Angular.X,0.0001,0.1);
vel_msg.Angular.Y = limit_speed(vel_msg.Angular.Y,0.0001,0.1);
vel_msg.Angular.Z = limit_speed(vel_msg.Angular.Z,0.0001,0.1);


send(vel_pub,vel_msg);


end


