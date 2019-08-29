function stable_check_callback(~, ~)

global loop_rate_hz
global ref_pose_data
global current_pose_data
global pre_pose_data
persistent robot_stable_count robot_in_goal_count
if isempty(robot_stable_count)
    robot_stable_count = 0;
end
if isempty(robot_in_goal_count)
    robot_in_goal_count = 0;
end


% get value
% get value
ref_trans = ref_pose_data(1:3);
refRotm = quat2rotm(ref_pose_data(4:7)');   % quaternion value;  w x y z
cur_trans = current_pose_data(1:3);
curRotm = quat2rotm(current_pose_data(4:7)');
pre_trans = pre_pose_data(1:3);
preRotm = quat2rotm(pre_pose_data(4:7)');


    % judge stable, if stable for 3 seconds set /robot_stable true
    linear_dis = norm(cur_trans - pre_trans);
    angular_dis = max(abs(preRotm - curRotm),[],'all');
    if linear_dis < 0.001 && angular_dis<0.001
        robot_stable_count = robot_stable_count+1;
    else
        rosparam('set','/robot_stable',false);
        robot_stable_count = 0;
    end
    
    if robot_stable_count > 3*loop_rate_hz
        rosparam('set','/robot_stable',true);
%         disp("stable")
    end
    
    % judge robot in goal, if stable for 3 seconds set /robot_in_goal true
    diff_largest_element = max(abs(refRotm - curRotm),[],'all');
    ref_distance = norm(ref_trans - cur_trans);
    if ref_distance < 0.001 && diff_largest_element<0.001
        robot_in_goal_count = robot_in_goal_count+1;
    else
        rosparam('set','/robot_in_goal',false);
        robot_in_goal_count = 0;
    end
    
    if robot_in_goal_count > 3*loop_rate_hz
        rosparam('set','/robot_in_goal',true);
%         disp("in goal")
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



end