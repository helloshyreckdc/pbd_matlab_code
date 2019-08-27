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
preX = pre_pose_data.Translation.X;
preY = pre_pose_data.Translation.Y;
preZ = pre_pose_data.Translation.Z;
preQ = pre_pose_data.Rotation;   % quaternion value;
preRotm = quat2rotm([preQ.W preQ.X preQ.Y preQ.Z]);
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
%         disp("stable")
    end
    
    % judge robot in goal, if stable for 3 seconds set /robot_in_goal true
    diff_largest_element = max(abs(refRotm - curRotm),[],'all');
    ref_distance = sqrt((refX - curX)^2+(refY - curY)^2+(refZ - curZ)^2);
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