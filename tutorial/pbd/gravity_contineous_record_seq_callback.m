function gravity_contineous_record_seq_callback(~, ~, ur_script_handle)

global ati_pose_data averaged_raw_force_data averaged_calibrated_force_data gravity_compensated_force
global raw_force_sensor_output force_sensor_output atiRotm_matrix max_gravity_seq_columns
global current_joint_data loop_rate_hz
global force_sensor_gravity_compensated_output
persistent seq_index;
if isempty(seq_index)
    seq_index = 1;
end

ur_script_pub = ur_script_handle.ur_script_pub;
ur_script_msg = ur_script_handle.ur_script_msg;

callback_time = 5/loop_rate_hz; % 0.1s   % 10 hz

joint_5 = current_joint_data(5)*180/pi;
start_recording_angle = 30;
end_recording_angle = 150;

move_robot_and_record = rosparam('get','/move_robot_and_record');
if move_robot_and_record
    % move joint 4 and 5 of ur with 2 degrees per second
    % joint 4 [20 160] degrees, record [30 150] degrees to avoid
    % vibration.  joint 4 [-160 -20] degrees, record [-150 -30] degrees 
    ur_script_msg.Data = "speedj([0,0,0,0.0349,0.0349,0],0.5,70.05)";
%     ur_script_msg.Data = "speedj([0,0,0,0.017453292519943,0.017453292519943,0],0.5,140.05)";
    send(ur_script_pub,ur_script_msg);
    rosparam('set','/move_robot_and_record',false);
end


if(norm(joint_5-start_recording_angle) < 2*callback_time) % 2 degrees/s
     rosparam('set','/gravity_record_seq',true);
     disp("start");
end

if(norm(joint_5-end_recording_angle) < 2*callback_time)
     rosparam('set','/gravity_record_seq',false);
     disp("end");
end

clear_gravity_seq = rosparam('get','/clear_gravity_seq');
if clear_gravity_seq
    force_sensor_gravity_compensated_output = zeros(6,max_gravity_seq_columns);
    force_sensor_output = zeros(6,max_gravity_seq_columns);
    raw_force_sensor_output = zeros(6,max_gravity_seq_columns);
    atiRotm_matrix = zeros(3,3*max_gravity_seq_columns);
    seq_index = 1;
    rosparam('set','/clear_gravity_seq',false);
end


gravity_record_seq = rosparam('get','/gravity_record_seq');  %record sensor output and rotation matrix
if gravity_record_seq
    
    atiRotm = quat2rotm(ati_pose_data(4:7)');
    wrench_vector_compensated = gravity_compensated_force;
    raw_wrench_vector = averaged_raw_force_data;
    wrench_vector = averaged_calibrated_force_data;
    
    force_sensor_gravity_compensated_output(:,seq_index) = wrench_vector_compensated;
    force_sensor_output(:,seq_index) = wrench_vector;
    raw_force_sensor_output(:,seq_index) = raw_wrench_vector;
    atiRotm_matrix(:,(seq_index*3-2):(3*seq_index)) = atiRotm;
    seq_index = seq_index + 1;
    
    
    if seq_index > max_gravity_seq_columns
        seq_index = 1;
    end
    
end


end