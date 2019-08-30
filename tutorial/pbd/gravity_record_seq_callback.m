function gravity_record_seq_callback(~, ~)

global ati_pose_data averaged_raw_force_data averaged_calibrated_force_data
global force_sensor_output atiRotm_matrix max_gravity_seq_columns
persistent seq_index;
if isempty(seq_index)
    seq_index = 1;
end

clear_gravity_seq = rosparam('get','/clear_gravity_seq');
if clear_gravity_seq
    force_sensor_output = zeros(6,max_gravity_seq_columns);
    atiRotm_matrix = zeros(3,3*max_gravity_seq_columns);
    seq_index = 1;
    rosparam('set','/clear_gravity_seq',false);
end


gravity_record_seq = rosparam('get','/gravity_record_seq');  %record sensor output and rotation matrix
if gravity_record_seq
    
    atiRotm = quat2rotm(ati_pose_data(4:7)');
    
    choose_raw = rosparam('get','/choose_raw_force_data_in_gravity_record_seq');
    if choose_raw
        wrench_vector = averaged_raw_force_data;
    else
        wrench_vector = averaged_calibrated_force_data;
    end
    
    %judge if two poses are too near
    if seq_index > 1
        previous_wrench = force_sensor_output(:,seq_index-1);
        wrench_diff = norm(wrench_vector-previous_wrench);
    end
    % if too near, omit
    if (seq_index == 1) || (wrench_diff > 0.2)
        force_sensor_output(:,seq_index) = wrench_vector;
        atiRotm_matrix(:,(seq_index*3-2):(3*seq_index)) = atiRotm;
        seq_index = seq_index + 1;
    end
    
    if seq_index > max_gravity_seq_columns
        seq_index = 1;
    end
    
    rosparam('set','/gravity_record_seq',false);
end


end