function gravity_record_seq_callback(~, ~)

global ati_pose_data averaged_raw_force_data
global force_sensor_output atiRotm_matrix max_gravity_seq_columns
persistent seq_index;
if isempty(seq_index)
    seq_index = 1;
end

gravity_record_seq = rosparam('get','/gravity_record_seq');  %record sensor output and rotation matrix
if gravity_record_seq
    
    atiRotm = quat2rotm(ati_pose_data(4:7));
    wrench_vector = averaged_raw_force_data;
    
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
        rosparam('set','/gravity_record_seq',false);
        seq_index = 1;
    end
end


end