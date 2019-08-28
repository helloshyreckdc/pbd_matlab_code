function estimate_M_G_callback(~, ~)

global force_sensor_output atiRotm_matrix mass_matrix sensor_bias

calculate_compensate = rosparam('get','/calculate_compensate');
if calculate_compensate
    rosparam('set','/gravity_record_seq',false);
    % delete all zeros columns
    force_sensor_output(:,all(force_sensor_output==0,1))=[];
    atiRotm_matrix(:,all(atiRotm_matrix==0,1))=[];
    % calculate compensate
    [e_S0,calibrated_output,M,G,e_alpha,e_beta]=estimate_M_G(force_sensor_output,atiRotm_matrix)
    mass_matrix = M;
    sensor_bias = e_S0;
    % clear cache
    rosparam('set','/calculate_compensate',false);
    %         force_sensor_output = zeros(6,max_columns);
    %         atiRotm_matrix = zeros(3,3*max_columns);

end

end