function estimate_M_G_callback(~, ~)

global force_sensor_output atiRotm_matrix mass_matrix sensor_bias

calculate_compensate = rosparam('get','/calculate_compensate');
if calculate_compensate
    rosparam('set','/gravity_record_seq',false);
    % delete all zeros columns
    force_sensor_output(:,all(force_sensor_output==0,1))=[];
    atiRotm_matrix(:,all(atiRotm_matrix==0,1))=[];
    % calculate compensate
    [e_S0,calibrated_output,M,G,e_alpha,e_beta,e_m,e_r]=estimate_M_G(force_sensor_output,atiRotm_matrix)
    mass_matrix = M*e_m;
    sensor_bias = e_S0;
    e_Rw2b = rotz(0)*roty(e_beta)*rotx(e_alpha); 
%     rosparam('set','/gripper_mass_matrix',false);
    rosparam('set','/sensor_bias',num2cell(sensor_bias));
    rosparam('set','/mass_matrix',num2cell(reshape(mass_matrix,1,[])));
    rosparam('set','/Rworld2base',num2cell(reshape(e_Rw2b,1,[])));
    % clear cache
    rosparam('set','/calculate_compensate',false);
    %         force_sensor_output = zeros(6,max_columns);
    %         atiRotm_matrix = zeros(3,3*max_columns);

end

end