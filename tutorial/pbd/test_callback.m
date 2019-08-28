function []=test_callback(~,~,handles)

global raw_force_data


handles.calibrated_force_msg.Wrench.Force.X = raw_force_data(1);
handles.calibrated_force_msg.Wrench.Force.Y = raw_force_data(2);
handles.calibrated_force_msg.Wrench.Force.Z = raw_force_data(3);
handles.calibrated_force_msg.Wrench.Torque.X = raw_force_data(4);
handles.calibrated_force_msg.Wrench.Torque.Y = raw_force_data(5);
handles.calibrated_force_msg.Wrench.Torque.Z = raw_force_data(6);
send(handles.calibrated_force_pub,handles.calibrated_force_msg);
end