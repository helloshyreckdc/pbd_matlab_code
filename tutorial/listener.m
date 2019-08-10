clear node 
node = robotics.ros.Node('/listener');
current_pose = rossubscriber('current_pose')
current_pose_data = receive(current_pose,1)
showdetails(current_pose_data)
current_pose_data.Translation.Z
% rate = robotics.ros.Rate(node,50);
% i = 1
% y = zeros(1,1001);
% x = 0:0.001:1;
% while(1) 
%     current_pose_data = receive(current_pose,1)
%     y(i) = current_pose_data.Translation.Z;
%     i = i+1;
%     if i == 1000
%         break;
%     end
%     waitfor(rate);
% end
% 
% plot(x,y)

