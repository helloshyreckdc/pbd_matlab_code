node = robotics.ros.Node('/talker');

chatterpub = rospublisher('/chatter', 'std_msgs/String');
pause(2);
chattermsg = rosmessage(chatterpub);
chattermsg.Data = 'hello world';

rate = robotics.ros.Rate(node,1);

while(1) 
    send(chatterpub,chattermsg);
    waitfor(rate);
end