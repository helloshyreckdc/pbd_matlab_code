function [wx,wy,wz] = Quat2EulerVel(Oldquat,Newquat)
%Attention!!! Old and new quats should be w,x,y,z
OldEulerZYX = quat2eul(Oldquat);
NewEulerZYX = quat2eul(Newquat);
dt = 2;
phi = NewEulerZYX(1);
theta = NewEulerZYX(2);
sai = NewEulerZYX(3);
dotphi = (phi-OldEulerZYX(1))/dt;
dottheta = (theta-OldEulerZYX(2))/dt;
dotsai = (sai-OldEulerZYX(3))/dt;
wx = dotphi - sin(theta)*dotsai;
wy = cos(phi)*dottheta+cos(theta)*sin(phi)*dotsai;
wz = -sin(phi)*dottheta+cos(theta)*cos(phi)*dotsai;
end

