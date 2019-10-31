function [Vt] = veltfcam2tool0(cVWc,bRt,tRc,cPtc)
%Function is used to transfer velocity between camera and tool0. 
%Notation description
%tquatc: [x,y,z,w] between tool0 and camera w.r.t tool0 frame.
%Vt: velocity and angular vel of tool0 w.r.t base frame.
%bVt: velocity of tool0 w.r.t base frame. 
%cVc: velocity of camera w.r.t camera frame. 
cVc = cVWc(1:3);
cWc = cVWc(4:6);
%Original vel tf formula: cWc = (tRc)'*tWt
tWt = tRc*cWc;
%Original ang tf formula: cVc = cross(cWc, tPtc)+tRc'*tVt
tVt = tRc*(cVc-cross(cWc,cPtc));
bVt = bRt*tVt;
bWt = bRt*tWt;
Vt = [bVt;bWt];
end