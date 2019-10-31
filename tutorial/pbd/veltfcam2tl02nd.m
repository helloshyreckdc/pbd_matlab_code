function [Vt] = veltfcam2tl02nd(cVWc,bRc,bPtc) 
cVc = cVWc(1:3);
cWc = cVWc(4:6);

bWt = bRc*cWc;
bVt = bRc*cVc-cross((bRc*cWc),bPtc);

Vt = [bVt;bWt]; 
end