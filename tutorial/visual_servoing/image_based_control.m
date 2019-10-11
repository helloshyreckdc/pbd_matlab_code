cam = CentralCamera('default');
P = [1 1 5]';  % world point
p0 = cam.project(P) % image point
px = cam.project(P,'pose',SE3(0.1,0,0)) % move camera along x