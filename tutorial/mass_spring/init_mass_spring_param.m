k = 1;
b = 1;
m = 1;
A = [0 1;-k/m -b/m];
B = [0;1/m];
C = eye(2);
D = zeros(2,1);
Initial_conditions = [0;0];
set_param('mass_spring/State-Space','A',mat2str(A));
set_param('mass_spring/State-Space','B',mat2str(B));
set_param('mass_spring/State-Space','C',mat2str(C));
set_param('mass_spring/State-Space','D',mat2str(D));
% set_param('mass_spring/State-Space',['Initial',' ','conditions'],mat2str(Initial_conditions));