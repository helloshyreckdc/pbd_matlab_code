syms dtheta L epsilon_dot_dot Kx Ktheta mu l r;
dU = L*dtheta-epsilon_dot_dot
Fx = Kx*dU
M = (Ktheta*dtheta+Fx*L)*(-1)
lambda = l/(2*mu*r)
Fz = (M+Fx*mu*r*(1+lambda))/(lambda*r)