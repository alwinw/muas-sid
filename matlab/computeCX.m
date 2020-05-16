function CX = computeCX(m, V, v, w, S, theta, q, r, T, ax)

g = 9.81;
rho = 1.225;

qbar = 0.5*rho*V^2;

CX =  (m*ax + m*(q*w-r*v) + m*g*sin(theta) - T) / (qbar * S);

end