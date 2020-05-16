function CY = computeCY(m, V, u, w, S, theta, phi, p, r, ay)

g = 9.81;
rho = 1.225;

qbar = 0.5*rho*V^2;

CY =  (m*ay + m*(r*u-p*w) - m*g*cos(theta)*sin(phi)) / (qbar * S);

end