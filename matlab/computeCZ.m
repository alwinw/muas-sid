function CZ = computeCZ(m, V, u, v, S, theta, phi, p, q, az)

g = 9.81;
rho = 1.225;

qbar = 0.5*rho*V^2;

CZ =  (m*az + m*(p*v-q*u) - m*g*cos(theta)*cos(phi)) / (qbar * S);

end