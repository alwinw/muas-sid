function [u,v,w] = compute_body_velocity_body_axes(roll, pitch, yaw, vN, vE, vD)

assert(max([length(roll) length(pitch) length(yaw) length(vN) length(vE) length(vD)]) == ...
       min([length(roll) length(pitch) length(yaw) length(vN) length(vE) length(vD)]));

u = zeros(length(roll), 1);
v = zeros(length(roll), 1);
w = zeros(length(roll), 1);
   
for i=1:length(roll)
    sin_phi = sin(roll(i));
    cos_phi = cos(roll(i));
    sin_theta = sin(pitch(i));
    cos_theta = cos(pitch(i));
    sin_psi = sin(yaw(i));
    cos_psi = cos(yaw(i));

    R1 = [1, 0, 0;
          0, cos_phi, sin_phi;
          0, -sin_phi, cos_phi]';
    R2 = [cos_theta, 0, -sin_theta;
          0, 1, 0;
          sin_theta, 0, cos_theta]';
    R3 = [cos_psi, sin_psi, 0;
          -sin_psi, cos_psi, 0;
          0, 0, 1]';

    y = R3*R2*R1*[vN(i); vE(i); vD(i)];
    u(i) = y(1);
    v(i) = y(2);
    w(i) = y(3);
end

end