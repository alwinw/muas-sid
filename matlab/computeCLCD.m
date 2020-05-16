function [CL,CD] = computeCLCD(CX, CZ, alpha)

CL = -CZ*cos(alpha) + CX*sin(alpha);
CD = -CX*cos(alpha) - CZ*sin(alpha);

end