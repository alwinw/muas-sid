classdef MotorInterpolator
    properties
        Rm
        I0
        Kv
        Dp
        p_Dp
        
        Ct
    end

    methods
        function obj = MotorInterpolator(Rm, I0, Kv, Dp, p_Dp)
            obj.Rm = Rm;
            obj.I0 = I0;
            obj.Kv = Kv;
            obj.Dp = Dp;
            obj.p_Dp = p_Dp;
            
            p_D = [0.3:0.1:1.0]';
            J = linspace(0.0, 1.3, 100*length(p_D))';
            [J, p_D] = meshgrid(J, p_D);
            [r,c] = size(J);

            CtData = zeros( r,c);
            for i=1:r
                for j=1:c
                    CtData(i,j) = computeCtDiscrete(J(i,j), p_D(i,j));
                end
            end
            [J, p_D, CtData] = prepareSurfaceData(J(1,:), p_D(:,1), CtData);
            obj.Ct = fit([J, p_D], CtData, 'linearinterp');
        end
        
        function T = getThrust(obj, V, rho, V_mot_inp, I_mot_inp)
            n = obj.Kv / 60 * (V_mot_inp - I_mot_inp * obj.Rm);
            J = V / n / obj.Dp;
            T = obj.Ct(J, obj.p_Dp) * rho * n^2 * obj.Dp^4;
        end
    end
end