function Ct = computeCtDiscrete(J,p_D)

[r,c] = size(J);
Ct = zeros(r,c);

% ax^2+bx+c 
coeffs = [1; 1; 1];

for i=1:r
    for j=1:c
        switch round( p_D(i,j), 3)
            case 0.3
                coeffs = [-0.0319 -0.1385 0.0747];
            case 0.4
                coeffs = [-0.0283 -0.1543 0.1007];
            case 0.5
                coeffs = [-0.0247 -0.1484 0.1106];
            case 0.6
                coeffs = [-0.1001 -0.0773 0.1135];
            case 0.7
                coeffs = [-0.1071 -0.0658 0.1229];
            case 0.8
                coeffs = [-0.1406 -0.016 0.1211];
            case 0.9
                coeffs = [-0.1553 0.0246 0.1247];
            case 1.0
                coeffs = [-0.147 0.042 0.1268];
            otherwise
                error('Not an accepted p/D value: %f', p_D(i,j));
        end

        Ct(i,j) = coeffs(1)*J(i,j).^2 + coeffs(2)*J(i,j) + coeffs(3);
        if Ct(i,j) < 0
            Ct(i,j) = 0;
        end
    end
end

end