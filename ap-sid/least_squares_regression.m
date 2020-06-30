function [a, cov, corr, sigma, SSR] = least_squares_regression(X,z)
    % [a, cov, corr, sigma, SSR] = ms.least_squares_regression(X,z)
    %
    % Solves the ordinary least-squares regression problem
    % that minimises z - Xa

    Ns = size(X,1);
    Np = size(X,2);

    % least-squares regression
    D = pinv(X'*X);
    a = D * (X'*z);

    % model estimate
    yhat = X*a;

    % measurement error variance estimate
    nu = z - yhat;
    sigma = sqrt((nu'*nu) / (Ns-Np));

    % covariance matrix
    cov = sigma^2 * D;

    % standard deviation estimates
    s_j = sigma * sqrt(diag(D));

    % parameter correlation matrix
    Sr = diag(1./s_j);
    corr = Sr * cov * Sr;

    % regression sum of squares
    z_mean = mean(z);
    SSR = sum( (yhat - z_mean).^2 );
end