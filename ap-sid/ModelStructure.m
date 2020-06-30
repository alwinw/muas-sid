classdef ModelStructure
properties (Access = public)
    z    % (Ns,1) Measured coefficients (e.g., CL, CD, Cm, etc.)
    zeta % (Np,1) Parameter estimates
    X    % (Ns,Np) Regressor time histories
    K    % (Nr) Set of potential regressors
    Kb   % (Nrb) Set of base potential regressors (from which to build K)
    Xb   % (Ns,Nrb) Regressor measurements for the base 
    A    % (Np+1) Set of accepted regressors
    yhat % (Ns,1) Current model of the measured data, z
    
    Ns  % Number of samples (discrete points in time)
    % Np: Number of model parameters
    % Nr: Number of potential regressors
    Nrb % Number of base potential regressors
    
    conf_in  % Confidence level to accept a regressor
    conf_out % Confidence level to remove a regressor
    
    SST         % Total sum of squares
end

properties (Access = private)
    sigma_max   % Estimate of the maximum variation for PSE calculation
    PSE_history % History of the predicted squared error as terms were added
    R2_history  % Hisotry of the coefficient of determination
end
    
methods (Access = public)
    function obj = ModelStructure(z, Kb, Xb, varargin)
        % ms = ModelStructure(z, Kb, Xb)
        %
        % z        : Measured coefficient values (one of e.g., CL, CD, Cm, etc.)
        %            (Ns,1)
        % Kb       : Set of base potential regressors (symbolic).
        %            These regressors are combined to produce a larger set
        %            up to power N
        %            (Nrb,1)
        % Xb       : Measured values for the base potential regressors
        %            (Ns, Nrb)
        %
        % Optional Arguments
        % 'Order' : Maximum order of any of the base regressors
        %           0 : Use only the base regressors (default)
        %               e.g., [a; b]
        %           1 : Use regressor products up to order 1
        %               e.g., [a; b; ab]
        %           2 : Use regressor products up to order 2
        %               e.g., [a; b; ab; a^2b; ab^2; a^2b^2]
        %           etc.
        % 'Confidence' : Confidence levels to accept/remove a regressor
        %                - e.g., ModelStructure(z, Kb, Xb, 'Confidence', [0.99, 0.95])
        %                   accepts regressors with 99% confidence, and
        %                   removes regressors with 95% confidence
        %                - Default: [0.99, 0.95]
        
        
        obj.Ns = size(z, 1);
        obj.Nrb = length(Kb);
        
        % defaults for optional arguments
        N = 0;
        conf_in = 0.99;
        conf_out = 0.95;
        % optional arguments
        for i=1:2:length(varargin)
            if strcmp(varargin{i}, 'Order')
                N = varargin{i+1};
            elseif strcmp(varargin{i}, 'Confidence')
                conf = varargin{i+1};
                conf_in = conf(1);
                conf_out = conf(2);
            else
                error('Unknown argument: ''%s''', varargin{i});
            end
        end
        
        assert(all(size(Xb) == [obj.Ns, obj.Nrb]), 'z,Kb,Xb have inconsistent dimensions');
        assert(N>=0, 'Order, N, must be non-negative');
        assert(size(z,2)==1, 'z should only have one column');
        assert(all([conf_in conf_out] > 0 & [conf_in conf_out] < 1), 'Confidence levels must be in (0,1)');
                
        % public member variables
        obj.z = z;
        obj.Kb = Kb;
        obj.Xb = Xb;
        obj.K = obj.create_regressor_set(N);
        % accept the constant term as a regressor by default
        obj.A = 1;
        obj.conf_in = conf_in;
        obj.conf_out = conf_out;
        z_mean = mean(z);
        obj.SST = sum((z - z_mean).^2);
        
        % private member variables
        obj.sigma_max = obj.SST / obj.Ns;
        
        %%% Find the first non-constant regressor to add 
        % evaluate the correlations of each potential regressor with z
        corr = obj.compute_regressor_correlations(obj.K, obj.z);
        [~,idx] = max(abs(corr));
        % check that this regressor passes the confidence criterion
        X = obj.compute_regressor_time_history_matrix([obj.A; obj.K(idx)]);
        [~, ~, ~, s, SSR] = obj.least_squares_regression(X,z);
        F0 = SSR / s^2;
        F_in = finv(obj.conf_in, 1, obj.Ns - length(obj.A) - 1);
        if F0 < F_in
            fprintf('WARNING: F0 < F_in for first regressor - check data');
        end
        % add the first accepted regressor
        obj = obj.add_accepted_regressor(obj.K(idx));
        % compute the initial PSE
        obj.X = obj.compute_regressor_time_history_matrix(obj.A);
        [obj.zeta,~,~,~,SSR] = obj.least_squares_regression(obj.X, obj.z);
        obj.yhat = obj.X * obj.zeta;
        PSE = obj.compute_predicted_squared_error(obj.yhat, length(obj.A));
        obj.PSE_history = PSE;
        obj.R2_history = SSR / obj.SST;
    end
    
    function [A,zeta,cov,corr,sigma,R2,obj] = stepwise_regression(obj)
        % Returns a model structure estimate with both symbolic functions
        % and parameter estimates
        
        iter = 0;
        max_iter = 10 * length(obj.K);
        
        while iter < max_iter
            have_added_new_regressor = false;
            have_removed_regressor = false;
                        
            % compute the current model
            if iter==0
                obj.X = obj.compute_regressor_time_history_matrix(obj.A);
                obj.zeta = obj.least_squares_regression(obj.X, obj.z);
                obj.yhat = obj.X * obj.zeta;
            end
            
            if length(obj.K) > 0
                
                % create a set of orthogonal regressors w.r.t. the current
                % model
                P = sym(zeros(length(obj.K), 1));
                for i=1:length(P)
                    % orthogonalise each regressor w.r.t. the current model
                    y_Ki = obj.compute_regressor_time_history_matrix(obj.K(i));
                    a = obj.least_squares_regression(obj.X, y_Ki);
                    P(i) = obj.K(i) - transpose(obj.A)*a;
                end

                % compute the residual of the measurements w.r.t. the current
                % model
                nu_z = obj.z - obj.yhat;

                % add a new regressor (if F0 > F_in)
                % first pick the orthogonalised regressor with the maximum
                % absolute correlation
                corr = obj.compute_regressor_correlations(P, nu_z);
                [~,idx] = max(abs(corr));
                % then check that the partitioned model meets the confidence
                % level
                X = obj.compute_regressor_time_history_matrix([obj.A; obj.K(idx)]);
                [~, ~, ~, ~, SSR_curr] = obj.least_squares_regression(X(:,1:end-1), obj.z);
                [zeta, ~, ~, s, SSR_new] = obj.least_squares_regression(X, obj.z);
                F0 = (SSR_new - SSR_curr) / s^2;
                F_in = finv(obj.conf_in, 1, obj.Ns - length(obj.A) - 1);
                % also check that the predicted squared error doesn't increase
                yhat = X*zeta;
                PSE = obj.compute_predicted_squared_error(yhat, length(obj.A) + 1);
                R2 = SSR_new / obj.SST;
                % check whether the process has a reached a stopping criterion
                % PSE must decrease
                % No R^2 change stopping criterion, as this often drops
                % terms
                if PSE > obj.PSE_history(end)

                    break;
                end

                % only add the new regressor if is statistically significant
                if F0 > F_in
                    obj = obj.add_accepted_regressor(obj.K(idx));
                    obj.PSE_history = [obj.PSE_history; PSE];
                    obj.R2_history = [obj.R2_history; R2];
                    have_added_new_regressor = true;
                end

                % update the model
                obj.X = obj.compute_regressor_time_history_matrix(obj.A);
                [obj.zeta,~,~,s,SSR] = obj.least_squares_regression(obj.X, obj.z);
                obj.yhat = obj.X * obj.zeta;
            
            end
                
            % check if any existing regressors should be removed
            % skip the bias term (in position 1)
            F0 = zeros(length(obj.A), 1);
            F0(1) = Inf;
            for i=2:length(obj.A)
                % SSR_i is missing the ith regressor
                idx = 1:length(obj.A) ~= i;
                X_i = obj.X(:, idx);
                [~,~,~,~,SSR_i] = obj.least_squares_regression(X_i, obj.z);
                F0(i) = (SSR - SSR_i)/s^2;
            end
            [F0_min, i_min] = min(F0);
            F_out = finv(obj.conf_out, 1, obj.Ns - length(obj.A));
            if F0_min < F_out
                obj = obj.remove_accepted_regressor(obj.A(i_min));
                have_removed_regressor = true;
            end
            
            % if an accepted was removed, the model needs to updated
            if have_removed_regressor
                obj.X = obj.compute_regressor_time_history_matrix(obj.A);
                [obj.zeta,~,~,~,SSR] = obj.least_squares_regression(obj.X, obj.z);
                obj.yhat = obj.X * obj.zeta;
                % also update the PSE history
                PSE = obj.compute_predicted_squared_error(obj.yhat, length(obj.A));
                obj.PSE_history = [obj.PSE_history; PSE];
                R2 = SSR / obj.SST;
                obj.R2_history = [obj.R2_history; R2];
            end
            
            % plot the progress
            figure(1); clf;
            subplot(211);
            semilogy(obj.PSE_history, 'k-o');
            grid on;
            title('Predicted Squared Error (PSE)');
            subplot(212);
            semilogy(obj.R2_history, 'k-o');
            grid on;
            title('Coefficient of Determination (R^2)');
            drawnow;
            
            % if no more regressors are added or removed, then stop the
            % process
            if ~have_added_new_regressor && ~have_removed_regressor
                break;
            end
           
            iter = iter + 1;
        end
        
        A = obj.A;
        obj.X = obj.compute_regressor_time_history_matrix(obj.A);
        [zeta,cov,corr,sigma,SSR] = obj.least_squares_regression(obj.X, obj.z);
        cov = diag(cov);
        R2 = SSR / obj.SST;
    end
end

methods (Access = private) 
    function K = create_regressor_set(obj, N)
        % creates the set of potential regressors from base regressors
        
        K = obj.Kb;
        if N==0
            return;
        end
        for i=1:obj.Nrb
            for j=i+1:obj.Nrb
                % this implements the Cartesian product
                % if Kb{i}^p * Kb{j}^q, then {p,q}={1,...,n}x{1,...n}
                [a,b] = meshgrid(obj.Kb(i).^(1:N), obj.Kb(j).^(1:N));
                K = [K; reshape(a.*b, [], 1)];
            end
        end
        
    end
    
    function X = compute_regressor_time_history_matrix(obj, K)
        % ms.compute_regressor_time_history_matrix(K)
        %
        X = zeros(obj.Ns, length(K));
        Kb_syms = sym(zeros(length(obj.Kb), 1));
        for i=1:length(obj.Kb)
            Kb_syms(i) = symvar(obj.Kb(i));
        end
        for i=1:length(K)
            if K(i) == 1
                X(:,i) = ones(obj.Ns, 1);
                continue;
            end
            fn = matlabFunction(K(i), 'vars', {Kb_syms});
            X(:,i) = fn(obj.Xb')';
        end
    end
    
    function obj = add_accepted_regressor(obj, xi)
        obj.A = [obj.A; xi];
        obj.K = obj.K(obj.K ~= xi);
    end
    
    function obj = remove_accepted_regressor(obj, xi)
        obj.A = obj.A(obj.A ~= xi);
    end
    
    function [a, cov, corr, sigma, SSR] = least_squares_regression(obj,X,z)
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
    
    function corr = compute_regressor_correlations(obj, P, z)
        % P : Set of regressors
        % z : Measurements
        
        X = obj.compute_regressor_time_history_matrix(P);
        corr = zeros(length(P), 1);
        for j=1:length(P)
            xi_j_diff = X(:,j) - mean(X(:,j));
            z_diff = z - mean(z);
            S_jj = sum(xi_j_diff.^2);
            S_zz = sum(z_diff.^2);
            corr(j) = 1/sqrt(S_jj*S_zz) * sum(xi_j_diff .* z_diff);
        end
    end
    
    function PSE = compute_predicted_squared_error(obj, yhat, p)
        % yhat : Current model predictions of z
        % Np   : Current number of terms in the model
        
        nu = obj.z - yhat;
        MSFE = nu'*nu / obj.Ns;
        PSE = MSFE + obj.sigma_max^2 * p / obj.Ns;
    end
end
end