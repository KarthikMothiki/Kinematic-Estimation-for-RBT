function [y1, y2] = kalmanFilter(V, V2, V3, T)
    % V: Velocity
    % T: Number of time steps

    disp('Velocity:')    
    disp(V)
    % Generate synthetic data for D (drag) and D_ (drag derivative)
    D_true = zeros(1, 50);
    D_true(1:25) = 10;
    D_true(26:end) = 5;
    D_noise = 0.5 * randn(1, 50);
    D = D_true + D_noise;

    D_ = diff(D) ./ diff(1:50); % Compute D_ using finite differences
    D_ = padarray(D_, [0 1], 'replicate', 'post'); % Pad one element at the end
    
    % Bootstrap estimation using Kalman filter
    gamma_est = zeros(1, T);
    gamma_dot_est = zeros(1, T);
    
    % Initial guess for gamma and gamma_dot
    gamma_est(1) = 0.5;
    gamma_dot_est(1) = 0.1;
    
    % Process noise covariance
    Q = diag([0.01, 0.01]);
    
    % Measurement noise covariance
    R = 0.1;

    % Kalman filter loop
    for t = 2:T
        % Prediction step
        gamma_pred = gamma_est(t-1);
        gamma_dot_pred = gamma_dot_est(t-1);
    
        % Compute pseudo measurements
        D_pred = 0.5 * V2 * gamma_pred;
        D_dot_pred = ((2 * D_pred * gamma_dot_pred) / V2) - ((4 * D_pred * gamma_dot_pred^2) / V3);
    
        % Update step
        K = [gamma_dot_pred * V2 / (2 * D_pred), 0];
        gamma_est(t) = gamma_est(t-1) + K(1) * (D(t) - D_pred);
%         gamma_est(t) = gamma_est(t-1) + K(1) * (D_(t) - D_pred(t));
    
        % Fix indexing for D_
        D_dot_index = min(t, T-1);
        gamma_dot_est(t) = (gamma_dot_pred + K(2) * (D_(D_dot_index) - D_dot_pred));
    end    

    y1 = gamma_est; 
    y2 = gamma_dot_est;

% Plot the estimated values
% figure;
% subplot(2,1,1);
% plot(gamma_est);
% xlabel('Time step');
% ylabel('Estimated gamma');
% 
% subplot(2,1,2);
% plot(gamma_dot_est);
% xlabel('Time step');
% ylabel('Estimated gamma_dot');
