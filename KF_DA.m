function y = KF(V, gamma_est, gamma_dot_est, T)

% y = u;

% Generate synthetic data for D (drag) and D_ (drag derivative)
D_true = zeros(1, T);
D_true(1:T/2) = 10;
D_true((T/2)+1:end) = 5;
D_noise = 0.5 * randn(1, T);
D = D_true + D_noise;

D_ = diff(D) ./ diff(1:T); % Compute D_ using finite differences
D_(T-1) = D_(T-2); % Extend D_ to match the length of D

% Initial guess for gamma and gamma_dot
gamma_est(1) = 0.5;
gamma_dot_est(1) = 0.1;

% Kalman filter loop
for t = 2:T
    % Prediction step
    gamma_pred = gamma_est(t-1);
    gamma_dot_pred = gamma_dot_est(t-1);
    
    % Compute pseudo measurements
    D_pred = 0.5 * (V.*V) * gamma_pred;
    D_dot_pred = 2 * D_pred * gamma_dot_pred / (V.*V) - 4 * D_pred * gamma_dot_pred^2 / ((V.*V).*V);
    
    % Update step
    K = [gamma_dot_pred * (V.*V) / (2 * D_pred), 0];
    gamma_est(t) = gamma_pred + K(1) * (D(t) - D_pred);
    
    % Fix indexing for D_
    if t == T
        D_dot_index = t - 1;
    else
        D_dot_index = t;
    end
    gamma_dot_est(t) = gamma_dot_pred + K(2) * (D_(D_dot_index) - D_dot_pred);
end

% Output the final results
y = [gamma_est; gamma_dot_est];
disp('y from Kalman Filter:')
disp(y)

gamma_est = y(1,:);
gamma_dot_est = y(2,:);

disp('gamma_est:')
disp(gamma_est)

disp('gamma_dot_est:')
disp(gamma_dot_est)

% Plot results
% time = 1:T;
% 
% figure;
% subplot(2, 1, 1);
% plot(time, gamma_est, 'b-', 'LineWidth', 2);
% hold on;
% plot(time, gamma_true, 'r--', 'LineWidth', 2);
% xlabel('Time');
% ylabel('\gamma');
% legend('Estimated \gamma', 'True \gamma');
% title('Bootstrapped Estimation of \gamma');
% 
% subplot(2, 1, 2);
% plot(time, gamma_dot_est, 'b-', 'LineWidth', 2);
% hold on;
% plot(time, gamma_dot_true, 'r--', 'LineWidth', 2);
% xlabel('Time');
% ylabel('\gamma_{\dot}');
% legend('Estimated \gamma_{\dot}', 'True \gamma_{\dot}');
% title('Bootstrapped Estimation of \gamma_{\dot}');
