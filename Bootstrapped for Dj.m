function [gamma_est, gamma_dot_est, gamma_dot2_est] = bootstrappedKalmanFilter(T, measurements_gamma, measurements_gamma_dot, XDJ, PDJ)
    num_measurements_gamma = length(measurements_gamma);
    num_measurements_gamma_dot = length(measurements_gamma_dot);
    num_measurements = num_measurements_gamma + num_measurements_gamma_dot;
    measurements = [measurements_gamma_dot measurements_gamma];

    state_size = size(XDJ, 1);
    filtered_state = zeros(state_size, num_measurements);
    filtered_covariance = zeros(state_size, state_size, num_measurements);
    filtered_state(:, 1) = XDJ(:, 1);
    filtered_covariance(:, :, 1) = PDJ;

    

    R = ones(3,3);

    T_matrix = [1 T T^2; 0 1 T; 0 0 1];
    T_ = transpose(T_matrix);

    for k = 2:num_measurements
        % Prediction step
        predicted_state = T * filtered_state(:, k-1);
        predicted_covariance = eye(2);
%         predicted_covariance = T * squeeze(filtered_covariance(:, :, k-1)) * T_;

        % Update step
        innovation = measurements(:, k) - [predicted_state(1); predicted_state(2)];
        innovation_covariance = eye(2);
        % innovation_covariance = T * squeeze(predicted_covariance(1:2, 1:2)) * T_ + R;
        kalman_gain = predicted_covariance(1:2, 1:2) .* T_ / innovation_covariance;

        filtered_state(:, k) = predicted_state + kalman_gain * innovation;
        filtered_covariance = eye(3);
        % filtered_covariance(:, :, k) = (eye(state_size) - kalman_gain * T) * predicted_covariance;
    end

    gamma_est = filtered_state(1, :);
    gamma_dot_est = filtered_state(2, :);
    gamma_dot2_est = filtered_state(3, :);
end
