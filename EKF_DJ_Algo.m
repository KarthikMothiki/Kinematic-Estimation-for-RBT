function [x_new, P] = ekf(u, z, T, x, P, Q)
%#codegen
% EKF   Extended Kalman Filter for nonlinear dynamic systems
% [x, P] = ekf(f,x,P,h,z,Q,R) returns state estimate, x and state covariance, P 
% for nonlinear dynamic system:
% x_k+1 = f(x_k) + w_k
% z_k   = h(x_k) + v_k
% where w ~ N(0,Q) meaning w is gaussian noise with covariance Q
%       v ~ N(0,R) meaning v is gaussian noise with covariance R
%
% Inputs:   u: input vector
%           z: current measurement
%           T: time interval between filter steps
%           x: "a priori" state estimate
%           P: "a priori" estimated state covariance
%
% Output:   x: "a posteriori" state estimate
%           P: "a posteriori" state covariance

    
    % Compute the Jacobian of the state transition function
    F = [1 T T^2/2; 0 1 T; 0 0 1];
    
    % Compute the Jacobian of the measurement function
    H = [100 0 10; 0 1 0];

    % Reshaping the matrices
    x_new = reshape(x, 3, 4);
    z_new = reshape(z, 12, 2);
    
    % Compute the measurement noise variance R(k)
    r11 = 1;
    r22 = 1;
    % r11 = (Dx * P(1,1) * Dx') + (Dx * P(1,2) * Dx') + (Dx * P(1,3) * Dx');
    % r22 = (Dy * P(2,1) * Dy') + (Dy * P(2,2) * Dy') + (Dy * P(2,3) * Dy');
    R = [r11 0; 0 r22];
    
    % Prediction update
    x_new = F * x_new;
    P = F * P * F' + Q;
    
    % Measurement update
    y = z_new - (H * x_new);
    S = H * P * H' + R;
    K = P * H' / S;
    x_new = x_new + K * y;
    P = (eye(size(x_new,1)) - K*H) * P;
end
