function y = Direct_Acceleration(V, V_dot, g_x_dot, x_dot, y_dot, z_dot, x_dot2, y_dot2, z_dot2, gamma_est, w, gamma_dot_est)

% V_dot = diff(V);

% x_dot = ones(1, 50);
% y_dot = ones(1, 50);
% z_dot = ones(1, 50);

w_x_bar = w;
w_y_bar = w;
w_z_bar = w;

T = 10;
for t = 1:T
    f_x_bar = (-0.5 .* gamma_dot_est(t) .* x_dot .* V) + (-0.5 .* gamma_est(t) .* x_dot2 .* V) - 0.5 .* (gamma_est(t) .* x_dot .* V_dot) - g_x_dot;
    f_y_bar = (-0.5 .* gamma_dot_est(t) .* y_dot .* V) + (-0.5 .* gamma_est(t) .* y_dot2 .* V) - 0.5 .* (gamma_est(t) .* y_dot .* V_dot);
    f_z_bar = (-0.5 .* gamma_dot_est(t) .* z_dot .* V) + (-0.5 .* gamma_est(t) .* z_dot2 .* V) - 0.5 .* (gamma_est(t) .* z_dot .* V_dot);
    
    % f_x_bar = 1;
    % f_y_bar = 1;
    % f_z_bar = 1;
    
    fDA_XDA = [x_dot; y_dot; z_dot; x_dot2; y_dot2; z_dot2; f_x_bar; f_y_bar; f_z_bar];
    W_DA = [0; 0; 0; 0; 0; 0; w_x_bar; w_y_bar; w_z_bar];
    
    y = fDA_XDA + W_DA;
    
    disp('Direct Acceleration Model at t = ')
    disp(t)
    disp(y)
end

