function y = fcn(V, g_x_dot, x_dot, y_dot, z_dot, x_dot2, y_dot2, z_dot2, gamma_est, w, gamma_dot_est)

V_dot = diff(V);
% gamma_dot_est = 1;
% gamma_est = 1;

w_x_bar = w;
w_y_bar = w;
w_z_bar = w;

f_x_bar = (-0.5 .* gamma_dot_est .* x_dot .* V) + (-0.5 .* gamma_est .* x_dot2 .* V) - 0.5 * (gamma_est * x_dot * V_dot) - g_x_dot;
f_y_bar = (-0.5 .* gamma_dot_est .* y_dot .* V) + (-0.5 .* gamma_est .* y_dot2 .* V) - 0.5 * (gamma_est * y_dot * V_dot);
f_z_bar = (-0.5 .* gamma_dot_est .* z_dot .* V) + (-0.5 .* gamma_est .* z_dot2 .* V) - 0.5 * (gamma_est * z_dot * V_dot);
% f_x_bar = 1;
% f_y_bar = 1;
% f_z_bar = 1;

fDA_XDA = [x_dot; y_dot; z_dot; x_dot2; y_dot2; z_dot2; f_x_bar; f_y_bar; f_z_bar];
disp("f-x-bar:")
disp(gamma_dot_est)
W_DA = [0; 0; 0; 0; 0; 0; w_x_bar; w_y_bar; w_z_bar];

% disp("f_DA")
% disp(fDA_XDA)
% disp("W_DA")
% disp(W_DA)
disp("Direct Acceleration Model")
y = fDA_XDA + W_DA;
disp(y)

