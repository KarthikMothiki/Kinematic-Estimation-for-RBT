function [x_dot2, y_dot2, z_dot2] = Eq_of_Motion(V, x_dot, y_dot, z_dot, beta, g_x, row_x)

x_dot2 = ((-0.5 * row_x * x_dot .* V)/beta) - g_x;
y_dot2 = (-0.5 * row_x * y_dot .* V)/beta;
z_dot2 = (-0.5 * row_x * z_dot .* V)/beta;

disp("Velocity, V = ")
disp(V)

disp("x_dot2= ")
disp(x_dot2)
disp("y_dot2= ")
disp(y_dot2)
disp("z_dot2= ")
disp(z_dot2)

% y = [x_dot2; y_dot2; z_dot2];
% disp(y)
