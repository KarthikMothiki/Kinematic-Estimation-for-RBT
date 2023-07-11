function DJ = DirectJerk(x, y, z, x_dot, y_dot, z_dot, x_dot2, y_dot2, z_dot2, x_dot3, y_dot3, z_dot3, w)

X_DJ_normal = [x y z x_dot y_dot z_dot x_dot2 y_dot2 z_dot2 x_dot3 y_dot3 z_dot3];
X_DJ = transpose(X_DJ_normal);
A = [0 0 0 0 0 0 0 0 0 w w w];
wDJ = transpose(A);

DJ = X_DJ + wDJ;
disp("DJ Model")
disp(DJ)
