function [dx_dot2_dx, dy_dot2_dx, dz_dot2_dx] = calculatePartialDerivatives(x, x_dot, y_dot, z_dot, row_x, V, beta, g_x)
    % Calculate the partial derivatives of x_dot2, y_dot2, and z_dot2 with respect to x
    
    % Calculate intermediate terms
    term1 = (-0.5 * row_x * x_dot .* V) / beta;
    
    % Calculate partial derivatives
    dx_dot2_dx = (-0.5 * row_x * V) / beta;
    dy_dot2_dx = zeros(size(x));
    dz_dot2_dx = zeros(size(x));
    
    % Iterate over each element of x
    for i = 1:numel(x)
        % Calculate partial derivatives for y_dot2 and z_dot2
        dy_dot2_dx(i) = (term1(i) * y_dot(i)) / x_dot(i);
        dz_dot2_dx(i) = (term1(i) * z_dot(i)) / x_dot(i);
    end
end
