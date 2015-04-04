% Compute thrust given current inputs and thrust coefficient.
function T = thrust(inputs, k)
    % Inputs are values for ${\omega_i}^2$
    T = [0; 0; k * sum(inputs)];
end

% Compute torques, given current inputs, length, drag coefficient, and thrust coefficient.
function tau = torques(inputs, L, b, k)
    % Inputs are values for ${\omega_i}^2$
    tau = [
        L * k * (inputs(1) - inputs(3))
        L * k * (inputs(2) - inputs(4))
        b * (inputs(1) - inputs(2) + inputs(3) - inputs(4))
    ];
end

function a = acceleration(inputs, angles, xdot, m, g, k, kd)
    gravity = [0; 0; -g];
    R = rotation(angles);
    T = R * thrust(inputs, k);
    Fd = -kd * xdot;
    a = gravity + 1 / m * T + Fd;
end

function omegadot = angular_acceleration(inputs, omega, I, L, b, k)
    tau = torques(inputs, L, b, k);
    omegaddot = inv(I) * (tau - cross(omega, I * omega));
end