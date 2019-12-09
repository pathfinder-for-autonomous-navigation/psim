
a  = 6793137.0;  % Semimajor axis                        (m)
e  = 0.0;        % Eccentricity                          (unitless)
i  = 45*pi/180;  % Inclination angle                     (rad)
O  = 0.0;        % Right ascension of the ascending node (rad)
o  = 0.0;        % Argument of perigee                   (rad)
nu = 0*pi/180;   % True anamoly                          (rad)
[   r1,...  % Position (m)   [eci]
    v1,...  % Velocity (m/s) [eci]
] = utl_orb2rv(a * (1 - e), e, i, O, o, nu, 3.986e14);
n = sqrt(3.986e14 / (a * a * a)); % Orbital rate
w_hill = [0.0; 0.0; n];

% Add initial velocity difference
V_rel   = 0.5; % Relative velocity at deployment (m/s)
r2 = r1;
v2 = randn(3, 1);
v2 = v1 + V_rel * v2 / norm(v2);

T = 5.0 * 90.0 * 60.0;
opt = odeset('RelTol', 1e-8, 'AbsTol', 1e-2, 'InitialStep', 0.1);
[t, y] = ode45(@frhs, [0.0, T], [r1; v1; r2; v2], opt);
R1 = y(:, 1:3)';
V1 = y(:, 4:6)';
R2 = y(:, 7:9)';
V2 = y(:, 10:12)';

[~, c] = size(R1);
R = zeros(3, c);
V = zeros(3, c);
for i = 1:c
    Q_eci_hill = eci_to_hill(R1(:, i), V1(:, i));
    R(:, i) = Q_eci_hill * (R2(:, i) - R1(:, i));
    V(:, i) = Q_eci_hill * (V2(:, i) - V1(:, i)) - cross(w_hill, R(:, i));
end

figure
hold on
plot(t, R(1, :), '-r')
plot(t, R(2, :), '-g')
plot(t, R(3, :), '-b')
hold off
title('Truth Position of Satellite Two in the Hill Frame (rgb ~ xyz)')

figure
hold on
plot(t, R2(1, :) - R1(1, :), '-r')
plot(t, R2(2, :) - R1(2, :), '-g')
plot(t, R2(3, :) - R1(3, :), '-b')
hold off
title('Truth Difference in Position in ECI (rgb ~ xyz)')

figure
hold on
plot(t, V(1, :), '-r')
plot(t, V(2, :), '-g')
plot(t, V(3, :), '-b')
hold off
title('Truth Velocity of Satellite Two in the Hill Frame (rgb ~ xyz)')

figure
hold on
plot(t, V2(1, :) - V1(1, :), '-r')
plot(t, V2(2, :) - V1(2, :), '-g')
plot(t, V2(3, :) - V1(3, :), '-b')
hold off
title('Truth Difference in Velocity in ECI (rgb ~ xyz)')

[rr, rv, vr, vv] = clohessywiltshire(30.0, n); % State update matrices
A = [rr, rv; vr, vv];

N = floor(T / 30.0);
t = zeros(1, N);
X = zeros(6, N);

Q_eci_hill = eci_to_hill(r1, v1);
X(:, 1) = [
    Q_eci_hill * (r2 - r1);
    Q_eci_hill * (v2 - v1)
];
for i = 2:N
    t(i) = i * 30.0;
    X(:, i) = A * X(:, i - 1);
end

figure
hold on
plot(t, X(1, :), '-r')
plot(t, X(2, :), '-g')
plot(t, X(3, :), '-b')
hold off
title('CW Position of Satellite Two in the Hill Frame (rgb ~ xyz)')

figure
hold on
plot(t, X(4, :), '-r')
plot(t, X(5, :), '-g')
plot(t, X(6, :), '-b')
hold off
title('CW Velocity of Satellite Two in the Hill Frame (rgb ~ xyz)')


function Q_eci_hill = eci_to_hill(r, v)

r_hat = -r / norm(r);
n_hat = cross(r, v) / norm(cross(r, v));
v_hat = cross(n_hat, r_hat);

Q_eci_hill = [r_hat'; v_hat'; n_hat'];

end


function dy = frhs(~, y)

dy = zeros(12, 1);

%mu_earth = 3.986e14;

[gx, gy, gz] = gravityzonal(y(1:3)', 'Earth', 4, 'Error');
dy(1:3, 1) = y(4:6, 1);
dy(4:6, 1) = [gx; gy; gz];
%dy(4:6, 1) = -mu_earth * y(1:3, 1) / norm(y(1:3, 1))^3;

[gx, gy, gz] = gravityzonal(y(7:9)', 'Earth', 4, 'Error');
dy(7:9, 1) = y(10:12, 1);
dy(10:12, 1) = [gx; gy; gz];
%dy(10:12, 1) = -mu_earth * y(7:9, 1) / norm(y(7:9, 1))^3;

end
