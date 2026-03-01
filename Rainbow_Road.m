%%
% Exercise 14.1

syms t
n = 4
u = t / n
x = 0.6561 * sin(u);
y = 0.3280 * sin(2*u);
vx = simplify(diff(x));
max(vx);
vy = simplify(diff(y));
max(vy);
speed = sqrt(vx^2+vy^2);
Tx = vx ./ speed;
Ty = vy ./ speed;
T_mag = sqrt(diff(Tx)^2+diff(Ty)^2);
Nx = diff(Tx) ./ T_mag;
Ny = diff(Ty) ./ T_mag;

figure()
fplot(x, y, [0, n * 2 * pi])
hold on
quiver(subs(x, 1), subs(y, 1), subs(Tx, 1), subs(Ty, 1))
quiver(subs(x, 15), subs(y, 15), subs(Tx, 15), subs(Ty, 15))
quiver(subs(x, 1), subs(y, 1), subs(Nx, 1), subs(Ny, 1))
quiver(subs(x, 15), subs(y, 15), subs(Nx, 15), subs(Ny, 15))
xlabel('i (m)')
ylabel('j (m)')
legend('neato path', 'T1', 'T2', 'N1', 'N2')
title('parametric curve')
subtitle('plot of r(t) = 0.6561*sin(u)i + y = 0.3280*sin(2*u)j with tangent and unit vectors')
hold off

qx = diff(Tx);
qy = diff(Ty);

theta_dot = Tx .* qy - Ty .* qx;

v_left = speed - 0.5 * 0.25 * theta_dot;
v_right = speed + 0.5 * 0.25 * theta_dot;

figure()
fplot(v_right, [0, n * 2 * pi])
hold on
fplot(v_left, [0, n * 2 * pi])
fplot(speed, [0, n * 2 * pi])
xlabel('time (s)')
ylabel('velocity (m/s)')
legend('right wheel velocity', 'left wheel velocity', 'speed')
title('planned neato velocity')
%%

neatov3.connect('192.168.16.70')


vl_func = matlabFunction(v_left);
vr_func = matlabFunction(v_right);
tic;

encoders = zeros(0, 3);

while toc <= n * 2 * pi
    t_in = toc;
    vl_out = vl_func(t_in);
    vr_out = vr_func(t_in);
    neatov3.setVelocities(vl_out, vr_out)
    sensors = neatov3.receive();
    encoders(end+1, :) = [t_in, sensors.encoders(1), sensors.encoders(2)];
    pause(0.01)
end

neatov3.disconnect()

%%

tlist = encoders(:, 1);
left_wheel_encoder_list = encoders(:, 2);
right_wheel_encoder_list = encoders(:, 3);
dt_list = diff(tlist);
right_wheel_speed = diff(right_wheel_encoder_list) ./ dt_list;
left_wheel_speed = diff(left_wheel_encoder_list) ./ dt_list;
theta_dot2 = (right_wheel_speed - left_wheel_speed) ./ 0.25;
speed_recorded = sqrt(x_vel.^2+y_vel.^2);

figure()
subplot(2, 1, 1)
plot(tlist(1:end-1), theta_dot2, '--')
hold on
fplot(theta_dot, [0, n * 2 * pi])
legend('recorded', 'planned')
xlabel('time (s)')
ylabel('angular velocity (rad/s)')
title('recorded vs planned angular velocity')
axis([0, 25, -1, 1])


subplot(2, 1, 2)
plot(tlist(1:end-1), speed_recorded, '--')
hold on
fplot(speed, [0, n * 2 * pi])
legend('recorded', 'planned')
xlabel('time (s)')
ylabel('speed (m/s)')
title('recorded vs planned speed')
axis([0, 25, 0, 0.5])


theta_dot2 = (right_wheel_speed - left_wheel_speed) ./ 0.25;
figure()
theta = cumsum(theta_dot2.*dt_list);
x_vel = ((right_wheel_speed + left_wheel_speed) / 2) .* cos(theta);
y_vel = ((right_wheel_speed + left_wheel_speed) / 2) .* sin(theta);


x_disp = cumsum(x_vel.*dt_list);
y_disp = cumsum(y_vel.*dt_list);

Tx_recorded = x_vel ./ speed_recorded;
Ty_recorded = y_vel ./ speed_recorded;


figure()
subplot(2, 1, 1)
plot(x_disp, y_disp, '--');
hold on
quiver(x_disp(10), y_disp(10), Tx_recorded(10), Ty_recorded(10))
quiver(x_disp(100), y_disp(100), Tx_recorded(100), Ty_recorded(100))
xlabel('x displacement (m)')
ylabel('y displacement (m)')
title('recorded neato path')
legend('neato path', 'T hat1', 'T hat2')

%%

% Reconstructed path appears to be rotated compared to planned path as it is
% relative to the frame of the neato as opposed to the path. When the neato first
% starts moving, it is going straight which is considered the positive x direction.
% To the observer standing roughly perpendicular to the path, the neato travels
% up at an angle. Although the paths look different they are the same, just projected
% onto different reference frames.

subplot(2, 1, 2)
fplot(x, y, [0, n * 2 * pi])
hold on
quiver(subs(x, 1), subs(y, 1), subs(Tx, 1), subs(Ty, 1))
quiver(subs(x, 15), subs(y, 15), subs(Tx, 15), subs(Ty, 15))
xlabel('i (m)')
ylabel('j (m)')
legend('neato path', 'T hat1', 'T hat2')
title('planned neato path')
hold off
v_left_recorded = speed_recorded - 0.5 * 0.25 * theta_dot2;
v_right_recorded = speed_recorded + 0.5 * 0.25 * theta_dot2;

figure()
subplot(2, 1, 1)
plot(tlist(1:end-1), left_wheel_speed, '--')
hold on
fplot(v_left, [0, n * 2 * pi])
xlabel('time(s)')
ylabel('velocity(m/s)')
legend('recorded', 'planned')
title('recorded vs planned left wheel velocities')
axis([0, 25, 0, 0.3])

subplot(2, 1, 2)
plot(tlist(1:end-1), right_wheel_speed, '--')
hold on
fplot(v_right, [0, n * 2 * pi])
xlabel('time(s)')
ylabel('velocity(m/s)')
legend('recorded', 'planned')
title('recorded vs planned right wheel velocities')
axis([0, 25, 0, 0.3])
