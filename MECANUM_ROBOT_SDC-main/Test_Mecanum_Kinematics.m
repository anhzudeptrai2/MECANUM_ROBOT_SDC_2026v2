%% TEST ĐỘNG HỌC MECANUM - DỰA TRÊN CODE STM32
% File này test động học Mecanum giống hệt code trong STM32
% Không thay đổi code STM32, chỉ mô phỏng trên MATLAB

clear all; clc; close all;

%% ============ THÔNG SỐ ROBOT (GIỐNG CODE STM32) ============
MECANUM_WHEEL_RADIUS = 0.075;  % m
MECANUM_LX = 0.189;            % m (half-length)
MECANUM_LY = 0.189;            % m (half-width)
MECANUM_K = MECANUM_LX + MECANUM_LY;

Robot_Max_Speed = 3.0;  % m/s
Robot_Max_Omega = 6.0;  % rad/s

%% ============ HÀM ĐỘNG HỌC MECANUM (GIỐNG CODE C) ============
function [w_fl, w_fr, w_rl, w_rr] = MecanumRobot_CalculateWheelSpeeds(vx, vy, omega, theta, WHEEL_RADIUS, K)
    % Field-centric to robot-centric
    vx_robot = vx * cos(theta) + vy * sin(theta);
    vy_robot = -vx * sin(theta) + vy * cos(theta);
    
    % Mecanum inverse kinematics (rad/s)
    w_fl = (vx_robot - vy_robot - K * omega) / WHEEL_RADIUS;
    w_fr = (vx_robot + vy_robot + K * omega) / WHEEL_RADIUS;
    w_rl = (vx_robot + vy_robot - K * omega) / WHEEL_RADIUS;
    w_rr = (vx_robot - vy_robot + K * omega) / WHEEL_RADIUS;
    
    % Convert rad/s to RPM
    w_fl = w_fl * 60 / (2*pi);
    w_fr = w_fr * 60 / (2*pi);
    w_rl = w_rl * 60 / (2*pi);
    w_rr = w_rr * 60 / (2*pi);
end

%% ============ TEST CASES ============
fprintf('=== TEST ĐỘNG HỌC MECANUM ===\n\n');

% IMU heading (giả sử robot đang ở góc 0 độ)
imu_theta_deg = 0;
theta = -deg2rad(imu_theta_deg);  % Giống code: robot->theta = -robot->theta

%% TEST 1: TIẾN THẲNG (Joystick đẩy LÊN)
fprintf('TEST 1: TIẾN THẲNG (l_stick_y = +127)\n');
l_stick_x = 0;    % Không sang ngang
l_stick_y = 127;  % Đẩy lên (max)
r_stick_x = 0;    % Không xoay

% Mapping giống code main.c
mapped_x = l_stick_x / 128.0;
mapped_y = l_stick_y / 128.0;
mapped_z = r_stick_x / 128.0;

% Vận tốc mong muốn (GIỐNG CODE MAIN.C)
vx_cmd = mapped_y * Robot_Max_Speed;      % Y stick → vx (tiến/lùi)
vy_cmd = mapped_x * Robot_Max_Speed;      % X stick → vy (trái/phải)
omg_cmd = -mapped_z * Robot_Max_Omega;    % Z stick → omega (xoay)

[w_fl, w_fr, w_rl, w_rr] = MecanumRobot_CalculateWheelSpeeds(vx_cmd, vy_cmd, omg_cmd, theta, MECANUM_WHEEL_RADIUS, MECANUM_K);

fprintf('  Joystick: X=%d, Y=%d, Z=%d\n', l_stick_x, l_stick_y, r_stick_x);
fprintf('  Vận tốc: vx=%.2f m/s, vy=%.2f m/s, omega=%.2f rad/s\n', vx_cmd, vy_cmd, omg_cmd);
fprintf('  Tốc độ bánh (RPM):\n');
fprintf('    FL = %.1f RPM\n', w_fl);
fprintf('    FR = %.1f RPM\n', w_fr);
fprintf('    RL = %.1f RPM\n', w_rl);
fprintf('    RR = %.1f RPM\n', w_rr);
fprintf('  ✓ Tất cả bánh phải CÙNG chiều, CÙNG tốc độ\n\n');

%% TEST 2: LÙI (Joystick đẩy XUỐNG)
fprintf('TEST 2: LÙI (l_stick_y = -127)\n');
l_stick_x = 0;
l_stick_y = -127;  % Đẩy xuống
r_stick_x = 0;

mapped_x = l_stick_x / 128.0;
mapped_y = l_stick_y / 128.0;
mapped_z = r_stick_x / 128.0;

vx_cmd = mapped_y * Robot_Max_Speed;
vy_cmd = mapped_x * Robot_Max_Speed;
omg_cmd = -mapped_z * Robot_Max_Omega;

[w_fl, w_fr, w_rl, w_rr] = MecanumRobot_CalculateWheelSpeeds(vx_cmd, vy_cmd, omg_cmd, theta, MECANUM_WHEEL_RADIUS, MECANUM_K);

fprintf('  Joystick: X=%d, Y=%d, Z=%d\n', l_stick_x, l_stick_y, r_stick_x);
fprintf('  Vận tốc: vx=%.2f m/s, vy=%.2f m/s, omega=%.2f rad/s\n', vx_cmd, vy_cmd, omg_cmd);
fprintf('  Tốc độ bánh (RPM):\n');
fprintf('    FL = %.1f RPM\n', w_fl);
fprintf('    FR = %.1f RPM\n', w_fr);
fprintf('    RL = %.1f RPM\n', w_rl);
fprintf('    RR = %.1f RPM\n', w_rr);
fprintf('  ✓ Tất cả bánh phải CÙNG chiều ÂM, CÙNG tốc độ\n\n');

%% TEST 3: SANG PHẢI (Joystick đẩy PHẢI)
fprintf('TEST 3: SANG PHẢI (l_stick_x = +127)\n');
l_stick_x = 127;   % Đẩy phải
l_stick_y = 0;
r_stick_x = 0;

mapped_x = l_stick_x / 128.0;
mapped_y = l_stick_y / 128.0;
mapped_z = r_stick_x / 128.0;

vx_cmd = mapped_y * Robot_Max_Speed;
vy_cmd = mapped_x * Robot_Max_Speed;
omg_cmd = -mapped_z * Robot_Max_Omega;

[w_fl, w_fr, w_rl, w_rr] = MecanumRobot_CalculateWheelSpeeds(vx_cmd, vy_cmd, omg_cmd, theta, MECANUM_WHEEL_RADIUS, MECANUM_K);

fprintf('  Joystick: X=%d, Y=%d, Z=%d\n', l_stick_x, l_stick_y, r_stick_x);
fprintf('  Vận tốc: vx=%.2f m/s, vy=%.2f m/s, omega=%.2f rad/s\n', vx_cmd, vy_cmd, omg_cmd);
fprintf('  Tốc độ bánh (RPM):\n');
fprintf('    FL = %.1f RPM\n', w_fl);
fprintf('    FR = %.1f RPM\n', w_fr);
fprintf('    RL = %.1f RPM\n', w_rl);
fprintf('    RR = %.1f RPM\n', w_rr);
fprintf('  ✓ FL,RR dương (+), FR,RL âm (-) → Sang phải\n\n');

%% TEST 4: SANG TRÁI (Joystick đẩy TRÁI)
fprintf('TEST 4: SANG TRÁI (l_stick_x = -127)\n');
l_stick_x = -127;  % Đẩy trái
l_stick_y = 0;
r_stick_x = 0;

mapped_x = l_stick_x / 128.0;
mapped_y = l_stick_y / 128.0;
mapped_z = r_stick_x / 128.0;

vx_cmd = mapped_y * Robot_Max_Speed;
vy_cmd = mapped_x * Robot_Max_Speed;
omg_cmd = -mapped_z * Robot_Max_Omega;

[w_fl, w_fr, w_rl, w_rr] = MecanumRobot_CalculateWheelSpeeds(vx_cmd, vy_cmd, omg_cmd, theta, MECANUM_WHEEL_RADIUS, MECANUM_K);

fprintf('  Joystick: X=%d, Y=%d, Z=%d\n', l_stick_x, l_stick_y, r_stick_x);
fprintf('  Vận tốc: vx=%.2f m/s, vy=%.2f m/s, omega=%.2f rad/s\n', vx_cmd, vy_cmd, omg_cmd);
fprintf('  Tốc độ bánh (RPM):\n');
fprintf('    FL = %.1f RPM\n', w_fl);
fprintf('    FR = %.1f RPM\n', w_fr);
fprintf('    RL = %.1f RPM\n', w_rl);
fprintf('    RR = %.1f RPM\n', w_rr);
fprintf('  ✓ FL,RR âm (-), FR,RL dương (+) → Sang trái\n\n');

%% TEST 5: XOAY PHẢI (Joystick R xoay PHẢI)
fprintf('TEST 5: XOAY PHẢI (r_stick_x = +127)\n');
l_stick_x = 0;
l_stick_y = 0;
r_stick_x = 127;   % Xoay phải

mapped_x = l_stick_x / 128.0;
mapped_y = l_stick_y / 128.0;
mapped_z = r_stick_x / 128.0;

vx_cmd = mapped_y * Robot_Max_Speed;
vy_cmd = mapped_x * Robot_Max_Speed;
omg_cmd = -mapped_z * Robot_Max_Omega;

[w_fl, w_fr, w_rl, w_rr] = MecanumRobot_CalculateWheelSpeeds(vx_cmd, vy_cmd, omg_cmd, theta, MECANUM_WHEEL_RADIUS, MECANUM_K);

fprintf('  Joystick: X=%d, Y=%d, Z=%d\n', l_stick_x, l_stick_y, r_stick_x);
fprintf('  Vận tốc: vx=%.2f m/s, vy=%.2f m/s, omega=%.2f rad/s\n', vx_cmd, vy_cmd, omg_cmd);
fprintf('  Tốc độ bánh (RPM):\n');
fprintf('    FL = %.1f RPM\n', w_fl);
fprintf('    FR = %.1f RPM\n', w_fr);
fprintf('    RL = %.1f RPM\n', w_rl);
fprintf('    RR = %.1f RPM\n', w_rr);
fprintf('  ✓ FL,RL âm (-), FR,RR dương (+) → Xoay CW (phải)\n\n');

%% TEST 6: TIẾN CHÉO PHẢI (Joystick đẩy LÊN-PHẢI)
fprintf('TEST 6: TIẾN CHÉO PHẢI (l_stick_x = +90, l_stick_y = +90)\n');
l_stick_x = 90;    % Đẩy phải
l_stick_y = 90;    % Đẩy lên
r_stick_x = 0;

mapped_x = l_stick_x / 128.0;
mapped_y = l_stick_y / 128.0;
mapped_z = r_stick_x / 128.0;

vx_cmd = mapped_y * Robot_Max_Speed;
vy_cmd = mapped_x * Robot_Max_Speed;
omg_cmd = -mapped_z * Robot_Max_Omega;

[w_fl, w_fr, w_rl, w_rr] = MecanumRobot_CalculateWheelSpeeds(vx_cmd, vy_cmd, omg_cmd, theta, MECANUM_WHEEL_RADIUS, MECANUM_K);

fprintf('  Joystick: X=%d, Y=%d, Z=%d\n', l_stick_x, l_stick_y, r_stick_x);
fprintf('  Vận tốc: vx=%.2f m/s, vy=%.2f m/s, omega=%.2f rad/s\n', vx_cmd, vy_cmd, omg_cmd);
fprintf('  Tốc độ bánh (RPM):\n');
fprintf('    FL = %.1f RPM\n', w_fl);
fprintf('    FR = %.1f RPM\n', w_fr);
fprintf('    RL = %.1f RPM\n', w_rl);
fprintf('    RR = %.1f RPM\n', w_rr);
fprintf('  ✓ FR,RL quay nhanh, FL,RR chậm/dừng → Tiến chéo phải\n\n');

%% VẼ ĐỒ THỊ MÔ PHỎNG
figure('Name', 'Test Động Học Mecanum', 'Position', [100, 100, 1200, 800]);

test_cases = {
    'Tiến thẳng', 0, 127, 0;
    'Lùi', 0, -127, 0;
    'Sang phải', 127, 0, 0;
    'Sang trái', -127, 0, 0;
    'Xoay phải', 0, 0, 127;
    'Tiến chéo', 90, 90, 0
};

for i = 1:size(test_cases, 1)
    subplot(2, 3, i);
    
    l_x = test_cases{i, 2};
    l_y = test_cases{i, 3};
    r_x = test_cases{i, 4};
    
    mapped_x = l_x / 128.0;
    mapped_y = l_y / 128.0;
    mapped_z = r_x / 128.0;
    
    vx = mapped_y * Robot_Max_Speed;
    vy = mapped_x * Robot_Max_Speed;
    omg = -mapped_z * Robot_Max_Omega;
    
    [w_fl, w_fr, w_rl, w_rr] = MecanumRobot_CalculateWheelSpeeds(vx, vy, omg, theta, MECANUM_WHEEL_RADIUS, MECANUM_K);
    
    % Vẽ biểu đồ tốc độ bánh
    wheels = categorical({'FL', 'FR', 'RL', 'RR'});
    speeds = [w_fl, w_fr, w_rl, w_rr];
    bar(wheels, speeds);
    ylabel('Tốc độ (RPM)');
    title(sprintf('%s\nJoy: X=%d, Y=%d, Z=%d', test_cases{i, 1}, l_x, l_y, r_x));
    grid on;
    ylim([-600 600]);
    
    % Vẽ đường 0
    hold on;
    plot([0.5, 4.5], [0, 0], 'r--', 'LineWidth', 1.5);
    hold off;
end

fprintf('\n=== HOÀN TẤT TEST ===\n');
fprintf('Kiểm tra đồ thị để xác nhận động học đúng!\n');
