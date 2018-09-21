fs = 100; % 10Hz

% convert timetable to array and extract data
acceleration_table = timetable2table(Acceleration);
accel = table2array(acceleration_table(:,2:4));
orientation_table = timetable2table(Orientation);
orientation = table2array(orientation_table(:,2:4)); 
gyro_table = timetable2table(AngularVelocity);
gyro = table2array(gyro_table(:,2:4));
accel_x = accel(:,1);
accel_y = accel(:,2);
accel_z = accel(:,3);
gyro_x = gyro(:,1);
gyro_y = gyro(:,2);
gyro_z = gyro(:,3);
yaw = orientation(:,1);
pitch = orientation(:,2);
roll = orientation(:,3);

gravity = [0;0;9.8];
N = length(orientation);
g_rot = [];

for i = 1:N
    Xnew = pitchrot(pitch(i));
    Ynew = rollrot(roll(i));
    Znew = yawrot(yaw(i));
    
    g_new =  Ynew* Xnew * Znew * gravity; 
    
    
    g_rot = [g_rot g_new];
end

% plot effect of gravity
figure
subplot(3,2,1)
plot(g_rot(1,:))
title("Accel X Gravity")

subplot(3,2,3)
plot(g_rot(2,:))
title("Accel X Gravity")

subplot(3,2,5)
plot(g_rot(3,:))
title("Accel X Gravity")

subplot(3,2,2)
plot(accel_x)
title("Accel X Raw Data")

subplot(3,2,4)
plot(accel_y)
title("Accel Y Raw Data")

subplot(3,2,6)
plot(accel_z)
title("Accel Z Raw Data")

% Cancel Effect of Gravity
accel_x_filter_g = accel_x - transpose(g_rot(1,:));
accel_y_filter_g = accel_y - transpose(g_rot(2,:));
accel_z_filter_g = accel_z - transpose(g_rot(3,:));

% plot gravity filtered
figure
subplot(3,2,1)
plot(accel_x_filter_g)
title("Accel X Gravity Filtered")

subplot(3,2,3)
plot(accel_y_filter_g)
title("Accel X Gravity Filtered")

subplot(3,2,5)
plot(accel_z_filter_g)
title("Accel X Gravity Filtered")

subplot(3,2,2)
plot(accel_x)
title("Accel X Raw Data")

subplot(3,2,4)
plot(accel_y)
title("Accel Y Raw Data")

subplot(3,2,6)
plot(accel_z)
title("Accel Z Raw Data")

% fft raw data
N_accel = length(accel_x);
N_gyro = length(gyro_x);
N_orien = length(yaw);
f_accel = linspace(-fs/2, fs/2 - fs/N_accel, N_accel) + fs/(2*N_accel)*mod(N_accel, 2);
f_gyro = linspace(-fs/2, fs/2 - fs/N_gyro, N_gyro) + fs/(2*N_gyro)*mod(N_gyro, 2);
f_orien = linspace(-fs/2, fs/2 - fs/N_orien, N_orien) + fs/(2*N_orien)*mod(N_orien, 2);
X_accel = fft(accel_x_filter_g);
Y_accel = fft(accel_y_filter_g);
Z_accel = fft(accel_z_filter_g);
X_gyro = fft(gyro_x);
Y_gyro = fft(gyro_y);
Z_gyro = fft(gyro_z);
YAW = fft(yaw);
PITCH = fft(pitch);
ROLL = fft(roll);

% plot accel frequency domain
figure
subplot(3,1,1);
plot(f_accel, fftshift(abs(X_accel)));
xlabel("Frequency(Hz)");
ylabel("Amplitude");
title("Accel x")
subplot(3,1,2);
plot(f_accel, fftshift(abs(Y_accel)));
xlabel("Frequency(Hz)");
ylabel("Amplitude");
title("Accel y")
subplot(3,1,3);
plot(f_accel, fftshift(abs(Z_accel)));
xlabel("Frequency(Hz)");
ylabel("Amplitude");
title("Accel z")


function res = pitchrot(alpha) % pitch
    res = [1 0 0; 0 cosd(alpha) sind(alpha); 0 -sind(alpha) cosd(alpha)];
end 

function res = rollrot(beta) % roll
    res = [cosd(beta) 0 sind(beta); 0 1 0; -sind(beta) 0 cosd(beta)];
end

function res = yawrot(theta) % yaw
    res = [cosd(theta) -sind(theta) 0; sind(theta) cosd(theta) 0; 0 0 1];
end