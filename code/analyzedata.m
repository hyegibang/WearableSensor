fs = 10; % 10Hz

% convert timetable to array and extract data
acceleration_table = timetable2table(Acceleration);
accel = table2array(acceleration_table(:,2:4));
orientation_table = timetable2table(Orientation);
gyro = table2array(orientation_table(:,2:4));
accel_x = accel(:,1);
accel_y = accel(:,2);
accel_z = accel(:,3);
gyro_x = gyro(:,1);
accel_y = accel(:,2);
accel_z = accel(:,3);

% subtract mean of data
accel_x = accel_x - mean(accel_x);
accel_y = accel_y - mean(accel_y);
accel_z = accel_z - mean(accel_z);

% fft raw data
N = length(accel_x);
f = linspace(-fs/2, fs/2 - fs/N, N) + fs/(2*N)*mod(N, 2);
X_accel = fft(accel_x);
Y_accel = fft(accel_y);
Z_accel = fft(accel_z);

% low pass
accel_x_lp = lowpass(accel_x,0.06,fs);
accel_y_lp = lowpass(accel_y, 0.06,fs);
accel_z_lp = lowpass(accel_z, 0.06,fs);

% fft low pass data
X_accel_lp = fft(accel_x_lp);
Y_accel_lp = fft(accel_y_lp);
Z_accel_lp = fft(accel_z_lp);

%figure
%hold on
%plot(f,fftshift(abs(X_accel)))
%plot(f,fftshift(abs(X_accel_lp)))
%hold off
%title("fft-lowpass")
%legend("unfiltered","filtered")

% plot data time domain
figure
subplot(3,2,1)
plot(accel_x)
title("x -time")
subplot(3,2,3)
plot(accel_y)
title("y - time")
subplot(3,2,5)
plot(accel_z)
title("z - time")
subplot(3,2,2)
plot(accel_x_lp)
title("x low pass")
subplot(3,2,4)
plot(accel_y_lp)
title("y low pass")
subplot(3,2,6)
plot(accel_z_lp)
title("z low pass")

% plot orientation 

yaw = gyro(:,1);
pitch = gyro(:,2);
roll = gyro(:,3);

figure;
subplot(3,1,1)
plot(yaw)
title("yaw")
subplot(3,1,2)
plot(pitch)
title("pitch")
subplot(3,1,3)
plot(roll)s
title("roll")


