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

% extract gravity component
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
title("Accel Y Gravity")
subplot(3,2,5)
plot(g_rot(3,:))
title("Accel Z Gravity")
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
title("Accel Y Gravity Filtered")
subplot(3,2,5)
plot(accel_z_filter_g)
title("Accel Z Gravity Filtered")
subplot(3,2,2)
plot(accel_x)
title("Accel X Raw Data")
subplot(3,2,4)
plot(accel_y)
title("Accel Y Raw Data")
subplot(3,2,6)
plot(accel_z)
title("Accel Z Raw Data")

% Analyze Gravity filtered data
accel_x = accel_x_filter_g;
accel_y = accel_y_filter_g;
accel_z = accel_z_filter_g;

% fft raw data
N_accel = length(accel_x);
N_gyro = length(gyro_x);
N_orien = length(yaw);
f_accel = linspace(-fs/2, fs/2 - fs/N_accel, N_accel) + fs/(2*N_accel)*mod(N_accel, 2);
f_gyro = linspace(-fs/2, fs/2 - fs/N_gyro, N_gyro) + fs/(2*N_gyro)*mod(N_gyro, 2);
f_orien = linspace(-fs/2, fs/2 - fs/N_orien, N_orien) + fs/(2*N_orien)*mod(N_orien, 2);
X_accel = fft(accel_x);
Y_accel = fft(accel_y);
Z_accel = fft(accel_z);
X_gyro = fft(gyro_x);
Y_gyro = fft(gyro_y);
Z_gyro = fft(gyro_z);
YAW = fft(yaw);
PITCH = fft(pitch);
ROLL = fft(roll);

% identify peak frequency 
% accel
[X_accel_sorted,X_accel_I] = sort(fftshift(abs(X_accel)),'descend');
maxfreq_X_accel = abs(f_accel(X_accel_I(1)));
maxfreq_X_accel_mag = X_accel_sorted(1);
[Y_accel_sorted,Y_accel_I] = sort(fftshift(abs(Y_accel)),'descend');
maxfreq_Y_accel = abs(f_accel(Y_accel_I(1)));
maxfreq_Y_accel_mag = Y_accel_sorted(1);
[Z_accel_sorted,Z_accel_I] = sort(fftshift(abs(Z_accel)),'descend');
maxfreq_Z_accel = abs(f_accel(Z_accel_I(1)));
maxfreq_Z_accel_mag = Z_accel_sorted(1);
% gyro
[X_gyro_sorted,X_gyro_I] = sort(fftshift(abs(X_gyro)),'descend');
maxfreq_X_gyro = abs(f_gyro(X_gyro_I(1)));
maxfreq_X_gyro_mag = X_gyro_sorted(1);
[Y_gyro_sorted,Y_gyro_I] = sort(fftshift(abs(Y_gyro)),'descend');
maxfreq_Y_gyro = abs(f_gyro(Y_gyro_I(1)));
maxfreq_Y_gyro_mag = Y_gyro_sorted(1);
[Z_gyro_sorted,Z_gyro_I] = sort(fftshift(abs(Z_gyro)),'descend');
maxfreq_Z_gyro = abs(f_gyro(Z_gyro_I(1)));
maxfreq_Z_gyro_mag = Z_gyro_sorted(1);
% orientation
[YAW_sorted,YAW_I] = sort(fftshift(abs(YAW)),'descend');
maxfreq_YAW = abs(f_orien(YAW_I(1)));
maxfreq_YAW_mag = YAW_sorted(1);
[PITCH_sorted,PITCH_I] = sort(fftshift(abs(PITCH)),'descend');
maxfreq_PITCH = abs(f_orien(PITCH_I(1)));
maxfreq_PITCH_mag =  PITCH_sorted(1);
[ROLL_sorted,ROLL_I] = sort(fftshift(abs(ROLL)),'descend');
maxfreq_ROLL = abs(f_orien(ROLL_I(1)));
maxfreq_ROLL_mag =  ROLL_sorted(1);

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

% plot gyro frequency domain
figure
subplot(3,1,1);
plot(f_gyro, fftshift(abs(X_gyro)));
xlabel("Frequency(Hz)");
ylabel("Amplitude");
title("Gyro x")
subplot(3,1,2);
plot(f_gyro, fftshift(abs(Y_gyro)));
xlabel("Frequency(Hz)");
ylabel("Amplitude");
title("Gyro y")
subplot(3,1,3);
plot(f_gyro, fftshift(abs(Z_gyro)));
xlabel("Frequency(Hz)");
ylabel("Amplitude");
title("Gyro z")

% plot orientation frequency domain
figure
subplot(3,1,1);
plot(f_orien, fftshift(abs(YAW)));
xlabel("Frequency(Hz)");
ylabel("Amplitude");
title("Yaw")
subplot(3,1,2);
plot(f_orien, fftshift(abs(PITCH)));
xlabel("Frequency(Hz)");
ylabel("Amplitude");
title("PITCH")
subplot(3,1,3);
plot(f_orien, fftshift(abs(ROLL)));
xlabel("Frequency(Hz)");
ylabel("Amplitude");
title("Roll")

% plot acceleration gyro data time domain
figure
subplot(3,3,1)
plot(gyro_x)
title("Gyro x")
subplot(3,3,4)
plot(gyro_y)
title("Gyro y")
subplot(3,3,7)
plot(gyro_z)
title("Gyro z")
subplot(3,3,2)
plot(accel_x)
title("Acceleration x")
subplot(3,3,5)
plot(accel_y)
title("Acceleration y")
subplot(3,3,8)
plot(accel_z)
title("Acceleration z")
subplot(3,3,3)
plot(yaw)
title("Yaw")
subplot(3,3,6)
plot(pitch)
title("Pitch")
subplot(3,3,9)
plot(roll)
title("Roll")

%%bandpass
% offset = 0.05;
% accel_x_bp = lowpass(highpass(accel_x,maxfreq_X_accel,fs),maxfreq_X_accel + offset,fs);
% accel_y_bp = lowpass(highpass(accel_y,maxfreq_Y_accel,fs),maxfreq_Y_accel + offset,fs);
% accel_z_bp = lowpass(highpass(accel_z,maxfreq_Z_accel,fs),maxfreq_Z_accel + offset,fs);
% gyro_x_bp = lowpass(highpass(gyro_x,maxfreq_X_gyro,fs),maxfreq_X_gyro + offset,fs);
% gyro_y_bp = lowpass(highpass(gyro_y,maxfreq_Y_gyro,fs),maxfreq_Y_gyro + offset,fs);
% gyro_z_bp = lowpass(highpass(gyro_z,maxfreq_Z_gyro,fs),maxfreq_Z_gyro + offset,fs);
% yaw_bp = lowpass(highpass(yaw,maxfreq_YAW,fs),maxfreq_YAW + offset,fs);
% pitch_bp = lowpass(highpass(pitch,maxfreq_PITCH,fs),maxfreq_PITCH + offset,fs);
% roll_bp = lowpass(highpass(roll,maxfreq_ROLL,fs),maxfreq_ROLL + offset,fs);

% bandpass second way
offset = 0.01;
accel_x_bp = bandpass(accel_x,[maxfreq_X_accel - offset, maxfreq_X_accel + offset],fs);
accel_y_bp = bandpass(accel_y,[maxfreq_Y_accel - offset, maxfreq_Y_accel + offset],fs);
accel_z_bp = bandpass(accel_z,[maxfreq_Z_accel - offset, maxfreq_Z_accel + offset],fs);
gyro_x_bp = bandpass(gyro_x,[maxfreq_X_gyro - offset, maxfreq_X_gyro + offset],fs);
gyro_y_bp = bandpass(gyro_y,[maxfreq_Y_gyro - offset, maxfreq_Y_gyro + offset] ,fs);
gyro_z_bp = bandpass(gyro_z,[maxfreq_Z_gyro - offset, maxfreq_Z_gyro + offset],fs);
yaw_bp = bandpass(yaw,[maxfreq_YAW - offset, maxfreq_YAW + offset],fs);
pitch_bp = bandpass(pitch,[maxfreq_PITCH - offset, maxfreq_PITCH + offset],fs);
roll_bp = bandpass(roll,[maxfreq_ROLL - offset, maxfreq_ROLL + offset], fs);

% fft bandpass data
X_accel_bp = fft(accel_x_bp);
Y_accel_bp = fft(accel_y_bp);
Z_accel_bp = fft(accel_z_bp);
X_gyro_bp = fft(gyro_x_bp);
Y_gyro_bp = fft(gyro_y_bp);
Z_gyro_bp = fft(gyro_z_bp);
YAW_bp = fft(yaw_bp);
PITCH_bp = fft(pitch_bp);
ROLL_bp = fft(roll_bp);


% plot accel band pass frequency domain
figure
subplot(3,1,1);
plot(f_accel, fftshift(abs(X_accel_bp)));
xlabel("Frequency(Hz)");
ylabel("Amplitude");
title("Accel x band pass")
subplot(3,1,2);
plot(f_accel, fftshift(abs(Y_accel_bp)));
xlabel("Frequency(Hz)");
ylabel("Amplitude");
title("Accel y band pass")
subplot(3,1,3);
plot(f_accel, fftshift(abs(Z_accel_bp)));
xlabel("Frequency(Hz)");
ylabel("Amplitude");
title("Accel z band pass")

% plot gyro band pass frequency domain
figure
subplot(3,1,1);
plot(f_gyro, fftshift(abs(X_gyro_bp)));
xlabel("Frequency(Hz)");
ylabel("Amplitude");
title("Gyro x band pass")
subplot(3,1,2);
plot(f_gyro, fftshift(abs(Y_gyro_bp)));
xlabel("Frequency(Hz)");
ylabel("Amplitude");
title("Gyro y band pass")
subplot(3,1,3);
plot(f_gyro, fftshift(abs(Z_gyro_bp)));
xlabel("Frequency(Hz)");
ylabel("Amplitude");
title("Gyro z band pass")

% plot orientation band pass frequency domain
figure
subplot(3,1,1);
plot(f_orien, fftshift(abs(YAW_bp)));
xlabel("Frequency(Hz)");
ylabel("Amplitude");
title("YAW band pass")
subplot(3,1,2);
plot(f_orien, fftshift(abs(PITCH_bp)));
xlabel("Frequency(Hz)");
ylabel("Amplitude");
title("Pitch band pass")
subplot(3,1,3);
plot(f_orien, fftshift(abs(ROLL_bp)));
xlabel("Frequency(Hz)");
ylabel("Amplitude");
title("Roll band pass")

% plot accel gyro orientation bandpass time domain
figure
subplot(3,3,1)
plot(gyro_x_bp)
title("Gyro x band pass")
subplot(3,3,4)
plot(gyro_y_bp)
title("Gyro y band pass")
subplot(3,3,7)
plot(gyro_z_bp)
title("Gyro z band pass")
subplot(3,3,2)
plot(accel_x_bp)
title("Acceleration x band pass")
subplot(3,3,5)
plot(accel_y_bp)
title("Acceleration y band pass")
subplot(3,3,8)
plot(accel_z_bp)
title("Acceleration z band pass")
subplot(3,3,3)
plot(yaw_bp)
title("Yaw band pass")
subplot(3,3,6)
plot(pitch_bp)
title("Pitch band pass")
subplot(3,3,9)
plot(roll_bp)
title("Roll band pass")



function res = pitchrot(alpha) % pitch
    res = [1 0 0; 0 cosd(alpha) sind(alpha); 0 -sind(alpha) cosd(alpha)];
end 

function res = rollrot(beta) % roll
    res = [cosd(beta) 0 sind(beta); 0 1 0; -sind(beta) 0 cosd(beta)];
end

function res = yawrot(theta) % yaw
    res = [cosd(theta) -sind(theta) 0; sind(theta) cosd(theta) 0; 0 0 1];
end
