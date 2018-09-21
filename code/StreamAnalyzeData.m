fs = 10; % 10Hz

while 1 == 1 
   [orient, t] = orientlog(m);
   [accel, t] = accellog(m);
    accel_x = accel(:,1);
    accel_y = accel(:,2);
    accel_z = accel(:,3);
    
    yaw = orient(:,1);
    pitch = orient(:,2);
    roll = orient(:,3);

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

    
    subplot(3,2,1)
    plot(accel_x)
    title("x")
    subplot(3,2,3)
    plot(accel_y)
    title("y")
    subplot(3,2,5)
    plot(accel_z)
    title("z")
    subplot(3,2,2)
    plot(accel_x_lp)
    title("x_lp")
    subplot(3,2,4)
    plot(accel_y_lp)
    title("y_lp")
    subplot(3,2,6)
    plot(accel_z_lp)
    title("z_lp")
    
    pause(0.05)
end 
