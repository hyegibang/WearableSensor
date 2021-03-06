fs = 100; % 100Hz

% convert timetable to array and extract data
orientation = [seconds(Orientation.Timestamp - Orientation.Timestamp(1)) Orientation.Variables];
yaw = orientation(:,2);
pitch = orientation(:,3);
roll = orientation(:,4);
world_x = [1,0,0];
world_y = [0,1,0];
world_z = [0,0,1];

% change world axis to phone axis
N = length(orientation);
phone_x = rpy_world2local(N,world_x,orientation);
phone_y = rpy_world2local(N,world_y,orientation);
phone_z = rpy_world2local(N,world_z,orientation);

% subtract mean
phone_x = phone_x - mean(phone_x);
phone_y = phone_y - mean(phone_y);
phone_z = phone_z - mean(phone_z);

% fft on phone axis
phone_X = fftn(phone_x);
phone_Y = fftn(phone_y);
phone_Z = fftn(phone_z);
N_orien = length(yaw);
f_orien = linspace(-fs/2, fs/2 - fs/N_orien, N_orien) + fs/(2*N_orien)*mod(N_orien, 2);

% plot orientation frequency domain y axis
figure
subplot(3,1,1);
plot(f_orien, fftshift(abs(phone_Y(:,1))));
xlabel("Frequency(Hz)");
ylabel("Amplitude");
title("Phone Y axis i")
subplot(3,1,2);
plot(f_orien, fftshift(abs(phone_Y(:,2))));
xlabel("Frequency(Hz)");
ylabel("Amplitude");
title("Phone Y axis j")
subplot(3,1,3);
plot(f_orien, fftshift(abs(phone_Y(:,3))));
xlabel("Frequency(Hz)");
ylabel("Amplitude");
title("Phone Y axis k")

function res = pitchrot(alpha) % pitch
    res = [1 0 0; 0 cosd(alpha) sind(alpha); 0 -sind(alpha) cosd(alpha)];
end 

function res = rollrot(beta) % roll
    res = [cosd(beta) 0 sind(beta); 0 1 0; -sind(beta) 0 cosd(beta)];
end

function res = yawrot(theta) % yaw
    res = [cosd(theta) -sind(theta) 0; sind(theta) cosd(theta) 0; 0 0 1];
end

function res = rpy_world2local(n, v,orientation)
    axis = [];
    for i = 1:n
        YawMat = yawrot(orientation(i,2));
        PitchMat = pitchrot(orientation(i,3));
        RollMat = rollrot(orientation(i,4));
        new_axis =  RollMat'*PitchMat'*YawMat'*transpose(v); 
        axis = [axis; transpose(new_axis)]; 
    end
    res = axis;
end