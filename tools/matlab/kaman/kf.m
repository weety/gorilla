% Generate fuzzy control lookup table
% Author: weety
% 2019/06/13

clear all;
PI = 3.1415926;

% read data from file
fid = fopen('sensor_data_delta2.txt');    % change the file name to your test file
data = fscanf(fid, '%f %f %f %f %f %f', [6 inf]);
data = data';
fclose(fid);

acc = data(:, 1:3);
gyr = data(:, 4:6);

dt = 0.01;

A = [1, -dt; 0,1];
B = [dt; 0];
Q = [0.00001, 0; 0, 0.00003];
R = 0.5;
H = [1,0];
I = eye(2,2);

Gyro = gyr(:,1);
zk = atan(acc(:,2)./sqrt(acc(:,1).^2 + acc(:,3).^2));

X=[0;0];
P=[[1, 0]; [0, 1]];
K=[[0]; [0]];

data_len = size(acc,1);
data_range = data_len;

Angle = zeros(data_len,1);
Qbias = zeros(data_len,1);
Gyro_final = zeros(data_len,1);

% kalman filter
for i = 1:data_range
    % X(k|k-1) = A ? X(k?1|k-1) + B?u(k)
    X = A * X + B * (Gyro(i) - X(2));
    %P(k|k-1) = A ? P(k?1|k-1) ? AT + Q
    P = A * P * A' + Q;
    %K(k) = P(k|k-1) ? HT / (H ? P(k|k-1) ? HT + R)
    K = P * H' * pinv(H * P * H' + R);
    %e = Z(k) 每 H ? X(k|k-1)
    %X(k|k) = X(k|k-1) + K ? e = X(k|k-1) + K ? (Z(k) 每 H ? X(k|k-1))
    X = X + K * (zk(i) - H * X);
    %P(k|k) = P(k|k-1) 每 K ? H ? P(k|k-1) = (I 每 K * H) * P(k|k-1)
    P = (I - K * H) * P;
    %Y(k) = H ? X(k|k)
    Angle(i) = X(1);
    Qbias(i) = X(2);
    Gyro_final(i) = Gyro(i) - X(2);
end

%draw wave picture
figure;
h1=plot(zk*180/PI, 'g--');
%plot(zk*180/PI, 'g--', Angle*180/PI, 'b-','LineWidth',2);
hold on;
h2=plot(Angle*180/PI, 'b-','LineWidth',1.2);
legend('Angle\_acc','Angle\_filter');
xlabel('Time(ms)');
ylabel('Angle(～)');
title('Angle filter');
grid on;

figure;
hold on;
plot(Gyro, 'r--');
hold on;
plot(Gyro_final, 'b-');
hold on;
plot(Qbias, 'g--');
hold on;
legend('Gyro','Gyro\_final','Qbias');
xlabel('Time(ms)');
ylabel('Angular velocity(rad)');
title('Angle velocity filter');
grid on;