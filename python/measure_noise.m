clear 

T = 0.01;
N = 10000;
%% IMU setup

imu.COMPort = 'COM3';
imu.baudrate = 921600;
imu.handle = lpms();

% Connect to sensor
if ( ~imu.handle.connect(imu.COMPort, imu.baudrate) )
    return
end
disp('sensor connected')

% Set streaming mode
imu.handle.setStreamingMode();

%% Retrieve data 
D = zeros(9,N);
fprintf("gathering data for %f seconds\n",N*T);
for i = 1:N
    pause(T);
    d = imu.handle.getCurrentSensorData();
    
    D(:,i) = [d.gyr,d.acc,d.mag]';
end

%% Means and Vars
medias = zeros(9,1);
varianzas = zeros(9,1);
for i = 1:9
    [medias(i),varianzas(i)] = medVar(D(i,:));
end

fprintf("\nMEDIAS Y VARIANZAS\n");
fprintf("gyrX: %f, %f\n",medias(1),varianzas(1));
fprintf("gyrY: %f, %f\n",medias(2),varianzas(2));
fprintf("gyrZ: %f, %f\n",medias(3),varianzas(3));
fprintf("\n");
fprintf("accX: %f, %f\n",medias(4),varianzas(4));
fprintf("accY: %f, %f\n",medias(5),varianzas(5));
fprintf("accZ: %f, %f\n",medias(6),varianzas(6));
fprintf("\n");
fprintf("magX: %f, %f\n",medias(7),varianzas(7));
fprintf("magY: %f, %f\n",medias(8),varianzas(8));
fprintf("magZ: %f, %f\n",medias(9),varianzas(9));

%% Functions
function [media,varianza] = medVar(X)

sumaX = 0;
sumX2 = 0;
n = length(X);

for i=1:n
    sumaX = sumaX + X(i);
    sumX2 = sumX2 + X(i)^2;
end

media = sumaX/n;
varianza = (sumX2 - (media^2)*n)/(n-1);
end