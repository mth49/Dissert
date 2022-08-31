
T = two_one.Time;
Accel_x = two_one.myIMUax - Base.myIMUax(1:length(two_one.myIMUax));
Accel_y = two_one.myIMUay - Base.myIMUay(1:length(two_one.myIMUay));
Accel_z= two_one.myIMUaz - Base.myIMUaz(1:length(two_one.myIMUaz));

figure; hold on

a1 = plot(T,Accel_x,'g'); M1 = ['IMU_x ' newline] ;
a2 = plot(T,Accel_y,'--ro'); M2 = ['IMU_y ' newline];
a3 =plot(T,Accel_z,'c'); M3 = 'IMU_z'
legend([a1,a2,a3],[M1,M2,M3]);
xlabel("Time")
ylabel("Acceleration in g's")
title("Normalised Accel 0.2-0.1m/s")

Len = length(T)
Grav = [9.81]
time = 1:1:Len
time = time'
%convert into m/s^2
Accel_x = Accel_x * Grav
Accel_y = Accel_y * Grav
Accel_z = Accel_z * Grav

Q = cumtrapz(Accel_x); QQ = cumtrapz(Q);
K = cumtrapz(Accel_y); KK = cumtrapz(K);
L = cumtrapz(Accel_z); LL = cumtrapz(L)
QQ = QQ(1:60:end); KK = KK(1:60:end); LL=LL(1:60:end);
T = time(1:60:end);

Q = cumtrapz(Accel_x); QQ = cumtrapz(Q);
K = cumtrapz(Accel_y); KK = cumtrapz(K);
L = cumtrapz(Accel_z); LL = cumtrapz(L)
QQ = QQ(1:60:end); KK = KK(1:60:end); LL=LL(1:60:end);
T = time(1:60:end);

figure; hold on
%Accel_x = linspace(-2,2,200); 
%Accel_y = linspace(-2,2,200); 
%Accel_z = linspace(-2,2,200); 
%axis(T(0) T(-1) -1 1)

a1 = plot(T,QQ,'g'); M1 = ['IMU_x ' newline] ;
a2 = plot(T,KK,'--ro'); M2 = ['IMU_y ' newline];
a3 =plot(T,LL,'c'); M3 = 'IMU_z'
legend([a1,a2,a3],[M1,M2,M3]);
xlabel("Time");
ylabel("Displacement");
title("Displacement (double integration) Data 0.2-0.1m/s")