T = Base.Time;
Accel_x = Base.myIMUax;
Accel_y = Base.myIMUay;
Accel_z= Base.myIMUaz;

a1 = plot(T,Accel_x,'--'); M1 = ['IMU_x ' newline] ;
a2 = plot(T,Accel_y,'--ro'); M2 = ['IMU_y ' newline];
a3 =plot(T,Accel_z,'c*'); M3 = 'IMU_z'
legend([a1,a2,a3],[M1,M2,M3]);

xlabel("Time");
ylabel("Acceleration");
title("Displacement (double integration) Data 0m/s")



Q = cumtrapz(Accel_x); QQ = cumtrapz(Q);
K = cumtrapz(Accel_y); KK = cumtrapz(K);
L = cumtrapz(Accel_z); LL = cumtrapz(L)


figure; hold on
%Accel_x = linspace(-2,2,200); 
%Accel_y = linspace(-2,2,200); 
%Accel_z = linspace(-2,2,200); 
%axis(T(0) T(-1) -1 1)

a1 = plot(T,QQ,'--'); M1 = ['IMU_x ' newline] ;
a2 = plot(T,KK,'--ro'); M2 = ['IMU_y ' newline];
a3 =plot(T,LL,'c*'); M3 = 'IMU_z'
legend([a1,a2,a3],[M1,M2,M3]);
xlabel("Time");
ylabel("Displacement");
title("Displacement (double integration) Data 0m/s")