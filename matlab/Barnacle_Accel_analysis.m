
T = Check.Time;
%trans 0.4
Accel_x = Check.myIMUax - Base.myIMUax(1:length(Check.myIMUax));
Accel_y = Check.myIMUay - Base.myIMUay(1:length(Check.myIMUay));
Accel_z = Check.myIMUaz - Base.myIMUaz(1:length(Check.myIMUaz));
r_chan1 = Check.Channel1 - Base.Channel1(1:length(Check.Channel1));
r_chan2 = Check.Channel2 - Base.Channel2(1:length(Check.Channel2));
r_chan3 = Check.Channel3 - Base.Channel3(1:length(Check.Channel3));
r_chan4 = Check.Channel4 - Base.Channel4(1:length(Check.Channel4));
%convert into magnitudes
r_chan1 = abs(r_chan1);
r_chan2 = abs(r_chan2);
r_chan3 = abs(r_chan3);
r_chan4 = abs(r_chan4);
%convert into meter per second squared and absolute values
Accel_x =abs(Accel_x * 9.81);
Accel_y = abs(Accel_y * 9.81);
Accel_z = abs(Accel_z * 9.81);
r_chan1 = r_chan1(1:60:end); r_chan2 = r_chan2(1:60:end); r_chan3=r_chan3(1:60:end); r_chan4 = r_chan4(1:60:end);
Accel_x = Accel_x(1:60:end); Accel_y = Accel_y(1:60:end); Accel_z=Accel_z(1:60:end);
T = T(1:60:end);
%create plot to show case changes present
figure; hold on
a1 = plot(T,Accel_x,'g'); M1 = ['IMU_x ' newline] ;
a2 = plot(T,Accel_y,'--ro'); M2 = ['IMU_y ' newline];
a3 =plot(T,Accel_z,'c*'); M3 = ['IMU_z' newline];
r1 = plot(T,r_chan1,'k'); M4 = ['Channel1 ' newline] ;
r2 = plot(T,r_chan2,'m'); M5 = ['Channel2 ' newline];
r3 =plot(T,r_chan3,'--b'); M6 = ['Channel3' newline];
r4 =plot(T,r_chan4,'k*'); M7 = ['Channel4' newline];
legend([a1,a2,a3,r1,r2,r3,r4],[M1,M2,M3,M4,M5,M6,M7]);
xlabel("Time")
ylabel("magnitude")
title("Barnacle output alongside Acceleration 0.4-0.3m/s")%


Q = cumtrapz(Accel_x); QQ = cumtrapz(Q);
K = cumtrapz(Accel_y); KK = cumtrapz(K);
L = cumtrapz(Accel_z); LL = cumtrapz(L);
figure; hold on
a1 = plot(T,QQ,'g'); M1 = ['IMU_x ' newline] ;
a2 = plot(T,KK,'--ro'); M2 = ['IMU_y ' newline];
%a3 =plot(T,LL,'c*'); M3 = 'IMU_z'
R1 = plot(T,r_chan1,'k'); M4 = ['Channel1 ' newline] ;
R2 = plot(T,r_chan2,'m'); M5 = ['Channel2 ' newline];
R3 =plot(T,r_chan3,'--r'); M6 = ['Channel3' newline];
R4 =plot(T,r_chan4,'k*'); M7 = ['Channel4' newline]
legend([a1,a2,R1,R2,R3,R4],[M1,M2,M4,M5,M6,M7]);
set(gca, 'YScale', 'log')
xlabel("Time")
ylabel("magnitude")
title("Barnacle signal and positioning data from double integration")