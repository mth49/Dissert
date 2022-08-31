%trans 0.1
r_chan1 = trans.Channel1 - Base.Channel1(1:length(trans.Channel1));
r_chan2 = trans.Channel2 - Base.Channel2(1:length(trans.Channel2));
r_chan3 = trans.Channel3 - Base.Channel3(1:length(trans.Channel3));
r_chan4 = trans.Channel4 - Base.Channel4(1:length(trans.Channel4));
%trans 0.2
r1_chan1 = trans1.Channel1 - Base.Channel1(1:length(trans1.Channel1));
r1_chan2 = trans1.Channel2 - Base.Channel2(1:length(trans1.Channel2));
r1_chan3 = trans1.Channel3 - Base.Channel3(1:length(trans1.Channel3));
r1_chan4 = trans1.Channel4 - Base.Channel4(1:length(trans1.Channel4));
%trans 0.3
r2_chan1 = trans2.Channel1 - Base.Channel1(1:length(trans2.Channel1));
r2_chan2 = trans2.Channel2 - Base.Channel2(1:length(trans2.Channel2));
r2_chan3 = trans2.Channel3 - Base.Channel3(1:length(trans2.Channel3));
r2_chan4 = trans2.Channel4 - Base.Channel4(1:length(trans2.Channel4));
%trans 0.4
r3_chan1 = trans3.Channel1 - Base.Channel1(1:length(trans3.Channel1));
r3_chan2 = trans3.Channel2 - Base.Channel2(1:length(trans3.Channel2));
r3_chan3 = trans3.Channel3 - Base.Channel3(1:length(trans3.Channel3));
r3_chan4 = trans3.Channel4 - Base.Channel4(1:length(trans3.Channel4));
%%resultant values
R = ((r_chan1 + r_chan2 +r_chan3 + r_chan4) / 4)
R1 = ((r1_chan1 + r1_chan2 +r1_chan3 + r1_chan4) / 4)
R2 = ((r2_chan1 + r2_chan2 +r2_chan3 + r2_chan4) / 4)
R3 = ((r3_chan1 + r3_chan2 +r3_chan3 + r3_chan4) / 4)


r_timer = trans.Time;
%%0.5 * density * 0.1^2 = ldyn delta V
Const = zeros(1,4)
for i = 1:4
    speed = [0.1,0.2,0.3,0.4];
    Const(1,i) = 0.5*1000*speed(i)^2;
end
Resultant = [mean(R)*(-1),mean(R1)*(-1),mean(R2)*(-1),mean(R3)*(-1)]
%Const ./ 
plot(Const,Resultant)
xlabel("Signal value")
ylabel("Dynamic head")
title("Verification of five-point probe values")
%plot(r_timer(1:60:end), resultr(1:60:end))
%xlabel("Time");
%ylabel("signal");
%title("Channel readings Data 0.1 - 0m/s")