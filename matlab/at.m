r_chan1 = trans2.Channel1 - Base.Channel1(1:length(trans2.Channel1));
r_chan2 = trans2.Channel2 - Base.Channel2(1:length(trans2.Channel2));
r_chan3 = trans2.Channel3 - Base.Channel3(1:length(trans2.Channel3));
r_chan4 = trans2.Channel4 - Base.Channel4(1:length(trans2.Channel4));
r_time = trans2.Time;
result = ((r_chan1 + r_chan2 +r_chan3 + r_chan4) / 4)
plot(r_time(1:60:23606), result(1:60:23606))
xlabel("Time");
ylabel("signal");
title("Channel Data 0.2-0.1 m/s")