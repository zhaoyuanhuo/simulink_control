
%%
J = 0.01;
b =  0.1;
Ke = 0.01;
Kt = 0.01;
R = 1;
L = 0.5;

subplot(2,1,1);
plot(out.tout,out.ulag);
xlabel('time (seconds)')
ylabel('control effort (volts)')
title('Control Effort Under Lag Compensation')
subplot(2,1,2)
plot(out.tout,out.ulead);
xlabel('time (seconds)')
ylabel('control effort (volts)')
title('Control Effort Under Lead Compensation')