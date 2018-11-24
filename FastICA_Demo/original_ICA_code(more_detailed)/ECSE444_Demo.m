Ts = 16000; % samples/second
time = 0.1; % seconds
t = 0:1/Ts:time-1/Ts;

n=2;
N=Ts*time;

f1 = 261.63;
w1 = 2*pi*f1;
s1 = sin(w1*t);

f2 = 392;
w2 = 2*pi*f2;
s2 = sin(w2*t);

s = [s1;s2];

randn('seed', 1);
A = randn(2,2);
x = A*s;

[UU, AA, WW] = fastica(x);

subplot(3,2,1);
plot(s(1,:));
title('s1(t)');

subplot(3,2,2);
plot(s(2,:));
title('s2(t)');

subplot(3,2,3);
plot(x(1,:));
title('x1(t)');

subplot(3,2,4);
plot(x(2,:));
title('x2(t)');

subplot(3,2,5);
plot(UU(1,:));
title('u1(t)');

subplot(3,2,6);
plot(UU(2,:));
title('u2(t)');