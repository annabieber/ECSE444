clear;
clc;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Sine wave generation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Mixing the sine waves
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
s = [s1;s2];

randn('seed', 1);
A = randn(2,2);
x = A*s;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Receive and transmit from keil
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
s = serial('COM1');
set(s,'BaudRate',4800);
fopen(s);
fprintf(s,'*IDN?');
out = fscanf(s);
fclose(s);
delete(s);
clear s

%todo need to be able to receive values from keil project 


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Performing FastICA
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
[u, A_est, W] = fastica(x);

%transmit values
%fprintf(keil,'%d', u);
%fclose(keil);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Plotting and display
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

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
plot(u(1,:));
title('u1(t)');

subplot(3,2,6);
plot(u(2,:));
title('u2(t)');


display('The estimated mixing matrix is '); display(A_est);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% OPTIONALLY, figure out how to transform the extimate to the original
% YOU CANNOT DO THIS IF YOU DID NOT KNOW WHAT THE ORIGINAL 'A' WAS !!!
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
display('Original mixing matrix: ');
display(A);

swap_transform = [[0,1];[1,0]];
t1 = A./A_est;
t2 = A./(A_est*swap_transform);
t3 = A./(swap_transform*A_est);

x_t1 = t1-t1(1,1);
x_t2 = t2-t2(1,1);
x_t3 = t3-t3(1,1);

dets = [abs(det(x_t1)), abs(det(x_t2)), abs(det(x_t3))];
[min,idx] = min([abs(det(x_t1)), abs(det(x_t2)), abs(det(x_t3))]);

if(idx == 1)
   display(...
   sprintf('The estimated matrix has been scaled by a factor of %.2f', ...
   abs(mean2(t1))));
   A_est_fixed = A_est.*t1;
elseif(idx == 2)
   display('The columns of the estimated matrix have been swapped and');
   display(...
   sprintf('the estimated matrix has been scaled by a factor of %.2f', ...
   abs(mean2(t2))));
   A_est_fixed = A_est*swap_transform.*t2;
elseif(idx == 3)
   display('The rows of the estimated matrix have been swapped and');
   display(...
   sprintf('the estimated matrix has been scaled by a factor of %.2f', ...
   abs(mean2(t3))));
   A_est_fixed = swap_transform*A_est.*t2;
end

display('Fixed estimated mixing matrix: ');
display(A_est_fixed);

display('NOTE: Not reporting whether the sign of the mixing columns have');
display('been flipped, this is left for you to figure out as an exercise');