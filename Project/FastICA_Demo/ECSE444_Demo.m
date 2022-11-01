clear;
clc;

A = [0.5,0.5;
    0.34,0.66];
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Sine wave generation
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
Ts = 16000; % samples/second
time = 0.1; % seconds
t = 0:1/Ts:time-1/Ts;

n=2;
N=Ts*time;

f1 = 440;
w1 = 2*pi*f1;
s1 = sin(w1*t);

f2 = 392;
w2 = 2*pi*f2;
s2 = sin(w2*t);
% 
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% % Mixing the sine waves
% %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
 s = [s1;s2];
% 
% randn('seed', 1);
% A = randn(2,2);
% x = A*s;

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Receiving mixed signals from the board
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Create a serial port object.
obj1 = instrfind('Type', 'serial', 'Port', 'COM7', 'Tag', '');

% Create the serial port object if it does not exist
% otherwise use the object that was found.
if isempty(obj1)
    obj1 = serial('COM7', 'BaudRate', 115200);
    obj1.OutputBufferSize = 2*32000*32;
    obj1.InputBufferSize = 2*32000*32;
    obj1.timeout = 100.0;
else
    fclose(obj1);
    obj1 = obj1(1);
end

% Connect to instrument object, obj1.
fopen(obj1);

% s_1 = fread(obj1,32000*32/4,'uint32');
% s_2 = fread(obj1,32000*32/4,'uint32');
% s = [s_1;s_2];

% Receiving 1 sample at a time
x_1 =zeros(1,32000);
for i = 1:32000
    x_1(1,i) = fread(obj1, 1,'float32');
end
x_2 =zeros(1,32000);
for j= 1:32000
    x_2(1,j) = fread(obj1, 1,'float32');
end
x = [x_1;x_2];
		


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Performing FastICA
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
[u, A_est, W] = fastica(x);


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%Attempting to send values back to keil
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% u_1 = u(1,:);
% for r = 1:32000
%     u_1(1,r) = fprintf(obj1, 'float32',1);
% end
% u_2 = u(2,:);
% for s= 1:32000
%     u_2(2,s) = fprintf(obj1,'float32',1);
% end
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

u_fixed = inv(A_est_fixed)*x;
 fwrite(obj1, 's');
 pause(0.5);
for i=1:16000
    fwrite(obj1, u_fixed(1,i), 'float32');
    fwrite(obj1, u_fixed(2,i), 'float32');
    pause(0.010);
end

%fwrite(obj1, u_fixed(1,1:1000), 'float32', 'sync');
%fwrite(obj1, u_fixed(2,1:5000), 'float32', 'sync');

% Disconnect from instrument object, obj1.
fclose(obj1);

% Clean up all objects.
delete(obj1);

subplot(3,2,5);
plot(u_fixed(1,:));
title('u1(t)');

subplot(3,2,6);
plot(u_fixed(2,:));
title('u2(t)');