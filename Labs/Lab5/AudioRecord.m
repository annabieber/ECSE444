% Record your voice for 5 seconds.
recObj = audiorecorder;
disp('Start speaking.')
recordblocking(recObj, 5);
disp('End of Recording.');

% Play back the recording.
play(recObj);

% Store data in double-precision array.
myRecording = getaudiodata(recObj);
figure; plot(myRecording); % Plot the original waveform.

% cut the area you want and convert it into integers
MyAudioArray = uint16((myRecording(1000:15000)+1)*1024/2); 
csvwrite('C:\Users\annab\Documents\MATLAB\AudioArray.csv',MyAudioArray');
% Plot the modified waveform.
figure;plot(MyAudioArray);
