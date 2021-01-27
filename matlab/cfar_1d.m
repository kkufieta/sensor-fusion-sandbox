% Implement 1D CFAR using lagging cells on the given noise and target scenario.
% Steps to follow:
% 1. Define the number of training cells and guard cells
% 2. Start sliding the window one cell at a time across the complete 
%    FFT 1D array. Total window size should be: 2(T+G)+CUT
% 3. For each step, sum the signal (noise) within all the leading or 
%    lagging training cells
% 4. Average the sum to determine the noise threshold
% 5. Using an appropriate offset value to scale the threshold
% 6. Now, measure the signal in the CUT, which is T+G+1 from the window 
%    starting point
% 7. Compare the signal measured in 5 against the threshold measured in 4
% 8. If the level of signal measured in CUT is smaller than the threshold 
%    measured, then assign 0 value to the signal within CUT.

% Close and delete all currently open figures
close all;

% Data_points
Ns = 1000;

% Generate random noise
s=abs(randn(Ns,1));

%Targets location. Assigning bin 100, 200, 300 and 700 as Targets with the amplitudes of 8, 9, 4, 11.
s([100 ,200, 300, 700])=[8 9 4 11];

%plot the output
plot(s);
title('Original signal');

% Apply CFAR to detect the targets by filtering the noise.

% 1. Define the number of training cells and guard cells
T = 12;
G = 4;

% Offset : Adding room above noise threshold for desired SNR 
offset=5;

% Vector to hold threshold values 
threshold_cfar = [];

%Vector to hold final signal after thresholding
signal_cfar = [];

% 2. Slide window across the signal length
for i = 1:(Ns-(G+T))     

    % 2. - 5. Determine the noise threshold by measuring it within the
    % training cells:
    
    % 2. Start sliding the window one cell at a time across the complete 
    %    FFT 1D array. Total window size should be: 2(T+G)+CUT
    % 3. For each step, sum the signal (noise) within all the leading or 
    %    lagging training cells. Here: use lagging cells
    % 4. Average the sum to determine the noise threshold
    % 5. Using an appropriate offset value to scale the threshold
    noise_threshold = mean(s(i:i+T-1)) * offset;
    
    % 6. Now, measure the signal within the CUT, which is T+G+1 from the
    % window starting point
    signal = s(i+T+G);
    % 7. Compare the signal measured in 5 against the threshold measured in
    % 4 (??)
    % 8. Filter the signal above the threshold
    if signal < noise_threshold
        signal = 0;
    end
    threshold_cfar = [threshold_cfar, {noise_threshold}];
    signal_cfar = [signal_cfar, {signal}];
end



figure
% plot the filtered signal
plot (cell2mat(signal_cfar),'g--');
title('Filtered signal')

% plot original sig, threshold and filtered signal within the same figure.
figure,plot(s);
title('CFAR 1D')
hold on,plot(cell2mat(circshift(threshold_cfar,G)),'r--','LineWidth',2)
hold on, plot (cell2mat(circshift(signal_cfar,(T+G))),'g--','LineWidth',4);
legend('Signal','CFAR Threshold','detection')