clear all, close all
clc;

%% Radar Specifications 
%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Frequency of operation = 77GHz
% Max Range = 200m
% Range Resolution = 1 m
% Max Velocity = 100 m/s
%%%%%%%%%%%%%%%%%%%%%%%%%%%

c = 3e8;                % speed of light [m/s]

%Operating carrier frequency of Radar 
fc = 77e9;               % carrier freq [1/s], 77GHz
dres = 1;               % range resolution [m]
Rmax = 200;             % max range [m]

%% User Defined Range and Velocity of target
% Define the target's initial position and velocity. Note : Velocity
% remains constant

% Note: Range can not exceed 200 m
% Velocity can be between -70 to 70 m/s
R = 110;    % Initial distance of the target [m]
v = -20;    % Speed of the target [m/s]

%% FMCW Waveform Generation

% Design the FMCW waveform by giving the specs of each of its parameters.
% Calculate the Bandwidth (B), Chirp Time (Tchirp) and Slope (slope) of the FMCW
% chirp using the requirements above.

B = c / (2 * dres);
Tchirp = 5.5 * 2 * Rmax / c;
Slope = B / Tchirp;
disp(["Slope is around 2e13? ", Slope/2e13]);

% The number of chirps in one sequence. Its ideal to have 2^ value for the 
% ease of running the FFT for Doppler Estimation. 
Nd = 128;                 % #of doppler cells OR 
                        % #of sent periods % number of chirps

% The number of samples on each chirp. 
Nr = 1024;                  %for length of time OR # of range cells

% Timestamp for running the displacement scenario for every sample on each
% chirp
t = linspace(0,Nd*Tchirp,Nr*Nd); %total time for samples


% Creating the vectors for Tx, Rx and Mix based on the total samples input.
Tx = zeros(1,length(t));      % transmitted signal
Rx = zeros(1,length(t));      % received signal
Mix = zeros(1,length(t));   % beat signal

% Similar vectors for range_covered and time delay.
r_t = zeros(1,length(t));
td = zeros(1,length(t));

%% Signal generation and Moving Target simulation
% Running the radar scenario over the time. 

for i=1:length(t)         
    
    % For each time stamp update the Range of the Target for constant velocity. 
    r_t(i) = R + v * t(i);
    
    % For each time sample we need update the transmitted and
    % received signal. 
    td(i) = 2 * r_t(i) / c;
    Tx(i) = cos(2 * pi * (fc * t(i) + Slope * t(i)^2 / 2));
    Rx(i) = cos(2 * pi * (fc * (t(i) - td(i)) + Slope * (t(i) - td(i))^2 / 2));
    
    % Now by mixing the Transmit and Receive generate the beat signal
    % This is done by element wise matrix multiplication of Transmit and
    % Receiver Signal
    Mix(i) = Tx(i) .* Rx(i);
    
end

%% RANGE MEASUREMENT

% Reshape the vector into Nr*Nd array. Nr and Nd here would also define 
% the size of Range and Doppler FFT respectively.
signal = reshape(Mix, [Nr * Nd, 1]);

% Run the FFT on the beat signal along the range bins dimension (Nr) and
% normalize.
signal_fft = fft(signal, Nr)./Nr;

% Take the absolute value of FFT output
signal_fft = abs(signal_fft);

% Output of FFT is double sided signal, but we are interested in only one 
% side of the spectrum. Hence we throw out half of the samples.
signal_fft = signal_fft(1:Nr/2+1);

% Plotting the range
figure ('Name','Range from First FFT')
subplot(2,1,1)
plot(r_t);

% plot FFT output 
subplot(2,1,2)
plot(signal_fft);
 
axis ([0 200 0 0.4]);

%% RANGE DOPPLER RESPONSE
% The 2D FFT implementation is already provided here. This will run a 2DFFT
% on the mixed signal (beat signal) output and generate a range doppler
% map.You will implement CFAR on the generated RDM

% Range Doppler Map Generation.

% The output of the 2D FFT is an image that has reponse in the range and
% doppler FFT bins. So, it is important to convert the axis from bin sizes
% to range and doppler based on their Max values.
Mix = reshape(Mix,[Nr,Nd]);

% 2D FFT using the FFT size for both dimensions.
sig_fft2 = fft2(Mix,Nr,Nd);

% Taking just one side of signal from Range dimension.
sig_fft2 = sig_fft2(1:Nr/2,1:Nd);
sig_fft2 = fftshift (sig_fft2);
RDM = abs(sig_fft2);
RDM = 10*log10(RDM);

% Use the surf function to plot the output of 2DFFT and to show axis in
% both dimensions.
doppler_axis = linspace(-100,100,Nd);
range_axis = linspace(-200,200,Nr/2)*((Nr/2)/400);
figure ('Name','Range Doppler Map')
surf(doppler_axis,range_axis,RDM);

%% CFAR implementation

% Slide Window through the complete Range Doppler Map

% Select the number of Training Cells in both the dimensions.
Tr = 10;
Tc = 8;

% Select the number of Guard Cells in both dimensions around the Cell under 
% test (CUT) for accurate estimation
Gr = 4;
Gc = 4;

% offset the threshold by SNR value in dB
offset = 10;

% Create a vector to store noise_level for each iteration on training cells
noise_level = zeros(1,1);


% Design a loop such that it slides the CUT across range doppler map by
% giving margins at the edges for Training and Guard Cells.

% For every iteration sum the signal level within all the training
% cells. To sum convert the value from logarithmic to linear using db2pow
% function. Average the summed values for all of the training
% cells used. After averaging convert it back to logarithimic using pow2db.
% Further add the offset to it to determine the threshold. Next, compare the
% signal under CUT with this threshold. If the CUT level > threshold assign
% it a value of 1, else equate it to 0.

% Use RDM[x,y] as the matrix from the output of 2D FFT for implementing
% CFAR
for i = 1 : Nr/2
    for j = 1 : Nd
        % This process will generate a thresholded block, which is smaller 
        % than the Range Doppler Map as the CUT cannot be located at the 
        % edges of matrix. Hence,few cells will not be thresholded. To keep
        % the map size same set those values to 0. 
        if (i < 1+Tr+Gr || i > Nr/2-(Tr+Gr) || j < 1+Tc+Gc || j > Nd-(Tc+Gc))
            RDM(i,j) = 0;
            continue;
        end
        
        noise_level = 0;
        for p = i-(Tr+Gr) : i+Tr+Gr
            for q = j-(Tc+Gc) : j+Tc+Gc
                % Sum up all the values in the training cells
                if (abs(i-p)>Gr || abs(j-q)>Gc)
                    noise_level = noise_level + db2pow(RDM(p,q));
                end
            end
        end
        
       % Average the summed values for all the training cells
       threshold = pow2db(noise_level / (2*(Tc+Gc+1)*2*(Tr+Gr+1)-(Gr*Gc)-1));
       
       % Add the SNR offset to the threshold
       threshold = threshold + offset;
       
       % Measure the signal in in Cell Under Test (CUT) and compare against
       CUT = RDM(i,j);
       if (CUT < threshold)
           RDM(i,j) = 0;
       else
           RDM(i,j) = 1;
       end
    end
end

% Display the CFAR output using the Surf function like we did for Range
% Doppler Response output.
doppler_axis = linspace(-100,100,Nd);
range_axis = linspace(-200,200,Nr/2)*((Nr/2)/400);
figure ('Name','CA-CFAR filtered RDM')
surf(doppler_axis,range_axis,RDM);
colorbar; 