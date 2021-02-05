# Radar Target Generation and Detection

This is the writeup for this [project](https://github.com/kkufieta/sensor-fusion-sandbox/blob/main/matlab/Radar_Target_Generation_and_Detection.m).

The given task was to generate and detect a moving object. The steps to do this are the following:
1. Configure the FMCW waveform based on the system requirements
2. Define the range and velocity of the target and simulate its displacement
3. Process the transmit and receive signals to determine the beat signal
4. Perform range FFT on the received signal to determine the range
5. Perform CFAR processing on the output of FFT2 to display the target

## 2D CFAR

The following is the implementation for the 2D CFAR.

```octave
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
```

I have selected the following number of training cells and guard cells:
* `Tr = 10` Training Band Rows
* `Tc = 8` Training Band Columns
* `Gr = 4` Guard Band Rows
* `Gc = 4` Guard Band Columns

I have selected the offset to be `10`, since the signal is well detected with this threshold and the noise is suppressed.

I then loop over the values in the Range Doppler Map `RDM`. For the non-thresholded cells at the edges, I set the value of the RDM to `0`. 

Otherwise, I sum up all the values in the training cells in `noise_level`, after I converted them from logarithmic to linear using this command: `noise_level = noise_level + db2pow(RDM(p,q));`. 
I make sure I sum up only training cells by checking this condition: `if (abs(i-p)>Gr || abs(j-q)>Gc)`.

Once I've summed up all the values in the training cells for a given cell under test (CUT), I averge them and convert them back to logarithmic like this: `threshold = pow2db(noise_level / (2*(Tc+Gc+1)*2*(Tr+Gr+1)-(Gr*Gc)-1));`.

After that, I add the SNR offset to the threshold: `threshold = threshold + offset;`.

Next, I'm comparing the value at CUT with the threshold. If the CUT value is below the threshold, I set the RDM to 0. Otherwise, I set it to 1.
