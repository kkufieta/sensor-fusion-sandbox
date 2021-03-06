% Doppler Velocity Calculation
c = 3*10^8;         % speed of light
frequency = 77e9;   % frequency in Hz (77 GHz)
fd = [3, -4.5, 11, -3] * 10^3; % doppler frequency shifts

% Find: velocity of four targets in [m/s]
% Equation for relative velocity: v_r = lambda/2 * fd

% Calculate the wavelength
lambda = c/frequency;

% Calculate the velocity of the targets  fd = 2*vr/lambda
v_r = lambda/2 * fd;

% Display results
disp(v_r);