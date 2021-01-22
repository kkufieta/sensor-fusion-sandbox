% Constants and provided values
c = 3 * 10^8;               % Speed of light [m/s]
R_max = 300;                % Radar maximum range [m]
d_res = 1;                  % Range resolution [m]
f_b = [0, 1.1, 13, 24];     % frequency shifts [MHz]

% Find the Bsweep of chirp for 1 m resolution
B_sweep = c / (2 * d_res);

% Calculate the chirp time based on the Radar's Max Range
T_s = 5.5 * 2 * R_max / c;

% Calculate the range
range = c * f_b * 10^6 * T_s / (2 * B_sweep); % Range [m]

% Display the calculated range
disp(range);