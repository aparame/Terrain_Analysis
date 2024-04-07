
%% Load data from CSV file and clean data
clc
clear
data = readtable('.\csv\450_1.500.csv');
data_eddie = readtable('.\csv\450_1.500.csv');
% Remove duplicate rows
clean_data = unique(data,'rows', 'stable');
noisy_data = clean_data(clean_data{:,4} > 0.0 & clean_data{:,4} < 2.5,:);
clean_data = clean_data(clean_data{:,4} >= 2.5 & clean_data{:, 4} <= 13, :);
writetable(clean_data, '.\csv\450_1.500_clean.csv');
% Extract time stamps and signal from data
time = clean_data{:, 6} + 1e-9.*clean_data{:,7};  % Assuming the first column contains time stamps
mean_front_signal = clean_data{:,1};  % Assuming the second column contains the signal data

%% Processing the noisy data
mean_front = mean((noisy_data{:,1}+noisy_data{:,3})/2);
mean_rear = mean((noisy_data{:,2}+noisy_data{:,4})/2);
std_front = std((noisy_data{:,1}+noisy_data{:,3})/2);
std_rear = std((noisy_data{:,2}+noisy_data{:,4})/2);
% Model the noise using Gaussian distribution
noise_model_front = @(x) normpdf(x, mean_front, std_front);
noise_model_rear = @(x) normpdf(x, mean_rear, std_rear);

filtered_data = mean_front_signal - noise_model_front(1:length(mean_front_signal))';
%% Compute time differences to estimate sampling intervals
time_diff = diff(time);
avg_sampling_interval = mean(time_diff); % Average sampling interval

% Perform FFT analysis
Fs = 1 / avg_sampling_interval;  % Sampling frequency (assuming uniform time stamps)
L = length(filtered_data);  % Length of signal
f = Fs * (0:L/2)/L;  % Frequency vector

Y = fft(filtered_data);  % Compute FFT
P2 = abs(Y/L);  % Compute two-sided spectrum
P1 = P2(1:L/2+1);  % Compute single-sided spectrum
P1(2:end-1) = 2*P1(2:end-1);  % Double spectrum (excluding DC component)


% Find the maximum frequency
[~, max_index] = max(P1);
remove_frequency = f(max_index);


% Define bandpass filter parameters
filter_range = 2;  % Range around max frequency to keep
filter_index = ~(f >= remove_frequency - filter_range & f <= remove_frequency + filter_range);

% Apply bandpass filter
filtered_signal = ifft(Y .* filter_index);

%% New FFT
L_new = length(filtered_signal);  % Length of signal
f_new = Fs * (0:L_new/2)/L_new;  % Frequency vector
Y_new = fft(filtered_signal);  % Compute FFT
P2_new = abs(Y_new/L_new);  % Compute two-sided spectrum
P1_new = P2_new(1:L_new/2+1);  % Compute single-sided spectrum
P1_new(2:end-1) = 2*P1_new(2:end-1);  % Double spectrum (excluding DC component)


% Find the maximum frequency
[new_max_amplitude, new_max_index] = max(P1);
new_max_frequency = f_new(new_max_index);

% Define bandpass filter parameters
filter_range = 2;  % Range around max frequency to keep
new_filter_index = (f_new >= new_max_frequency - filter_range) & (f_new <= new_max_frequency + filter_range);
%% Plot NEW FFT results
figure;
plot(f_new, P1, "LineWidth",1.5);
title('Single-Sided Amplitude Spectrum of Signal');
xlabel('Frequency (Hz)');
ylabel('Amplitude');

% Plot maximum frequency point
hold on;
plot(new_max_frequency, new_max_amplitude, 'ro', 'MarkerSize', 10);
text(new_max_frequency, new_max_amplitude, sprintf('(%.2f Hz, %.2f)', new_max_frequency, new_max_amplitude), 'VerticalAlignment', 'bottom');
hold off;

% Plot filtered data
new_filtered_signal = ifft(Y_new .* new_filter_index);
figure;
plot(time, abs(new_filtered_signal));
title('Filtered Data');
xlabel('Time');
ylabel('Amplitude');
hold on

