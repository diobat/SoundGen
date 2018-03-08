
clc;
clear all;
close all;

%% ===========================================================================
%%  AUDIO BEEP SETUP  (outdated & redundant)
%% ===========================================================================

fsinal = 0.5;
amp=0.5;
fs_beep=20500; % sampling frequency
duration=0.5;
freq_beep=500;
values=0:1/fs_beep:duration;
a=amp*sin(2*pi*freq_beep*values);
sound(a,fs_beep)
flag = 0;

%% ===========================================================================
%% SDR RTL HARDWARE SETUP
%% ===========================================================================

fmRxParams = getParamsSdrrFMExamples;

fmRxParams.StopTime = 7;
fmRxParams.RadioSampleRate = 2e6;  %% Not being used
fmRxParams.FrequencyDeviation = 1e6; %% Not being used
fmRxParams.SamplesPerFrame = 512*100;
fmRxParams.AudioFrameTime = 1 / (fmRxParams.RadioSampleRate/fmRxParams.SamplesPerFrame) ;

Fc = Define_Carrier();
Fs = 226e3;
radioTime = 0;

radio = comm.SDRRTLReceiver('CenterFrequency', Fc, ...
                            'SampleRate', Fs, ...
                            'EnableTunerAGC', true, ...
                            'SamplesPerFrame', fmRxParams.SamplesPerFrame, ...  %potencia de 2
                            'OutputDataType', 'single');

%% ===========================================================================
%% DECLARATION/INITIALIZATION/PREALOCATION OF VARIABLES
%% ===========================================================================

ssm=zeros(1000, 1);
ssm2=zeros(1000, 1);
endresult = zeros(500000,1);
allsamples = zeros(150000,1);

signal_frequency = 1.85e3; % <------ frequency of the unmodulated signal sent by the sensor
signal_period = 1/signal_frequency;
samples_per_bit = Fs * signal_period; % <----- number of samples in each bit of the recieved message, this number is not necessarily an integer

envelope_offset = 0;
decimation_order = 8; % <----- METHOD WILL BE OBSOLETE SOON
m = 1;
rising_edge_counter = 0;


thresh_gain= 1;
thresh_offset = 0;


frame_start = zeros(1000,1);


desired_result = transpose([1 1 0 1 0 1 0 1 0 1 0]);
reset_pattern = transpose([0 0 0 0 0 0 0 0 0 0 0 0 1]);

%% ===========================================================================
%% GUI SETUP
%% ===========================================================================

Interface = SoundGen_GUI;
Interface.GUI_display_fc(num2str(Fc));

%% ===========================================================================
%% MULTITHREADING (Gave up on this, unessecary and impossible to implement)
%% ===========================================================================

  %parallel_worker = batch('parallel_plotting');
  %parallel_worker_ID =

%% ===========================================================================
%% MAIN CYCLE
%% ===========================================================================
tic;
while radioTime < fmRxParams.StopTime
  % Receive baseband samples (Signal Source)


  if ~isempty(sdrinfo(radio.RadioAddress))
    [rcv,~] = step(radio);


    for n=1:radio.SamplesPerFrame
        m = m + 1;

        allsamples(m) = rcv(n);
        %adjusted = (rcv(n)/2) + 0.65;
        endresult(m) = ssm(2);

        ssm(2:end) = ssm(1:end-1);
        ssm(1) = AM_Demod(rcv(n), 1, 0);

        %=================================
        % Custom functions go here
        %=================================

        if red(ssm) == 1
          %display ('Rising Edge detectado')
          m;
          rising_edge_counter = rising_edge_counter + 1;
        end


    end

    %lost = 0;
    %late = 1;
  end
  %plot(ssm)
  Interface.GUI_plot(ssm);
  Interface.GUI_RE_counter(num2str(rising_edge_counter));
  pause(1e-6);

  % Update radio time.
  radioTime = radioTime + toc
  tic;
  %radioTime = radioTime + fmRxParams.AudioFrameTime + toc %+ ...

   % double(lost)/fmRxParams.RadioSampleRate;
end

%% ===========================================================================
%% END OF MAIN CYCLE
%% ===========================================================================

%% ===========================================================================
%% RELEASE SYSTEM RESOURCES
%% ===========================================================================

%wait(parallel_worker);
%delete(parallel_worker);
release(radio);
delete(Interface);

rising_edge_counter

%% ===========================================================================
%% FINAL CHARTS
%% ===========================================================================

%power_amp = abs(allsamples(round(length(allsamples)*0.2) : end));


absolute_signal = abs(allsamples);
absolute_signal = absolute_signal(round(length(absolute_signal)*0.25):end);



[peaks, index] = findpeaks(absolute_signal);
[yupper,ylower] = envelope(absolute_signal, 50000 ,'peak');
fronteira = [yupper,ylower];
threshold = mean(fronteira, 2) .* thresh_gain + thresh_offset;

[highest_quality, offset_index] = Synchronize(absolute_signal, samples_per_bit , threshold)


%index_peak = single(index);
%figure(2);
%title('Number of samples per power')
%xlabel('Power (after AGC)')
%ylabel('Total number of samples collected')

%histogram(power_amp)
%figure(3);
%FourierT(absolute_signal, Fs);

figure(5);
%ax = gca;
%ax.XTicks = index
%ax.XTickMode = 'manual'

grid on;
plot(absolute_signal);
hold on;
plot(yupper, 'r');
plot(ylower, 'r');
plot(threshold, 'k');
%plot(index, peaks, 'og')
ylim([0 1.5])

decimated_peaks = peaks(1:decimation_order:end);
decimated_index = index(1:decimation_order:end);

plot(decimated_index, decimated_peaks, 'om')

result = zeros(length(decimated_peaks), 1);


%% THRESHOLD APPLICATION

for b1 = 1 : (length(decimated_index)-1)

b2 = mean(absolute_signal(decimated_index(b1):decimated_index(b1+1)));

if b2 > threshold(decimated_index(b1))
  result(b1) = 1;
end

z = plot([decimated_index(b1), decimated_index(b1+1)], [b2,b2] , '--r');
end


figure(6);
plot(result);
ylim([-0.5, 1.5]);

patterns_found_map = FindPattern(desired_result, result);
reset_pattern_map = FindPattern(reset_pattern, result);

tries = nnz(reset_pattern_map);
hits = nnz(patterns_found_map);

sucess_rate = hits/tries

%scatter(efficiency(:,1)*512, efficiency(:,2));


%subplot(2,1,2);
%plot (endresult)

%xlabel('Samples')

%ylabel('Bit')
%k = find(allsamples);  % encontrar a posi�ao de todos os valores diferentes de zero no array allsamples
%k1 = max(k);  % encontrar a posi��o o ultimo valor diferente de zero no array allsamples
%axis([0 k1 -0.1 1.1])

%% ===========================================================================
%% END OF PROGRAM
%% ===========================================================================
sound(a,fs_beep)
