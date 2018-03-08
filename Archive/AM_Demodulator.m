clc;
close all; 
clear all;

fmRxParams = getParamsSdrrFMExamples

Fc = 1e9;
Fs = 3.2e6;

radio = comm.SDRRTLReceiver('CenterFrequency', Fc, ...
                            'SampleRate', Fs, ...
                            'EnableTunerAGC', true, ...
                            'SamplesPerFrame', 1024, ...
                            'OutputDataType', 'single');



% demodulates the amplitude modulated signal y from a carrier signal with
% frequency Fc (Hz). The carrier signal and y have sample frequency Fs
% (Hz). The modulated signal y has zero initial phase and zero carrier
% amplitude, so it represents suppressed carrier modulation. The
% demodulation process uses the lowpass filter specified by [num,den] =
% butter(5,Fc*2/Fs).

player = audioDeviceWriter('SampleRate',fmRxParams.AudioSampleRate);

%z = amdemod(y,Fc,Fs);


% Create audio player
player = audioDeviceWriter('SampleRate',fmRxParams.AudioSampleRate);

% Initialize radio time
radioTime = 0;

buffer = [];

% Main loop
while radioTime < fmRxParams.StopTime
  % Receive baseband samples (Signal Source)
  if ~isempty(sdrinfo(radio.RadioAddress))
    [rcv,~,lost,late] = step(radio);
  else
    %rcv = sigSrc();
    lost = 0;
    late = 1;
  end

  % Demodulate FM broadcast signals and play the decoded audio
  buffer1 = cat(1, buffer, rcv);
  buffer = buffer1;
  


  % Update radio time. If there were lost samples, add those too.
  radioTime = radioTime + fmRxParams.AudioFrameTime + ...
    double(lost)/fmRxParams.RadioSampleRate;
end

  signal = amdemod(buffer, Fc, Fs);
  player(rcv);

% 
% fs = 2000;                              % Sampling frequency
% f = 5;                                  % Signal frequency
% fc = 250;                               % Carrier frequency
% N = 2000;                               % Use 1 sec of data
% t = (1:N)/fs;                           % Time axis for plotting
% wn = .02;                               % PSD lowpass filter cut - off frequency
% [b,a] = butter(2,wn);                   % Design lowpass filter
% % Generate AM signal
% w = (1:N)* 2*pi*fc/fs;                  % Carrier frequency = 250 Hz
% w1 = (1:N)*2*pi*f/fs;                   % Signal frequency = 5 Hz
% vc = sin(w);                            % Define carrier
% vsig = sawtooth(w1,.5);                 % Define signal
% vm = (1 + .5 * vsig) .* vc;             % Create modulated signal with a Modulation constant = 0.5
% subplot(3,1,1);
% plot(t,vm,'k');                         % Plot AM Signal....axis, label,title.......
% % Add noise with 3.16 times power (10 db) of signal for SNR of -10 db
% noise = randn(1,N);
% scale = (var(vsig)/var(noise)) * 3.16;
% vm = vm + noise * scale;                % Add noise to modulated signal
% subplot(3,1,2);
% plot(t,vm,'k');                         % Plot AM signal..axis, label,title.......
% % Phase sensitive detection
% ishift = fix(.125 * fs/fc);             % Shift carrier by 1/4
% vc = [vc(ishift:N) vc(1:ishift-1)];     % period (45 deg) using periodic shift
% v1 = vc .* vm;                          % Multiplier
% vout = filter(b,a,v1);                  % Apply lowpass filter
% subplot(3,1,3);
% plot(t,vout,'k');       


release(sigSrc)
release(fmBroadcastDemod)
release(player)