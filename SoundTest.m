
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
%% BEGIN BENCHMARK CYCLING
%% ===========================================================================

c = 50;
efficiency = zeros(c,3);
t = 1;

while t <= c

%% ===========================================================================
%% SDR RTL HARDWARE SETUP
%% ===========================================================================

fmRxParams = getParamsSdrrFMExamples

fmRxParams.StopTime = 10;
fmRxParams.RadioSampleRate = 2e6;
fmRxParams.FrequencyDeviation = 1e6;
fmRxParams.SamplesPerFrame = 512 * 5;
fmRxParams.AudioFrameTime = 1 / (fmRxParams.RadioSampleRate/fmRxParams.SamplesPerFrame) ;

Fc = 1e9;
Fs = 2e6;
radioTime = 0;


radio = comm.SDRRTLReceiver('CenterFrequency', Fc, ...
                            'SampleRate', Fs, ...
                            'EnableTunerAGC', true, ...
                            'SamplesPerFrame', fmRxParams.SamplesPerFrame*t, ...  %potencia de 2
                            'OutputDataType', 'single');

%% ===========================================================================
%% DECLARATION/INITIALIZATION/PREALOCATION OF VARIABLES
%% ===========================================================================

ssm=zeros(1000, 1);
ssm2=zeros(1000, 1);
endresult = zeros(500000,1);
allsamples = zeros(500000,1);
m = 1;
red_flag = 0;
rising_edge_counter = 0 ;
tresh_offset = 1;
etime = 0;
frame_start=zeros(1000,1);

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

        if ssm(1) == 1 && red_flag == 0
          %display ('Rising Edge detectado')
          red_flag = 1;
          rising_edge_counter = rising_edge_counter + 1;
        elseif ssm(1) == 0 && red_flag == 1
          red_flag = 0;
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

efficiency(t,1) = t;
efficiency(t,2) = m;
efficiency(t,3) = rising_edge_counter;
t = t + 1;

end  %% <- end the bnechmark cycling

%% ===========================================================================
%% RELEASE SYSTEM RESOURCES
%% ===========================================================================

%power_amp = abs(allsamples(length(allsamples)*0.2 : end));

scatter(efficiency(:,1)*fmRxParams.SamplesPerFrame, efficiency(:,2));
title('Optimal Frame Size tests - 10 seconds')
xlabel('Frame Size')
ylabel('Total number of samples collected')

figure(2);
scatter(efficiency(:,1)*fmRxParams.SamplesPerFrame, efficiency(:,3));
title('Optimal Frame Size tests - 10 seconds')
xlabel('Frame Size')
ylabel('Total number of rising edges detected.')

%histogram(power_amp)
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
