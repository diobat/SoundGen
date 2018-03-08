
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

fmRxParams.StopTime = 20;
fmRxParams.RadioSampleRate = 2e6;  %% Not being used
fmRxParams.FrequencyDeviation = 1e6; %% Not being used
fmRxParams.SamplesPerFrame = 512*10*3;
SPF = fmRxParams.SamplesPerFrame;
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

signal_frequency = 1.85e3 * 2; % <------ frequency of the unmodulated signal sent by the sensor
bits_per_word = 11; % <----- has to be adjusted to the expected recieved transmission


signal_period = 1/signal_frequency; % <----- period of the unmodulated signal
samples_per_bit = Fs * signal_period; % <----- number of samples in each bit of the recieved message, this number is not necessarily an integer
bits_per_frame = SPF/samples_per_bit;
samples_per_word = samples_per_bit * bits_per_word;

frame_absolute = zeros(SPF, 1);  % <-------- store here each frame as it is harvested from the analisar
n = 40; % <------ select number of previous frames to be kept in a buffer. Bigger n = better envelope + bigger processing time;
last_n_frames = zeros (SPF * n, 1); %<----- store here last n frames for posterior envelope calculating purposes
unmodulated_signal = zeros(1000, 1);  % <----- Demodulated signal gets stored here for later use in pattern recognition

f = 0;  % <----- total frames processed counter;
endresult = [];
allsamples = [];
allthreshold = [];
allbit_frontier = [];
allenvelope = [,];
allvariance = [];
allword_frontier = [];


m = 6; % <--- it's advisable to select m such as bilmf > 1000 to facilitate the UI generation. under normal circunstances this means m >= 6
bilmf = zeros(ceil(bits_per_frame * m), 1);  % <-------- Bits in Last m Frames: stores last 3 frames worth of bits for pattern recognition

% WORD DETECTION VARIABLES

word_window = zeros( ceil(samples_per_bit*(bits_per_word+2)) , 1);
window_variance = [];
word_map = [];
word_frontier = [];
number_of_slices = 0;

envelope_function = zeros(SPF, 1);

first_frontier = 0;
final_frontier = 0;

initial_frame_discard = 5; % <  ---- specifies the number of disregarded initial frames at program startup, improves envelope stabilization and ultimately the sucess rate
rising_edge_counter = 0;
sucess_rate = 0; % <------ hits/tries (NOT 100% TRUSTABLE);

thresh_gain= 1;  % <------ threshold line control
thresh_offset = 0;
envelope_offset = 0; % <------ envelope line control

frame_start = zeros(1000,1);
bit_frontier = zeros(floor(SPF / samples_per_bit), 1); % <--- space allocation for the array that will store the indexes of the samples that separate two different bits

desired_result = transpose([1 1 0 1 0 1 0 1 0 1 0]);
reset_pattern = transpose([0 0 0 0 0 0 0 0 0 1]);
rising_edge = transpose([0 1]);
patterns = {desired_result; reset_pattern; rising_edge};
counter_patterns = zeros(1, length(patterns));  % <---- counts how many patterns are detected in the signal    MAP = [hits, tries]


%% ===========================================================================
%% GUI SETUP
%% ===========================================================================

Interface = SoundGen_GUI;
Interface.GUI_display_fc(num2str(Fc));

debug_mode = 1; % <- binary variable that determines if permanent samples are stored for later debugging. WARNING: PROGRAM WILL SHARPLY DECREASE IN PERFORMANCE AFTER THE FIRST 20 SECONDS WHILE IN THIS MODE

%% ===========================================================================
%% MULTITHREADING (Gave up on this, unessecary and impossible to implement)
%% ===========================================================================

  %parallel_worker = batch('parallel_plotting');
  %parallel_worker_ID =

%% ===========================================================================
%% MAIN CYCLE
%% ===========================================================================

pause(2);

tic;
while radioTime < fmRxParams.StopTime
  % Receive baseband samples (Signal Source)

  if ~isempty(sdrinfo(radio.RadioAddress))

    [rcv,~] = step(radio);
    frame_absolute = abs(rcv);

    f = f + 1; % increase frames processed counter


    if f > initial_frame_discard  % <----- discard first frames from processing to discard initial hardware calibration phase and smoothen the envelope

      allsamples = cat(1, allsamples, frame_absolute);

      last_n_frames(1:end-SPF) = last_n_frames(SPF+1:end);
      last_n_frames(SPF*(n-1) + 1 :end) = frame_absolute;

      % VERTICAL FRAMING (ENVELOPE)

      [yupper,ylower] = envelope(last_n_frames, 100000 ,'peak');
      envelope_function = [yupper,ylower];

      threshold = mean(envelope_function(SPF*(n-1):end, 1:2), 2) .* thresh_gain + thresh_offset;

      % HORIZONTAL FRAMING (bit_frontier, word_window)

      window_variance = zeros(length(frame_absolute), 1);

      for a0 = 1 : length(frame_absolute) - length(word_window) - 1
        %window_stop = a0+length(word_window)-1;
        word_window = frame_absolute(a0:a0+length(word_window)-1);
        window_variance(a0, 1) = var(word_window);
      end


      split = (max(window_variance)-min(window_variance))*0.2;
      word_map = im2bw(window_variance, split);
      word_frontier = abs(word_map(1:end-1) - word_map(2:end));  % <------ these three lines create an array that is filled with zeros except when a word begins or ends
      word_frontier(round(samples_per_word/1.66)+1 : end) = word_frontier(1:end-round(samples_per_word/1.66));
      word_frontier(end-round(samples_per_word/1.66)+1:end) = 0;

      number_of_slices = nnz(word_frontier);

      if number_of_slices > 2

        frame_absolute_slices = cell(number_of_slices-1, 4);     % <<<< ----- allocate space for an array that will be stores in slices,
        % each slice begins when a word begins or ends, thus the signal extracts the samples that have words on them. One of the slices
        %is ignored, it corresponds to the cropped part of the signal ant the beggining and end of frame.
        % COLUMNS : 1 - SIGNAL; 2 - BIT FRONTIERS; 3 - RELEVANT THRESHOLD VALUES; 4 - DEMODULATED SIGNAL

        slice_index = find(word_frontier, number_of_slices);  % returns the index of the first n ones in the variable word_frontier, where n = number of slices

        for a1 = 1 : length(slice_index)-1;
          frame_absolute_slices{a1, 1} = frame_absolute(slice_index(a1, 1):slice_index(a1+1, 1));    % < -- save the signal itself in the first column of the cell array
          frame_absolute_slices{a1, 3} = threshold(slice_index(a1, 1):slice_index(a1+1, 1));
          [~, slice_offset] = Synchronize(frame_absolute_slices{a1, 1}, samples_per_bit , frame_absolute_slices{a1, 3});   % <--- save the corresponding bit frontiers for that slice and word in the respective row of the second column

          slice_bit_frontiers =  zeros(floor(length(frame_absolute_slices{a1, 1})/samples_per_bit), 1);

          for a2 = 1 : length(slice_bit_frontiers)
            slice_bit_frontiers(a2) = slice_offset + round((a2-1) * samples_per_bit);
          end

          frame_absolute_slices{a1, 2} = slice_bit_frontiers;



        end
      end

      %Crop irrelevant data (at beggining and end) from the frame itself, threshold and envelope
      first_frontier = 0;
      for a6 = 1 : length(frame_absolute_slices)
        if ~isempty(frame_absolute_slices{a6, 2});
          first_frontier = frame_absolute_slices{1, 2}(1);
        end
      end
        first_frontier = 1;
      final_frontier = first_frontier;
      for a5 = 1 : length(frame_absolute_slices)
        final_frontier =  final_frontier + ((length(frame_absolute_slices{a5, 2})+2) * samples_per_bit);
      end
      final_frontier = round(final_frontier);

      envelope_function = envelope_function( SPF*(n-1):end , 1:2 );  %<<--- envelope uses the last n frames, but after its used to create a stable threshold we only need the data relevant to the nth (last) frame.

      for a3 = 1 : length(frame_absolute_slices)
        frame_absolute_slices{a3, 4} = ASK_Demod(frame_absolute_slices{a3, 1}, frame_absolute_slices{a3, 3}, frame_absolute_slices{a3, 2});

        for a4 = 1 : length(frame_absolute_slices{a3, 4})  % <------- detect patterns, bit by bit
          bilmf(1:end-1) = bilmf(2:end);  % <---- Será vantajoso fazer isto de um a um ou seria melhor passar este ciclo para dentro de uma funçao?
          bilmf(end) = frame_absolute_slices{a3, 4}(a4);

          if f > initial_frame_discard + n/3 + 10% <---- the program has been running AND processing data for a while before it starts actively trying to identify patters, provides more accurate pattern recognition output data
            counter_patterns =  counter_patterns + FindPattern(patterns, bilmf);
          end
        end
      end

      if debug_mode == 1
        allenvelope = cat(1, allenvelope, envelope_function);
        allthreshold = cat(1, allthreshold, threshold);
        allbit_frontier = cat(1, allbit_frontier, (bit_frontier(1:end-1) + ((f-1)*SPF) ) );
        endresult = cat(1, endresult, unmodulated_signal);
        allvariance = cat(1, allvariance, window_variance);
        allword_frontier = cat(1, allword_frontier, word_frontier);
      end

      sucess_rate = counter_patterns(1, 1)/counter_patterns(1, 2);

      pause(1e-8);


      %=================================
      % Custom functions go here
      %=================================

      %lost = 0;
      %late = 1;

    end

  end
  Interface.GUI_plot(bilmf);
  Interface.GUI_RE_counter(num2str(counter_patterns(3)));
  pause(1e-7);

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


release(radio);
delete(Interface);

disp(sucess_rate);

%% ===========================================================================
%% FINAL CHARTS
%% ===========================================================================

%power_amp = abs(allsamples(round(length(allsamples)*0.2) : end));


%absolute_signal = abs(allsamples);
%absolute_signal = absolute_signal(round(length(absolute_signal)*0.25):end);

%[yupper,ylower] = envelope(absolute_signal, 50000 ,'peak');

%fronteira = [yupper,ylower];
%threshold = mean(fronteira, 2) .* thresh_gain + thresh_offset;

%[highest_quality, offset_index] = Synchronize(absolute_signal, samples_per_bit , threshold)


%index_peak = single(index);
%figure(2);
%title('Number of samples per power')
%xlabel('Power (after AGC)')
%ylabel('Total number of samples collected')

%histogram(power_amp)
%figure(3);
%FourierT(absolute_signal, Fs);

%figure(5);
%ax = gca;
%ax.XTicks = index
%ax.XTickMode = 'manual'

%grid on;
%plot(absolute_signal);
%hold on;
%plot(yupper, 'r');
%plot(ylower, 'r');
%plot(threshold, 'k');
%plot(index, peaks, 'og')
%ylim([0 1.5])


%% Descobrir os limites de separação entre bits
%bit_frontier = zeros(floor(length(absolute_signal) / samples_per_bit), 1);

%for a = 1 : length(bit_frontier)
%  bit_frontier(a) = offset_index + round(((a-1) * samples_per_bit));
%end

%result = zeros(length(bit_frontier), 1);


%% THRESHOLD APPLICATION

%for b1 = 1 : (length(bit_frontier)-1)
%
%b2 = mean(absolute_signal(bit_frontier(b1):bit_frontier(b1+1)));
%
%if b2 > threshold(bit_frontier(b1))
%  result(b1) = 1;
%end

%z = plot([bit_frontier(b1), bit_frontier(b1+1)], [b2,b2] , '--r');
%z1 = plot([bit_frontier(b1), bit_frontier(b1)], [0, 1.5], 'k');  %<---- comment to turn off black vertical separation lines
%end

%figure(6);

%result = ~result;  % invert the result

%plot(result);
%ylim([-0.5, 1.5]);

%patterns_found_map = FindPattern(desired_result, result);
%reset_pattern_map = FindPattern(reset_pattern, result);

%tries = nnz(reset_pattern_map);
%hits = nnz(patterns_found_map);
%bits_per_cycle = length(reset_pattern_map)/tries;

%sucess_rate = hits/tries

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
