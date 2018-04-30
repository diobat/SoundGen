
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

%signal_frequency = 1.7857e3 * 2;
%signal_frequency = 1785 * 2; % <------ frequency of the unmodulated signal sent by the sensor
signal_frequency = 3650; % <------ frequency of the unmodulated signal sent by the sensor
bits_per_word = 32; % <----- has to be adjusted to the expected recieved transmission

signal_period = 1/signal_frequency; % <----- period of the unmodulated signal
samples_per_bit = Fs * signal_period; % <----- number of samples in each bit of the recieved message, this number is not necessarily an integer
bits_per_frame = SPF/samples_per_bit;
samples_per_word = samples_per_bit * bits_per_word;

frame_absolute = zeros(SPF, 1);  % <-------- store here each frame as it is harvested from the analisar
n = 20; % <------ select number of previous frames to be kept in a buffer. Bigger n = better envelope + bigger processing time;
last_n_frames = zeros (SPF * n, 1); %<----- store here last n frames for posterior envelope calculating purposes
unmodulated_signal = zeros(1000, 1);  % <----- Demodulated signal gets stored here for later use in pattern recognition

f = 0;  % <----- total frames processed counter;
data = [];
alldata = [];

m = 6; % <--- it's advisable to select m such as bilmf > 1000 to facilitate the UI generation. under normal circunstances this means m >= 6
bilmf = zeros(ceil(bits_per_frame * m), 1);  % <-------- Bits in Last m Frames: stores last 3 frames worth of bits for pattern recognition


% WORD DETECTION VARIABLES

word_window = zeros( ceil(samples_per_bit*(15)) , 1);
window_variance = [];
word_map = [];
word_frontier = [];
number_of_slices = 0;

envelope_function = zeros(SPF, 1);   % < -- pre allocate space for the envelope function

first_frontier = 1;
final_frontier = 1;

initial_frame_discard = 5; % <  ---- specifies the number of disregarded initial frames at program startup, improves envelope stabilization and ultimately the sucess rate
word_success_rate = 0; % <------ hits/tries (NOT 100% TRUSTABLE);
bit_success_rate = 0; % <------ nthroot(bits_per_word, word_success_rate)

thresh_gain= 1;  % <------ threshold line control
thresh_offset = -0.003;
envelope_offset = 0; % <------ envelope line control

frame_start = zeros(1000,1);

desired_result = transpose([0 0 0 1 1 0 1 0 0 0 1 0 0 0 1 0 1 0 0 1 1 0 0 0 1 1 1 0 0 0 0 1 1 1 1 0 0 0]);
reset_pattern = transpose([0 0 0 0 0 0 0 0 0 1]);
rising_edge = transpose([0 1]);
patterns = {desired_result; reset_pattern; rising_edge};
counter_patterns = zeros(1, length(patterns));  % <---- counts how many patterns are detected in the signal    MAP = [hits, tries]

message = cell(1,1); % < ------ first column for the data, second one for the hexadecimal translation

%% ===========================================================================
%% GUI SETUP
%% ===========================================================================

Interface = SoundGen_GUI;
Interface.GUI_display_fc(num2str(Fc));

debug_mode = 1; % <- binary variable that determines if permanent samples are stored for later debugging. WARNING: PROGRAM WILL SHARPLY DECREASE IN PERFORMANCE AFTER THE FIRST 20 SECONDS WHILE IN THIS MODE

%% ===========================================================================
%% MAIN CYCLE
%% ===========================================================================

tic;
while radioTime < fmRxParams.StopTime
  % Receive baseband samples (Signal Source)
  if ~isempty(sdrinfo(radio.RadioAddress))

    [rcv,~] = step(radio);
    frame_absolute = abs(rcv);
    f = f + 1; % increase frames processed counter

    if f > initial_frame_discard  % <----- discard first frames from processing to discard initial hardware calibration phase and smoothen the envelope

      last_n_frames(1:end-SPF) = last_n_frames(SPF+1:end);
      last_n_frames(end-SPF + 1 :end) = frame_absolute;

      % VERTICAL FRAMING (ENVELOPE)

      nz = find(last_n_frames,1,'first');
      [yupper,ylower] = envelope(last_n_frames(max(1, nz-1):end), SPF * 2 ,'peak');
      envelope_function = [yupper,ylower];
      envelope_function = envelope_function(end-SPF+1:end , 1:2);
      threshold = (mean(envelope_function, 2) .* thresh_gain) + thresh_offset;

      % HORIZONTAL FRAMING (bit_frontier, word_window)

      %window_variance = define_windowvariance(SPF, frame_absolute, length(word_window));

      window_variance = zeros(SPF, 1);

      for a0 = 1 : SPF - length(word_window)
        %window_stop = a0+length(word_window)-1;
        word_window = frame_absolute(a0:a0+length(word_window)-1);
        window_variance(a0, 1) = var(word_window);
      end
      window_variance(a0+1:end) = window_variance(a0);



      split = max(window_variance)*0.40;
      word_map = im2bw(window_variance, split);
      word_frontier = [(abs(word_map(1:end-1)-word_map(2:end))); 0];  % <------ these three lines create an array that is filled with zeros except when a word begins or ends
      word_frontier(round(length(word_window)/1.66)+1 : end) = word_frontier(1:end-round(length(word_window)/1.66));
      word_frontier(1:round(length(word_window)/1.66)) = 0;
      word_frontier(1) = 1;
      word_frontier(end) = 1;

      number_of_slices = nnz(word_frontier);

      if number_of_slices > 0

        data = cell(number_of_slices-1, 8);     % <<<< ----- allocate space for an array that will be stores in slices,
        % each slice begins when a word begins or ends, thus the signal extracts the samples that have words on them. One of the slices
        %is ignored, it corresponds to the cropped part of the signal ant the beggining and end of frame.
        % COLUMNS :
          %1 - SIGNAL;
          %2 - BIT FRONTIERS;
          %3 - RELEVANT THRESHOLD VALUES;
          %4 - DEMODULATED SIGNAL
          %5 - INTRA FRONTIERS AVERAGE VALUES
          %6 - UPPER AND LOWER ENVELOPES
          %7 - WINDOW VARIANCE
          %8 - WORD FRONTIER

          [nrData,ncData]=size(data);

        slice_index = find(word_frontier, number_of_slices);  % returns the index of the first n ones in the variable word_frontier, where n = number of slices
        %slice_index = [1; slice_index; SPF];

        for a1 = 1 : length(slice_index)-1;
          data{a1, 1} = frame_absolute(slice_index(a1):slice_index(a1+1)-1);            % < -- save the signal itself in the first column of the cell array
          data{a1, 3} = threshold(slice_index(a1):slice_index(a1+1)-1);                 % < -- threshold levels
          data{a1, 6} = envelope_function(slice_index(a1):slice_index(a1+1)-1, 1:2);    % < -- envelope levels, both upper and lower
          data{a1, 7} = window_variance(slice_index(a1):slice_index(a1+1)-1);           % < -- variance of the signal in a given interval (defined by length(word_window))
          data{a1, 8} = word_frontier(slice_index(a1):slice_index(a1+1)-1);             % < -- map of the places where a word begins or ends

          [~, slice_offset] = Synchronize(data{a1, 1}, samples_per_bit , data{a1, 3});   % <--- save the corresponding bit frontiers for that slice and word in the respective row of the second column

          slice_bit_frontiers = define_bitfrontiers(length(data{a1, 1}), slice_offset, samples_per_bit);

          data{a1, 2} = slice_bit_frontiers;

        end

        % DATA PROCESSING

        for a3 = 1 : nrData

          data{a3, 5} = intervals_average(data{a3, 1}, data{a3, 2});
          data{a3, 4} = ASK_Demod(data{a3, 1}, (data{a3, 3} .* thresh_gain) + thresh_offset, data{a3, 2});      % < -- demodulation of the signal

          for a4 = 1 : length(data{a3, 4})  % <------- detect patterns, bit by bit
            bilmf(1:end-1) = bilmf(2:end);
            bilmf(end) = data{a3, 4}(a4);

            if f > initial_frame_discard + n/3 + 10   % <---- the program has been running AND processing data for a while before it starts actively trying to identify patters, provides more accurate pattern recognition output data
              patterns_buffer = FindPattern(patterns, bilmf);
              counter_patterns =  counter_patterns + patterns_buffer;

              if patterns_buffer(1,1) == 1
                window1 = length(bilmf)-length(desired_result)+3;
                %window2 = length(bilmf)-length(desired_result)+10;
                message{counter_patterns(1)*2-1, 1} = transpose(bilmf(window1:window1+3));
                message{counter_patterns(1)*2-1, 2} = binaryVectorToHex(message{counter_patterns(1)*2-1, 1});
                message{counter_patterns(1)*2, 1} = transpose(bilmf(window1+4:window1+7));
                message{counter_patterns(1)*2, 2} = binaryVectorToHex(message{counter_patterns(1)*2, 1});
              end

            end
          end
        end
      end


      if debug_mode == 1
        alldata = cat (1, alldata, data);
      end

      word_success_rate = (counter_patterns(1, 1)/counter_patterns(1, 2));
      bit_success_rate = nthroot(word_success_rate, bits_per_word);

      word_success_rate = word_success_rate * 100;
      bit_success_rate = bit_success_rate * 100;

      pause(1e-8);

      %====================================================
      % Custom functions go here
      %====================================================

    end

  end

  Interface.GUI_plot(bilmf);
  Interface.GUI_RE_counter(num2str(counter_patterns(3)));

  Interface.GUI_stats(num2str(counter_patterns(1)), strcat(num2str(word_success_rate),'%'), strcat(num2str(bit_success_rate),'%'));
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

word_success_rate
bit_success_rate

%% ===========================================================================
%% FINAL CHARTS
%% ===========================================================================


%% ===========================================================================
%% END OF PROGRAM
%% ===========================================================================
sound(a,fs_beep)
