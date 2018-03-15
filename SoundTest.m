
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

jj = 20
signal_frequency= 3550;
%testing_results = zeros (jj, 3);

for j = 12 : jj

  signal_frequency = 3550 + 10 * j


  fmRxParams = getParamsSdrrFMExamples;

  fmRxParams.StopTime = 20;
  fmRxParams.RadioSampleRate = 2e6;  %% Not being used
  fmRxParams.FrequencyDeviation = 1e6; %% Not being used
  fmRxParams.SamplesPerFrame = 512*10*6;
  SPF = fmRxParams.SamplesPerFrame;
  fmRxParams.AudioFrameTime = 1 / (fmRxParams.RadioSampleRate/fmRxParams.SamplesPerFrame) ;

  Fc = 200e6+16200;
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
  bits_per_word = 11; % <----- has to be adjusted to the expected recieved transmission


  signal_period = 1/signal_frequency; % <----- period of the unmodulated signal
  samples_per_bit = Fs * signal_period; % <----- number of samples in each bit of the recieved message, this number is not necessarily an integer
  bits_per_frame = SPF/samples_per_bit;
  samples_per_word = samples_per_bit * bits_per_word;

  frame_absolute = zeros(SPF, 1);  % <-------- store here each frame as it is harvested from the analisar
  n = 10; % <------ select number of previous frames to be kept in a buffer. Bigger n = better envelope + bigger processing time;
  last_n_frames = zeros (SPF * n, 1); %<----- store here last n frames for posterior envelope calculating purposes
  unmodulated_signal = zeros(1000, 1);  % <----- Demodulated signal gets stored here for later use in pattern recognition

  f = 0;  % <----- total frames processed counter;
  data = [];
  alldata = [];
  endresult = [];
  allvariance = [];
  allword_frontier = [];

  m = 6; % <--- it's advisable to select m such as bilmf > 1000 to facilitate the UI generation. under normal circunstances this means m >= 6
  bilmf = zeros(ceil(bits_per_frame * m), 1);  % <-------- Bits in Last m Frames: stores last 3 frames worth of bits for pattern recognition

  % WORD DETECTION VARIABLES

  word_window = zeros( ceil(samples_per_bit*(bits_per_word)) , 1);
  window_variance = [];
  word_map = [];
  word_frontier = [];
  number_of_slices = 0;

  envelope_function = zeros(SPF, 1);

  first_frontier = 1;
  final_frontier = 1;

  initial_frame_discard = 5; % <  ---- specifies the number of disregarded initial frames at program startup, improves envelope stabilization and ultimately the sucess rate
  rising_edge_counter = 0;
  word_success_rate = 0; % <------ hits/tries (NOT 100% TRUSTABLE);
  bit_success_rate = 0; % <------ nthroot(bits_per_word, word_success_rate)

  thresh_gain= 1;  % <------ threshold line control
  thresh_offset = -0.006;
  envelope_offset = 0; % <------ envelope line control

  frame_start = zeros(1000,1);

  desired_result = transpose([1 1 0 1 0 1 0 1 0 1]);
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

        last_n_frames(1:end-SPF) = last_n_frames(SPF+1:end);
        last_n_frames(end-SPF + 1 :end) = frame_absolute;

        % VERTICAL FRAMING (ENVELOPE)

        nz = find(last_n_frames,1,'first');
        [yupper,ylower] = envelope(last_n_frames(max(1, nz-1):end), 100000 ,'peak');
        envelope_function = [yupper,ylower];
        envelope_function = envelope_function(end-SPF+1:end , 1:2);
        threshold = (mean(envelope_function, 2) .* thresh_gain) + thresh_offset;

        % HORIZONTAL FRAMING (bit_frontier, word_window)

        window_variance = zeros(SPF, 1);

        for a0 = 1 : SPF - length(word_window)
          %window_stop = a0+length(word_window)-1;
          word_window = frame_absolute(a0:a0+length(word_window)-1);
          window_variance(a0, 1) = var(word_window);
        end
        window_variance(a0+1:end) = window_variance(a0);

        split = (max(window_variance)-min(window_variance))*0.25;
        word_map = im2bw(window_variance, split);
        word_frontier = [(abs(word_map(1:end-1)-word_map(2:end))); 0];  % <------ these three lines create an array that is filled with zeros except when a word begins or ends
        word_frontier(round(samples_per_word/1.66)+1 : end) = word_frontier(1:end-round(samples_per_word/1.66));
        word_frontier(1:round(samples_per_word/1.66)) = 0;


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

          slice_index = find(word_frontier, number_of_slices);  % returns the index of the first n ones in the variable word_frontier, where n = number of slices
          slice_index = [1; slice_index; SPF];


          for a1 = 1 : length(slice_index)-1;
            data{a1, 1} = frame_absolute(slice_index(a1):slice_index(a1+1)-1);    % < -- save the signal itself in the first column of the cell array
            data{a1, 3} = threshold(slice_index(a1):slice_index(a1+1)-1);
            data{a1, 6} = envelope_function(slice_index(a1):slice_index(a1+1)-1, 1:2);
            data{a1, 7} = window_variance(slice_index(a1):slice_index(a1+1)-1);
            data{a1, 8} = word_frontier(slice_index(a1):slice_index(a1+1)-1);

            [~, slice_offset] = Synchronize(data{a1, 1}, samples_per_bit , data{a1, 3});   % <--- save the corresponding bit frontiers for that slice and word in the respective row of the second column

            slice_bit_frontiers =  zeros(floor(length(data{a1, 1})/samples_per_bit), 1);

            for a2 = 1 : length(slice_bit_frontiers)
              slice_bit_frontiers(a2) = slice_offset + round((a2-1) * samples_per_bit);
            end

            slice_bit_frontiers = [slice_bit_frontiers; length(data{a1, 1})];

            if slice_bit_frontiers(1) ~= 1
            slice_bit_frontiers = [1; slice_bit_frontiers]; % <- include 0 and end on the frontiers, this means the first and last interval will be uneven.
            end

            data{a1, 2} = slice_bit_frontiers;

          end

          % DATA PROCESSING

          for a3 = 1 : length(data)

            data{a3, 5} = intervals_average(data{a3, 1}, data{a3, 2});
            data{a3, 4} = ASK_Demod(data{a3, 1}, (data{a3, 3} .* thresh_gain) + thresh_offset, data{a3, 2});

            for a4 = 1 : length(data{a3, 4})  % <------- detect patterns, bit by bit
              bilmf(1:end-1) = bilmf(2:end);  % <---- Será vantajoso fazer isto de um a um ou seria melhor passar este ciclo para dentro de uma funçao?
              bilmf(end) = data{a3, 4}(a4);

              if f > initial_frame_discard + n/3 + 10% <---- the program has been running AND processing data for a while before it starts actively trying to identify patters, provides more accurate pattern recognition output data
                counter_patterns =  counter_patterns + FindPattern(patterns, bilmf);
              end
            end
          end
        end

        if debug_mode == 1

          alldata = cat (1, alldata, data);

          for a7 = 1 : length(data)

            %allthreshold = cat(1, allthreshold, data{a7, 3});
            %endresult = cat(1, endresult, data{a7, 4});

          end
          %allsamples = cat(1, allsamples, frame_absolute);
          allword_frontier = cat(1, allword_frontier, word_frontier);
          %allenvelope = cat(1, allenvelope, envelope_function);
          %allvariance = cat(1, allvariance, window_variance);
          %allthreshold = cat(1, allthreshold, threshold);
          %allbit_frontier = cat(1, allbit_frontier, (data{a7, 2} + ((f-1)*SPF) ) );
        end

        word_success_rate = (counter_patterns(1, 1)/counter_patterns(1, 2));
        bit_success_rate = nthroot(word_success_rate, bits_per_word);

        word_success_rate = word_success_rate * 100;
        bit_success_rate = bit_success_rate * 100;

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

  disp(word_success_rate);
  disp(bit_success_rate);

  %% ===========================================================================
  %% FINAL CHARTS
  %% ===========================================================================
  testing_results(j, 1) = signal_frequency;
  testing_results(j, 2) = word_success_rate;
  testing_results(j, 3) = bit_success_rate;

end
%% ===========================================================================
%% END OF PROGRAM
%% ===========================================================================
sound(a,fs_beep)
