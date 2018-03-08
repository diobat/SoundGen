%% FM Receiver with RTL-SDR Radio
% This example shows how to use the RTL-SDR radio with MATLAB(R) to build
% an FM mono or stereo receiver.

% Copyright 2013-2016 The MathWorks, Inc.

%% Initialization
% The getParamsSdrrFMExamples function initializes some simulation
% parameters and generates a structure, fmRxParams. The fields of this
% structure are the parameters of the FM Broadcast Demodulator Baseband and
% RTL-SDR Receiver System objects(TM).

fmRxParams = getParamsSdrrFMExamples

%% RTL-SDR
% The script communicates with the RTL-SDR radio using the RTL-SDR System
% object. You set the properties using name-value pair arguments. Set the
% center frequency to 102.5 MHz and enable the tuner AGC.

%% 
% The RTL-SDR receiver System object takes in baseband discrete-time
% complex samples from RTL-SDR hardware. Set the sample rate to 240 kHz. 
% Frame length controls the number of samples at the output of the RTL-SDR
% receiver, which is the input to the FM Broadcast Demodulator Baseband 
% block. The frame length must be an integer multiple of the decimation 
% factor, which is 30, and 256. Set the frame length to lcm(30, 256), 
% which is 3840 samples. Select the output data type as single to reduce 
% the required memory and speed up execution. 

radio = comm.SDRRTLReceiver('CenterFrequency', 105.6e6, ...
    'SampleRate', fmRxParams.RadioSampleRate, ...
    'EnableTunerAGC', true, ...
    'SamplesPerFrame', fmRxParams.RadioFrameLength, ...
    'OutputDataType', 'single')

%%
% You can obtain information about the radio using the info method
% of the object. This method returns a structure with fields that specify
% the RTL-SDR radio hardware information and settings of the radio. Note
% that some values, such as the center frequency, may be different then the
% CenterFrequency property value of the System object. This is due to
% quantization and hardware limitations.

hwInfo = info(radio)

%% FM Demodulation
% The FM Broadcast Demodulator Baseband block converts the sampling rate of
% 240 kHz to 48 kHz, a native sampling rate for your host computer's audio
% device. According to the FM broadcast standard in the United States, the
% deemphasis lowpass filter time constant is set to 75 microseconds. Set up
% the demodulator to process stereo signals. The demodulator can also
% process the signals in a mono fashion.

fmBroadcastDemod = comm.FMBroadcastDemodulator(...
    'SampleRate', fmRxParams.RadioSampleRate, ...
    'FrequencyDeviation', fmRxParams.FrequencyDeviation, ...
    'FilterTimeConstant', fmRxParams.FilterTimeConstant, ...
    'AudioSampleRate', fmRxParams.AudioSampleRate, ...
    'PlaySound', true, ...
    'BufferSize', fmRxParams.BufferSize, ...
    'Stereo', true);

%%
% To perform stereo decoding, the FM Broadcast Demodulator Baseband block 
% uses a peaking filter which picks out the 19 kHz pilot tone from which 
% the 38 kHz carrier is created. Using the obtained carrier signal, the 
% FM Broadcast Demodulator Baseband block downconverts the L-R signal, 
% centered at 38 kHz, to baseband. Afterwards, the L-R and L+R signals 
% pass through a 75 microsecond deemphasis filter . The FM Broadcast 
% Demodulator Baseband block separates the L and R signals and converts 
% them to the 48 kHz audio signal.

%% Stream Processing Loop
%
% Capture FM signals and apply FM demodulation for 10 seconds which is
% specified by fmRxParams.StopTime.

%%
% Check for the status of the RTL-SDR radio
if ~isempty(sdrinfo(radio.RadioAddress))
    % Loop until the example reaches the simulation stop time
    timeCounter = 0;
    while timeCounter < fmRxParams.StopTime
        % Get baseband samples from RTL-SDR radio
        [x, ~] = step(radio);  % no 'len' output needed for blocking 
                               % operation
        step(fmBroadcastDemod, x);
        % Update counter
        timeCounter = timeCounter + fmRxParams.AudioFrameTime;
    end
else
    warning(message('SDR:sysobjdemos:MainLoop'))
end

%% 
% Release the audio and RTL-SDR resources.

release(radio)
release(fmBroadcastDemod)

%% Conclusion
% In this example, you used Communications System Toolbox(TM) System
% objects to build an FM receiver utilizing the RTL-SDR radio. The example
% showed that the MATLAB script can process signals captured by the RTL-SDR
% radio in real time.

%% Further Exploration
% To further explore the example, you can vary the center frequency of the
% RTL-SDR radio and listen to other radio stations.
%
% If you have your own FM transmitter that can transmit .wma files, you can
% duplicate the test that shows the channel separation result above. Load
% the sdruFMStereoTestSignal.wma file into your transmitter. The channel
% separation can be easily observed from the spectrum and heard from the
% audio device. You can also adjust the gain compensation to see its effect
% on stereo separation.
%
% You can set the Stereo property of the FM demodulator object to false to
% process the signals in mono fashion and compare the sound quality.

%% Appendix
% The following function is used in this example:
%
% getParamsSdrrFMExamples.m

%% Selected Bibliography 
% 
% http://en.wikipedia.org/wiki/FM_broadcasting
