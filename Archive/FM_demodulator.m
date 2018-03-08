% % Request user input from the command-line for application parameters
% userInput = helperFMUserInput;
% 
% % Calculate FM system parameters based on the user input
% [fmRxParams,sigSrc] = helperFMConfig(userInput);

fmRxParams = getParamsSdrrFMExamples

% Create FM broadcast receiver object and configure based on user input
fmBroadcastDemod = comm.FMBroadcastDemodulator(...
    'SampleRate', fmRxParams.FrontEndSampleRate, ...
    'FrequencyDeviation', fmRxParams.FrequencyDeviation, ...
    'FilterTimeConstant', fmRxParams.FilterTimeConstant, ...
    'AudioSampleRate', fmRxParams.AudioSampleRate, ...
    'Stereo', true);

% Create audio player
player = audioDeviceWriter('SampleRate',fmRxParams.AudioSampleRate);

% Initialize radio time
radioTime = 0;

% Main loop
while radioTime < userInput.Duration
  % Receive baseband samples (Signal Source)
  if fmRxParams.isSourceRadio
    [rcv,~,lost,late] = sigSrc();
  else
    rcv = sigSrc();
    lost = 0;
    late = 1;
  end

  % Demodulate FM broadcast signals and play the decoded audio
  audioSig = fmBroadcastDemod(rcv);
  player(audioSig);

  % Update radio time. If there were lost samples, add those too.
  radioTime = radioTime + fmRxParams.FrontEndFrameTime + ...
    double(lost)/fmRxParams.FrontEndSampleRate;
end

% Release the audio and the signal source
release(sigSrc)
release(fmBroadcastDemod)
release(player)