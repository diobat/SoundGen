clc;
clear all;
close all;

% Captura e possivel playback de sinais de radio 105.6 FM (R�dio Regi�o de Bastos

fc = 105600000; %   105.6 FM
fa = fc * 10;
fd = 110000; % frequency deviation de 100khz


soc = rtl_sdr_connect;  % open local tcp socket to rtl_tcp
dummy = rtl_sdr_getData(soc,2.4e6); % get 1 second worth of samples, and throw away>> dummy = rtl_sdr_getData(soc,2.4e6); % get 1 second worth of samples, and throw away
y = rtl_sdr_getData(soc, fa); % get 1 seconds worth of samples
soc.close;  % close socket

mistura = fmdemod(y, fc, fa, fd);

figure(1);
plot(y);

figure(2);
plot(mistura);

soundsc(mistura)

%close all;