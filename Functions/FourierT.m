function [result] = FourierT(signal, Fs)

L = length(signal);

Y = fft(signal);

P2 = abs(Y/L);
P1 = P2(1:L/2+1);
P1(2:end-1) = 2*P1(2:end-1);

figure(7);
f = Fs*(0:(L/2))/L;
%f = f/100; %% <--- Remover esta linha eventualmente
plot(f,P1)
title('Single-Sided Amplitude Spectrum of X(t)')
xlabel('f (Hz)')
ylabel('|P1(f)|')

%o = ifft(P1);

%figure(8);
%plot(abs(o));
%title('Signal Salvaged from the Single-Sided Amplitude Spectrum of X(t)')
%xlabel('Sample number')
%ylabel('A')
