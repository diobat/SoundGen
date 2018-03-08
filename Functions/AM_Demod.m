function [result] = AM_Demod(input, divider, offset)

%% DIVIDER != 0

%% USAGE => Detects the presence or absence of a carrier wave. Divider and
%% offset parameters are used to adjust the input window so It is centered
%% on 0.5 with a width below +-1.

x = abs(input)/divider + offset;

result = round(abs(x));

end
