function [highest_quality, offset_index] = Synchronize(signal, samples_per_bit, threshold)


quality = zeros(ceil(samples_per_bit),1);

number_of_bits = floor(length(signal)/ samples_per_bit);

for i = 1 : samples_per_bit
amplitudeSum = 0;
  for a = 0 : (number_of_bits-2)
    for b = (floor(a*samples_per_bit)+i) : floor((a+1)*samples_per_bit+i)
      amplitudeSum = abs(signal(b) - threshold(b));
      quality(i) = quality(i) + amplitudeSum;
    end
  end
end

[highest_quality, offset_index] = max(quality);
