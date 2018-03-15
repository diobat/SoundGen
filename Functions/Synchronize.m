function [highest_quality, offset_index] = Synchronize(signal, samples_per_bit, threshold)


quality = zeros(ceil(samples_per_bit),1);

number_of_bits = floor(length(signal)/ samples_per_bit);

for i = 1 : ceil(samples_per_bit)
amplitudeSum = 0;
  for a = 0 : (number_of_bits-2)

      b1 = floor(a*samples_per_bit)+i;;
      b2 = floor((a+1)*samples_per_bit)+i;

      signal_mean = mean(signal(b1:b2));
      threshold_mean = mean(threshold(b1:b2));

      amplitudeSum = abs(signal_mean - threshold_mean);
      quality(i) = quality(i) + amplitudeSum;

  end
end

[highest_quality, offset_index] = max(quality);
