function [highest_quality, offset_index] = Synchronize(signal, blockSampleSize, threshold)


quality = zeros(ceil(blockSampleSize),1);

numberOfBlocks = floor(length(signal)/ blockSampleSize);

for i = 1 : blockSampleSize
amplitudeSum = 0;
  for a = 0 : (numberOfBlocks-2)
    for b = (floor(a*blockSampleSize)+i) : floor((a+1)*blockSampleSize+i)
      amplitudeSum = abs(signal(b) - threshold(b));
      quality(i) = quality(i) + amplitudeSum;
    end
  end
end

[highest_quality, offset_index] = max(quality);
