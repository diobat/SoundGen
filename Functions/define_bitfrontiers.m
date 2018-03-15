function [result] = define_bitfrontiers(dataLength, offset, samples_per_bit)

  slice_bit_frontiers =  zeros(floor(dataLength/samples_per_bit), 1);

  for a2 = 1 : length(slice_bit_frontiers)
    slice_bit_frontiers(a2) = offset + round((a2-1) * samples_per_bit);
  end

  slice_bit_frontiers = [slice_bit_frontiers; dataLength];

  if  ~isempty(slice_bit_frontiers)
    if slice_bit_frontiers(1) ~= 1
      slice_bit_frontiers = [1; slice_bit_frontiers]; % <- include 1 and end on the frontiers, this means the first and last interval will be uneven.
    end
  end

  result = slice_bit_frontiers;

end
