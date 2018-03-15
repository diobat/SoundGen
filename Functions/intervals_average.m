function [result] = intervals_average(input, bit_frontier)

  result = zeros(length(bit_frontier)-1, 1); % << -- intervals, not borders, hence the -2
  left_discard = 0; % <--- percentage of samples to cut from the left side of the window;
  right_discard = 0;


  for b1 = 1 : length(result) % << -- last interval is ignored, so the last relevant interval is between frontier(end-2) and frontier(end-1)

    size = bit_frontier(b1+1) - bit_frontier(b1);
    begin1 = floor(bit_frontier(b1) + (size-1) * left_discard);
    end1 = ceil(bit_frontier(b1) + (size-1) * (1-right_discard));

    interval = input(begin1:end1);
    result(b1) = mean(interval);
  end
end
