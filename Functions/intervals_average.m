function [result] = intervals_average(input, bit_frontier)

  result = zeros(length(bit_frontier)-1, 1); % << -- intervals, not borders, hence the -1

  for b1 = 1 : (length(bit_frontier)-2) % << -- last interval is ignored, so the last relevant interval is between frontier(end-2) and frontier(end-1)

    window = bit_frontier(b1):bit_frontier(b1+1);
    interval = input(window);
    result(b1) = mean(interval);
  end
end
