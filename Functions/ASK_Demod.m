function [result] = ASK_Demod(input, threshold, bit_frontier)

  result = zeros(length(bit_frontier)-1, 1); % << -- intervals, not borders, hence the -1
  left_discard = 0; % <--- percentage of samples to cut from the left side of the window;
  right_discard = 0;


for b1 = 1 : (length(bit_frontier)-1) % << -- last interval is ignored, so the last relevant interval is between frontier(end-2) and frontier(end-1)

  window = bit_frontier(b1):bit_frontier(b1+1);
  interval = input(window);
  p1 = max( round( length(interval)*left_discard ) , 1); % <---- funçao max para evitar que o valor dê zero
  p2 = round( length(interval)*(1-right_discard));

  interval = interval(p1:p2);
  b2 = mean(interval);

  if b2 > threshold(bit_frontier(b1))
    result(b1) = 1;
  end
end
