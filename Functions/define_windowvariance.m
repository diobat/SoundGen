function [result] = define_windowvariance(SPF, frame, window_size)

  window_variance = zeros(SPF, 1);

  for a0 = 1 : SPF - window_size
    word_window = frame(a0:a0+window_size-1);
    window_variance(a0, 1) = var(word_window);
  end
  window_variance(a0+1:end) = window_variance(a0);

  result = window_variance;

end
