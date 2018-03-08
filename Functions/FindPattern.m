function [result] = FindPattern(patterns, signal)

number_of_patterns = length(patterns);
Ls = length(signal);
result = zeros(1, number_of_patterns);

for x = 1: number_of_patterns

  pattern_size = length(patterns{x,:});

  if signal(end-pattern_size+1:end) == patterns{x,:}
    result(1, x) = 1;
  end
end
