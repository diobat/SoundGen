function [result] = Define_Carrier()
result = -1;
while result <= 0 || result >= 1.4e9
result = input('Carrier Frequency:')
end
end
