
figure(10);


plot(allsamples);
hold on;
p1 = allenvelope(:, 1);
p2 = allenvelope(:, 2);
plot(p1, 'r');
plot(p2, 'r');
plot(allthreshold, 'k');
%plot(index, peaks, 'og')
ylim([0 1.5])
grid on;

for x=1:fmRxParams.SamplesPerFrame:length(allsamples)

plot([x x], [0 1], 'm');

end

offset = 0.5;
pos = 1:length(allvariance);


plot(pos,allvariance * 40 + offset, 'c');

split= (max(allvariance)-min(allvariance))*0.2;
split_bin = im2bw(allvariance, split);

split = split * 40 + offset;
pos3 = 1:1:length(split_bin);

plot([1 length(allvariance)], [split split], 'k');
%plot(pos3, split_bin * 0.1 + 0.5, 'r');

pos4 = 1:1:length(allword_frontier);
plot(pos4, allword_frontier* 0.1 + offset, 'r');
