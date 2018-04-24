extensive_debug = 1;

alldata2 = alldata;

%alldata2 = cell(floor(length(alldata)/2), 8);
%alldata2 = alldata(floor(length(alldata)/2):end, 1:end);

figure(10);

samples_processed = 0;
sub_frames_processed = 0;

allsamples = [];
allthreshold = [];
allbit_frontier = [];
allenvelope = [,];
endresult = [];
allintra_frontier_averages = [];
allvariance = [];
allword_frontier = [];

for x = 1 : length(alldata2);

  allsamples = cat(1, allsamples, alldata2{x, 1});
  allenvelope = cat(1, allenvelope, alldata2{x, 6});
  allthreshold = cat (1, allthreshold, alldata2{x, 3});
  endresult = cat(1, endresult, alldata2{x, 4});
  allbit_frontier = cat(1, allbit_frontier, alldata2{x, 2} + samples_processed);
  allintra_frontier_averages = cat(1, allintra_frontier_averages , alldata2{x, 5});
  allvariance = cat(1, allvariance, alldata2{x, 7});
  allword_frontier = cat(1, allword_frontier, alldata2{x, 8});

  samples_processed = samples_processed + length(alldata2{x, 1});
  sub_frames_processed = sub_frames_processed + 1;
end




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

y = 1:SPF:length(allsamples);

for x=1:SPF:length(allsamples)

  plot([x x], [0 1.5], 'm');

end



offset = 0.4;
pos = 1:length(allvariance);


%plot(pos, allvariance * 20 + offset, 'c');

split = (max(allvariance)-min(allvariance))*0.30;
split_bin = im2bw(allvariance, split);

split = split * 20 + offset;
pos3 = 1:1:length(split_bin);

%plot([1 length(allvariance)], [split split], 'k');


%%%%%%%%%%%%%%%plot(pos3, split_bin * 0.1 + 0.5, 'r');

pos4 = 1:1:length(allword_frontier);
%plot(pos4, allword_frontier* -0.6 + offset, 'r');


if extensive_debug == 1
  for b1 = 1 : length(endresult)-1 % <---- corta um bocado o fim para evitar warnings the mismatched lengths, isto pode significar que as fronteiras apresentadas nao sejam fidedignas
    b2 = mean(allsamples (allbit_frontier(b1):allbit_frontier(b1+1)));
    b3 = endresult(b1);
  plot([allbit_frontier(b1), allbit_frontier(b1+1)], [b2,b2] , '--r');
  %plot([allbit_frontier(b1), allbit_frontier(b1+1)], [b3, b3] .*0.4 + 0.7, 'k')
  %z1 = plot([bit_frontier(b1), bit_frontier(b1)], [0, 1.5], 'k');  %<---- comment to turn off black vertical separation lines
  end
end
