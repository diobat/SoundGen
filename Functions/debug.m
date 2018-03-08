debug_interval1 = length(allsamples) * 0.65;
%debug_interval2 = length(allsamples) * 0.75;

debug_interval2 = debug_interval1 + 512*20*2; %% <---- 2 frames inteiras


%debug_allsamples = allsamples(debug_interval1:debug_interval2);

p1 = allenvelope(:, 1);
p2 = allenvelope(:, 2);



%debug_allthreshold = allthreshold(debug_interval1:debug_interval2);

%debug_allbit_frontier

figure(10);

plot(allsamples);
hold on;
plot(p1, 'r');
plot(p2, 'r');
plot(allthreshold, 'k');
%plot(index, peaks, 'og')
ylim([0 0.5])
xlim([debug_interval1 debug_interval2])
grid on;

for b1 = 1 : (floor(length(allbit_frontier)*0.8)) % <---- corta um bocado o fim para evitar warnings the mismatched lengths, isto pode significar que as fronteiras apresentadas nao sejam fidedignas
  b2 = mean( allsamples (allbit_frontier(b1):allbit_frontier(b1+1)));
plot([allbit_frontier(b1), allbit_frontier(b1+1)], [b2,b2] , '--r');
%z1 = plot([bit_frontier(b1), bit_frontier(b1)], [0, 1.5], 'k');  %<---- comment to turn off black vertical separation lines
end
