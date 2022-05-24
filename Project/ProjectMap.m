%% Create plot from recording and map. 
% Get the car track.
recording = readmatrix('CID-111-recording-2022-05-23_151230.rec.csv/opendlv.sim.Frame-0.csv');
x = recording(:,7);
y = recording(:,8);

% Then the cones. 
map = readmatrix('map.csv');
blueCones = map(1,:);
blueCones = blueCones(6:187);
blueCones = nonzeros(blueCones');
xBlueCones = blueCones(1:2:end);
yBlueCones = blueCones(2:2:end);
blueCones = [xBlueCones, yBlueCones];

yellowCones = map(2,:);
yellowCones = yellowCones(6:171);
yellowCones = nonzeros(yellowCones');
xYellowCones = yellowCones(1:2:end);
yYellowCones = yellowCones(2:2:end);
yellowCones = [xYellowCones, yYellowCones];

redCones = map(3,:);
redCones = [redCones(6), redCones(7);
            redCones(10), redCones(11)];

% The plot.
hold on
%set(gca,'Color','k')
scatter(blueCones(:,1), blueCones(:,2), 'b.')
scatter(yellowCones(:,1), yellowCones(:,2), 'y.')
scatter(redCones(:,1), redCones(:,2), 'r.')
plot(x, y)
xlabel('x-coord')
ylabel('y-coord')
title('Plot of the map with an example simulation.')
axis equal
hold off
