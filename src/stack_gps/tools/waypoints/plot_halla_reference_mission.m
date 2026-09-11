%% Plot Halla reference paths, GPS-only rectangles, and parking points.
clear;
close all;
clc;

scriptDir = fileparts(mfilename('fullpath'));
waypointDir = fullfile(scriptDir, '..', '..', 'waypoints');

pathColors = lines(7);
zoneColors = [1.00 0.15 0.10; 1.00 0.72 0.00; 0.00 0.80 0.90];
zoneCorners = readtable(fullfile(waypointDir, ...
    'halla_reference_zone_corners.csv'), 'VariableNamingRule', 'preserve');

fig = figure('Color', 'w', 'Name', 'Halla reference mission');
ax = axes(fig);
hold(ax, 'on');

zoneHandles = gobjects(3, 1);
for zoneId = 1:3
    rows = zoneCorners.zone_id == zoneId;
    east = zoneCorners.east_m(rows);
    north = zoneCorners.north_m(rows);
    east(end + 1) = east(1);
    north(end + 1) = north(1);
    zoneHandles(zoneId) = patch(ax, east, north, zoneColors(zoneId, :), ...
        'FaceAlpha', 0.16, 'EdgeColor', zoneColors(zoneId, :), ...
        'LineWidth', 2.2, 'DisplayName', sprintf('Zone %d', zoneId));
    text(ax, mean(east(1:4)), mean(north(1:4)), sprintf('ZONE %d', zoneId), ...
        'HorizontalAlignment', 'center', 'FontWeight', 'bold', ...
        'Color', zoneColors(zoneId, :) .* 0.65);
end

pathHandles = gobjects(7, 1);
for pathId = 1:7
    pathFile = fullfile(waypointDir, ...
        sprintf('waypoints_halla_reference_path_%02d.csv', pathId));
    T = readtable(pathFile, 'VariableNamingRule', 'preserve');
    pathHandles(pathId) = plot(ax, T.east_m, T.north_m, '-', ...
        'Color', pathColors(pathId, :), 'LineWidth', 2.0, ...
        'DisplayName', sprintf('Path %d', pathId));
end

parkingEast = [-31.8932, -62.7363];
parkingNorth = [-33.4497, -65.9110];
parkingLabels = ["T PARKING", "PARALLEL PARKING"];
parkingMarkers = ['p', 's'];
for k = 1:2
    plot(ax, parkingEast(k), parkingNorth(k), parkingMarkers(k), ...
        'MarkerSize', 13, 'MarkerFaceColor', 'w', ...
        'MarkerEdgeColor', 'k', 'LineWidth', 1.8, ...
        'HandleVisibility', 'off');
    text(ax, parkingEast(k), parkingNorth(k), "  " + parkingLabels(k), ...
        'FontWeight', 'bold', 'Color', 'k', 'VerticalAlignment', 'bottom');
end

axis(ax, 'equal');
grid(ax, 'on');
box(ax, 'on');
xlabel(ax, 'east\_m (m)');
ylabel(ax, 'north\_m (m)');
title(ax, 'Halla Reference Paths, GPS-only Zones, and Parking Points');
legend(ax, [pathHandles; zoneHandles], 'Location', 'eastoutside');

outputFile = fullfile(waypointDir, 'halla_reference_mission.png');
exportgraphics(ax, outputFile, 'Resolution', 200);
fprintf('Saved: %s\n', outputFile);
