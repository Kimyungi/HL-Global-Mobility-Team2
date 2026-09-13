%% Plot Halla reference paths, GPS-only zones, and mission state points.
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
stateEast = nan(3, 1);
stateNorth = nan(3, 1);
for pathId = 1:7
    pathFile = fullfile(waypointDir, ...
        sprintf('waypoints_halla_reference_path_%02d.csv', pathId));
    T = readtable(pathFile, 'VariableNamingRule', 'preserve');
    pathHandles(pathId) = plot(ax, T.east_m, T.north_m, '-', ...
        'Color', pathColors(pathId, :), 'LineWidth', 2.0, ...
        'DisplayName', sprintf('Path %d', pathId));
    for zoneId = 1:3
        zoneRows = T.zone_id == zoneId;
        plot(ax, T.east_m(zoneRows), T.north_m(zoneRows), '.', ...
            'Color', zoneColors(zoneId, :), 'MarkerSize', 7, ...
            'HandleVisibility', 'off');
    end
    for stateId = 1:3
        stateRow = find(T.state == stateId, 1);
        if ~isempty(stateRow)
            stateEast(stateId) = T.east_m(stateRow);
            stateNorth(stateId) = T.north_m(stateRow);
        end
    end
end

stateLabels = ["STATE 1: T PARKING", "STATE 2: PARALLEL PARKING", ...
    "STATE 3: TRAFFIC SIGNAL"];
stateMarkers = ['p', 's', 'd'];
stateColors = [0 0 0; 0.55 0 0.75; 0.85 0 0];
stateHandles = gobjects(3, 1);
for stateId = 1:3
    stateHandles(stateId) = plot(ax, stateEast(stateId), stateNorth(stateId), ...
        stateMarkers(stateId), ...
        'MarkerSize', 13, 'MarkerFaceColor', 'w', ...
        'MarkerEdgeColor', stateColors(stateId, :), 'LineWidth', 1.8, ...
        'DisplayName', stateLabels(stateId));
    text(ax, stateEast(stateId), stateNorth(stateId), ...
        "  " + stateLabels(stateId), 'FontWeight', 'bold', ...
        'Color', stateColors(stateId, :), 'VerticalAlignment', 'bottom');
end

axis(ax, 'equal');
grid(ax, 'on');
box(ax, 'on');
xlabel(ax, 'east\_m (m)');
ylabel(ax, 'north\_m (m)');
title(ax, 'Halla Reference Paths, GPS-only Zones, and Mission States');
legend(ax, [pathHandles; zoneHandles; stateHandles], 'Location', 'eastoutside');

outputFile = fullfile(waypointDir, 'halla_reference_mission.png');
exportgraphics(ax, outputFile, 'Resolution', 200);
fprintf('Saved: %s\n', outputFile);
