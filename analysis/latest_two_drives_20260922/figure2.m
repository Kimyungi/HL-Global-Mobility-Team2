function fig = figure2(csvFile, outputDir)
%FIGURE2 Local GPS x-y trajectory, colored by replayed TargetRef.state.
% figure2() plots the 16:05 drive. Pass a CSV path for another drive.
% Outputs: <CSV name>_figure2.png and an editable MATLAB .fig file.
baseDir = fileparts(mfilename('fullpath'));
if nargin < 1 || isempty(csvFile)
    csvFile = fullfile(baseDir, 'v2_20260920_160548_762545.csv');
end
if nargin < 2 || isempty(outputDir)
    outputDir = fullfile(baseDir, 'figures');
end
opts = detectImportOptions(csvFile);
opts = setvartype(opts, {'elapsed_s','x','y','state','gps_position_valid','matched'}, 'double');
T = readtable(csvFile, opts);
assert(all(isfinite(T.elapsed_s)) && all(diff(T.elapsed_s) >= 0), 'Invalid elapsed time.');
valid = T.matched == 1 & T.gps_position_valid == 1 & ...
    isfinite(T.x) & isfinite(T.y) & isfinite(T.state);
assert(any(valid),'No valid GPS/state samples.');
assert(all(ismember(T.state(valid),0:5)),'Unexpected state ID.');
% Fixed colors across both drives: 0 LANE ... 5 ESTOP.
colors = [0 0.447 0.741; 0.1 0.65 0.25; 0.929 0.55 0.05; ...
    0.494 0.184 0.556; 0.1 0.7 0.8; 0.85 0.1 0.15];
labels = {'0 LANE','1 WAYPOINT','2 AVOID','3 PARKING','4 TRAFFIC','5 ESTOP'};
[~, name] = fileparts(csvFile);
fig = figure(2); clf(fig);
set(fig,'Color','w','Position',[100 80 1100 850], ...
    'Name',['Figure 2 | ' name],'NumberTitle','off');
ax = axes(fig); hold(ax,'on');
% Only adjacent valid samples may be connected. Each segment takes the
% starting sample's state; every sample also has its own state-colored dot.
edgeOK = valid(1:end-1) & valid(2:end) & diff(T.elapsed_s) <= 0.1;
handles = gobjects(0); names = {};
for s = 0:5
    sample = valid & T.state == s;
    if ~any(sample), continue; end
    edge = find(edgeOK & T.state(1:end-1) == s);
    xx = [T.x(edge) T.x(edge+1) nan(numel(edge),1)]';
    yy = [T.y(edge) T.y(edge+1) nan(numel(edge),1)]';
    plot(ax,xx(:),yy(:),'-','Color',colors(s+1,:), ...
        'LineWidth',1.5,'HandleVisibility','off');
    handles(end+1) = plot(ax,T.x(sample),T.y(sample),'.', ...
        'Color',colors(s+1,:),'MarkerSize',5); %#ok<AGROW>
    names{end+1} = labels{s+1}; %#ok<AGROW>
end
first = find(valid,1,'first'); last = find(valid,1,'last');
handles(end+1) = plot(ax,T.x(first),T.y(first),'ko','MarkerFaceColor','w','MarkerSize',9,'LineWidth',1.5);
names{end+1} = 'Start';
handles(end+1) = plot(ax,T.x(last),T.y(last),'kx','MarkerSize',10,'LineWidth',2);
names{end+1} = 'End';
axis(ax,'equal'); grid(ax,'on'); box(ax,'on');
set(ax,'FontSize',11); xlabel(ax,'x (m)'); ylabel(ax,'y (m)');
title(ax,['Figure 2 | ' name],'Interpreter','none');
subtitle(ax,'Local GPS trajectory | color = TargetRef.state');
legend(ax,handles,names,'Location','eastoutside');
if ~isfolder(outputDir), mkdir(outputDir); end
drawnow;
exportgraphics(fig,fullfile(outputDir,[name '_figure2.png']),'Resolution',180);
savefig(fig,fullfile(outputDir,[name '_figure2.fig']));
end
