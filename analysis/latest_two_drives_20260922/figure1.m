function fig = figure1(csvFile, outputDir)
%FIGURE1 Speed (top) and steering (bottom), versus elapsed time.
% figure1() plots the 16:05 drive. Pass a CSV path for another drive.
% Outputs: <CSV name>_figure1.png and an editable MATLAB .fig file.
baseDir = fileparts(mfilename('fullpath'));
if nargin < 1 || isempty(csvFile)
    csvFile = fullfile(baseDir, 'v2_20260920_160548_762545.csv');
end
if nargin < 2 || isempty(outputDir)
    outputDir = fullfile(baseDir, 'figures');
end
opts = detectImportOptions(csvFile);
columns = {'elapsed_s','v_ref','v_act','str_ref','str_act'};
opts = setvartype(opts, columns, 'double');
T = readtable(csvFile, opts);
t = T.elapsed_s;
assert(all(isfinite(t)) && all(diff(t) >= 0), 'Invalid elapsed time.');
% Break lines across CAN gaps >100 ms, preserving both endpoint samples.
gap = [false; diff(t) > 0.1];
idx = (1:height(T))' + cumsum(gap);
tp = nan(height(T) + nnz(gap), 1);
yp = nan(numel(tp), 4);
tp(idx) = t;
yp(idx,:) = T{:,columns(2:end)};
[~, name] = fileparts(csvFile);
fig = figure(1); clf(fig);
set(fig, 'Color','w', 'Position',[80 80 1400 780], ...
    'Name',['Figure 1 | ' name], 'NumberTitle','off');
layout = tiledlayout(fig, 2, 1, 'TileSpacing','compact', 'Padding','compact');
ax1 = nexttile(layout);
plot(ax1, tp, yp(:,1), '-', 'Color',[0 0.35 0.75], 'LineWidth',1.3);
hold(ax1,'on');
plot(ax1, tp, yp(:,2), '-', 'Color',[0.9 0.3 0.1], 'LineWidth',0.9);
ylabel(ax1,'Speed (m/s)'); title(ax1,'Speed tracking');
legend(ax1,{'v_ref','v_act'},'Interpreter','none','Location','northeast');
ax2 = nexttile(layout);
plot(ax2, tp, yp(:,3), '-', 'Color',[0 0.35 0.75], 'LineWidth',1.3);
hold(ax2,'on');
plot(ax2, tp, yp(:,4), '-', 'Color',[0.9 0.3 0.1], 'LineWidth',0.9);
ylabel(ax2,'Steering angle (rad)'); xlabel(ax2,'Elapsed time (s)');
title(ax2,'Steering tracking');
legend(ax2,{'str_ref','str_act'},'Interpreter','none','Location','northeast');
linkaxes([ax1 ax2],'x');
for ax = [ax1 ax2]
    grid(ax,'on'); box(ax,'on');
    set(ax,'FontSize',11); xlim(ax,[t(1) t(end)]);
end
title(layout, ['Figure 1 | ' name], 'Interpreter','none');
subtitle(layout,'Original units; missing values and CAN gaps > 0.1 s are not connected');
if ~isfolder(outputDir), mkdir(outputDir); end
drawnow;
exportgraphics(fig,fullfile(outputDir,[name '_figure1.png']),'Resolution',180);
savefig(fig,fullfile(outputDir,[name '_figure1.fig']));
end
