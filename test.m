clear; clc; close all;

scriptDir = fileparts(mfilename('fullpath'));
if isempty(scriptDir)
    scriptDir = pwd;
end

cfg.playbackSpeed = 1.0;
cfg.startFrame = 1;
cfg.endFrame = inf;
cfg.frameStep = 1;

cfg.tofMaxRangeM = 2.5;
cfg.tofMaxRangeMm = round(cfg.tofMaxRangeM * 1000.0);
cfg.maxProcessRangeMm = 2500;
cfg.maxTargetNum = 6;

cfg.DIS_REACT = 1400;
cfg.DIS_SLOW = 700;
cfg.DIS_STOP = 500;
cfg.DIS_FEAR = 150;
cfg.DIS_GROUND_MIN = 400;
cfg.DIS_CEILING_MIN = 600;
cfg.GROUND_BORDER = 6;
cfg.CELLING_BORDER = 2;
cfg.MIN_PIXEL_NUMBER = 1;

cfg.TURN_MAX = 1.0;
cfg.TURN_SLOW = 0.7;
cfg.TURN_FAST = 3.0;
cfg.TURN_NOT = 0.0;
cfg.VEL_STOP = 0.0;
cfg.VEL_FEAR = -0.2;
cfg.VEL_SCALE_MEDIUM = 1.0;
cfg.VEL_SCALE_SLOW = 0.7;
cfg.VEL_UP = 0.2;
cfg.VEL_DOWN = -0.5;
cfg.MAX_TURN_RATIO = 0.8;
cfg.EPSILON = 1e-4;

cfg.droneZoneRows = 4:6;
cfg.droneZoneCols = 4:5;
cfg.careZoneRows = 3:6;
cfg.careZoneCols = 3:6;
cfg.middlePosY = 3.5;

cfg.channelLength = 10.0;
cfg.channelWidth = 2.0;
cfg.channelHeight = 1.8;
cfg.robotRadius = 0.175;
cfg.fovH = deg2rad(45.0);
cfg.fovV = deg2rad(45.0);
cfg.xMin3D = -1.5;
cfg.xMax3D = 12.5;
cfg.yMax3D = 12.5;

logFile = fullfile(scriptDir, 'tof_sim_log.csv');
obsFile = fullfile(scriptDir, 'tof_obstacles.csv');
if ~isfile(logFile)
    logFile = fullfile(scriptDir, 'build', 'tof_sim_log.csv');
    obsFile = fullfile(scriptDir, 'build', 'tof_obstacles.csv');
end

if ~isfile(logFile) || ~isfile(obsFile)
    error('Simulation data not found. Run the C++ program first.');
end

T = readtable(logFile);
O = readtable(obsFile);

frameCount = height(T);
cfg.startFrame = max(1, cfg.startFrame);
cfg.endFrame = min(frameCount, cfg.endFrame);
cfg.frameStep = max(1, round(cfg.frameStep));
if cfg.startFrame > cfg.endFrame
    error('startFrame must be less than or equal to endFrame.');
end

baseDt = median(diff(T.time));
if isempty(baseDt) || ~isfinite(baseDt) || baseDt <= 0
    baseDt = 0.02;
end
cfg.baseDt = baseDt;

analysis = preprocessFrames(T, cfg);
hasDesktop = usejava('desktop');

figVisible = 'on';
if ~hasDesktop
    figVisible = 'off';
end

fig = figure( ...
    'Name', 'ToF Corridor Avoidance Viewer', ...
    'Color', 'w', ...
    'Position', [70 40 1560 950], ...
    'Visible', figVisible, ...
    'CloseRequestFcn', @(src, evt) closeViewer(src));

ui = struct();
ui.ax3d = axes(fig, 'Position', [0.04 0.10 0.56 0.80]);
setup3DAxes(ui.ax3d, cfg);
drawStaticEnvironment(ui.ax3d, O, cfg);
ui.pathLine = plot3(ui.ax3d, nan, nan, nan, 'b-', 'LineWidth', 1.8);
[sx0, sy0, sz0] = sphere(24);
ui.robotSphere = surf(ui.ax3d, sx0 * 0 + nan, sy0 * 0 + nan, sz0 * 0 + nan, ...
    'FaceColor', [0.10 0.55 0.95], 'FaceAlpha', 0.35, 'EdgeColor', 'none');
ui.robotPoint = plot3(ui.ax3d, nan, nan, nan, 'o', 'MarkerSize', 6, 'MarkerFaceColor', [0.10 0.55 0.95], 'MarkerEdgeColor', 'k');
ui.headingLine = plot3(ui.ax3d, nan, nan, nan, 'Color', [0.05 0.05 0.05], 'LineWidth', 2.0);
ui.rayHandles = gobjects(64, 1);
for k = 1:64
    ui.rayHandles(k) = plot3(ui.ax3d, nan, nan, nan, 'Color', [0.52 0.70 0.86], 'LineWidth', 0.8);
end
ui.statusText = text(ui.ax3d, cfg.xMin3D + 0.1, -0.3, cfg.channelHeight + 0.10, '', ...
    'FontWeight', 'bold', 'FontName', 'Consolas', 'FontSize', 11);

ui.axTop = axes(fig, 'Position', [0.65 0.72 0.31 0.11]);
setupTopAxes(ui.axTop, T, O, cfg);
ui.topMarker = plot(ui.axTop, nan, nan, 'o', 'MarkerSize', 7, ...
    'MarkerFaceColor', [0.13 0.42 0.85], 'MarkerEdgeColor', 'k');

ui.axInfo = axes(fig, 'Position', [0.65 0.47 0.31 0.18]);
axis(ui.axInfo, 'off');
title(ui.axInfo, 'Connected Domains & Commands');
ui.infoText = text(ui.axInfo, 0.00, 1.00, '', ...
    'Units', 'normalized', ...
    'HorizontalAlignment', 'left', ...
    'VerticalAlignment', 'top', ...
    'FontName', 'Consolas', ...
    'FontSize', 10.5, ...
    'Interpreter', 'none');

ui.axTof = axes(fig, 'Position', [0.62 0.10 0.35 0.33]);
setupTofAxes(ui.axTof);
ui.tofRects = gobjects(8, 8);
ui.bandRects = gobjects(8, 8);
ui.distText = gobjects(8, 8);
ui.compTag = gobjects(8, 8);
for r = 1:8
    for c = 1:8
        ui.tofRects(r, c) = rectangle(ui.axTof, ...
            'Position', [c - 0.5, r - 0.5, 1.0, 1.0], ...
            'FaceColor', [0.95 0.95 0.95], ...
            'EdgeColor', [0.20 0.20 0.20], ...
            'LineWidth', 1.0);
        ui.bandRects(r, c) = rectangle(ui.axTof, ...
            'Position', [c - 0.48, r + 0.25, 0.96, 0.20], ...
            'FaceColor', [0.8 0.8 0.8], ...
            'EdgeColor', 'none');
        ui.distText(r, c) = text(ui.axTof, c, r + 0.03, '', ...
            'HorizontalAlignment', 'center', ...
            'VerticalAlignment', 'middle', ...
            'FontName', 'Consolas', ...
            'FontSize', 9, ...
            'FontWeight', 'bold');
        ui.compTag(r, c) = text(ui.axTof, c - 0.43, r - 0.35, '', ...
            'HorizontalAlignment', 'left', ...
            'VerticalAlignment', 'top', ...
            'FontName', 'Consolas', ...
            'FontSize', 7.5, ...
            'FontWeight', 'bold', ...
            'Color', [0.10 0.10 0.10]);
    end
end

careRect = zoneRectangle(cfg.careZoneRows, cfg.careZoneCols);
droneRect = zoneRectangle(cfg.droneZoneRows, cfg.droneZoneCols);
ui.careZone = rectangle(ui.axTof, ...
    'Position', careRect, ...
    'EdgeColor', [0.95 0.49 0.07], ...
    'LineStyle', '--', ...
    'LineWidth', 2.0, ...
    'Curvature', 0);
ui.droneZone = rectangle(ui.axTof, ...
    'Position', droneRect, ...
    'EdgeColor', [0.84 0.16 0.16], ...
    'LineStyle', '-', ...
    'LineWidth', 2.5, ...
    'Curvature', 0);
ui.careLabel = text(ui.axTof, min(cfg.careZoneCols), min(cfg.careZoneRows) - 0.18, 'CARE_ZONE', ...
    'HorizontalAlignment', 'left', 'VerticalAlignment', 'bottom', ...
    'FontName', 'Consolas', 'FontSize', 8.5, 'FontWeight', 'bold', ...
    'Color', [0.95 0.49 0.07], 'BackgroundColor', [1 1 1], 'Margin', 1.0);
ui.droneLabel = text(ui.axTof, min(cfg.droneZoneCols), min(cfg.droneZoneRows) - 0.18, 'DRONE_ZONE', ...
    'HorizontalAlignment', 'left', 'VerticalAlignment', 'bottom', ...
    'FontName', 'Consolas', 'FontSize', 8.5, 'FontWeight', 'bold', ...
    'Color', [0.84 0.16 0.16], 'BackgroundColor', [1 1 1], 'Margin', 1.0);

ui.componentBoxes = gobjects(0);
ui.componentLabels = gobjects(0);
ui.tofLegendText = text(ui.axTof, 0.55, -0.20, '', ...
    'HorizontalAlignment', 'left', ...
    'VerticalAlignment', 'top', ...
    'FontName', 'Consolas', ...
    'FontSize', 8.6, ...
    'Interpreter', 'none', ...
    'Color', [0.15 0.15 0.15]);

ui.legendBandFear = annotation(fig, 'textbox', [0.62 0.445 0.06 0.025], 'String', 'fear <=150', ...
    'Color', [0.10 0.10 0.10], 'BackgroundColor', [0.84 0.18 0.15], 'EdgeColor', [0.84 0.18 0.15], ...
    'FontName', 'Consolas', 'FontSize', 8.5, 'HorizontalAlignment', 'center');
ui.legendBandStop = annotation(fig, 'textbox', [0.685 0.445 0.06 0.025], 'String', 'stop <=500', ...
    'Color', [0.10 0.10 0.10], 'BackgroundColor', [0.95 0.45 0.18], 'EdgeColor', [0.95 0.45 0.18], ...
    'FontName', 'Consolas', 'FontSize', 8.5, 'HorizontalAlignment', 'center');
ui.legendBandSlow = annotation(fig, 'textbox', [0.75 0.445 0.06 0.025], 'String', 'slow <=700', ...
    'Color', [0.10 0.10 0.10], 'BackgroundColor', [0.99 0.77 0.28], 'EdgeColor', [0.99 0.77 0.28], ...
    'FontName', 'Consolas', 'FontSize', 8.5, 'HorizontalAlignment', 'center');
ui.legendBandReact = annotation(fig, 'textbox', [0.815 0.445 0.07 0.025], 'String', 'react <=1400', ...
    'Color', [0.10 0.10 0.10], 'BackgroundColor', [0.87 0.91 0.54], 'EdgeColor', [0.87 0.91 0.54], ...
    'FontName', 'Consolas', 'FontSize', 8.5, 'HorizontalAlignment', 'center');
ui.legendBandClear = annotation(fig, 'textbox', [0.89 0.445 0.07 0.025], 'String', '>1400 / max', ...
    'Color', [0.10 0.10 0.10], 'BackgroundColor', [0.86 0.92 0.97], 'EdgeColor', [0.86 0.92 0.97], ...
    'FontName', 'Consolas', 'FontSize', 8.5, 'HorizontalAlignment', 'center');

ui.zoneNote = annotation(fig, 'textbox', [0.62 0.415 0.34 0.026], ...
    'String', 'red border = DRONE_ZONE(front speed) | orange dashed = CARE_ZONE(reference) | Ck = connected domain id', ...
    'EdgeColor', 'none', 'FontName', 'Consolas', 'FontSize', 8.8, 'Color', [0.18 0.18 0.18]);

ui.btnPrev = [];
ui.btnPlay = [];
ui.btnNext = [];
ui.slider = [];
ui.frameEdit = [];
ui.frameInfo = [];
ui.speedEdit = [];
ui.stepEdit = [];

playTimer = [];
if hasDesktop
    ui.btnPrev = uicontrol(fig, 'Style', 'pushbutton', 'String', 'Prev', ...
        'Units', 'normalized', 'Position', [0.04 0.93 0.05 0.035], ...
        'Callback', @(src, evt) stepFrame(fig, -1));
    ui.btnPlay = uicontrol(fig, 'Style', 'togglebutton', 'String', 'Play', ...
        'Units', 'normalized', 'Position', [0.095 0.93 0.06 0.035], ...
        'Callback', @(src, evt) togglePlayback(src, fig));
    ui.btnNext = uicontrol(fig, 'Style', 'pushbutton', 'String', 'Next', ...
        'Units', 'normalized', 'Position', [0.16 0.93 0.05 0.035], ...
        'Callback', @(src, evt) stepFrame(fig, 1));
    ui.slider = uicontrol(fig, 'Style', 'slider', ...
        'Units', 'normalized', 'Position', [0.23 0.934 0.34 0.025], ...
        'Min', cfg.startFrame, 'Max', cfg.endFrame, 'Value', cfg.startFrame, ...
        'SliderStep', sliderStep(frameCount), ...
        'Callback', @(src, evt) onSliderChanged(src, fig));
    ui.frameInfo = uicontrol(fig, 'Style', 'text', 'String', '', ...
        'Units', 'normalized', 'Position', [0.58 0.93 0.08 0.03], ...
        'BackgroundColor', 'w', 'HorizontalAlignment', 'left', ...
        'FontName', 'Consolas', 'FontSize', 10);
    uicontrol(fig, 'Style', 'text', 'String', 'Jump', ...
        'Units', 'normalized', 'Position', [0.67 0.93 0.03 0.025], ...
        'BackgroundColor', 'w', 'HorizontalAlignment', 'left');
    ui.frameEdit = uicontrol(fig, 'Style', 'edit', 'String', num2str(cfg.startFrame), ...
        'Units', 'normalized', 'Position', [0.705 0.932 0.05 0.03], ...
        'Callback', @(src, evt) jumpToFrame(fig));
    uicontrol(fig, 'Style', 'pushbutton', 'String', 'Go', ...
        'Units', 'normalized', 'Position', [0.76 0.93 0.04 0.035], ...
        'Callback', @(src, evt) jumpToFrame(fig));
    uicontrol(fig, 'Style', 'text', 'String', 'Speed', ...
        'Units', 'normalized', 'Position', [0.81 0.93 0.035 0.025], ...
        'BackgroundColor', 'w', 'HorizontalAlignment', 'left');
    ui.speedEdit = uicontrol(fig, 'Style', 'edit', 'String', num2str(cfg.playbackSpeed, '%.2f'), ...
        'Units', 'normalized', 'Position', [0.848 0.932 0.045 0.03], ...
        'Callback', @(src, evt) updatePlaybackRate(fig));
    uicontrol(fig, 'Style', 'text', 'String', 'Step', ...
        'Units', 'normalized', 'Position', [0.90 0.93 0.03 0.025], ...
        'BackgroundColor', 'w', 'HorizontalAlignment', 'left');
    ui.stepEdit = uicontrol(fig, 'Style', 'edit', 'String', num2str(cfg.frameStep), ...
        'Units', 'normalized', 'Position', [0.932 0.932 0.035 0.03], ...
        'Callback', @(src, evt) updateFrameStep(fig));

    playTimer = timer( ...
        'ExecutionMode', 'fixedSpacing', ...
        'BusyMode', 'drop', ...
        'Period', timerPeriod(cfg), ...
        'TimerFcn', @(src, evt) timerAdvance(fig));
end

state = struct( ...
    'T', T, ...
    'O', O, ...
    'cfg', cfg, ...
    'analysis', analysis, ...
    'ui', ui, ...
    'frameCount', frameCount, ...
    'currentFrame', cfg.startFrame, ...
    'isPlaying', false, ...
    'timer', playTimer);
guidata(fig, state);

renderFrame(fig, cfg.startFrame);

if ~hasDesktop
    drawnow;
    close(fig);
end

function analysis = preprocessFrames(T, cfg)
frameCount = height(T);
analysis(frameCount, 1) = struct();
history.values = [ -1, 0, 0, 0, 0 ];
history.index = 1;
for idx = 1:frameCount
    tofFrameMm = zeros(8, 8);
    for r = 1:8
        for c = 1:8
            tofFrameMm(r, c) = min(round(T.(sprintf('tof%d%d', r - 1, c - 1))(idx)), cfg.tofMaxRangeMm);
        end
    end
    [labels, components, targets] = extractTargetsFromTof(tofFrameMm, cfg);
    [command, metrics, history, explanation] = applyDecisionLogic(targets, history, cfg);

    summaryLine = sprintf( ...
        '[t=%.2fs] pos=(%.3f, %.3f, %.3f) psi=%.3f obj=%d min(front/left/right)=(%s, %s, %s)mm cmd(vx,vz,turn)=(%.3f, %.3f, %.3f) clr=%.3f', ...
        T.time(idx), T.x(idx), T.y(idx), T.z(idx), T.psi(idx), numel(targets), ...
        formatDistance(metrics.min_front), formatDistance(metrics.min_left), formatDistance(metrics.min_right), ...
        command.vx, command.vz, command.turn, T.clearance(idx));

    logged = struct( ...
        'vx', T.cmd_vx(idx), ...
        'vz', T.cmd_vz(idx), ...
        'turn', T.cmd_turn(idx), ...
        'min_front', T.min_front_mm(idx), ...
        'min_left', T.min_left_mm(idx), ...
        'min_right', T.min_right_mm(idx), ...
        'min_up', T.min_up_mm(idx), ...
        'min_down', T.min_down_mm(idx), ...
        'min_global', T.min_global_mm(idx), ...
        'object_count', T.object_count(idx));

    analysis(idx).tofMm = tofFrameMm;
    analysis(idx).labels = labels;
    analysis(idx).components = components;
    analysis(idx).targets = targets;
    analysis(idx).command = command;
    analysis(idx).metrics = metrics;
    analysis(idx).explanation = explanation;
    analysis(idx).summaryLine = summaryLine;
    analysis(idx).logged = logged;
end
end

function [labels, components, targets] = extractTargetsFromTof(tofFrameMm, cfg)
labels = zeros(8, 8);
visited = false(8, 8);
components = struct('id', {}, 'rows', {}, 'cols', {}, 'pixelCount', {}, 'minDistance', {}, ...
    'top', {}, 'bottom', {}, 'left', {}, 'right', {}, 'rowCenter', {}, 'colCenter', {});
targets = struct('position', {}, 'borders', {}, 'min_distance', {}, 'pixels_number', {}, 'component_id', {});
neighbors = [ -1 -1; -1 0; -1 1; 0 -1; 0 1; 1 -1; 1 0; 1 1 ];
componentId = 0;

for startRow = 1:8
    for startCol = 1:8
        if visited(startRow, startCol) || tofFrameMm(startRow, startCol) > cfg.maxProcessRangeMm
            continue;
        end

        componentId = componentId + 1;
        queue = zeros(64, 2);
        head = 1;
        tail = 1;
        queue(tail, :) = [startRow, startCol];
        visited(startRow, startCol) = true;

        rows = zeros(64, 1);
        cols = zeros(64, 1);
        pixelCount = 0;
        minDistance = inf;
        top = inf;
        bottom = -inf;
        left = inf;
        right = -inf;

        while head <= tail
            row = queue(head, 1);
            col = queue(head, 2);
            head = head + 1;

            pixelCount = pixelCount + 1;
            rows(pixelCount) = row;
            cols(pixelCount) = col;
            labels(row, col) = componentId;
            minDistance = min(minDistance, double(tofFrameMm(row, col)));
            top = min(top, row - 1);
            bottom = max(bottom, row - 1);
            left = min(left, col - 1);
            right = max(right, col - 1);

            for k = 1:size(neighbors, 1)
                nr = row + neighbors(k, 1);
                nc = col + neighbors(k, 2);
                if nr < 1 || nr > 8 || nc < 1 || nc > 8
                    continue;
                end
                if visited(nr, nc) || tofFrameMm(nr, nc) > cfg.maxProcessRangeMm
                    continue;
                end
                tail = tail + 1;
                queue(tail, :) = [nr, nc];
                visited(nr, nc) = true;
            end
        end

        rows = rows(1:pixelCount);
        cols = cols(1:pixelCount);
        rowCenter = mean(rows - 1);
        colCenter = mean(cols - 1);

        components(componentId).id = componentId;
        components(componentId).rows = rows;
        components(componentId).cols = cols;
        components(componentId).pixelCount = pixelCount;
        components(componentId).minDistance = minDistance;
        components(componentId).top = top;
        components(componentId).bottom = bottom;
        components(componentId).left = left;
        components(componentId).right = right;
        components(componentId).rowCenter = rowCenter;
        components(componentId).colCenter = colCenter;

        if numel(targets) < cfg.maxTargetNum
            target.position.x = rowCenter;
            target.position.y = colCenter;
            target.borders.top = top;
            target.borders.bottom = bottom;
            target.borders.left = left;
            target.borders.right = right;
            target.min_distance = minDistance;
            target.pixels_number = pixelCount;
            target.component_id = componentId;
            targets(end + 1) = target; %#ok<AGROW>
        end
    end
end
end

function [command, metrics, history, explanation] = applyDecisionLogic(targets, history, cfg)
command.vx = cfg.VEL_SCALE_MEDIUM;
command.vz = cfg.VEL_STOP;
command.turnRaw = cfg.TURN_NOT;
command.turn = cfg.TURN_NOT;

metrics.min_global = inf;
metrics.min_front = inf;
metrics.min_left = inf;
metrics.min_right = inf;
metrics.min_up = inf;
metrics.min_down = inf;
metrics.object_count = numel(targets);

activeTargetIds = [];
for k = 1:numel(targets)
    target = targets(k);
    isCriticalClose = target.min_distance <= cfg.DIS_STOP;
    isBoundaryThreat = target.borders.top >= cfg.GROUND_BORDER || target.borders.bottom <= cfg.CELLING_BORDER;
    if ~isCriticalClose && ~isBoundaryThreat && target.pixels_number < cfg.MIN_PIXEL_NUMBER
        continue;
    end

    activeTargetIds(end + 1) = target.component_id; %#ok<AGROW>
    dist = target.min_distance;
    metrics.min_global = min(metrics.min_global, dist);

    inDroneZone = ...
        target.borders.right >= (min(cfg.droneZoneCols) - 1) && ...
        target.borders.left <= (max(cfg.droneZoneCols) - 1) && ...
        target.borders.bottom >= (min(cfg.droneZoneRows) - 1) && ...
        target.borders.top <= (max(cfg.droneZoneRows) - 1);
    if inDroneZone
        metrics.min_front = min(metrics.min_front, dist);
    end

    if target.position.y < cfg.middlePosY
        metrics.min_left = min(metrics.min_left, dist);
    else
        metrics.min_right = min(metrics.min_right, dist);
    end

    if target.borders.bottom <= cfg.CELLING_BORDER
        metrics.min_up = min(metrics.min_up, dist);
    end
    if target.borders.top >= cfg.GROUND_BORDER
        metrics.min_down = min(metrics.min_down, dist);
    end
end

explanation.activeTargetIds = activeTargetIds;
explanation.decisionName = 'Clear / no active avoidance';
explanation.coreReason = 'No active target after connected-domain filtering.';
explanation.vxDesc = 'VEL_SCALE_MEDIUM';
explanation.vzDesc = 'VEL_STOP';
explanation.turnDesc = 'TURN_NOT';
explanation.turnPath = 'turn channel idle';
explanation.note = 'Yaw turning is based on global left/right minima, not only cells inside CARE_ZONE or DRONE_ZONE.';

if metrics.object_count == 0
    command.turnRaw = cfg.TURN_NOT;
    [command.turn, history] = handleExceptionCommand(command.turnRaw, history, cfg);
    explanation.turnDesc = describeTurn(command.turn);
    return;
end

if metrics.min_global <= cfg.DIS_FEAR
    command.vx = cfg.VEL_FEAR;
    command.vz = cfg.VEL_STOP;
    command.turnRaw = cfg.TURN_NOT;
    [command.turn, history] = handleExceptionCommand(command.turnRaw, history, cfg);
    explanation.decisionName = 'Fear override';
    explanation.coreReason = sprintf('min_global=%smm <= DIS_FEAR=%dmm, reverse and cancel turn.', ...
        formatDistance(metrics.min_global), cfg.DIS_FEAR);
    explanation.vxDesc = 'VEL_FEAR';
    explanation.vzDesc = 'VEL_STOP';
    explanation.turnDesc = 'TURN_NOT';
    explanation.turnPath = 'fear override dominates all channels';
    return;
end

vzActive = false;
if metrics.min_down < cfg.DIS_GROUND_MIN
    command.vz = cfg.VEL_UP;
    vzActive = true;
    explanation.vzDesc = 'VEL_UP';
end
if metrics.min_up < cfg.DIS_CEILING_MIN
    command.vz = cfg.VEL_DOWN;
    vzActive = true;
    explanation.vzDesc = 'VEL_DOWN';
end

if metrics.min_front < cfg.DIS_REACT
    if metrics.min_front <= cfg.DIS_STOP
        command.vx = cfg.VEL_STOP;
        explanation.vxDesc = 'VEL_STOP';
    elseif metrics.min_front <= cfg.DIS_SLOW
        command.vx = ((metrics.min_front - cfg.DIS_STOP) / (cfg.DIS_SLOW - cfg.DIS_STOP)) * cfg.VEL_SCALE_SLOW;
        explanation.vxDesc = 'scaled in stop-slow band';
    else
        command.vx = ((metrics.min_front - cfg.DIS_SLOW) / (cfg.DIS_REACT - cfg.DIS_SLOW)) * ...
            (cfg.VEL_SCALE_MEDIUM - cfg.VEL_SCALE_SLOW) + cfg.VEL_SCALE_SLOW;
        explanation.vxDesc = 'scaled in slow-react band';
    end
end

if metrics.min_left < cfg.DIS_REACT || metrics.min_right < cfg.DIS_REACT
    if metrics.min_left < metrics.min_right
        if metrics.min_left <= cfg.DIS_STOP
            command.turnRaw = cfg.TURN_MAX;
        elseif metrics.min_left <= cfg.DIS_SLOW
            command.turnRaw = cfg.TURN_MAX * 0.8;
        else
            command.turnRaw = cfg.TURN_SLOW;
        end
        explanation.turnPath = sprintf('left side nearer (%smm < %smm), so turn right.', ...
            formatDistance(metrics.min_left), formatDistance(metrics.min_right));
    else
        if metrics.min_right <= cfg.DIS_STOP
            command.turnRaw = -cfg.TURN_MAX;
        elseif metrics.min_right <= cfg.DIS_SLOW
            command.turnRaw = -cfg.TURN_MAX * 0.8;
        else
            command.turnRaw = -cfg.TURN_SLOW;
        end
        explanation.turnPath = sprintf('right side nearer (%smm <= %smm), so turn left.', ...
            formatDistance(metrics.min_right), formatDistance(metrics.min_left));
    end
end

if vzActive
    command.vx = min(command.vx, cfg.VEL_SCALE_SLOW);
    command.turnRaw = max(min(command.turnRaw, cfg.TURN_SLOW), -cfg.TURN_SLOW);
    explanation.decisionName = 'Vertical avoidance active';
    explanation.coreReason = 'Height avoidance is active, so vx and turn are clamped before exception handling.';
else
    explanation.decisionName = 'Horizontal avoidance';
    explanation.coreReason = 'Front speed uses DRONE_ZONE, yaw uses global left/right minima from all active connected domains.';
end

 [command.turn, history] = handleExceptionCommand(command.turnRaw, history, cfg);
explanation.turnDesc = describeTurn(command.turn);
end

function [refined, history] = handleExceptionCommand(currentTurn, history, cfg)
rightCount = sum(history.values < -cfg.TURN_MAX + cfg.EPSILON);
leftCount = sum(history.values > cfg.TURN_MAX - cfg.EPSILON);
refined = currentTurn;
if (currentTurn < -cfg.TURN_MAX + cfg.EPSILON || currentTurn > cfg.TURN_MAX - cfg.EPSILON) && ...
        (rightCount + leftCount) >= cfg.MAX_TURN_RATIO * numel(history.values)
    if rightCount > leftCount
        refined = -cfg.TURN_FAST;
    else
        refined = cfg.TURN_FAST;
    end
end
history.values(history.index) = currentTurn;
history.index = mod(history.index, numel(history.values)) + 1;
end

function out = describeTurn(turnValue)
if abs(turnValue - 3.0) < 1e-6
    out = 'TURN_FAST';
elseif abs(turnValue + 3.0) < 1e-6
    out = '-TURN_FAST';
elseif abs(turnValue - 1.0) < 1e-6
    out = 'TURN_MAX';
elseif abs(turnValue + 1.0) < 1e-6
    out = '-TURN_MAX';
elseif abs(turnValue - 0.8) < 1e-6
    out = 'TURN_MAX * 0.8';
elseif abs(turnValue + 0.8) < 1e-6
    out = '-TURN_MAX * 0.8';
elseif abs(turnValue - 0.7) < 1e-6
    out = 'TURN_SLOW';
elseif abs(turnValue + 0.7) < 1e-6
    out = '-TURN_SLOW';
else
    out = 'TURN_NOT';
end
end

function setup3DAxes(ax, cfg)
hold(ax, 'on');
grid(ax, 'on');
box(ax, 'on');
view(ax, 42, 24);
xlabel(ax, 'x lateral (m)');
ylabel(ax, 'y forward (m)');
zlabel(ax, 'z height (m)');
xlim(ax, [cfg.xMin3D cfg.xMax3D]);
ylim(ax, [-0.5 cfg.yMax3D]);
zlim(ax, [0 cfg.channelHeight + 0.2]);
daspect(ax, [1 1 1]);
title(ax, '3D corridor, ToF rays, robot body');

% Right wall: (1,0)->(1,10)->(11,10)->(11,0)
% Left wall:  (-1,0)->(-1,12)->(12,12)->(12,0)

% Segment 1 floor: x in [-1, 1], y in [0, 10]
patch(ax, [-1 1 1 -1], [0 0 10 10], [0 0 0 0], [0.92 0.92 0.92], 'FaceAlpha', 0.55, 'EdgeColor', 'none');
plot3(ax, [-1 -1], [0 10], [0 0], 'k-', 'LineWidth', 1.5);
plot3(ax, [1 1], [0 10], [0 0], 'k-', 'LineWidth', 1.5);

% Segment 2 floor: x in [1, 11], y in [10, 12]
patch(ax, [1 11 11 1], [10 10 12 12], [0 0 0 0], [0.92 0.92 0.92], 'FaceAlpha', 0.55, 'EdgeColor', 'none');
plot3(ax, [1 11], [10 10], [0 0], 'k-', 'LineWidth', 1.5);
plot3(ax, [-1 12], [12 12], [0 0], 'k-', 'LineWidth', 1.5);

% Segment 3 floor: x in [11, 12], y in [0, 12]
patch(ax, [11 12 12 11], [0 0 12 12], [0 0 0 0], [0.92 0.92 0.92], 'FaceAlpha', 0.55, 'EdgeColor', 'none');
plot3(ax, [11 11], [0 10], [0 0], 'k-', 'LineWidth', 1.5);
plot3(ax, [12 12], [0 12], [0 0], 'k-', 'LineWidth', 1.5);

% Turn markers
plot3(ax, [-1 1], [10 10], [0.02 0.02], 'g-', 'LineWidth', 2.0);
plot3(ax, [11 11], [10 12], [0.02 0.02], 'g-', 'LineWidth', 2.0);
end

function drawStaticEnvironment(ax, O, cfg)
for i = 1:height(O)
    boxColor = [0.75 0.18 0.14];
    alphaValue = 0.70;
    if strcmp(string(O.name(i)), 'ceiling')
        alphaValue = 0.0;
    elseif contains(string(O.name(i)), 'wall')
        boxColor = [0.25 0.28 0.34];
        alphaValue = 0.18;
    end
    drawBox(ax, O.xmin(i), O.xmax(i), O.ymin(i), O.ymax(i), O.zmin(i), O.zmax(i), boxColor, alphaValue);
end
end

function setupTopAxes(ax, T, O, cfg)
hold(ax, 'on');
grid(ax, 'on');
box(ax, 'on');
plot(ax, T.y, T.x, 'b-', 'LineWidth', 1.1);
for i = 1:height(O)
    rectangle(ax, 'Position', [O.ymin(i), O.xmin(i), O.ymax(i) - O.ymin(i), O.xmax(i) - O.xmin(i)], ...
        'FaceColor', [1.0 0.78 0.74], 'EdgeColor', [0.55 0.1 0.1]);
end
xlabel(ax, 'y (m)');
ylabel(ax, 'x (m)');
title(ax, 'Top view path');
ylim(ax, [-1.5, 12.5]);
xlim(ax, [-0.5, 12.5]);
end

function setupTofAxes(ax)
hold(ax, 'on');
box(ax, 'on');
axis(ax, [0.5 8.5 0.1 8.5]);
axis(ax, 'ij');
axis(ax, 'equal');
xticks(ax, 1:8);
yticks(ax, 1:8);
xlabel(ax, 'column');
ylabel(ax, 'row');
title(ax, 'ToF 8x8 range bands, connected domains, care/drone zones');
end

function rect = zoneRectangle(rows1Based, cols1Based)
rect = [min(cols1Based) - 0.5, min(rows1Based) - 0.5, numel(cols1Based), numel(rows1Based)];
end

function drawBox(ax, xmin, xmax, ymin, ymax, zmin, zmax, color, alphaValue)
verts = [xmin ymin zmin;
         xmax ymin zmin;
         xmax ymax zmin;
         xmin ymax zmin;
         xmin ymin zmax;
         xmax ymin zmax;
         xmax ymax zmax;
         xmin ymax zmax];
faces = [1 2 3 4;
         5 6 7 8;
         1 2 6 5;
         2 3 7 6;
         3 4 8 7;
         4 1 5 8];
patch(ax, 'Vertices', verts, 'Faces', faces, 'FaceColor', color, ...
    'FaceAlpha', alphaValue, 'EdgeColor', [0.2 0.2 0.2], 'LineWidth', 0.8);
end

function renderFrame(fig, requestedFrame)
state = guidata(fig);
idx = max(state.cfg.startFrame, min(state.cfg.endFrame, round(requestedFrame)));
state.currentFrame = idx;
guidata(fig, state);

T = state.T;
cfg = state.cfg;
ui = state.ui;
frame = state.analysis(idx);

set(ui.pathLine, 'XData', T.x(1:idx), 'YData', T.y(1:idx), 'ZData', T.z(1:idx));
set(ui.topMarker, 'XData', T.y(idx), 'YData', T.x(idx));

[sx, sy, sz] = sphere(24);
robotColor = [0.10 0.55 0.95];
if T.collision(idx) > 0
    robotColor = [0.90 0.05 0.05];
end
set(ui.robotSphere, 'XData', cfg.robotRadius * sx + T.x(idx), ...
    'YData', cfg.robotRadius * sy + T.y(idx), ...
    'ZData', cfg.robotRadius * sz + T.z(idx), ...
    'FaceColor', robotColor);
set(ui.robotPoint, 'XData', T.x(idx), 'YData', T.y(idx), 'ZData', T.z(idx), ...
    'MarkerFaceColor', robotColor);
set(ui.headingLine, ...
    'XData', [T.x(idx), T.x(idx) + 0.6 * sin(T.psi(idx))], ...
    'YData', [T.y(idx), T.y(idx) + 0.6 * cos(T.psi(idx))], ...
    'ZData', [T.z(idx), T.z(idx)]);

rayIndex = 0;
for r = 1:8
    for c = 1:8
        rayIndex = rayIndex + 1;
        d = frame.tofMm(r, c) / 1000.0;
        horiz = (((c - 0.5) / 8) - 0.5) * cfg.fovH;
        vert = (0.5 - ((r - 0.5) / 8)) * cfg.fovV;
        yaw = T.psi(idx) + horiz;
        dx = sin(yaw) * cos(vert);
        dy = cos(yaw) * cos(vert);
        dz = sin(vert);
        set(ui.rayHandles(rayIndex), ...
            'XData', [T.x(idx), T.x(idx) + d * dx], ...
            'YData', [T.y(idx), T.y(idx) + d * dy], ...
            'ZData', [T.z(idx), T.z(idx) + d * dz]);
    end
end

for r = 1:8
    for c = 1:8
        dMm = frame.tofMm(r, c);
        [cellColor, bandColor, textColor] = cellAppearance(dMm, cfg);
        set(ui.tofRects(r, c), 'FaceColor', cellColor);
        set(ui.bandRects(r, c), 'FaceColor', bandColor);
        set(ui.distText(r, c), 'String', sprintf('%d', dMm), 'Color', textColor);
        compId = frame.labels(r, c);
        if compId > 0
            set(ui.compTag(r, c), 'String', sprintf('C%d', compId));
        else
            set(ui.compTag(r, c), 'String', '');
        end
    end
end

if ~isempty(ui.componentBoxes)
    delete(ui.componentBoxes(ishandle(ui.componentBoxes)));
end
if ~isempty(ui.componentLabels)
    delete(ui.componentLabels(ishandle(ui.componentLabels)));
end
ui.componentBoxes = gobjects(0);
ui.componentLabels = gobjects(0);

for k = 1:numel(frame.components)
    rows = frame.components(k).rows;
    cols = frame.components(k).cols;
    boxColor = [0.08 0.08 0.08];
    if ismember(frame.components(k).id, frame.explanation.activeTargetIds)
        boxColor = [0.02 0.02 0.02];
    end
    ui.componentBoxes(end + 1) = rectangle(ui.axTof, ...
        'Position', [min(cols) - 0.5, min(rows) - 0.5, max(cols) - min(cols) + 1, max(rows) - min(rows) + 1], ...
        'EdgeColor', boxColor, 'LineStyle', '-', 'LineWidth', 1.9); 
    ui.componentLabels(end + 1) = text(ui.axTof, mean(cols), min(rows) - 0.15, sprintf('C%d', frame.components(k).id), ...
        'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', ...
        'FontName', 'Consolas', 'FontSize', 9.5, 'FontWeight', 'bold', ...
        'BackgroundColor', [1 1 1], 'Margin', 1.0, 'Color', boxColor); 
end

componentLines = strings(0, 1);
for k = 1:numel(frame.components)
    componentLines(end + 1) = sprintf('C%d cells=%d min=%dmm', ...
        frame.components(k).id, frame.components(k).pixelCount, round(frame.components(k).minDistance)); %#ok<AGROW>
end
if isempty(componentLines)
    componentLines = "no connected domain";
end
set(ui.tofLegendText, 'String', strjoin(cellstr(componentLines), newline));

decisionText = buildDecisionText(frame, T, idx, cfg);
set(ui.infoText, 'String', decisionText);

if T.collision(idx) > 0
    msg = sprintf('frame %d/%d | FAIL at t=%.2fs | clearance=%.3fm', idx, state.frameCount, T.time(idx), T.clearance(idx));
elseif idx == state.frameCount || T.y(idx) <= 0.0
    msg = sprintf('frame %d/%d | SUCCESS window | t=%.2fs | x=%.2fm y=%.2fm', idx, state.frameCount, T.time(idx), T.x(idx), T.y(idx));
else
    msg = sprintf('frame %d/%d | t=%.2fs | y=%.2fm | sim turn=%.2f | algo turn=%.2f', ...
        idx, state.frameCount, T.time(idx), T.y(idx), T.cmd_turn(idx), frame.command.turn);
end
set(ui.statusText, 'String', msg);

if ~isempty(ui.slider) && isgraphics(ui.slider)
    set(ui.slider, 'Value', idx);
end
if ~isempty(ui.frameEdit) && isgraphics(ui.frameEdit)
    set(ui.frameEdit, 'String', num2str(idx));
end
if ~isempty(ui.frameInfo) && isgraphics(ui.frameInfo)
    set(ui.frameInfo, 'String', sprintf('%d / %d', idx, state.frameCount));
end

state.ui = ui;
guidata(fig, state);
drawnow;
end

function txt = buildDecisionText(frame, T, idx, cfg)
compLines = strings(0, 1);
for k = 1:numel(frame.components)
    comp = frame.components(k);
    compLines(end + 1) = sprintf('  C%d: pixels=%d  min=%dmm  bounds=[%d,%d]-[%d,%d]', ...
        comp.id, comp.pixelCount, round(comp.minDistance), comp.top, comp.left, comp.bottom, comp.right); %#ok<AGROW>
end
if isempty(compLines)
    compLines = "  (none)";
end

txt = sprintf([ ...
    'Connected domains (%d):\n%s\n\n' ...
    'Commands:\n' ...
    '  vx   = %.3f\n' ...
    '  vz   = %.3f\n' ...
    '  turn = %.3f'], ...
    numel(frame.components), strjoin(cellstr(compLines), newline), ...
    T.cmd_vx(idx), T.cmd_vz(idx), T.cmd_turn(idx));
end

function [cellColor, bandColor, textColor] = cellAppearance(distanceMm, cfg)
if distanceMm <= cfg.DIS_FEAR
    cellColor = [0.98 0.89 0.89];
    bandColor = [0.84 0.18 0.15];
    textColor = [0.25 0.05 0.05];
elseif distanceMm <= cfg.DIS_STOP
    cellColor = [0.99 0.93 0.88];
    bandColor = [0.95 0.45 0.18];
    textColor = [0.23 0.10 0.05];
elseif distanceMm <= cfg.DIS_SLOW
    cellColor = [1.00 0.97 0.87];
    bandColor = [0.99 0.77 0.28];
    textColor = [0.22 0.18 0.06];
elseif distanceMm <= cfg.DIS_REACT
    cellColor = [0.95 0.98 0.88];
    bandColor = [0.87 0.91 0.54];
    textColor = [0.13 0.18 0.08];
else
    cellColor = [0.92 0.95 0.98];
    bandColor = [0.62 0.78 0.92];
    textColor = [0.08 0.14 0.21];
end
end

function value = timerPeriod(cfg)
value = max(cfg.baseDt * cfg.frameStep / max(cfg.playbackSpeed, 0.05), 0.02);
end

function step = sliderStep(frameCount)
if frameCount <= 1
    step = [1 1];
else
    step = [1 / max(frameCount - 1, 1), min(0.10, 10 / max(frameCount - 1, 1))];
end
end

function onSliderChanged(src, fig)
renderFrame(fig, get(src, 'Value'));
end

function stepFrame(fig, deltaSign)
state = guidata(fig);
delta = deltaSign * state.cfg.frameStep;
renderFrame(fig, state.currentFrame + delta);
end

function togglePlayback(src, fig)
state = guidata(fig);
if isempty(state.timer) || ~isvalid(state.timer)
    return;
end
state.isPlaying = logical(get(src, 'Value'));
if state.isPlaying
    set(src, 'String', 'Pause');
    start(state.timer);
else
    set(src, 'String', 'Play');
    stop(state.timer);
end
guidata(fig, state);
end

function timerAdvance(fig)
if ~ishandle(fig)
    return;
end
state = guidata(fig);
nextFrame = state.currentFrame + state.cfg.frameStep;
if nextFrame > state.cfg.endFrame
    nextFrame = state.cfg.endFrame;
    state.isPlaying = false;
    if ~isempty(state.timer) && isvalid(state.timer)
        stop(state.timer);
    end
    if ~isempty(state.ui.btnPlay) && isgraphics(state.ui.btnPlay)
        set(state.ui.btnPlay, 'Value', 0, 'String', 'Play');
    end
    guidata(fig, state);
end
renderFrame(fig, nextFrame);
end

function jumpToFrame(fig)
state = guidata(fig);
if isempty(state.ui.frameEdit) || ~isgraphics(state.ui.frameEdit)
    return;
end
requested = str2double(get(state.ui.frameEdit, 'String'));
if ~isfinite(requested)
    requested = state.currentFrame;
end
renderFrame(fig, round(requested));
end

function updatePlaybackRate(fig)
state = guidata(fig);
if isempty(state.ui.speedEdit) || ~isgraphics(state.ui.speedEdit)
    return;
end
value = str2double(get(state.ui.speedEdit, 'String'));
if ~isfinite(value) || value <= 0
    value = state.cfg.playbackSpeed;
else
    state.cfg.playbackSpeed = value;
end
set(state.ui.speedEdit, 'String', num2str(state.cfg.playbackSpeed, '%.2f'));
if ~isempty(state.timer) && isvalid(state.timer)
    wasRunning = strcmp(state.timer.Running, 'on');
    if wasRunning
        stop(state.timer);
    end
    state.timer.Period = timerPeriod(state.cfg);
    if wasRunning
        start(state.timer);
    end
end
guidata(fig, state);
end

function updateFrameStep(fig)
state = guidata(fig);
if isempty(state.ui.stepEdit) || ~isgraphics(state.ui.stepEdit)
    return;
end
value = round(str2double(get(state.ui.stepEdit, 'String')));
if ~isfinite(value) || value < 1
    value = state.cfg.frameStep;
else
    state.cfg.frameStep = value;
end
set(state.ui.stepEdit, 'String', num2str(state.cfg.frameStep));
if ~isempty(state.ui.slider) && isgraphics(state.ui.slider)
    set(state.ui.slider, 'SliderStep', sliderStep(state.frameCount));
end
if ~isempty(state.timer) && isvalid(state.timer)
    wasRunning = strcmp(state.timer.Running, 'on');
    if wasRunning
        stop(state.timer);
    end
    state.timer.Period = timerPeriod(state.cfg);
    if wasRunning
        start(state.timer);
    end
end
guidata(fig, state);
end

function out = formatDistance(value)
if ~isfinite(value) || value >= 65535
    out = 'INF';
else
    out = sprintf('%d', round(value));
end
end

function out = yesNo(flag)
if flag
    out = 'match';
else
    out = 'DIFF ';
end
end

function closeViewer(fig)
if ~ishandle(fig)
    return;
end
state = guidata(fig);
if isstruct(state) && isfield(state, 'timer') && ~isempty(state.timer) && isvalid(state.timer)
    try
        stop(state.timer);
    catch
    end
    delete(state.timer);
end
delete(fig);
end
