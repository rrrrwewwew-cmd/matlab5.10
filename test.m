clear; clc; close all;

scriptDir = fileparts(mfilename('fullpath'));
if isempty(scriptDir)
    scriptDir = pwd;
end

playbackSpeed = 1.0;
startFrame = 1;
endFrame = inf;
frameStep = 1;
tofMaxRange = 2.5;

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
startFrame = max(1, startFrame);
endFrame = min(frameCount, endFrame);
frameStep = max(1, frameStep);
if startFrame > endFrame
    error('startFrame must be less than or equal to endFrame.');
end

baseDt = median(diff(T.time));
if isempty(baseDt) || ~isfinite(baseDt) || baseDt <= 0
    baseDt = 0.02;
end

channelLength = 10.0;
channelHeight = 1.8;
robotRadius = 0.175;
fovH = deg2rad(45.0);
fovV = deg2rad(45.0);

fig = figure('Name', 'ToF Corridor Avoidance Simulation', 'Color', 'w');
set(fig, 'Position', [80 60 1500 920]);

ax3d = axes(fig, 'Position', [0.05 0.10 0.56 0.82]);
hold(ax3d, 'on');
grid(ax3d, 'on');
box(ax3d, 'on');
view(ax3d, 42, 24);
xlabel(ax3d, 'x lateral (m)');
ylabel(ax3d, 'y forward (m)');
zlabel(ax3d, 'z height (m)');
xlim(ax3d, [-1.25 1.25]);
ylim(ax3d, [-0.2 channelLength + 0.2]);
zlim(ax3d, [0 channelHeight + 0.2]);
daspect(ax3d, [1 1 1]);
title(ax3d, '3D corridor, ToF rays, robot safety sphere');

patch(ax3d, [-1 1 1 -1], [0 0 channelLength channelLength], [0 0 0 0], [0.92 0.92 0.92], ...
    'FaceAlpha', 0.55, 'EdgeColor', 'none');
plot3(ax3d, [-1 -1], [0 channelLength], [0 0], 'k-', 'LineWidth', 1.5);
plot3(ax3d, [1 1], [0 channelLength], [0 0], 'k-', 'LineWidth', 1.5);

for i = 1:height(O)
    color = [0.75 0.18 0.14];
    alphaValue = 0.70;
    if contains(string(O.name(i)), 'wall')
        color = [0.25 0.28 0.34];
        alphaValue = 0.18;
    end
    drawBox(ax3d, O.xmin(i), O.xmax(i), O.ymin(i), O.ymax(i), O.zmin(i), O.zmax(i), color, alphaValue);
end

pathLine = plot3(ax3d, T.x(1), T.y(1), T.z(1), 'b-', 'LineWidth', 1.8);
plot3(ax3d, [-1 1], [channelLength channelLength], [0.02 0.02], 'g-', 'LineWidth', 2.0);

axTop = axes(fig, 'Position', [0.68 0.68 0.27 0.17]);
hold(axTop, 'on');
grid(axTop, 'on');
box(axTop, 'on');
plot(axTop, T.y, T.x, 'b-', 'LineWidth', 1.3);
for i = 1:height(O)
    rectangle(axTop, 'Position', [O.ymin(i), O.xmin(i), O.ymax(i) - O.ymin(i), O.xmax(i) - O.xmin(i)], ...
        'FaceColor', [1.0 0.78 0.74], 'EdgeColor', [0.55 0.1 0.1]);
end
xlabel(axTop, 'y (m)');
ylabel(axTop, 'x (m)');
title(axTop, 'Top view path');
xlim(axTop, [0 channelLength]);
ylim(axTop, [-1.1 1.1]);

axCmd = axes(fig, 'Position', [0.68 0.46 0.27 0.17]);
hold(axCmd, 'on');
grid(axCmd, 'on');
box(axCmd, 'on');
plot(axCmd, T.time, T.cmd_turn, 'Color', [0.10 0.45 0.85], 'LineWidth', 1.1);
plot(axCmd, T.time, T.cmd_vx, 'Color', [0.15 0.65 0.25], 'LineWidth', 1.1);
plot(axCmd, T.time, T.clearance, 'Color', [0.85 0.35 0.10], 'LineWidth', 1.1);
yline(axCmd, 0, 'k:');
legend(axCmd, {'turn cmd', 'vx cmd', 'clearance'}, 'Location', 'best');
xlabel(axCmd, 'time (s)');
title(axCmd, 'Decision and safety margin');

axTof = axes(fig, 'Position', [0.64 0.08 0.33 0.30]);
tofImage = imagesc(axTof, zeros(8, 8), [0 tofMaxRange]);
axis(axTof, 'image');
set(axTof, 'YDir', 'normal');
colormap(axTof, tofDistanceMap());
colorbar(axTof);
xticks(axTof, 1:8);
yticks(axTof, 1:8);
xlabel(axTof, 'column');
ylabel(axTof, 'row');
title(axTof, 'ToF 8x8 range (m)');
hold(axTof, 'on');

tofText = gobjects(8, 8);
for r = 1:8
    for c = 1:8
        tofText(r, c) = text(axTof, c, r, '0.00', ...
            'HorizontalAlignment', 'center', ...
            'VerticalAlignment', 'middle', ...
            'FontName', 'Consolas', ...
            'FontSize', 10, ...
            'FontWeight', 'bold', ...
            'Color', [0.07 0.12 0.20]);
    end
end

robotHandles = gobjects(0);
rayHandles = gobjects(0);
headingHandle = gobjects(0);
statusText = text(ax3d, -1.18, -0.05, channelHeight + 0.12, '', 'FontWeight', 'bold');

for idx = startFrame:frameStep:endFrame
    if ~isempty(robotHandles)
        delete(robotHandles(ishandle(robotHandles)));
    end
    if ~isempty(rayHandles)
        delete(rayHandles(ishandle(rayHandles)));
    end
    if ~isempty(headingHandle) && all(ishandle(headingHandle))
        delete(headingHandle);
    end

    x = T.x(idx);
    y = T.y(idx);
    z = T.z(idx);
    psi = T.psi(idx);
    set(pathLine, 'XData', T.x(1:idx), 'YData', T.y(1:idx), 'ZData', T.z(1:idx));

    tofFrame = zeros(8, 8);
    for r = 1:8
        for c = 1:8
            name = sprintf('tof%d%d', r - 1, c - 1);
            d = T.(name)(idx) / 1000.0;
            tofFrame(r, c) = min(d, tofMaxRange);
            set(tofText(r, c), 'String', sprintf('%.2f', tofFrame(r, c)));
        end
    end
    set(tofImage, 'CData', tofFrame);

    [sx, sy, sz] = sphere(24);
    robotColor = [0.10 0.55 0.95];
    if T.collision(idx) > 0
        robotColor = [0.90 0.05 0.05];
    end
    robotHandles(end + 1) = surf(ax3d, robotRadius * sx + x, robotRadius * sy + y, robotRadius * sz + z, ...
        'FaceColor', robotColor, 'FaceAlpha', 0.35, 'EdgeColor', 'none');
    robotHandles(end + 1) = plot3(ax3d, x, y, z, 'o', 'MarkerSize', 6, ...
        'MarkerFaceColor', robotColor, 'MarkerEdgeColor', 'k');

    headingHandle = plot3(ax3d, [x x + 0.6 * sin(psi)], [y y + 0.6 * cos(psi)], [z z], ...
        'Color', [0.05 0.05 0.05], 'LineWidth', 2.0);

    rayList = gobjects(0);
    for r = 1:8
        for c = 1:8
            d = tofFrame(r, c);
            horiz = (((c - 0.5) / 8) - 0.5) * fovH;
            vert = (0.5 - ((r - 0.5) / 8)) * fovV;
            yaw = psi + horiz;
            dx = sin(yaw) * cos(vert);
            dy = cos(yaw) * cos(vert);
            dz = sin(vert);
            rayList(end + 1) = plot3(ax3d, [x x + d * dx], [y y + d * dy], [z z + d * dz], ...
                'Color', [0.52 0.70 0.86], 'LineWidth', 0.7);
        end
    end
    rayHandles = rayList;

    if T.collision(idx) > 0
        msg = sprintf('frame %d/%d | FAIL at t = %.2fs, clearance = %.3fm', idx, frameCount, T.time(idx), T.clearance(idx));
    elseif idx == frameCount || T.y(idx) >= channelLength
        msg = sprintf('frame %d/%d | SUCCESS, t = %.2fs, y = %.2fm', idx, frameCount, T.time(idx), T.y(idx));
    else
        msg = sprintf('frame %d/%d | t = %.2fs, y = %.2fm, cmd turn = %.2f', idx, frameCount, T.time(idx), T.y(idx), T.cmd_turn(idx));
    end
    set(statusText, 'String', msg);
    drawnow;
    pause(baseDt * frameStep / max(playbackSpeed, 0.05));
end

function h = drawBox(ax, xmin, xmax, ymin, ymax, zmin, zmax, color, alphaValue)
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
h = patch(ax, 'Vertices', verts, 'Faces', faces, 'FaceColor', color, ...
    'FaceAlpha', alphaValue, 'EdgeColor', [0.2 0.2 0.2], 'LineWidth', 0.8);
end

function cmap = tofDistanceMap()
base = [1.00 1.00 1.00;
        0.90 0.94 0.98;
        0.75 0.84 0.93;
        0.56 0.70 0.86;
        0.32 0.53 0.76;
        0.10 0.29 0.55];
x = linspace(0, 1, size(base, 1));
xq = linspace(0, 1, 256);
cmap = interp1(x, base, xq, 'pchip');
end
