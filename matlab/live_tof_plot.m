% live_tof_plot.m
%
% Live-plots the MicroMouse's 3 ToF sensor distances (front/right/left)
% streamed from the ESP32 over serial, for sensor bring-up / tuning.
%
% Firmware side: main.cpp calls telemetry_send() every loop, which prints
% one line per reading over Serial:
%   DATA,<millis>,<front_mm>,<right_mm>,<left_mm>
% This script filters for lines starting with "DATA," and ignores every
% other line on the port (e.g. the [SENSOR]/[MAIN] debug logs), so both
% can share the same USB serial connection with no extra setup.
%
% Requires: MATLAB R2019b+ (uses the built-in serialport object, no
% Instrument Control Toolbox needed).
%
% Usage:
%   1. Set PORT_NAME below (run serialportlist("available") if unsure).
%   2. Run this script.
%   3. Close the plot window to stop.

clear; clc;

%% --- Config ---
PORT_NAME  = "/dev/ttyUSB0";  % Linux: /dev/ttyUSB0 or /dev/ttyACM0
                               % Windows: "COM3" etc.
BAUD_RATE  = 115200;
WINDOW_SEC = 15;               % how many seconds of history to show

%% --- Connect ---
fprintf("Connecting to %s @ %d...\n", PORT_NAME, BAUD_RATE);
port = serialport(PORT_NAME, BAUD_RATE);
configureTerminator(port, "LF");
flush(port);

%% --- Plot setup ---
% Zoomed to the 0-10cm near-field range (fine-tuning sensor placement
% happens at close range, where mm-scale noise matters most).
fig = figure('Name', 'Live ToF Sensor Readings', 'NumberTitle', 'off');
ax = axes(fig);
hold(ax, 'on');
grid(ax, 'on');
ax.YMinorGrid = 'on';
xlabel(ax, 'Time (s)');
ylabel(ax, 'Distance (cm)');
title(ax, 'Front / Right / Left ToF Distance (0-10cm)');
ylim(ax, [0 10]);
yticks(ax, 0:0.5:10);  % 0.5cm gridlines for a detailed near-field view

lineFront = animatedline(ax, 'Color', [0.85 0.10 0.10], 'DisplayName', 'Front', ...
    'Marker', '.', 'MarkerSize', 10);
lineRight = animatedline(ax, 'Color', [0.10 0.55 0.85], 'DisplayName', 'Right', ...
    'Marker', '.', 'MarkerSize', 10);
lineLeft  = animatedline(ax, 'Color', [0.10 0.70 0.30], 'DisplayName', 'Left', ...
    'Marker', '.', 'MarkerSize', 10);
legend(ax, 'show', 'Location', 'northeastoutside');

cleanupObj = onCleanup(@() cleanupSerial(port)); %#ok<NASGU>

%% --- Read loop (stops when the figure is closed) ---
t0 = tic;
while isvalid(fig) && isgraphics(fig)
    if port.NumBytesAvailable == 0
        drawnow limitrate;
        pause(0.01);
        continue;
    end

    line = readline(port);
    if ~startsWith(line, "DATA,")
        continue;  % ignore [SENSOR]/[MAIN] debug logs
    end

    fields = sscanf(line, "DATA,%f,%f,%f,%f");
    if numel(fields) ~= 4
        continue;  % malformed/partial line, skip it
    end

    t = toc(t0);
    front_cm = fields(2) / 10;
    right_cm = fields(3) / 10;
    left_cm  = fields(4) / 10;

    addpoints(lineFront, t, front_cm);
    addpoints(lineRight, t, right_cm);
    addpoints(lineLeft,  t, left_cm);

    xlim(ax, [max(0, t - WINDOW_SEC), max(WINDOW_SEC, t)]);
    drawnow limitrate;
end

function cleanupSerial(port)
    fprintf("Closing serial port.\n");
    delete(port);
end
