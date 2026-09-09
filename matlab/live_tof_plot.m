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
fig = figure('Name', 'Live ToF Sensor Readings', 'NumberTitle', 'off');
ax = axes(fig);
hold(ax, 'on');
grid(ax, 'on');
xlabel(ax, 'Time (s)');
ylabel(ax, 'Distance (mm)');
title(ax, 'Front / Right / Left ToF Distance');
ylim(ax, [0 2000]);  % matches SENSOR_MAX_RANGE_MM in sensor.h

lineFront = animatedline(ax, 'Color', [0.85 0.10 0.10], 'DisplayName', 'Front');
lineRight = animatedline(ax, 'Color', [0.10 0.55 0.85], 'DisplayName', 'Right');
lineLeft  = animatedline(ax, 'Color', [0.10 0.70 0.30], 'DisplayName', 'Left');
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
    front_mm = fields(2);
    right_mm = fields(3);
    left_mm  = fields(4);

    addpoints(lineFront, t, front_mm);
    addpoints(lineRight, t, right_mm);
    addpoints(lineLeft,  t, left_mm);

    xlim(ax, [max(0, t - WINDOW_SEC), max(WINDOW_SEC, t)]);
    drawnow limitrate;
end

function cleanupSerial(port)
    fprintf("Closing serial port.\n");
    delete(port);
end
