%% Problem 2 (AV HW#1) — Accelerometer Analysis Template
% This script:
% 1) Loads your acc_data.csv
% 2) Auto-detects time column
% 3) Auto-detects accel columns (x,y,z) if possible
% 4) Identifies "vertical" axis via gravity offset (~9.81 m/s^2)
% 5) Builds lateral (one of remaining), longitudinal (the other)
% 6) Plots raw + filtered accel vs time
% 7) Computes stats (mean/std/RMS/peaks) and time-scales (PSD dominant freq)
% 8) Splits low-frequency maneuvers vs high-frequency vibration
% 9) Exports figures + summary table

clear; clc; close all;

%% ---- USER SETTINGS ----
csvFile = "acc_data.csv";    % <-- if needed, set full path: "/mnt/data/acc_data.csv"
assumeUnits = "auto";        % "auto", "m/s^2", or "g"
lpCutoff_Hz = 2.0;           % low-pass cutoff: maneuvers (turns/braking)
hpCutoff_Hz = 5.0;           % high-pass cutoff: vibration-ish content
minFs_Hz    = 5;             % sanity check: expected >= 5 Hz
exportFigures = true;        % save PNGs
outDir = "problem2_outputs";
if exportFigures && ~exist(outDir, 'dir'); mkdir(outDir); end

%% ---- LOAD CSV ----
T = readtable(csvFile, 'PreserveVariableNames', true);

% Display columns for quick sanity
disp("Columns found in CSV:");
disp(string(T.Properties.VariableNames)');

%% ---- FIND / BUILD TIME VECTOR ----
timeVarCandidates = ["time","Time","timestamp","Timestamp","t","T","seconds","Seconds"];
timeCol = find(ismember(lower(string(T.Properties.VariableNames)), lower(timeVarCandidates)), 1);

if ~isempty(timeCol)
    t = T{:, timeCol};
else
    % If no explicit time, assume first column is time if it looks monotonic,
    % otherwise create sample index time using an estimated sample rate.
    first = T{:,1};
    if isnumeric(first) && numel(first) > 10 && all(isfinite(first)) && isMonotonicNondecreasing(first)
        t = first;
        timeCol = 1;
    else
        warning("No obvious time column found. Creating time from sample index using estimated Fs.");
        % Estimate Fs from any "dt" column if exists
        dtCol = find(contains(lower(string(T.Properties.VariableNames)), "dt"), 1);
        if ~isempty(dtCol)
            dt = median(T{:,dtCol}, 'omitnan');
            Fs = 1/dt;
        else
            Fs = 50; % fallback guess
        end
        n = height(T);
        t = (0:n-1)'/Fs;
    end
end

t = double(t(:));
t = t - t(1); % start at 0

% Estimate sampling rate from time
dt = diff(t);
dt = dt(isfinite(dt) & dt>0);
Fs = 1/median(dt);
fprintf("Estimated sampling rate Fs = %.2f Hz\n", Fs);
if Fs < minFs_Hz
    warning("Estimated Fs looks low (%.2f Hz). Check your time column.", Fs);
end

%% ---- FIND ACCEL COLUMNS ----
% Try common naming conventions
vars = string(T.Properties.VariableNames);
lowerVars = lower(vars);

% Candidate groups
axCandidates = ["ax","accx","acc_x","accelx","accelerationx","x","acceleration_x"];
ayCandidates = ["ay","accy","acc_y","accely","accelerationy","y","acceleration_y"];
azCandidates = ["az","accz","acc_z","accelz","accelerationz","z","acceleration_z"];

ix = find(ismember(lowerVars, axCandidates), 1);
iy = find(ismember(lowerVars, ayCandidates), 1);
iz = find(ismember(lowerVars, azCandidates), 1);

if isempty(ix) || isempty(iy) || isempty(iz)
    % Fallback: pick 3 numeric columns excluding time-like columns
    numericMask = varfun(@isnumeric, T, 'OutputFormat','uniform');
    candidateIdx = find(numericMask);
    candidateIdx(candidateIdx == timeCol) = [];

    if numel(candidateIdx) < 3
        error("Couldn't find 3 numeric accel columns. Rename columns or add ax/ay/az.");
    end

    % Choose the 3 with highest variance (often accel channels)
    v = zeros(size(candidateIdx));
    for k = 1:numel(candidateIdx)
        col = T{:,candidateIdx(k)};
        v(k) = var(double(col), 'omitnan');
    end
    [~, order] = sort(v, 'descend');
    accIdx = candidateIdx(order(1:3));
    ix = accIdx(1); iy = accIdx(2); iz = accIdx(3);

    fprintf("Auto-selected accel columns: %s, %s, %s\n", vars(ix), vars(iy), vars(iz));
else
    fprintf("Detected accel columns: %s, %s, %s\n", vars(ix), vars(iy), vars(iz));
end

ax = double(T{:,ix});
ay = double(T{:,iy});
az = double(T{:,iz});

% Clean NaNs (simple interpolation)
ax = fillmissing(ax,'linear','EndValues','nearest');
ay = fillmissing(ay,'linear','EndValues','nearest');
az = fillmissing(az,'linear','EndValues','nearest');

%% ---- UNITS: g vs m/s^2 ----
% Heuristic: if magnitudes near 1, it's probably g. If near 9.81, it's m/s^2.
g0 = 9.80665;

if assumeUnits == "auto"
    medAbs = median([median(abs(ax)), median(abs(ay)), median(abs(az))], 'omitnan');
    if medAbs < 3
        units = "g";
    else
        units = "m/s^2";
    end
else
    units = assumeUnits;
end

fprintf("Assuming accelerometer units: %s\n", units);

if units == "g"
    ax_ms2 = ax*g0; ay_ms2 = ay*g0; az_ms2 = az*g0;
else
    ax_ms2 = ax;    ay_ms2 = ay;    az_ms2 = az;
end

%% ---- IDENTIFY VERTICAL AXIS (closest mean to +/-g) ----
means = [mean(ax_ms2), mean(ay_ms2), mean(az_ms2)];
[~, vIdx] = min(abs(abs(means) - g0));   % closest to |g|
accAll = [ax_ms2, ay_ms2, az_ms2];

aVert = accAll(:, vIdx);
otherIdx = setdiff(1:3, vIdx);

% Remaining: pick lateral as the one with higher low-frequency energy (turns)
% vs longitudinal. This is a heuristic; you can swap later if needed.
a1 = accAll(:, otherIdx(1));
a2 = accAll(:, otherIdx(2));

a1_lp = lowpass(a1, lpCutoff_Hz, Fs);
a2_lp = lowpass(a2, lpCutoff_Hz, Fs);

E1 = rms(a1_lp);
E2 = rms(a2_lp);

if E1 >= E2
    aLat = a1; aLon = a2;
    latName = vars(otherIdx(1)); lonName = vars(otherIdx(2));
else
    aLat = a2; aLon = a1;
    latName = vars(otherIdx(2)); lonName = vars(otherIdx(1));
end

accColIdx = [ix iy iz];          % column indices of the 3 accel channels
accNames  = vars(accColIdx);     % their names as strings
vName     = accNames(vIdx);      % name of the chosen vertical channel

fprintf("Vertical axis: %s | Lateral axis (heuristic): %s | Longitudinal: %s\n", ...
    vName, latName, lonName);


% Remove gravity from vertical to focus on bumps
aVert_noG = aVert - mean(aVert); % simple detrend; keep it explainable in report

%% ---- FILTERS: MANEUVERS vs VIBRATION ----
aLat_lp  = lowpass(aLat,  lpCutoff_Hz, Fs);   % turns
aLon_lp  = lowpass(aLon,  lpCutoff_Hz, Fs);   % accel/brake
aVert_lp = lowpass(aVert_noG, lpCutoff_Hz, Fs);

aLat_hp  = highpass(aLat,  hpCutoff_Hz, Fs);  % vibration-ish
aVert_hp = highpass(aVert_noG, hpCutoff_Hz, Fs);

%% ---- BASIC STATS TABLE ----
channels = ["Vertical (no g)"; "Lateral"; "Longitudinal"; ...
            "Vert LP"; "Lat LP"; "Lon LP"; ...
            "Vert HP"; "Lat HP"];

dataCell = {aVert_noG, aLat, aLon, aVert_lp, aLat_lp, aLon_lp, aVert_hp, aLat_hp};

Mean_ms2 = cellfun(@(x) mean(x,'omitnan'), dataCell).';
Std_ms2  = cellfun(@(x) std(x,'omitnan'),  dataCell).';
RMS_ms2  = cellfun(@(x) rms(x),            dataCell).';
Max_ms2  = cellfun(@(x) max(x),            dataCell).';
Min_ms2  = cellfun(@(x) min(x),            dataCell).';
P95_ms2  = cellfun(@(x) prctile(x,95),     dataCell).';
P05_ms2  = cellfun(@(x) prctile(x,5),      dataCell).';

S = table(channels, Mean_ms2, Std_ms2, RMS_ms2, Max_ms2, Min_ms2, P95_ms2, P05_ms2, ...
          'VariableNames', ["Channel","Mean_ms2","Std_ms2","RMS_ms2","Max_ms2","Min_ms2","P95_ms2","P05_ms2"]);

disp("Summary stats (m/s^2):");
disp(S);

if exportFigures
    writetable(S, fullfile(outDir, "acc_stats_summary.csv"));
end


%% ---- TIME SCALE / DOMINANT FREQUENCIES (PSD) ----
% Welch PSD for vertical and lateral
[Px_v, f] = pwelch(aVert_noG, [], [], [], Fs);
[Px_l, ~] = pwelch(aLat,     [], [], [], Fs);

domFreq_v = f(argmax(Px_v));
domFreq_l = f(argmax(Px_l));

fprintf("Dominant freq (PSD peak): Vertical(no g) ~ %.2f Hz | Lateral ~ %.2f Hz\n", domFreq_v, domFreq_l);

%% ---- PLOTS ----
% 1) Raw accel signals
figure; 
plot(t, aVert_noG); hold on; plot(t, aLat); plot(t, aLon);
xlabel("Time (s)"); ylabel("Acceleration (m/s^2)");
title("Raw Accelerations (Vertical detrended, Lateral, Longitudinal)");
legend("Vertical (no g)", "Lateral", "Longitudinal", 'Location','best');
grid on;
if exportFigures, saveas(gcf, fullfile(outDir, "01_raw_accels.png")); end

% 2) Low-pass (maneuvers)
figure;
plot(t, aLat_lp); hold on; plot(t, aLon_lp); plot(t, aVert_lp);
xlabel("Time (s)"); ylabel("Acceleration (m/s^2)");
title(sprintf("Low-pass (<= %.1f Hz): Maneuvers / Large-scale motion", lpCutoff_Hz));
legend("Lateral LP", "Longitudinal LP", "Vertical LP", 'Location','best');
grid on;
if exportFigures, saveas(gcf, fullfile(outDir, "02_lowpass_maneuvers.png")); end

% 3) High-pass (vibration-ish)
figure;
plot(t, aVert_hp); hold on; plot(t, aLat_hp);
xlabel("Time (s)"); ylabel("Acceleration (m/s^2)");
title(sprintf("High-pass (>= %.1f Hz): Fine-scale vibration content", hpCutoff_Hz));
legend("Vertical HP", "Lateral HP", 'Location','best');
grid on;
if exportFigures, saveas(gcf, fullfile(outDir, "03_highpass_vibration.png")); end

% 4) PSD plots (time-scale evidence)
figure;
plot(f, 10*log10(Px_v)); hold on;
plot(f, 10*log10(Px_l));
xlabel("Frequency (Hz)"); ylabel("Power/Frequency (dB/Hz)");
title("Power Spectral Density (Welch) — Time-scale Evidence");
legend("Vertical(no g)", "Lateral", 'Location','best');
grid on; xlim([0, min(50, Fs/2)]);
if exportFigures, saveas(gcf, fullfile(outDir, "04_psd_timescales.png")); end

% 5) Simple event highlighting: turns (lateral LP threshold)
turnThresh = 5.0; % m/s^2 heuristic; tune this
turnMask = abs(aLat_lp) > turnThresh;

figure;
plot(t, aLat_lp, 'LineWidth', 1); hold on;
yline(turnThresh, '--'); yline(-turnThresh, '--');
scatter(t(turnMask), aLat_lp(turnMask), 10, 'filled');
xlabel("Time (s)"); ylabel("Lateral Accel LP (m/s^2)");
title(sprintf("Detected Turn-ish Segments (|Lat LP| > %.1f m/s^2)", turnThresh));
grid on;
if exportFigures, saveas(gcf, fullfile(outDir, "05_turn_detection.png")); end

%% ---- TEXT OUTPUT YOU CAN PASTE INTO REPORT ----
fprintf("\n---- REPORT-READY NOTES (edit these) ----\n");
fprintf("Phone accel units assumed: %s; converted to m/s^2.\n", units);
fprintf("Vertical axis chosen as channel whose mean was closest to ±g (%.2f m/s^2).\n", g0);
fprintf("Gravity removed from vertical via mean-detrend to focus on bumps/ride content.\n");
fprintf("Low-pass cutoff %.1f Hz used to represent maneuver time-scales (turns/braking).\n", lpCutoff_Hz);
fprintf("High-pass cutoff %.1f Hz used to represent fine-scale vibration content.\n", hpCutoff_Hz);
fprintf("Dominant frequencies (Welch PSD peak): Vertical(no g) ~ %.2f Hz; Lateral ~ %.2f Hz.\n", domFreq_v, domFreq_l);
fprintf("Turn-ish segments flagged when |Lat LP| > %.1f m/s^2.\n", turnThresh);

%% ---- HELPER FUNCTIONS (local) ----
function tf = isMonotonicNondecreasing(x)
    x = double(x(:));
    dx = diff(x);
    tf = all(dx >= -1e-9); % allow tiny numerical noise
end

function idx = argmax(v)
    [~, idx] = max(v);
end
