clear; clc; close all;

%% ================= USER SETTINGS =================
dataFolder = "C:\Users\joeje\OneDrive\Desktop\GDP 11 - Drone Base Noise Testing\3";

micCh  = 2:10;     % <-- exclude mics 1–2; use mics 2–10 only
tachCh = 17;

% Welch settings (match your original DAQ code)
window   = 2^13;
noverlap = 0;
nfft     = window;

% SPL conversion offset (kept same as your original)
SPL_offset = 94;

% Tach processing
pulsesPerRev = 1;       % change if tach outputs multiple pulses per rev
minEdgeSep_s = 0.001;   % debounce in seconds

% RPM grouping (binning)
rpmBinWidth = 50;       % RPM

% Frequency limits for plotting
fmin = 10;              % Hz

%% ================= FIND FILES =================
files = dir(fullfile(dataFolder, "*.mat"));
files = files(~startsWith({files.name}, "ZERO", "IgnoreCase", true));

if isempty(files)
    error("No MAT files found in: %s", dataFolder);
end

%% ================= EXTRACT RPM + LINEAR PSD PER FILE =================
spec = struct('rpm',{},'f',{},'PxxMean',{});

for k = 1:numel(files)

    S = load(fullfile(files(k).folder, files(k).name));
    if ~isfield(S,"data_LC") || ~isfield(S,"F_s")
        continue
    end

    X  = S.data_LC;
    Fs = S.F_s;

    if size(X,2) < 17
        continue
    end

    %% ---- RPM from tach ----
    tach = X(:,tachCh);
    thr = 0.5*(min(tach) + max(tach));
    edges = find(diff(tach > thr) == 1) + 1;

    if isempty(edges)
        continue
    end

    edgeTimes = edges / Fs;
    edgeTimes = edgeTimes([true; diff(edgeTimes) > minEdgeSep_s]);

    if numel(edgeTimes) < 3
        continue
    end

    dt = diff(edgeTimes);
    meddt = median(dt);
    dt = dt(dt > 0.2*meddt & dt < 5*meddt);

    if isempty(dt)
        continue
    end

    rpm = mean(60 ./ dt / pulsesPerRev);
    if ~isfinite(rpm)
        continue
    end

    %% ---- LINEAR PSD averaging across mics (3–10) ----
    Pxx = [];
    f = [];

    for m = 1:numel(micCh)
        ch = micCh(m);
        [pxx, f] = pwelch(X(:,ch), hann(window,'periodic'), noverlap, nfft, Fs);
        Pxx(:,m) = pxx; %#ok<SAGROW>   % <-- keep linear PSD (NOT dB)
    end

    PxxMean = mean(Pxx, 2);           % <-- mean in linear domain (power)
    PxxMean = max(PxxMean, eps);      % guard against log10(0) later

    spec(end+1).rpm = rpm; %#ok<SAGROW>
    spec(end).f = f(:);
    spec(end).PxxMean = PxxMean(:);
end

if isempty(spec)
    error("No usable files found (couldn't compute RPM + PSD).");
end

%% ================= GROUP BY RPM (BIN FILES) =================
rpms = [spec.rpm];
rpmCenters = round(rpms / rpmBinWidth) * rpmBinWidth;
uniqueRPM = unique(rpmCenters);

% Common frequency grid (use first file's frequency vector)
fCommon = spec(1).f;
mask = fCommon >= fmin & fCommon <= max(fCommon);

%% ================= PLOT: ONE CURVE PER RPM (MEAN OF MICS 3–10) =================
figure('Name','Mean noise spectra vs RPM (mics 3–10, linear-power averaged)');
ax = axes;
hold(ax,'on'); grid(ax,'on');

for i = 1:numel(uniqueRPM)

    idx = find(rpmCenters == uniqueRPM(i));
    if isempty(idx), continue; end

    % Average across files in this RPM bin (still in linear PSD domain)
    Pstack = [];

    for j = 1:numel(idx)
        s = spec(idx(j));

        % interpolate onto common frequency grid if needed
        if numel(s.f) ~= numel(fCommon) || any(abs(s.f - fCommon) > 1e-9)
            P_interp = interp1(s.f, s.PxxMean, fCommon, 'linear', 'extrap');
        else
            P_interp = s.PxxMean;
        end

        Pstack(:,end+1) = P_interp(:); %#ok<SAGROW>
    end

    Pxx_binMean = mean(Pstack, 2);         % mean in linear domain across files
    Pxx_binMean = max(Pxx_binMean, eps);

    % Convert to dB AFTER averaging
    SPL_binMean = 10*log10(Pxx_binMean) + SPL_offset;

    plot(ax, fCommon(mask), SPL_binMean(mask), ...
        'DisplayName', sprintf('%d RPM', round(uniqueRPM(i))));
end

% Force logarithmic frequency axis (robust)
set(ax,'XScale','log');
xlim([fmin max(fCommon)]);
xticks([10 20 50 100 200 500 1000 2000 5000 10000 20000]);

xlabel('Frequency (Hz)');
ylabel('SPL (dB) from mean linear PSD (mics 3–10)');
title(sprintf('Mean noise spectra vs RPM (mics %d–%d), linear-power averaged', micCh(1), micCh(end)));
legend('Location','best');