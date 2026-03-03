clear; clc; close all;

%% ================= USER SETTINGS =================
dataFolder = "C:\Users\joeje\OneDrive\Desktop\GDP 11 - Drone Base Noise Testing\3";

micCh  = 2:10;      % use only mics 2–10
tachCh = 17;

% Welch settings (match DAQ)
window   = 2^13;
noverlap = 0;
nfft     = window;

% SPL offset
SPL_offset = 94;

% Tach processing
pulsesPerRev = 1;
minEdgeSep_s = 0.001;

% RPM binning
rpmBinWidth = 50;   % RPM

% Frequency limits
fmin = 10;          % Hz

%% ================= FIND FILES =================
files = dir(fullfile(dataFolder, "*.mat"));
files = files(~startsWith({files.name}, "ZERO", "IgnoreCase", true));

if isempty(files)
    error("No MAT files found.");
end

%% ================= EXTRACT RPM + LINEAR PSD =================
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
    if isempty(dt), continue; end

    rpm = mean(60 ./ dt / pulsesPerRev);
    if ~isfinite(rpm), continue; end

    %% ---- Mean linear PSD across mics 3–10 ----
    Pxx = [];
    f = [];

    for m = 1:numel(micCh)
        [pxx,f] = pwelch(X(:,micCh(m)), hann(window,'periodic'), noverlap, nfft, Fs);
        Pxx(:,m) = pxx; %#ok<SAGROW>
    end

    PxxMean = mean(Pxx, 2);
    PxxMean = max(PxxMean, eps);

    spec(end+1).rpm = rpm; %#ok<SAGROW>
    spec(end).f = f(:);
    spec(end).PxxMean = PxxMean(:);
end

if isempty(spec)
    error("No usable files found.");
end

%% ================= GROUP FILES BY RPM =================
rpms = [spec.rpm];
rpmCenters = round(rpms / rpmBinWidth) * rpmBinWidth;
uniqueRPM = unique(rpmCenters);

fCommon = spec(1).f(:);
mask = fCommon >= fmin & fCommon <= max(fCommon);

%% ================= ONE FIGURE PER RPM =================
for i = 1:numel(uniqueRPM)

    idx = find(rpmCenters == uniqueRPM(i));
    if isempty(idx), continue; end

    % Average across files in this RPM bin (linear domain)
    Pstack = [];

    for j = 1:numel(idx)
        s = spec(idx(j));

        if numel(s.f) ~= numel(fCommon) || any(abs(s.f - fCommon) > 1e-9)
            P_interp = interp1(s.f, s.PxxMean, fCommon, 'linear', 'extrap');
        else
            P_interp = s.PxxMean;
        end

        Pstack(:,end+1) = P_interp(:); %#ok<SAGROW>
    end

    Pxx_binMean = mean(Pstack, 2);
    Pxx_binMean = max(Pxx_binMean, eps);

    SPL_binMean = 10*log10(Pxx_binMean) + SPL_offset;

    %% ---- Plot ----
    figure('Name', sprintf('RPM ~ %d', round(uniqueRPM(i))));
    ax = axes;
    plot(ax, fCommon(mask), SPL_binMean(mask), 'LineWidth', 1.5);

    set(ax,'XScale','log');
    grid(ax,'on');

    xlabel('Frequency (Hz)');
    ylabel('SPL (dB)');
    title(sprintf('Mean noise spectrum (mics 2–10) | RPM ~ %d', round(uniqueRPM(i))));

    xlim([fmin max(fCommon)]);
    xticks([10 20 50 100 200 500 1000 2000 5000 10000 20000]);
end