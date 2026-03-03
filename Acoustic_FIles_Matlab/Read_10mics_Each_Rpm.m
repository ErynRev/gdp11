clear; clc; close all;

%% ================= USER SETTINGS =================
dataFolder = "C:\Users\joeje\OneDrive\Desktop\GDP 11 - Drone Base Noise Testing\3";

micCh  = 1:10;     % mics
tachCh = 17;       % laser tach

% Welch settings (match your original)
window   = 2^13;
noverlap = 0;
nfft     = window;

% SPL conversion (same as your original script)
SPL_offset = 94;

% RPM estimation from tach
pulsesPerRev = 1;       % change if tach outputs >1 pulse per rev
minEdgeSep_s = 0.001;   % debounce (seconds)

% RPM grouping (bin files within +/- binWidth/2 into one RPM figure)
rpmBinWidth = 50;       % RPM

% Frequency plotting limits
fmin = 10;              % Hz

%% ================= FIND FILES =================
files = dir(fullfile(dataFolder, "*.mat"));
files = files(~startsWith({files.name}, "ZERO", "IgnoreCase", true));

if isempty(files)
    error("No MAT files found in: %s", dataFolder);
end

%% ================= COLLECT ONE SPECTRUM PER FILE =================
specList = struct('file',{},'rpm',{},'Fs',{},'f',{},'SPL10',{});

for k = 1:numel(files)
    fileName = files(k).name;
    filePath = fullfile(files(k).folder, fileName);

    S = load(filePath);

    if ~isfield(S,"data_LC") || ~isfield(S,"F_s")
        continue
    end

    X  = S.data_LC;
    Fs = S.F_s;

    if size(X,2) < 17
        continue
    end

    % -------- RPM from tach --------
    tach = X(:,tachCh);
    thr  = 0.5*(min(tach) + max(tach));
    edges = find(diff(tach > thr) == 1) + 1;   % rising edges (sample indices)

    if ~isempty(edges)
        edgeTimes = edges / Fs;
        keep = [true; diff(edgeTimes) > minEdgeSep_s];
        edgeTimes = edgeTimes(keep);
    else
        edgeTimes = [];
    end

    if numel(edgeTimes) < 3
        % If tach isn't usable, you can fall back to filename parsing:
        % tok = regexp(fileName,'(\d+)\.mat$','tokens','once');
        % if ~isempty(tok), rpm_mean = str2double(tok{1}); else, continue; end
        continue
    end

    dt = diff(edgeTimes);
    dt = dt(dt > 0);
    meddt = median(dt);
    dt = dt(dt > 0.2*meddt & dt < 5*meddt);
    rpm_inst = (60 ./ dt) / pulsesPerRev;
    rpm_mean = mean(rpm_inst);

    if ~isfinite(rpm_mean)
        continue
    end

    % -------- SPL spectra for mic channels 1–10 --------
    SPL10 = zeros(nfft/2+1, numel(micCh));
    f = [];

    for m = 1:numel(micCh)
        ch = micCh(m);
        [pxx,f] = pwelch(X(:,ch), hann(window,'periodic'), noverlap, nfft, Fs);
        SPL10(:,m) = 10*log10(pxx) + SPL_offset;
    end

    specList(end+1).file = fileName; %#ok<SAGROW>
    specList(end).rpm = rpm_mean;
    specList(end).Fs = Fs;
    specList(end).f = f;
    specList(end).SPL10 = SPL10;
end

if isempty(specList)
    error("No usable files found (couldn't compute RPM/spectra).");
end

%% ================= GROUP FILES INTO RPM BINS =================
rpms = [specList.rpm];
rpmCenters = round(rpms / rpmBinWidth) * rpmBinWidth;
uniqueBins = unique(rpmCenters);

% Use the frequency grid from the first file as the common grid
fCommon = specList(1).f(:);
mask = fCommon >= fmin & fCommon <= max(fCommon);

%% ================= MAKE ONE FIGURE PER RPM BIN =================
for b = 1:numel(uniqueBins)
    binRPM = uniqueBins(b);
    idx = find(rpmCenters == binRPM);

    % Average across all files that landed in this RPM bin
    SPL_stack = [];

    for j = 1:numel(idx)
        s = specList(idx(j));

        % interpolate onto fCommon if needed
        if numel(s.f) ~= numel(fCommon) || any(abs(s.f - fCommon) > 1e-9)
            SPL_interp = zeros(numel(fCommon), numel(micCh));
            for m = 1:numel(micCh)
                SPL_interp(:,m) = interp1(s.f, s.SPL10(:,m), fCommon, 'linear', 'extrap');
            end
        else
            SPL_interp = s.SPL10;
        end

        SPL_stack(:,:,j) = SPL_interp; %#ok<SAGROW>
    end

    SPL_avg = mean(SPL_stack, 3);  % (freq x 10 mics)

    % -------- Plot: 10 mics on one graph for this RPM --------
   figure('Name', sprintf('RPM ~ %d', round(binRPM)));
ax = axes;               % <-- explicitly create axes
hold(ax,'on');
grid(ax,'on');

for m = 1:numel(micCh)
    plot(ax, fCommon(mask), SPL_avg(mask,m), ...
        'DisplayName', sprintf('Mic %d', micCh(m)));
end

% FORCE logarithmic frequency axis
set(ax,'XScale','log');

xlabel('Frequency (Hz)');
ylabel('SPL (dB)');
title(sprintf('Noise spectra | RPM ~ %d', round(binRPM)));

xlim([fmin max(fCommon)]);
xticks([10 20 50 100 200 500 1000 2000 5000 10000 20000]);

legend('Location','best');

    xlabel('Frequency (Hz)');
    ylabel('SPL (dB)');
    title(sprintf('Noise spectra | RPM ~ %d (averaged over %d file(s))', round(binRPM), numel(idx)));

    xlim([fmin max(fCommon)]);
    legend('Location','best');
end