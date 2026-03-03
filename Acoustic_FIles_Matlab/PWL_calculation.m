clear; clc; close all;

%% ---------------- USER SETTINGS ----------------
dataFolder = "C:\Users\joeje\OneDrive\Desktop\GDP 11 - Drone Base Noise Testing\4";

micCh = 3:10;              % mic channels in data_LC
Wref  = 1e-12;             % reference sound power (W)

% Welch settings
window   = 2^13;
noverlap = 0;
nfft     = window;
win      = hann(window,'periodic');

% Air properties
rho = 1.2;                 % kg/m^3
c0  = 343;                 % m/s

% Band for overall PWL
fmin_OA = 20;
fmax_OA_target = 20000;

%% ---------------- CALIBRATION (auto from micxx.mat) ----------------
calFolder = "C:\Users\joeje\OneDrive\Desktop\GDP 11 - Drone Base Noise Testing\Mic Calibration";

sen = nan(1, numel(micCh));

for ii = 1:numel(micCh)
    m = micCh(ii);

    cand = { ...
        fullfile(calFolder, sprintf("mic%d.mat",   m)), ...
        fullfile(calFolder, sprintf("mic%02d.mat", m)), ...
        fullfile(calFolder, sprintf("mic%03d.mat", m)) };

    fpath = "";
    for c = 1:numel(cand)
        if isfile(cand{c})
            fpath = cand{c};
            break
        end
    end
    if fpath == ""
        error("Missing calibration file for mic %d (looked for mic%d/mic%02d/mic%03d.mat in %s).", ...
              m, m, m, m, calFolder);
    end

    S = load(fpath);

    vars = fieldnames(S);
    bestName = "";
    bestLen  = 0;

    for v = 1:numel(vars)
        x = S.(vars{v});
        if isnumeric(x)
            if isvector(x)
                L = numel(x);
                if L > bestLen
                    bestLen = L;
                    bestName = vars{v};
                end
            elseif ismatrix(x) && size(x,2)==1
                L = size(x,1);
                if L > bestLen
                    bestLen = L;
                    bestName = vars{v};
                end
            end
        end
    end

    if bestName == ""
        error("Couldn't find a numeric vector signal in %s. Open it and check variable names.", fpath);
    end

    sig = S.(bestName);
    sig = sig(:);
    sig = sig(isfinite(sig));

    if numel(sig) < 100
        error("Calibration signal in %s (%s) is too short (%d samples).", fpath, bestName, numel(sig));
    end

    sen(ii) = 10 / std(sig);

    fprintf("Mic %d: using %s from %s | std=%.6g | sen=%.6g Pa/V\n", ...
            m, bestName, string(fpath), std(sig), sen(ii));
end

if any(~isfinite(sen)) || any(sen <= 0)
    error("Invalid sen values computed. Check calibration files / signals.");
end

%% ---------------- MIC GEOMETRY (MUST match micCh order) ----------------
ang_deg = [70.4718 65.317 60.5543 54.6914 50.1041 45.6616 40.0572 35.5191]; % mics 3..10
r_m     = [2 2 2 2 2 2 2 2];

if numel(ang_deg) ~= numel(micCh) || numel(r_m) ~= numel(micCh) || numel(sen) ~= numel(micCh)
    error("ang_deg, r_m, and sen must all have length %d (same as micCh).", numel(micCh));
end

%% ---------------- FIND FILES ----------------
files = dir(fullfile(dataFolder, "*.mat"));
files = files(~startsWith({files.name}, "ZERO", "IgnoreCase", true));

if isempty(files)
    error("No MAT files found in: %s", dataFolder);
end

%% ---------------- PARSE RPM FROM FILENAME (GENERAL) ----------------
rpms = nan(numel(files),1);
for k = 1:numel(files)
    tok = regexp(files(k).name, "(\d+)\.mat$", "tokens", "once");
    if ~isempty(tok), rpms(k) = str2double(tok{1}); end
end
[~, idx] = sort(rpms);
files = files(idx);
rpms  = rpms(idx);

%% ---------------- PRECOMPUTE ANGLE INTEGRATION WEIGHTS ----------------
[angS, order] = sort(ang_deg, "descend");
rS   = r_m(order);
senS = sen(order);

dtheta = (angS(1:end-1) - angS(2:end)) * pi/180;
thavg  = 0.5*(angS(1:end-1) + angS(2:end)) * pi/180;
ravg   = 0.5*(rS(1:end-1) + rS(2:end));

dA = 2*pi .* (ravg.^2) .* sin(thavg) .* dtheta;     % m^2
w  = (dA ./ (rho*c0)).';                             % (M-1) x 1

%% ---------------- LOOP OVER FILES ----------------
PWL_f_all = cell(numel(files),1);
F_all     = cell(numel(files),1);
OAPWL     = nan(numel(files),1);

for k = 1:numel(files)
    S = load(fullfile(files(k).folder, files(k).name));
    if ~isfield(S,"data_LC")
        continue
    end
    X = S.data_LC;

    if isfield(S,"F_s")
        Fs = S.F_s;
    else
        Fs = 40000;
    end
    fmax_OA = min(fmax_OA_target, Fs/2);

    if size(X,2) < max(micCh)
        warning("Skipping %s (not enough channels).", files(k).name);
        continue
    end

    V = X(:, micCh);
    V = V(:, order);
    P = V .* reshape(senS, 1, []);

    [Spp, F] = pwelch(P, win, noverlap, nfft, Fs);

    Spp_avg = 0.5*(Spp(:,1:end-1) + Spp(:,2:end));
    Wf = Spp_avg * w;

    PWL_f = 10*log10(max(Wf, eps)/Wref);

    % ----- REMOVE 0 Hz BIN (log axis can't show 0) -----
    pos = F > 0;
    Fp = F(pos);
    PWLp = PWL_f(pos);
    Wfp = Wf(pos);

    mask = (Fp >= fmin_OA) & (Fp <= fmax_OA);
    W_band = trapz(Fp(mask), Wfp(mask));
    OAPWL(k) = 10*log10(max(W_band, eps)/Wref);

    PWL_f_all{k} = PWLp;
    F_all{k}     = Fp;

    fprintf("Done %s | RPM=%g | OAPWL(%.0f–%.0f Hz)=%.2f dB re 1 pW\n", ...
        files(k).name, rpms(k), fmin_OA, fmax_OA, OAPWL(k));
end

%% ---------------- PLOTS ----------------
keep = ~cellfun(@isempty, PWL_f_all) & isfinite(OAPWL);
PWL_f_all = PWL_f_all(keep);
F_all     = F_all(keep);
rpms      = rpms(keep);
OAPWL     = OAPWL(keep);

% --------- FIRST FIGURE: FORCE LOG FREQUENCY AXIS ----------
figure('Name','PWL spectra (All motor case)'); hold on; grid on;
for k = 1:numel(PWL_f_all)
    plot(F_all{k}, PWL_f_all{k}, 'LineWidth', 1.1);   % plot + forced log axis below
end
set(gca,'XScale','log');                               % <-- FORCE LOG SCALE
xlim([10 20000]);
xticks([10 20 50 100 200 500 1000 2000 5000 10000 20000]); % optional nicer ticks
xlabel('Frequency (Hz)');
ylabel('PWL (dB re 1 pW)');
title('PWL spectrum (All motor case)');
legend(compose("%g RPM", rpms), 'Location','bestoutside');

% --------- SECOND FIGURE: OAPWL vs RPM ----------
figure('Name','Overall PWL vs RPM (All motor case)'); grid on; hold on;
plot(rpms, OAPWL, '-o', 'LineWidth', 1.5);
set(gca,'XScale','log');
xlabel('RPM');
ylabel('OAPWL 20–20k Hz (dB re 1 pW)');
title('Overall sound power level vs RPM (All motor case)');