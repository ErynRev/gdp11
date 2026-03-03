clear; clc; close all;

%% ---------------- USER EDIT SECTION ----------------
calFolder = "C:\Users\joeje\OneDrive\Desktop\GDP 11 - Drone Base Noise Testing\Mic Calibration";

cases = struct([]);

cases(1).name   = "Rotor 3";
cases(1).folder = "C:\Users\joeje\OneDrive\Desktop\GDP 11 - Drone Base Noise Testing\3";

cases(2).name   = "Rotor 4";
cases(2).folder = "C:\Users\joeje\OneDrive\Desktop\GDP 11 - Drone Base Noise Testing\4";

cases(3).name   = "Rotor 3+4";
cases(3).folder = "C:\Users\joeje\OneDrive\Desktop\GDP 11 - Drone Base Noise Testing\3 & 4";

cases(4).name   = "All Rotors";
cases(4).folder = "C:\Users\joeje\OneDrive\Desktop\GDP 11 - Drone Base Noise Testing\All";

micCh = 3:10;

rho  = 1.2;
c0   = 343;
Wref = 1e-12;

window   = 2^13;
noverlap = 0;
nfft     = window;

fmin_OA = 20;
fmax_OA_target = 20000;

Fs_fallback = 40000;

ang_deg = [70.4718 65.317 60.5543 54.6914 50.1041 45.6616 40.0572 35.5191];
r_m     = [2 2 2 2 2 2 2 2];

%% ---------------- BUILD sen FROM micxx.mat CALIBRATION FILES ----------------
sen = buildSenFromCalibration(calFolder, micCh);

if numel(ang_deg) ~= numel(micCh) || numel(r_m) ~= numel(micCh) || numel(sen) ~= numel(micCh)
    error("ang_deg, r_m, and sen must all have length %d (same as micCh).", numel(micCh));
end

%% ---------------- LOOP ALL CASES, COMPUTE OAPWL vs RPM ----------------
results = repmat(struct("name","","rpms",[],"oapwl",[]), numel(cases), 1);

for i = 1:numel(cases)
    dataFolder = cases(i).folder;
    if ~isfolder(dataFolder)
        warning("Case '%s' folder not found: %s (skipping)", cases(i).name, dataFolder);
        continue
    end

    out = computeOAPWLForFolder( ...
        dataFolder, micCh, ang_deg, r_m, sen, ...
        Fs_fallback, window, noverlap, nfft, ...
        rho, c0, Wref, fmin_OA, fmax_OA_target);

    results(i).name  = cases(i).name;
    results(i).rpms  = out.rpms(:);
    results(i).oapwl = out.oapwl(:);

    fprintf("Case '%s': %d points\n", results(i).name, numel(results(i).rpms));
end

%% ---------------- CREATE THEORETICAL CASES ----------------
theory34  = struct("name","Theory 3+4 (R3+R4 power sum)","rpms",[],"oapwl",[]);
theoryAll = struct("name","Theory All Rotors (2x MEASURED 3+4)","rpms",[],"oapwl",[]);

% Robust name matching
names = string({results.name});
idxR3   = find(names == "Rotor 3", 1);
idxR4   = find(names == "Rotor 4", 1);
idxR34m = find(names == "Rotor 3+4", 1);  % measured 3+4

disp("Detected case names:");
disp(names.');

fprintf("idxR3 = %d, idxR4 = %d, idxR34m = %d\n", idxR3, idxR4, idxR34m);

%% ---- Theory 3+4 from Rotor 3 & Rotor 4 (paired by index, avg RPM)
if isempty(idxR3) || isempty(idxR4)
    warning("Cannot create Theory 3+4: Rotor 3 and/or Rotor 4 not found.");
else
    rpm3 = results(idxR3).rpms(:);  L3 = results(idxR3).oapwl(:);
    rpm4 = results(idxR4).rpms(:);  L4 = results(idxR4).oapwl(:);

    keep3 = isfinite(rpm3) & isfinite(L3);
    keep4 = isfinite(rpm4) & isfinite(L4);
    rpm3 = rpm3(keep3); L3 = L3(keep3);
    rpm4 = rpm4(keep4); L4 = L4(keep4);

    [rpm3, i3] = sort(rpm3); L3 = L3(i3);
    [rpm4, i4] = sort(rpm4); L4 = L4(i4);

    N = min(numel(rpm3), numel(rpm4));

    if N < 1
        warning("Cannot create Theory 3+4: no valid paired points.");
    else
        rpm_avg = 0.5*(rpm3(1:N) + rpm4(1:N));

        % Add in linear power (outside dB)
        P3 = 10.^(L3(1:N)/10);
        P4 = 10.^(L4(1:N)/10);
        P34 = P3 + P4;

        theory34.rpms  = rpm_avg;
        theory34.oapwl = 10*log10(P34);

        fprintf("Created '%s': %d points\n", theory34.name, numel(theory34.rpms));
    end
end

%% ---- Theory All Rotors from MEASURED Rotor 3+4 (double power)
if isempty(idxR34m) || isempty(results(idxR34m).rpms)
    warning("Cannot create Theory All Rotors: measured Rotor 3+4 not found.");
else
    rpm34m = results(idxR34m).rpms(:);
    L34m   = results(idxR34m).oapwl(:);

    keep = isfinite(rpm34m) & isfinite(L34m);
    rpm34m = rpm34m(keep);
    L34m   = L34m(keep);

    [rpm34m, is] = sort(rpm34m);
    L34m = L34m(is);

    % Double in linear power (outside dB)
    P34m = 10.^(L34m/10);
    Pall = 2 * P34m;

    theoryAll.rpms  = rpm34m;
    theoryAll.oapwl = 10*log10(Pall);

    fprintf("Created '%s': %d points\n", theoryAll.name, numel(theoryAll.rpms));
end

%% ---------------- PLOT MEASURED + THEORETICAL CASES ----------------
figure('Name','Overall PWL vs RPM (measured + theoretical)');
grid on; hold on;
anyPlotted = false;

% Measured cases
for i = 1:numel(results)
    if isempty(results(i).rpms), continue; end
    plot(results(i).rpms, results(i).oapwl, '-o', 'LineWidth', 1.5, ...
        'DisplayName', results(i).name);
    anyPlotted = true;
end

% Theory 3+4
if ~isempty(theory34.rpms)
    plot(theory34.rpms, theory34.oapwl, '--s', 'LineWidth', 2.0, ...
        'MarkerSize', 7, 'DisplayName', theory34.name);
    anyPlotted = true;
end

% Theory All Rotors (from measured 3+4)
if ~isempty(theoryAll.rpms)
    plot(theoryAll.rpms, theoryAll.oapwl, '--d', 'LineWidth', 2.0, ...
        'MarkerSize', 7, 'DisplayName', theoryAll.name);
    anyPlotted = true;
end

set(gca,'XScale','log');
xlim([1500 6000]);
xticks([1500 2000 2500 3000 4000 5000 6000]);
xticklabels({'1500','2000','2500','3000','4000','5000','6000'});

xlabel('RPM');
ylabel('OAPWL 20–20k Hz (dB re 1 pW)');
title('Overall sound power level vs RPM for each motor case');
legend('Location','best');

if ~anyPlotted
    warning("Nothing plotted. Check folders, file naming, and data_LC existence.");
end

%% ==============================================================
%                     LOCAL FUNCTIONS
% ==============================================================

function sen = buildSenFromCalibration(calFolder, micCh)
    sen = nan(1, numel(micCh));
    if ~isfolder(calFolder)
        error("Calibration folder not found: %s", calFolder);
    end

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
            error("Couldn't find a numeric vector signal in %s.", fpath);
        end

        signal = S.(bestName);
        signal = signal(:);
        signal = signal(isfinite(signal));

        if numel(signal) < 100
            error("Calibration signal in %s (%s) is too short (%d samples).", fpath, bestName, numel(signal));
        end

        sen(ii) = 10 / std(signal);

        fprintf("Cal mic %d: used var '%s' | std=%.6g | sen=%.6g Pa/V\n", ...
            m, bestName, std(signal), sen(ii));
    end

    if any(~isfinite(sen)) || any(sen <= 0)
        error("Invalid sen values computed. Check calibration signals / files.");
    end
end

function out = computeOAPWLForFolder(dataFolder, micCh, ang_deg, r_m, sen, Fs_fallback, window, noverlap, nfft, rho, c0, Wref, fmin_OA, fmax_OA_target)

    files = dir(fullfile(dataFolder, "*.mat"));
    files = files(~startsWith({files.name}, "ZERO", "IgnoreCase", true));

    if isempty(files)
        out.rpms = [];
        out.oapwl = [];
        return
    end

    rpms = nan(numel(files),1);
    for k = 1:numel(files)
        tok = regexp(files(k).name, "(\d+)\.mat$", "tokens", "once");
        if ~isempty(tok)
            rpms(k) = str2double(tok{1});
        end
    end

    [~, idx] = sort(rpms);
    files = files(idx);
    rpms  = rpms(idx);

    [angS, order] = sort(ang_deg, "descend");
    rS   = r_m(order);
    senS = sen(order);

    dtheta = (angS(1:end-1) - angS(2:end)) * pi/180;
    thavg  = 0.5*(angS(1:end-1) + angS(2:end)) * pi/180;
    ravg   = 0.5*(rS(1:end-1) + rS(2:end));

    dA = 2*pi .* (ravg.^2) .* sin(thavg) .* dtheta;  % m^2
    w  = (dA ./ (rho*c0)).';                         % (M-1)x1

    win = hann(window,'periodic');

    OAPWL = nan(numel(files),1);

    for k = 1:numel(files)
        S = load(fullfile(files(k).folder, files(k).name));
        if ~isfield(S,"data_LC")
            continue
        end
        X = S.data_LC;

        if isfield(S,"F_s")
            Fs = S.F_s;
        else
            Fs = Fs_fallback;
        end
        fmax_OA = min(fmax_OA_target, Fs/2);

        if size(X,2) < max(micCh)
            continue
        end

        V = X(:, micCh);
        V = V(:, order);
        P = V .* reshape(senS, 1, []);  % V -> Pa

        [Spp, F] = pwelch(P, win, noverlap, nfft, Fs);

        Spp_avg = 0.5*(Spp(:,1:end-1) + Spp(:,2:end));
        Wf = Spp_avg * w;

        mask = (F >= fmin_OA) & (F <= fmax_OA);
        if ~any(mask)
            continue
        end

        W_band = trapz(F(mask), Wf(mask));
        OAPWL(k) = 10*log10(max(W_band, eps)/Wref);
    end

    keep = isfinite(rpms) & isfinite(OAPWL);
    out.rpms  = rpms(keep);
    out.oapwl = OAPWL(keep);
end