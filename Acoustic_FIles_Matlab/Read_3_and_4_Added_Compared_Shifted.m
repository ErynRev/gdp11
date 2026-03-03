clear; clc; close all;

%% ================= PATHS =================
folder_R4  = "C:\Users\joeje\OneDrive\Desktop\GDP 11 - Drone Base Noise Testing\4";
folder_R3  = "C:\Users\joeje\OneDrive\Desktop\GDP 11 - Drone Base Noise Testing\3";
folder_R34 = "C:\Users\joeje\OneDrive\Desktop\GDP 11 - Drone Base Noise Testing\3 + 4";

%% ================= USER SETTINGS =================
micCh  = 3:10;   % apples-to-apples mic set for ALL cases
tachCh = 17;

window   = 2^13;
noverlap = 0;
nfft     = window;

SPL_offset = 94;

pulsesPerRev = 1;
minEdgeSep_s = 0.001;

fmin = 10;
fmax_plot = 5000;

Nb = 2;  % blades (BPF = Nb * RPM/60)

% Peak-pick settings around nominal pair BPF
bpfSearchHalfWidth_Hz = 120;   % +/- Hz around nominal BPF to look for two peaks
minPeakSeparation_Hz  = 15;    % avoid picking same peak twice
peakProminence_dB     = 1.5;   % minimum peak prominence (in plotted dB units)

%% ================= DEFINE COMPARISON POINTS =================
cases(1).label = "LOW";
cases(1).R3  = "A0_3_2908.mat";
cases(1).R4  = "A0_4_2758.mat";
cases(1).R34 = "A0_34_2785.mat";

cases(2).label = "MID";
cases(2).R3  = "A0_3_4555.mat";
cases(2).R4  = "A0_4_4572.mat";
cases(2).R34 = "A0_34_4516.mat";

cases(3).label = "HIGH";
cases(3).R3  = "A0_3_5854.mat";
cases(3).R4  = "A0_4_5925.mat";
cases(3).R34 = "A0_34_5904.mat";

%% ================= MAIN LOOP =================
for k = 1:numel(cases)

    % Load/process
    out3  = processOne(fullfile(folder_R3,  cases(k).R3),  micCh, tachCh, window, noverlap, nfft, pulsesPerRev, minEdgeSep_s);
    out4  = processOne(fullfile(folder_R4,  cases(k).R4),  micCh, tachCh, window, noverlap, nfft, pulsesPerRev, minEdgeSep_s);
    out34 = processOne(fullfile(folder_R34, cases(k).R34), micCh, tachCh, window, noverlap, nfft, pulsesPerRev, minEdgeSep_s);

    % Common frequency grid (use measured pair grid)
    f = out34.f(:);

    % Interp single-rotor PSDs onto the pair grid
    P3  = interp1(out3.f,  out3.PxxMean,  f, 'linear', 'extrap');
    P4  = interp1(out4.f,  out4.PxxMean,  f, 'linear', 'extrap');
    P34 = out34.PxxMean;

    P3  = max(P3,  eps);
    P4  = max(P4,  eps);
    P34 = max(P34, eps);

    % Measured SPL (your convention)
    L34 = 10*log10(P34) + SPL_offset;

    % Nominal pair BPF (for where to search for the two peaks)
    fbpf_pair_nom = Nb*(out34.rpm_mean/60);

    % ===== Find two peaks near the pair BPF =====
    searchBand = [max(fmin, fbpf_pair_nom - bpfSearchHalfWidth_Hz), ...
                  min(fmax_plot, fbpf_pair_nom + bpfSearchHalfWidth_Hz)];

    [fpk1, fpk2, df_bpf] = findTwoBpfPeaks(f, L34, searchBand, minPeakSeparation_Hz, peakProminence_dB);

    % Fallback if peak picking fails: use single-rotor tach BPFs
    fbpf3_nom = Nb*(out3.rpm_mean/60);
    fbpf4_nom = Nb*(out4.rpm_mean/60);

    if isnan(fpk1) || isnan(fpk2)
        fpk1 = min(fbpf3_nom, fbpf4_nom);
        fpk2 = max(fbpf3_nom, fbpf4_nom);
        df_bpf = fpk2 - fpk1;
        warning("%s: peak-pick failed; using tach-based BPFs (%.2f, %.2f Hz).", cases(k).label, fpk1, fpk2);
    end

    % ===== KEEP ROTOR 3 AS-IS, SHIFT ROTOR 4 TO THE LOWER PEAK =====
    % Shift so Rotor 4 nominal BPF lands on the lower measured peak fpk1:
    df_shift_R4 = fpk1 - fbpf4_nom;   % usually negative if R4 is the higher-frequency case

    P4_shift = shiftSpectrum(P4, f, df_shift_R4);

    % Expected sum ONLY (with shifted R4)
    Pexp = max(P3 + P4_shift, eps);

    Lexp = 10*log10(Pexp) + SPL_offset;

    % Interaction using shifted expected
    Lint = 10*log10(P34 ./ Pexp);

    mask = (f >= fmin) & (f <= min(fmax_plot, max(f)));

    %% ========== FIGURE 1: MEASURED vs EXPECTED (R4 SHIFTED) ==========
    fig1 = figure('Name', sprintf('%s: Measured vs Expected (R4->low)', cases(k).label), 'Color','w');
    ax1 = axes(fig1); hold(ax1,'on'); grid(ax1,'on');

    plot(ax1, f(mask), L34(mask), 'LineWidth', 1.8, 'DisplayName', 'Measured');
    plot(ax1, f(mask), Lexp(mask), 'LineWidth', 1.8, 'DisplayName', 'Expected');

    set(ax1,'XScale','log');
    xlabel(ax1,'Frequency (Hz)');
    ylabel(ax1,'SPL (dB)');
    title(ax1, sprintf('%s (R4 shifted by %+0.2f Hz to LOW peak)', cases(k).label, df_shift_R4));

    % Keep ONLY the two BPF markers (no extra dotted lines)
    yl = ylim(ax1);
    xline(ax1, fpk1, '--', 'BPF1', 'LabelVerticalAlignment','bottom');
    xline(ax1, fpk2, '--', 'BPF2', 'LabelVerticalAlignment','bottom');
    ylim(ax1, yl);

    xlim(ax1, [fmin fmax_plot]);
    legend(ax1,'Location','best');

    %% ========== FIGURE 2: INTERACTION ==========
    fig2 = figure('Name', sprintf('%s: Interaction', cases(k).label), 'Color','w');
    ax2 = axes(fig2); hold(ax2,'on'); grid(ax2,'on');

    plot(ax2, f(mask), Lint(mask), 'LineWidth', 1.6, 'DisplayName', 'Interaction');
    yline(ax2, 0, '-k', '0 dB');

    set(ax2,'XScale','log');
    xlabel(ax2,'Frequency (Hz)');
    ylabel(ax2,'dB: 10log10(P34/Pexp)');
    title(ax2, sprintf('%s Interaction (Expected uses shifted R4)', cases(k).label));

    yl2 = ylim(ax2);
    xline(ax2, fpk1, '--', 'BPF1', 'LabelVerticalAlignment','bottom');
    xline(ax2, fpk2, '--', 'BPF2', 'LabelVerticalAlignment','bottom');
    ylim(ax2, yl2);

    xlim(ax2, [fmin fmax_plot]);
    legend(ax2,'Location','best');

    fprintf("%s: BPF peaks %.2f Hz & %.2f Hz (Δf=%.2f). R4 BPF=%.2f -> shiftR4=%.2f Hz\n", ...
        cases(k).label, fpk1, fpk2, df_bpf, fbpf4_nom, df_shift_R4);
end

disp("Done. Rotor 3 kept fixed; Rotor 4 shifted to the LOW BPF peak. Only 2 BPF marker lines retained.");

%% ================= LOCAL FUNCTIONS =================
function out = processOne(filePath, micCh, tachCh, window, noverlap, nfft, pulsesPerRev, minEdgeSep_s)

    if ~isfile(filePath)
        error("File not found: %s", filePath);
    end

    S = load(filePath);
    if ~isfield(S,"data_LC") || ~isfield(S,"F_s")
        error("Missing data_LC or F_s in: %s", filePath);
    end

    X  = S.data_LC;
    Fs = S.F_s;

    if size(X,2) < max([micCh tachCh])
        error("Not enough channels in %s (has %d).", filePath, size(X,2));
    end

    % --- RPM from tach ---
    tach = X(:,tachCh);
    thr  = 0.5*(min(tach) + max(tach));
    edges = find(diff(tach > thr) == 1) + 1;

    if isempty(edges)
        error("No tach edges found in %s", filePath);
    end

    edgeTimes = edges / Fs;
    edgeTimes = edgeTimes([true; diff(edgeTimes) > minEdgeSep_s]);

    if numel(edgeTimes) < 3
        error("Too few tach edges in %s", filePath);
    end

    dt = diff(edgeTimes);
    dt = dt(dt > 0);

    meddt = median(dt);
    dt = dt(dt > 0.2*meddt & dt < 5*meddt);

    if isempty(dt)
        error("Filtered tach intervals empty in %s", filePath);
    end

    rpm_inst = (60 ./ dt) / pulsesPerRev;
    rpm_mean = mean(rpm_inst);

    % --- Mean linear PSD across selected mics ---
    Pxx = [];
    f = [];

    for m = 1:numel(micCh)
        [pxx,f] = pwelch(X(:,micCh(m)), hann(window,'periodic'), noverlap, nfft, Fs);
        Pxx(:,m) = pxx; %#ok<AGROW>
    end

    PxxMean = mean(Pxx, 2);
    PxxMean = max(PxxMean, eps);

    out.file = filePath;
    out.Fs = Fs;
    out.f = f(:);
    out.PxxMean = PxxMean(:);
    out.rpm_mean = rpm_mean;
end

function [fpk1, fpk2, df] = findTwoBpfPeaks(f, L, band, minSepHz, minPromDb)
% Find two strongest peaks in L(f) within a band, enforcing a minimum separation.

    fpk1 = NaN; fpk2 = NaN; df = NaN;

    in = (f >= band(1)) & (f <= band(2));
    if nnz(in) < 10
        return;
    end

    fb = f(in);
    Lb = L(in);

    df_samp = median(diff(fb));
    if ~isfinite(df_samp) || df_samp <= 0
        return;
    end
    minPeakDist_samp = max(1, round(minSepHz / df_samp));

    [pks, locs] = findpeaks(Lb, ...
        'MinPeakDistance', minPeakDist_samp, ...
        'MinPeakProminence', minPromDb);

    if numel(pks) < 2
        return;
    end

    % Take the two highest peaks
    [~, idx] = sort(pks, 'descend');
    locs = locs(idx(1:2));

    f1 = fb(locs(1));
    f2 = fb(locs(2));

    fpk1 = min(f1,f2);
    fpk2 = max(f1,f2);
    df   = fpk2 - fpk1;
end

function P_shift = shiftSpectrum(P, f, df)
% Frequency shift by df (Hz): P_shift(f) = P(f - df)
% positive df moves content RIGHT (higher freq). negative df moves LEFT.

    f_src = f - df;
    P_shift = interp1(f, P, f_src, 'linear', 'extrap');
    P_shift = max(P_shift, eps);
end