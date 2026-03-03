%% compare_R34_vs_expected_only.m
% Apples-to-apples mic set, but ONLY plots:
%   (1) Rotor 3+4 measured spectrum vs Expected (P3+P4)
%   (2) Interaction spectrum: 10log10(P34/(P3+P4))
%
% NOTE: We still LOAD rotor 3 and rotor 4 files to compute the expected sum,
% but we do NOT plot their isolated spectra.

clear; clc; close all;

%% ================= PATHS =================
folder_R4  = "C:\Users\joeje\OneDrive\Desktop\GDP 11 - Drone Base Noise Testing\4";
folder_R3  = "C:\Users\joeje\OneDrive\Desktop\GDP 11 - Drone Base Noise Testing\3";
folder_R34 = "C:\Users\joeje\OneDrive\Desktop\GDP 11 - Drone Base Noise Testing\3 + 4";
folder_Motr = "C:\Users\joeje\OneDrive\Desktop\GDP 11 - Drone Base Noise Testing\Motor";

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
fmax_plot = 5000;     % adjust if you prefer (e.g. 2000)

Nb = 2;               % blades (for markers)

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

    % Load/process (needed for expected sum)
    out3  = processOne(fullfile(folder_R3,  cases(k).R3),  micCh, tachCh, window, noverlap, nfft, pulsesPerRev, minEdgeSep_s);
    out4  = processOne(fullfile(folder_R4,  cases(k).R4),  micCh, tachCh, window, noverlap, nfft, pulsesPerRev, minEdgeSep_s);
    out34 = processOne(fullfile(folder_R34, cases(k).R34), micCh, tachCh, window, noverlap, nfft, pulsesPerRev, minEdgeSep_s);

    % Common frequency grid (use pair case)
    f = out34.f(:);

    P3  = interp1(out3.f,  out3.PxxMean,  f, 'linear', 'extrap');
    P4  = interp1(out4.f,  out4.PxxMean,  f, 'linear', 'extrap');
    P34 = out34.PxxMean;

    P3  = max(P3,  eps);
    P4  = max(P4,  eps);
    P34 = max(P34, eps);

    % Expected sum
    Pexp = P3 + P4;

    % Convert to your dB convention
    L34  = 10*log10(P34)  + SPL_offset;
    Lexp = 10*log10(Pexp) + SPL_offset;

    % Interaction
    Lint = 10*log10(P34 ./ Pexp);

    mask = (f >= fmin) & (f <= min(fmax_plot, max(f)));

    % Markers from measured pair RPM
    rpm_use = out34.rpm_mean;
    frot = rpm_use/60;
    fbpf = Nb*frot;

    %% ========== FIGURE 1: MEASURED vs EXPECTED ==========
    fig1 = figure('Name', sprintf('%s: R3+4 vs Expected', cases(k).label), 'Color','w');
    ax1 = axes(fig1); hold(ax1,'on'); grid(ax1,'on');

    plot(ax1, f(mask), L34(mask), 'LineWidth', 1.5, ...
        'DisplayName', sprintf('Measured 3+4 (%.0f RPM)', out34.rpm_mean));
    plot(ax1, f(mask), Lexp(mask),'--', 'LineWidth', 1.5, ...
        'DisplayName', sprintf('Expected (P3+P4) [R3 %.0f, R4 %.0f RPM]', out3.rpm_mean, out4.rpm_mean));

    set(ax1,'XScale','log');
    xlabel(ax1,'Frequency (Hz)');
    ylabel(ax1,'SPL (dB, per your formula)');
    title(ax1, sprintf('%s: Measured vs Expected (mics %d–%d)', cases(k).label, micCh(1), micCh(end)));

    yl = ylim(ax1);
    xline(ax1, frot, ':',  '1x',  'LabelVerticalAlignment','bottom');
    xline(ax1, fbpf, '--', 'BPF', 'LabelVerticalAlignment','bottom');
    ylim(ax1, yl);

    xlim(ax1, [fmin fmax_plot]);
    legend(ax1,'Location','best');

    %% ========== FIGURE 2: INTERACTION ==========
    fig2 = figure('Name', sprintf('%s: Interaction', cases(k).label), 'Color','w');
    ax2 = axes(fig2); hold(ax2,'on'); grid(ax2,'on');

    plot(ax2, f(mask), Lint(mask), 'LineWidth', 1.4, 'DisplayName', 'Interaction');
    yline(ax2, 0, '-k', 'DisplayName', '0 dB');

    set(ax2,'XScale','log');
    xlabel(ax2,'Frequency (Hz)');
    ylabel(ax2,'Interaction (dB): 10log10(P34/(P3+P4))');
    title(ax2, sprintf('%s: Interaction spectrum (positive = louder than sum)', cases(k).label));

    yl2 = ylim(ax2);
    xline(ax2, frot, ':',  '1x',  'LabelVerticalAlignment','bottom');
    xline(ax2, fbpf, '--', 'BPF', 'LabelVerticalAlignment','bottom');
    ylim(ax2, yl2);

    xlim(ax2, [fmin fmax_plot]);
    legend(ax2,'Location','best');
end

disp("Done. Plotted ONLY measured (3+4) vs expected and interaction for LOW/MID/HIGH.");

%% ================= LOCAL FUNCTION =================
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