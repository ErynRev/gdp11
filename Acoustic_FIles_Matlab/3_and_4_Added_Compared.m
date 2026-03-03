clear; clc; close all;

%% ================= PATHS =================
folder_R4  = "C:\Users\joeje\OneDrive\Desktop\GDP 11 - Drone Base Noise Testing\4";
folder_R3  = "C:\Users\joeje\OneDrive\Desktop\GDP 11 - Drone Base Noise Testing\3";
folder_R34 = "C:\Users\joeje\OneDrive\Desktop\GDP 11 - Drone Base Noise Testing\3 + 4";

%% ================= USER SETTINGS (match yours) =================
micCh  = 3:10;     % use mics 3–10 only
tachCh = 17;       % laser tach

window   = 2^13;
noverlap = 0;
nfft     = window;

SPL_offset = 94;   % your offset

pulsesPerRev = 1;
minEdgeSep_s = 0.001;

fmin = 10;         % Hz
fmax_plot = 5000;  % adjust if you want (e.g. 2000)

Nb = 2;            % rotor blade count (for BPF marker)

%% ================= DEFINE YOUR THREE COMPARISON POINTS =================
% (Using your chosen low/mid/high filenames)
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
    % --- Load + process each file ---
    out3  = processOne(fullfile(folder_R3,  cases(k).R3),  micCh, tachCh, window, noverlap, nfft, pulsesPerRev, minEdgeSep_s);
    out4  = processOne(fullfile(folder_R4,  cases(k).R4),  micCh, tachCh, window, noverlap, nfft, pulsesPerRev, minEdgeSep_s);
    out34 = processOne(fullfile(folder_R34, cases(k).R34), micCh, tachCh, window, noverlap, nfft, pulsesPerRev, minEdgeSep_s);

    % --- Put everything on a common frequency grid (use R34 as reference) ---
    f = out34.f(:);
    P3  = interp1(out3.f,  out3.PxxMean,  f, 'linear', 'extrap');
    P4  = interp1(out4.f,  out4.PxxMean,  f, 'linear', 'extrap');
    P34 = out34.PxxMean;

    % Guards
    P3  = max(P3,  eps);
    P4  = max(P4,  eps);
    P34 = max(P34, eps);

    % --- Expected (no interaction) in linear power ---
    Pexp = P3 + P4;

    % --- Convert to your "SPL" convention (dB of PSD + offset) ---
    L3   = 10*log10(P3)  + SPL_offset;
    L4   = 10*log10(P4)  + SPL_offset;
    L34  = 10*log10(P34) + SPL_offset;
    Lexp = 10*log10(Pexp)+ SPL_offset;

    % --- Interaction (dB ratio) ---
    Lint = 10*log10(P34 ./ Pexp);  % >0 means louder than expected

    % --- Plot mask ---
    mask = (f >= fmin) & (f <= min(fmax_plot, max(f)));

    % --- BPF markers using measured tach RPM (prefer pair RPM) ---
    rpm_use = out34.rpm_mean;
    frot = rpm_use/60;
    fbpf = Nb*frot;

    %% ================= FIGURE =================
    figure('Name', sprintf('%s comparison', cases(k).label), 'Color','w');

    % TOP: Spectra
    ax1 = subplot(2,1,1);
    hold(ax1,'on'); grid(ax1,'on');

    plot(ax1, f(mask), L3(mask),  'LineWidth', 1.2, 'DisplayName', sprintf('Rotor 3 (%.0f RPM)', out3.rpm_mean));
    plot(ax1, f(mask), L4(mask),  'LineWidth', 1.2, 'DisplayName', sprintf('Rotor 4 (%.0f RPM)', out4.rpm_mean));
    plot(ax1, f(mask), L34(mask), 'LineWidth', 1.4, 'DisplayName', sprintf('Rotor 3+4 (%.0f RPM)', out34.rpm_mean));
    plot(ax1, f(mask), Lexp(mask),'--',        'LineWidth', 1.4, 'DisplayName', 'Expected (P3+P4)');

    set(ax1,'XScale','log');
    xlabel(ax1,'Frequency (Hz)');
    ylabel(ax1,'SPL (dB, per your formula)');
    title(ax1, sprintf('%s: Mean spectra (mics %d–%d)', cases(k).label, micCh(1), micCh(end)));

    % Mark 1x and BPF
    yl = ylim(ax1);
    xline(ax1, frot, ':',  '1x',  'LabelVerticalAlignment','bottom');
    xline(ax1, fbpf, '--', 'BPF', 'LabelVerticalAlignment','bottom');
    ylim(ax1, yl);

    legend(ax1,'Location','best');

    % BOTTOM: Interaction
    ax2 = subplot(2,1,2);
    hold(ax2,'on'); grid(ax2,'on');

    plot(ax2, f(mask), Lint(mask), 'LineWidth', 1.3);
    yline(ax2, 0, '-k');

    set(ax2,'XScale','log');
    xlabel(ax2,'Frequency (Hz)');
    ylabel(ax2,'Interaction (dB): 10log10(P34/(P3+P4))');
    title(ax2,'Interaction spectrum (positive = louder than sum)');

    % Mark 1x and BPF again
    yl2 = ylim(ax2);
    xline(ax2, frot, ':',  '1x',  'LabelVerticalAlignment','bottom');
    xline(ax2, fbpf, '--', 'BPF', 'LabelVerticalAlignment','bottom');
    ylim(ax2, yl2);

    xlim(ax1, [fmin fmax_plot]);
    xlim(ax2, [fmin fmax_plot]);
end

disp("Done. Generated LOW/MID/HIGH comparison figures.");

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

    % --- RPM from tach (rising edges) ---
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

    % --- Mean linear PSD across chosen mics ---
    Pxx = [];
    f = [];

    for m = 1:numel(micCh)
        ch = micCh(m);
        [pxx,f] = pwelch(X(:,ch), hann(window,'periodic'), noverlap, nfft, Fs);
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
