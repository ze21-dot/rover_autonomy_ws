%% plot_matrix.m — SuRover KARASIMSEK Isaac 6 test matrix figures
% Reads runs/<run>/run.csv (60 Hz telemetry) and writes figures to runs/figures/.
% Usage (from repo root):   >> run('tools/rover_eval/plot_matrix.m')
% Requires MATLAB R2019b+ (readtable, tiledlayout). No toolboxes.

clear; clc; close all;
root = fileparts(fileparts(fileparts(mfilename('fullpath'))));   % repo root
runsDir = fullfile(root, 'runs');
outDir  = fullfile(runsDir, 'figures'); if ~exist(outDir,'dir'), mkdir(outDir); end

R = 0.145;                 % wheel radius [m]
DT = 1/60;                 % physics step [s]
RIDGE_X = [32 108];        % ridge section along course [m]

runs = {'A_rigid_7','B_passive_7','C_hybrid_7','speed_2','speed_13','speed_28','A_rigid_7_nolane','C_hybrid_13'};
lab  = containers.Map(runs, {'A: rigid','B: passive rocker','C: hybrid rocker', ...
                             '2 rad/s','13 rad/s','28 rad/s','A (lane off)','C @ 13 rad/s'});
T = struct();
for i = 1:numel(runs)
    f = fullfile(runsDir, runs{i}, 'run.csv');
    if ~isfile(f), warning('missing %s', f); continue; end
    T.(runs{i}) = readtable(f);
end
rms = @(v) sqrt(mean(v.^2));
col = struct('A',[0.55 0.55 0.55], 'B',[0.20 0.45 0.85], 'C',[0.85 0.30 0.20]);
set(groot,'defaultAxesFontSize',10,'defaultLineLineWidth',1.2);
savefig_ = @(h,name) (arrayfun(@(fmt) exportgraphics(h, fullfile(outDir,[name fmt{1}]), 'Resolution',200), {'.png','.pdf'}));

%% Fig 1 — Roll vs distance, A/B/C overlaid (ridge section shaded)
h = figure('Position',[100 100 900 380]); hold on; grid on;
patch([RIDGE_X(1) RIDGE_X(2) RIDGE_X(2) RIDGE_X(1)], [-15 -15 15 15], [0.95 0.95 0.95], 'EdgeColor','none');
plot(T.A_rigid_7.x,   T.A_rigid_7.roll,   'Color',col.A, 'DisplayName',lab('A_rigid_7'));
plot(T.B_passive_7.x, T.B_passive_7.roll, 'Color',col.B, 'DisplayName',lab('B_passive_7'));
plot(T.C_hybrid_7.x,  T.C_hybrid_7.roll,  'Color',col.C, 'DisplayName',lab('C_hybrid_7'));
xlabel('Course position x [m]'); ylabel('Chassis roll [deg]'); ylim([-15 15]);
title('Chassis roll — suspension ablation (7 rad/s, identical course)');
legend('Location','southwest'); savefig_(h,'fig1_roll_vs_x_ABC');

%% Fig 2 — Roll RMS / peak, A/B/C bars (ridge section only)
abc = {'A_rigid_7','B_passive_7','C_hybrid_7'};
rollRMS = zeros(1,3); rollMax = zeros(1,3); cmRMS = nan(1,3); artRMS = nan(1,3);
for k = 1:3
    t = T.(abc{k}); m = t.x > RIDGE_X(1) & t.x < RIDGE_X(2);
    rollRMS(k) = rms(t.roll(m)); rollMax(k) = max(abs(t.roll(m)));
    if any(t.qL ~= 0)
        cmRMS(k)  = rad2deg(rms(t.qL(m)+t.qR(m)));
        artRMS(k) = rad2deg(rms(t.qL(m)-t.qR(m)));
    end
end
h = figure('Position',[100 100 700 360]); tiledlayout(1,2,'Padding','compact');
nexttile; b = bar([rollRMS; rollMax]'); b(1).FaceColor=[0.3 0.3 0.3]; b(2).FaceColor=[0.75 0.75 0.75];
set(gca,'XTickLabel',{'A rigid','B passive','C hybrid'}); ylabel('deg'); grid on;
legend({'roll RMS','roll peak'},'Location','northeast'); title('Chassis roll (ridge section)');
text(1:3, rollRMS+0.3, compose('%.2f',rollRMS), 'HorizontalAlignment','center','FontSize',9);
nexttile; b = bar([cmRMS(2:3); artRMS(2:3)]'); b(1).FaceColor=col.C; b(2).FaceColor=col.B;
set(gca,'XTickLabel',{'B passive','C hybrid'}); ylabel('deg'); grid on;
legend({'common-mode  q_L+q_R','articulation  q_L−q_R'},'Location','northwest');
title(sprintf('Rocker: common-mode ↓%.1f×, articulation preserved', cmRMS(2)/cmRMS(3)));
savefig_(h,'fig2_ABC_bars');

%% Fig 3 — Rocker joints time series, B vs C (common-mode suppression, articulation preserved)
h = figure('Position',[100 100 900 460]); tiledlayout(2,1,'Padding','compact');
for k = 2:3
    t = T.(abc{k}); nexttile; hold on; grid on;
    plot(t.x, rad2deg(t.qL+t.qR), 'Color',col.C, 'DisplayName','q_L+q_R (common-mode)');
    plot(t.x, rad2deg(t.qL-t.qR), 'Color',col.B, 'DisplayName','q_L−q_R (articulation)');
    yline( rad2deg(0.5),'k:'); yline(-rad2deg(0.5),'k:');
    ylabel('deg'); ylim([-70 70]); title(lab(abc{k})); legend('Location','southwest');
end
xlabel('Course position x [m]'); savefig_(h,'fig3_rocker_modes_BC');

%% Fig 4 — Speed sweep (rigid): roll RMS, slip, distance vs speed; rollover marked
sp = {'speed_2','A_rigid_7','speed_13','speed_28'}; w = [2 7 13 28]; v = w*R;
rr = zeros(1,4); sl = zeros(1,4); dd = zeros(1,4); ro = false(1,4);
for k = 1:4
    t = T.(sp{k}); m = t.x > RIDGE_X(1) & t.x < RIDGE_X(2); if ~any(m), m = true(height(t),1); end
    rr(k) = rms(t.roll(m)); dd(k) = t.x(end)-t.x(1); ro(k) = any(abs(t.roll)>45 | abs(t.pitch)>45);
    vx = diff(t.x)/DT; wm = abs(t.wheel_mean(1:end-1))*R; g = wm>0.2;
    sl(k) = 100*mean(1 - abs(vx(g))./wm(g));
end
h = figure('Position',[100 100 900 340]); tiledlayout(1,3,'Padding','compact');
nexttile; plot(v,rr,'-o','Color',col.A,'MarkerFaceColor',col.A); hold on; grid on;
if any(ro), plot(v(ro),rr(ro),'rx','MarkerSize',12,'LineWidth',2); end
xlabel('Speed [m/s]'); ylabel('roll RMS [deg]'); title('Roll vs speed (rigid)');
nexttile; plot(v,sl,'-o','Color',col.B,'MarkerFaceColor',col.B); grid on; hold on;
if any(ro), plot(v(ro),sl(ro),'rx','MarkerSize',12,'LineWidth',2); end
xlabel('Speed [m/s]'); ylabel('slip [%]'); title('Wheel slip vs speed');
nexttile; plot(v,dd,'-o','Color',col.C,'MarkerFaceColor',col.C); grid on; hold on;
if any(ro), plot(v(ro),dd(ro),'rx','MarkerSize',12,'LineWidth',2); end
xlabel('Speed [m/s]'); ylabel('distance in 60 s [m]'); title('Traverse (× = rollover)');
savefig_(h,'fig4_speed_sweep');

%% Fig 5 — Lane keeping: lateral position A vs A(nolane)
h = figure('Position',[100 100 900 300]); hold on; grid on;
plot(T.A_rigid_7.x,        T.A_rigid_7.y,        'Color',col.B, 'DisplayName','lane keeping on');
plot(T.A_rigid_7_nolane.x, T.A_rigid_7_nolane.y, 'Color',col.C, 'DisplayName','lane keeping off');
xlabel('x [m]'); ylabel('lateral y [m]'); title('Lane keeping — lateral drift');
legend('Location','northwest'); savefig_(h,'fig5_lane_drift');

%% Fig 6 — Rollover event at 28 rad/s: pitch & z vs time
t = T.speed_28;
h = figure('Position',[100 100 900 340]); yyaxis left; plot(t.t, t.pitch,'Color',col.C); ylabel('pitch [deg]');
yyaxis right; plot(t.t, t.z,'Color',col.B); ylabel('chassis z [m]'); grid on;
xlabel('t [s]'); title('speed\_28: nose-in pitch-over (4.1 m/s)'); savefig_(h,'fig6_rollover_28');

%% Cross-check table (independent of analyze.py)
fprintf('\n%-18s %8s %8s %10s %10s\n','run','rollRMS','rollMax','cmRMS','artRMS');
for k = 1:3, fprintf('%-18s %8.2f %8.1f %10.2f %10.2f\n', abc{k}, rollRMS(k), rollMax(k), cmRMS(k), artRMS(k)); end
fprintf('\nFigures written to %s\n', outDir);
