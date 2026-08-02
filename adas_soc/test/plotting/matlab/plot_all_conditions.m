% Generate report-ready PNGs from SIL recorder CSV files.
script_dir = fileparts(mfilename('fullpath'));
soc_root = fileparts(fileparts(fileparts(script_dir)));
data_dir = fullfile(soc_root, 'test', 'plotting', 'data');
out_dir = fullfile(soc_root, 'test', 'plotting', 'output');
if ~exist(out_dir, 'dir'), mkdir(out_dir); end

set(groot, 'defaultFigureColor', 'w');
set(groot, 'defaultAxesFontName', 'Arial');
set(groot, 'defaultAxesFontSize', 10);

base = readtable(fullfile(data_dir, 'sil_baseline.csv'));
acc = readtable(fullfile(data_dir, 'sil_acc.csv'));
aeb = readtable(fullfile(data_dir, 'sil_aeb.csv'));
overtake = readtable(fullfile(data_dir, 'sil_overtake.csv'));
redundant = readtable(fullfile(data_dir, 'sil_redundant.csv'));
lqr = readtable(fullfile(data_dir, 'sil_lqr.csv'));

plot_baseline(base, fullfile(out_dir, '01_lka_baseline.png'));
plot_acc(acc, fullfile(out_dir, '02_acc_follow_stop_restart.png'));
plot_aeb(aeb, fullfile(out_dir, '03_aeb_pedestrian.png'));
plot_overtake(overtake, fullfile(out_dir, '04_overtake.png'));
plot_redundancy(redundant, fullfile(out_dir, '05_redundant_takeover.png'));
plot_lqr(base, lqr, fullfile(out_dir, '06_lqr_vs_pure_pursuit.png'));

function prepare_figure(name)
    figure('Name', name, 'Position', [100 100 1200 760], 'Visible', 'off');
end

function finish_figure(path)
    exportgraphics(gcf, path, 'Resolution', 220);
    close(gcf);
end

function plot_baseline(t, path)
    prepare_figure('LKA baseline');
    tiledlayout(3,1, 'TileSpacing', 'compact');
    nexttile; plot(t.t_s, t.lateral_offset_m, 'LineWidth', 1.4); yline(0, ':k');
    grid on; ylabel('Lateral offset (m)'); title('LKA baseline: lane convergence and fail-safe stop');
    nexttile; plot(t.t_s, t.speed_mps, 'LineWidth', 1.4); grid on; ylabel('Speed (m/s)');
    nexttile; stairs(t.t_s, t.gate_source, 'LineWidth', 1.2); grid on;
    yticks([0 1 2]); yticklabels({'Follower','AEB','Builtin stop'}); ylabel('Gate source'); xlabel('Time (s)');
    finish_figure(path);
end

function plot_acc(t, path)
    prepare_figure('ACC');
    tiledlayout(3,1, 'TileSpacing', 'compact');
    nexttile; plot(t.t_s, t.speed_mps, 'LineWidth', 1.4); grid on; ylabel('Ego speed (m/s)');
    title('ACC: follow, standstill and restart');
    nexttile; plot(t.t_s, t.lead_gap_m, 'LineWidth', 1.4); grid on; ylabel('Lead gap (m)');
    nexttile; stairs(t.t_s, t.behavior_state, 'LineWidth', 1.2); grid on;
    yticks(0:6); ylabel('Behavior state'); xlabel('Time (s)');
    finish_figure(path);
end

function plot_aeb(t, path)
    prepare_figure('AEB');
    tiledlayout(3,1, 'TileSpacing', 'compact');
    nexttile; plot(t.t_s, t.speed_mps, 'LineWidth', 1.4); grid on; ylabel('Speed (m/s)');
    title('AEB pedestrian crossing: trigger, braking and release');
    nexttile; plot(t.t_s, t.aeb_ttc_s, 'LineWidth', 1.4); ylim([0 5]); grid on; ylabel('TTC (s)');
    nexttile; stairs(t.t_s, t.aeb_state, 'LineWidth', 1.2); grid on;
    yticks(0:3); yticklabels({'Inactive','Monitoring','Warning','Emergency'}); ylabel('AEB state'); xlabel('Time (s)');
    finish_figure(path);
end

function plot_overtake(t, path)
    prepare_figure('Overtake');
    tiledlayout(3,1, 'TileSpacing', 'compact');
    nexttile; plot(t.t_s, t.lateral_offset_m, 'LineWidth', 1.4); yline(0, ':k'); yline(3.5, ':r');
    grid on; ylabel('Lateral offset (m)'); title('Overtake: lane change, pass and return');
    nexttile; stairs(t.t_s, t.behavior_state, 'LineWidth', 1.2); grid on; ylabel('Behavior state');
    nexttile; yyaxis left; plot(t.t_s, t.speed_mps, 'LineWidth', 1.4); ylabel('Speed (m/s)');
    yyaxis right; plot(t.t_s, t.lead_gap_m, 'LineWidth', 1.2); ylabel('Lead gap (m)'); grid on; xlabel('Time (s)');
    finish_figure(path);
end

function plot_redundancy(t, path)
    prepare_figure('Redundancy');
    tiledlayout(2,1, 'TileSpacing', 'compact');
    nexttile; plot(t.t_s, t.speed_mps, 'LineWidth', 1.4); grid on; ylabel('Ego speed (m/s)');
    title('Dual-stack redundancy: primary failure with backup continuity');
    nexttile; stairs(t.t_s, t.primary_gate_source, 'LineWidth', 1.2); hold on;
    stairs(t.t_s, t.backup_gate_source, 'LineWidth', 1.2); stairs(t.t_s, t.gate_source, '--k', 'LineWidth', 1.2);
    grid on; yticks([0 1 2]); yticklabels({'Follower','AEB','Builtin stop'});
    ylabel('Gate source'); xlabel('Time (s)');
    legend('Primary stack gate','Backup stack gate','Global gate-status topic', 'Location', 'best');
    finish_figure(path);
end

function plot_lqr(base, lqr, path)
    prepare_figure('LQR comparison');
    tiledlayout(2,1, 'TileSpacing', 'compact');
    nexttile; plot(base.t_s, base.lateral_offset_m, 'LineWidth', 1.3); hold on;
    plot(lqr.t_s, lqr.lateral_offset_m, 'LineWidth', 1.3); yline(0, ':k'); grid on;
    ylabel('Lateral offset (m)'); title('LQR versus Pure Pursuit on the same SIL route');
    legend('Pure Pursuit','LQR', 'Location', 'best');
    nexttile; plot(base.t_s, base.speed_mps, 'LineWidth', 1.3); hold on;
    plot(lqr.t_s, lqr.speed_mps, 'LineWidth', 1.3); grid on;
    ylabel('Speed (m/s)'); xlabel('Time (s)'); legend('Pure Pursuit','LQR', 'Location', 'best');
    finish_figure(path);
end
