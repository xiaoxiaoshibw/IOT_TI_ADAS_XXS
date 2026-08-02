function plot_navigation(csv_path, output_dir, can_metrics_path)
%PLOT_NAVIGATION 由真实 Phase 3 CSV 生成灰度论文图（MATLAB/Octave 风格）。
% 缺失列不插值、不补零；CAN 数据未提供时 Figure 5 仅显示 MCU CRC 计数。

if nargin < 3
    can_metrics_path = '';
end
if ~isfolder(output_dir)
    mkdir(output_dir);
end
T = readtable(csv_path);
set(groot, 'defaultAxesFontName', 'Microsoft YaHei');
set(groot, 'defaultAxesFontSize', 10);

f = figure('Color','w','Position',[100 100 720 480]);
plot(T.route_x,T.route_y,'--','Color',[.15 .15 .15]); hold on;
plot(T.vehicle_x,T.vehicle_y,'Color',[.5 .5 .5]); axis equal; grid on;
xlabel('X / m'); ylabel('Y / m'); title('Town04 全局路线跟踪');
legend('全局路线','车辆轨迹','Location','best');
exportgraphics(f,fullfile(output_dir,'figure1_global_route_tracking.png'),'Resolution',300); close(f);

f = figure('Color','w','Position',[100 100 720 400]);
plot(T.elapsed_s,T.lateral_error,'Color',[.2 .2 .2]); grid on;
xlabel('时间 / s'); ylabel('横向误差 e_y / m'); title('横向跟踪误差');
exportgraphics(f,fullfile(output_dir,'figure2_lateral_error.png'),'Resolution',300); close(f);

f = figure('Color','w','Position',[100 100 720 400]);
plot(T.elapsed_s,T.target_speed,'--','Color',[.15 .15 .15]); hold on;
plot(T.elapsed_s,T.velocity,'Color',[.5 .5 .5]); grid on;
xlabel('时间 / s'); ylabel('速度 / (m/s)'); title('速度跟踪');
legend('目标速度','实际速度','Location','best');
exportgraphics(f,fullfile(output_dir,'figure3_speed_tracking.png'),'Resolution',300); close(f);

f = figure('Color','w','Position',[100 100 720 400]);
stairs(T.elapsed_s,T.mcu_active_source,'Color',[.15 .15 .15]); hold on;
stairs(T.elapsed_s,T.mcu_state,'--','Color',[.6 .6 .6]); grid on;
xlabel('时间 / s'); ylabel('离散状态'); title('F280025C 安全接管时间轴');
legend('MCU 控制源','MCU 状态','Location','best');
exportgraphics(f,fullfile(output_dir,'figure4_mcu_takeover.png'),'Resolution',300); close(f);

f = figure('Color','w','Position',[100 100 720 400]); hold on;
labels = {};
if strlength(can_metrics_path) > 0 && isfile(can_metrics_path)
    C = readtable(can_metrics_path);
    t = C.time-C.time(1);
    stairs(t,C.drop,'Color',[.15 .15 .15]); labels{end+1}='丢帧计数';
    stairs(t,C.error,'--','Color',[.55 .55 .55]); labels{end+1}='CAN 错误';
end
stairs(T.elapsed_s,T.crc_error,':','Color',[.75 .75 .75]); labels{end+1}='MCU CRC 错误';
grid on; xlabel('时间 / s'); ylabel('累计计数'); title('CAN 可靠性');
legend(labels,'Location','best');
exportgraphics(f,fullfile(output_dir,'figure5_can_reliability.png'),'Resolution',300); close(f);
end

