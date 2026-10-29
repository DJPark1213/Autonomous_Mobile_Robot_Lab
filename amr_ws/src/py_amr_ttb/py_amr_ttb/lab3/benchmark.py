"""Reproducible, matched-limit comparison; all results are ideal simulations."""

import argparse
import csv
from dataclasses import asdict
import json
from pathlib import Path

from .core import Config
from .simulate import SCENARIOS, run_scenario


EXTRA_SCENARIOS = {
    'small_box': [(.75, -.10, .95, .10)],
    'near_box': [(.52, -.16, .82, .16)],
    'far_box': [(.85, -.20, 1.10, .20)],
}
CHALLENGES = {
    'two_staggered': [(.50, .02, .75, .25), (1., -.28, 1.20, -.05)],
}


def main(args=None):
    """Compare the same geometry, speed bounds, footprint and scan rate."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', default='lab3_comparison')
    parser.add_argument('--noise-seeds', type=int, default=3)
    opts = parser.parse_args(args)
    if opts.noise_seeds < 0:
        parser.error('--noise-seeds must be nonnegative')
    output = Path(opts.output)
    output.mkdir(parents=True, exist_ok=True)
    cases = {**SCENARIOS, **EXTRA_SCENARIOS, **CHALLENGES}
    metrics, histories = [], {}
    for name, boxes in cases.items():
        for planner in ('lecture', 'potential'):
            config = Config(planner=planner)
            metric, rows = run_scenario(name, boxes, config)
            metrics.append(metric)
            histories[name, planner] = rows
            folder = output/planner
            folder.mkdir(exist_ok=True)
            with (folder/f'{name}.csv').open('w', newline='') as stream:
                writer = csv.writer(stream)
                writer.writerow(('time_s', 'x_m', 'y_m', 'yaw_rad', 'v_m_s', 'w_rad_s', 'state'))
                writer.writerows(rows)
    noisy = []
    # The same noise model/seed for both controllers; no claim of identical beams
    # after their trajectories diverge. Noise is applied only on new 10-Hz scans.
    for seed in range(opts.noise_seeds):
        for name, boxes in {**SCENARIOS, **EXTRA_SCENARIOS}.items():
            if name == 'clear':
                continue
            for planner in ('lecture', 'potential'):
                metric, _ = run_scenario(name, boxes, Config(planner=planner),
                                          scan_noise=.005, seed=seed)
                noisy.append(metric)
    configuration = asdict(Config())
    configuration.pop('planner')
    result = {
        'kind': 'ideal_2d_simulation_NOT_physical_robot_data',
        'shared_configuration': configuration,
        'control_hz': 20, 'scan_hz': 10,
        'clean': metrics, 'noise_5mm': noisy,
        'limitations': [
            'Ideal unicycle and odometry; no motor dynamics, slip or network model.',
            'Both controllers keep the same velocity bounds and footprint margin.',
            'Actual minimum clearance is measured independently from exact rectangles.',
            'Two-staggered challenge is included even when neither controller completes.',
        ],
    }
    (output/'metrics.json').write_text(json.dumps(result, indent=2)+'\n')
    lookup = {(m['scenario'], m['algorithm']): m for m in metrics}
    table = []
    for name in cases:
        old, new = lookup[name, 'lecture'], lookup[name, 'potential']
        faster = 100*(old['elapsed_s']-new['elapsed_s'])/old['elapsed_s'] if old['reached'] and new['reached'] else None
        table.append((name, old, new, faster))
    with (output/'comparison.csv').open('w', newline='') as stream:
        writer = csv.writer(stream)
        writer.writerow(('scenario', 'lecture_reached', 'potential_reached', 'lecture_s',
                         'potential_s', 'time_reduction_percent', 'lecture_body_clearance_m',
                         'potential_body_clearance_m', 'potential_fallback_count'))
        writer.writerows((name, a['reached'], b['reached'], a['elapsed_s'], b['elapsed_s'],
                          '' if faster is None else faster, a['minimum_body_clearance_m'],
                          b['minimum_body_clearance_m'], b['fallback_count'])
                         for name, a, b, faster in table)
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    from matplotlib.patches import Rectangle
    fig, axes = plt.subplots(1, 4, figsize=(16, 4.5), constrained_layout=True)
    for ax, name in zip(axes, ('center_box', 'left_box', 'right_box', 'wide_box')):
        for planner, color in (('lecture', '#7b8191'), ('potential', '#087f8c')):
            rows = histories[name, planner]
            ax.plot([r[1] for r in rows], [r[2] for r in rows], color=color,
                    label=f'{planner}: {lookup[name, planner]["elapsed_s"]:.2f}s', lw=2)
        for x1, y1, x2, y2 in cases[name]:
            ax.add_patch(Rectangle((x1, y1), x2-x1, y2-y1, color='#ad7f51'))
        ax.scatter([1.5], [0], marker='*', s=100, color='#df6e31')
        ax.set(title=name, xlabel='x (m)', ylabel='y (m)', xlim=(-.15, 1.85), ylim=(-.55, 1.05))
        ax.set_aspect('equal')
        ax.grid(alpha=.2)
        ax.legend(fontsize=8, loc='upper left')
    fig.suptitle('Same v/w limits and footprint | 10-Hz lidar | Ideal simulation, NOT robot measurements')
    fig.savefig(output/'comparison.png', dpi=180)
    plt.close(fig)
    report = [
        '# Lab 3 同条件仿真比较', '',
        '**全部结果为理想二维仿真，不是实体机器人测量。**', '',
        '两种方法共用：v_max=0.18 m/s、w_max=0.70 rad/s、20 Hz 控制、10 Hz 雷达、'
        '0.19 m 车体半径及 0.08 m 安全余量。旧版使用 Lecture 5 沿墙状态机；'
        '新版使用势场、固定切向绕行与沿墙脱困。未通过提高速度上限取得时间改善。', '',
        '| 场景 | 旧版时间 s | 新版时间 s | 时间缩短 | 旧/新最小车体净空 m | 到达情况 |',
        '| --- | ---: | ---: | ---: | --- | --- |',
    ]
    for name, a, b, faster in table:
        improvement = '-' if faster is None else f'{faster:.1f}%'
        reached = f'{"到达" if a["reached"] else a["state"]} / {"到达" if b["reached"] else b["state"]}'
        report.append(f'| {name} | {a["elapsed_s"]:.2f} | {b["elapsed_s"]:.2f} | {improvement} | '
                      f'{a["minimum_body_clearance_m"]:.3f} / {b["minimum_body_clearance_m"]:.3f} | {reached} |')
    report += ['', '## 测距噪声检查', '',
               f'对 7 个单箱场景分别使用 {opts.noise_seeds} 个随机种子，加入标准差 5 mm 的测距噪声。', '']
    for planner in ('lecture', 'potential'):
        selected = [m for m in noisy if m['algorithm'] == planner]
        passed = sum(m['reached'] and m['minimum_body_clearance_m'] > Config().safety_margin for m in selected)
        report.append(f'- {planner}：{passed}/{len(selected)} 次到点且净空大于设定安全余量。')
    report += ['', '## 局限', '',
               '交错双障碍挑战中两种控制器都可能以 NO_PROGRESS 停车，不能据此声称完成任意障碍赛道。'
               '新版保持了停车保护，但没有全局路径搜索。', '',
               '车辆动力学、轮胎打滑、实际制动距离和网络延迟未被模拟。实机速度必须在实验室验证。', '',
               '具体轨迹、失败场景、随机种子和全部参数见 metrics.json 与两个控制器的 CSV。']
    (output/'REPORT.md').write_text('\n'.join(report)+'\n')
    for name, a, b, faster in table:
        print(name, a['elapsed_s'], b['elapsed_s'], faster, b['reached'], flush=True)
    print('Noisy completions:', {p: sum(m['reached'] for m in noisy if m['algorithm'] == p)
                                  for p in ('lecture', 'potential')}, flush=True)


if __name__ == '__main__':
    main()
