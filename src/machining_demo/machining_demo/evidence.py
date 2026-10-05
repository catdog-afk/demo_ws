"""不依赖 ROS 的 A2 验收规则，供记录器及集成测试复用。"""
import csv

REQUIRED_STAGES = ('PICK', 'PLACE', 'WAIT_MACHINING', 'RETRIEVE', 'RETURN', 'DONE')


def cycle_errors(stages):
    """核心阶段必须按顺序完成，允许工位等待及加工超时提示。"""
    compact = []
    for stage in stages:
        if not compact or compact[-1] != stage:
            compact.append(stage)
    if 'ABORTED' in compact:
        return ['任务中止']
    core = [stage for stage in compact if stage in REQUIRED_STAGES]
    # TIMEOUT 后重新进入 WAIT_MACHINING 是同一个加工阶段。
    core = [stage for index, stage in enumerate(core)
            if index == 0 or stage != core[index - 1]]
    if tuple(core) != REQUIRED_STAGES:
        return ['核心阶段不完整或顺序错误: %s' % ' -> '.join(compact)]
    return []


def log_errors(path, slot, expected_final='DONE'):
    with open(path, newline='', encoding='utf-8') as stream:
        rows = list(csv.DictReader(stream))
    task_rows = [row for row in rows if row.get('source') == 'task']
    if not task_rows:
        return ['缺少任务记录']
    if any(row.get('slot') != str(slot) for row in task_rows):
        return ['记录槽位与本次任务不一致']
    if task_rows[-1].get('event') != expected_final:
        return ['记录结束状态与预期不一致']
    if expected_final == 'ABORTED':
        return []
    return cycle_errors([row.get('event', '') for row in task_rows])
