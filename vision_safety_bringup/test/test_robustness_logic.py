"""Unit tests for the robustness study's pure logic (no ROS, no Gazebo).

If scenario sampling, world generation or the pass/fail judgement were wrong,
the study would produce confident-looking but meaningless numbers.
"""

import os
import random
import sys
import xml.etree.ElementTree as ET

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'scripts'))

import robustness_study as rs  # noqa: E402

TEMPLATE = open(os.path.join(
    HERE, '..', '..', 'vision_safety_sim', 'worlds', 'corridor.sdf')).read()


@pytest.fixture(scope='module')
def scenarios():
    rng = random.Random(1)
    return [rs.sample_scenario(rng, [0.2, 0.3, 0.5]) for _ in range(2000)]


def test_classification_boundaries():
    assert rs.classify(0.0, 0.3) == 'in_path'          # straddles the centre line
    assert rs.classify(0.25, 0.3) == 'in_path'         # inner edge 0.10
    assert rs.classify(0.30, 0.3) == 'edge'            # inner edge 0.15
    assert rs.classify(-0.50, 0.3) == 'clear'          # inner edge 0.35, other side
    assert rs.inner_edge(0.05, 0.2) == 0.0


def test_sampled_scenarios_match_their_category(scenarios):
    for s in scenarios:
        assert rs.classify(s['lateral_m'], s['size_m']) == s['category']


def test_sampled_boxes_fit_in_the_corridor_and_avoid_the_blue_box(scenarios):
    for s in scenarios:
        half = s['size_m'] / 2
        assert abs(s['lateral_m']) + half <= rs.CORRIDOR_INNER + 1e-9
        assert 0.10 <= s['size_m'] <= 0.40
        assert 1.5 <= s['initial_gap_m'] <= 3.5
        assert not rs._overlaps_blue_box(
            s['box_front_x'], s['box_front_x'] + s['size_m'],
            s['lateral_m'] - half, s['lateral_m'] + half)


def test_in_path_boxes_overlap_the_robot_body(scenarios):
    for s in scenarios:
        if s['category'] == 'in_path':
            assert s['inner_edge_m'] < rs.ROBOT_HALF_WIDTH
        if s['category'] == 'clear':
            assert s['inner_edge_m'] > rs.PATH_HALF_WIDTH + 0.05


def test_category_mix_and_both_sides_are_covered(scenarios):
    counts = {c: sum(1 for s in scenarios if s['category'] == c)
              for c in ('in_path', 'clear', 'edge')}
    assert 0.40 < counts['in_path'] / len(scenarios) < 0.60
    assert counts['clear'] > 300 and counts['edge'] > 200
    clear = [s for s in scenarios if s['category'] == 'clear']
    assert any(s['lateral_m'] > 0 for s in clear) and any(s['lateral_m'] < 0 for s in clear)


def test_sampling_is_reproducible_with_a_seed():
    a = [rs.sample_scenario(random.Random(7), [0.3]) for _ in range(5)]
    b = [rs.sample_scenario(random.Random(7), [0.3]) for _ in range(5)]
    assert a == b


def test_generated_world_replaces_only_the_red_box():
    sc = rs.sample_scenario(random.Random(3), [0.3])
    world = ET.fromstring(rs.make_world(TEMPLATE, sc))
    models = {m.get('name'): m for m in world.iter('model')}
    assert {'ground_plane', 'corridor_walls', 'red_patch', 'blue_box', 'red_obstacle'} <= set(models)
    assert len([m for m in world.iter('model') if m.get('name') == 'red_obstacle']) == 1

    box = models['red_obstacle']
    x, y, z = (float(v) for v in box.find('pose').text.split()[:3])
    assert x == pytest.approx(sc['box_center_x'])
    assert y == pytest.approx(sc['lateral_m'])
    assert z == pytest.approx(sc['size_m'] / 2)          # resting on the floor
    sizes = [float(v) for v in box.find('.//collision//size').text.split()]
    assert sizes == pytest.approx([sc['size_m']] * 3)


def test_make_world_rejects_a_template_without_the_box():
    with pytest.raises(ValueError):
        rs.make_world('<sdf><world name="x"></world></sdf>', {'size_m': 0.2})


@pytest.mark.parametrize('category,outcome,gap,stopped,expected', [
    ('in_path', 'stopped', 0.55, True, True),
    ('in_path', 'stopped', 0.10, True, False),     # too close
    ('in_path', 'stopped', 0.00, True, False),     # contact
    ('in_path', 'passed', None, False, False),     # drove through
    ('clear', 'passed', None, False, True),
    ('clear', 'stopped', 0.5, True, False),        # false stop
    ('edge', 'stopped', 0.5, True, True),
    ('edge', 'passed', None, False, True),
    ('in_path', 'no_perception', None, False, None),
    ('clear', 'timeout', None, False, None),
])
def test_evaluate(category, outcome, gap, stopped, expected):
    passed, note = rs.evaluate({'category': category}, outcome, gap, stopped)
    assert passed is expected
    assert note


HEALTHY = {'clock': 900, 'detections': 400, 'status': 600, 'last_state': 1,
           'cmd_raw': 600, 'cmd_out': 600, 'last_cmd_linear': 0.3, 'moved_m': 2.0}


@pytest.mark.parametrize('override,expected', [
    ({'clock': 0}, 'no /clock'),
    ({'detections': 0}, 'no /perception/obstacle'),
    ({'status': 0}, 'safety_monitor silent'),
    ({'last_state': 0}, 'stuck in INIT'),
    ({'last_state': 3}, 'STOP_STALE'),
    ({'cmd_raw': 0}, 'cruise_driver not publishing'),
    ({'last_cmd_linear': 0.0}, 'commanding zero'),
    ({'moved_m': 0.0}, 'did not move'),
    ({}, 'did not finish in time'),
])
def test_diagnose_names_the_first_broken_link(override, expected):
    assert expected in rs.diagnose(dict(HEALTHY, **override))


def _fake_proc(tmp_path, pid, cmdline, partition=None):
    d = tmp_path / str(pid)
    d.mkdir()
    (d / 'cmdline').write_bytes(b'\0'.join(p.encode() for p in cmdline.split(' ')) + b'\0')
    env = [b'HOME=/home/x'] + ([f'GZ_PARTITION={partition}'.encode()] if partition else [])
    (d / 'environ').write_bytes(b'\0'.join(env) + b'\0')


def test_finds_gazebo_servers_with_their_partition(tmp_path):
    _fake_proc(tmp_path, 101, 'gz sim -s -r corridor.sdf', 'vision_safety_test')
    _fake_proc(tmp_path, 102, 'ruby /opt/gz sim -s world.sdf', 'vision_safety')
    _fake_proc(tmp_path, 103, 'python3 something_else.py', 'vs_robust_1_2')
    (tmp_path / 'self').mkdir()          # non-numeric entries are ignored
    found = sorted(rs.find_gz_servers(str(tmp_path)))
    assert [(p, part) for p, part, _ in found] == [(101, 'vision_safety_test'), (102, 'vision_safety')]


def test_kills_only_test_leftovers_never_the_users_simulation(tmp_path):
    _fake_proc(tmp_path, 201, 'gz sim -s a.sdf', 'vision_safety_test')   # sim test orphan
    _fake_proc(tmp_path, 202, 'gz sim -s b.sdf', 'vs_robust_77_3')        # study orphan
    _fake_proc(tmp_path, 203, 'gz sim c.sdf', 'vision_safety')            # user's demo
    _fake_proc(tmp_path, 204, 'gz sim d.sdf')                             # no partition
    signalled = []
    killed = rs.kill_test_leftovers(str(tmp_path), kill=lambda pid, sig: signalled.append(pid))
    assert sorted(killed) == [201, 202] == sorted(signalled)


def test_restarts_are_always_reported():
    class Args:
        seed, speeds = 1, [0.3]
    base = {'category': 'clear', 'outcome': 'passed', 'passed': True, 'note': 'n',
            'speed_mps': 0.3, 'final_gap_m': None, 'travel_after_trigger_m': None,
            'size_m': 0.2, 'lateral_m': 0.5}
    s = rs.summarize([dict(base, run=0, attempts=2), dict(base, run=1, attempts=1)], [])
    assert s['runs_retried'] == 1
    assert '1 run(s) needed a second attempt' in rs.summary_markdown(s, Args)
    s = rs.summarize([dict(base, run=0, attempts=1)], [])
    assert 'No run needed a simulator restart' in rs.summary_markdown(s, Args)


def test_summary_and_markdown_from_synthetic_runs():
    rows = []
    for i, (cat, outcome, passed, speed, gap) in enumerate([
            ('in_path', 'stopped', True, 0.2, 0.58), ('in_path', 'stopped', True, 0.5, 0.50),
            ('clear', 'passed', True, 0.3, None), ('edge', 'stopped', True, 0.3, 0.55),
            ('in_path', 'passed', False, 0.3, None), ('clear', 'timeout', None, 0.2, None)]):
        rows.append({'run': i, 'category': cat, 'outcome': outcome, 'passed': passed,
                     'note': 'n', 'speed_mps': speed, 'final_gap_m': gap,
                     'travel_after_trigger_m': 0.03 if gap else None,
                     'size_m': 0.2, 'lateral_m': 0.0})
    errors = [{'run': 0, 'true_gap_m': g, 'error_m': e, 'size_m': 0.2}
              for g, e in [(0.5, 0.01), (1.0, -0.02), (2.0, 0.05)]]
    s = rs.summarize(rows, errors)
    assert s['runs_judged'] == 5 and s['infrastructure_failures'] == 1
    assert s['correct_total'] == 4
    assert s['by_category']['in_path'] == {'runs': 3, 'correct': 2, 'stopped': 2}
    assert set(s['stops_by_speed']) == {'0.2', '0.5'}
    assert len(s['failures']) == 1

    class Args:
        seed, speeds = 42, [0.2, 0.3, 0.5]
    md = rs.summary_markdown(s, Args)
    assert '**Correct decisions: 4 / 5**' in md
    assert 'did not stop' not in md or 'Failures' in md
