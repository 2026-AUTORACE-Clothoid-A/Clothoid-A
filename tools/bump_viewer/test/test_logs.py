from tools.bump_viewer.logs import read_file, merge, entry


def test_parser_preserves_untimed_lines_and_tracebacks(tmp_path):
    path = tmp_path/'run.log'
    path.write_text('[INFO] [launch]: starting\n[node-1] [WARN] [100.500] [mission]: waiting\n'
                    '[node-1] Traceback (most recent call last):\n  File "x.py", line 3\n\n')
    rows = read_file(path, 100_000_000_000)
    assert len(rows) == 5
    assert rows[1]['t'] == .5 and rows[1]['node'] == 'mission'
    assert rows[1]['level'] == 'WARN'
    assert rows[0]['t'] is None and rows[2]['t'] is None
    assert rows[3]['raw'] == '  File "x.py", line 3'
    assert rows[4]['raw'] == ''


def test_merge_only_cross_source_matching_records():
    file = entry(.5, '실제 파일', 'mission', 'INFO', 'armed', 'file original')
    ros = entry(.501, '실제 /rosout', 'mission', 'INFO', 'armed', 'ros original')
    later = entry(.6, '실제 /rosout', 'mission', 'INFO', 'armed')
    replay = entry(.5, '현재 재계산', 'mission', 'INFO', 'armed')
    rows = merge([file, ros, later, replay])
    assert len(rows) == 3
    merged = next(r for r in rows if 'rosout_raw' in r)
    assert merged['raw'] == 'file original' and merged['rosout_raw'] == 'ros original'
    assert file['source'] == '실제 파일'  # input remains reusable when switching files


def test_log_timestamps_before_bag_are_preserved(tmp_path):
    path = tmp_path/'run.log'
    path.write_text('[INFO] [99.0] [camera]: start\n[ERROR] [launch]: process exited\n')
    rows = read_file(path, 100_000_000_000)
    assert rows[0]['t'] == -1
    assert rows[1]['t'] is None and rows[1]['level'] == 'ERROR'
