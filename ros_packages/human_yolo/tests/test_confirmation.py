from human_yolo.confirmation import DetectionConfirmation

B = [10, 10, 100, 200]

def test_one_frame_and_repeated_stamp_never_confirm():
    gate = DetectionConfirmation()
    for _ in range(20):
        assert not gate.update(10., B)

def test_confirmation_needs_duration_and_frame_count():
    gate = DetectionConfirmation()
    for t in [10., 10.1, 10.2, 10.3, 10.4]:
        assert not gate.update(t, B)
    assert gate.update(10.7, B)

def test_miss_gap_target_change_and_reset_require_new_sequence():
    for event in ['miss', 'gap', 'target', 'reset']:
        gate = DetectionConfirmation()
        for t in [10., 10.2, 10.4, 10.6]:
            assert not gate.update(t, B)
        if event == 'miss': assert not gate.update(10.7, None)
        if event == 'gap': assert not gate.update(11.5, B)
        if event == 'target': assert not gate.update(10.8, [300, 10, 400, 200])
        if event == 'reset': gate.reset()
        assert not gate.update(11.6, B)
