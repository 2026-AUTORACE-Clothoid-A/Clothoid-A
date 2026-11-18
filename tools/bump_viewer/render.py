"""Diagnostic panels; the white and yellow masks come from production detectors."""
import cv2
import numpy as np

from .backend import read_image


def text(image, value, position=(8, 20), color=(235, 240, 245)):
    cv2.putText(image, value, position, cv2.FONT_HERSHEY_SIMPLEX, .45, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(image, value, position, cv2.FONT_HERSHEY_SIMPLEX, .45, color, 1, cv2.LINE_AA)


def panels(data, frame, hsv_settings=None, white_source='mission', color_source='lane'):
    from clothoid_lane.lane_detector import make_warp_matrix
    from clothoid_lane.lane_main import make_detector
    from clothoid_mission.line_stop import LineStopCore, Phase
    from clothoid_common.color import ColorPipeline, color_params
    import json

    raw = read_image(frame['reference'])
    camera = raw.copy()
    warp = data['warp']
    matrix = make_warp_matrix(warp)
    corners = np.array([[[0., 0.], [warp.warp_w-1., 0.],
                         [warp.warp_w-1., warp.warp_h-1.], [0., warp.warp_h-1.]]], np.float32)
    source = cv2.perspectiveTransform(corners, np.linalg.inv(matrix)).astype(np.int32)
    cv2.polylines(camera, source, True, (240, 200, 80), 2, cv2.LINE_AA)
    fresh = frame['detection_age'] is not None and frame['detection_age'] <= data['params']['input_timeout']
    for row in frame['detections']:
        x0, y0, x1, y1 = map(int, row['box'])
        color = (120, 120, 120) if not fresh else (
            (100, 235, 100) if row['status'] == '확정' else (40, 190, 250))
        cv2.rectangle(camera, (x0, y0), (x1, y1), color, 2)
        text(camera, f'{row["name"]} {row["confidence"]:.2f} '
             f'{row["hits"]}/{row["required"]}', (max(0, x0), max(15, y0-5)), color)

    warped = cv2.warpPerspective(cv2.GaussianBlur(raw, (5, 5), 0), matrix,
                                (warp.warp_w, warp.warp_h))
    settings = hsv_settings or data['hsv_applied']
    preview = settings != data['hsv_applied']
    values = {**data['detector_values'], **settings['lane']}
    mask_key = json.dumps(settings['lane'], sort_keys=True)
    if data.get('_masker_hsv') != mask_key:
        data['_masker'] = make_detector(values.__getitem__)
        data['_masker_hsv'] = mask_key
    masker = data['_masker']
    if hasattr(masker, '_masks'):
        yellow, lane_white = masker._masks(raw)
    else:
        # sliding2025 has no colour-only mask API; this panel shows its configured HSV stage.
        hsv = cv2.cvtColor(warped, cv2.COLOR_BGR2HSV)
        yellow = cv2.inRange(hsv, np.array(values['hsv_lower']), np.array(values['hsv_upper'])) > 0
        lane_white = ((hsv[..., 0] >= values['white_h_min']) &
                      (hsv[..., 0] <= values['white_h_max']) &
                      (hsv[..., 1] <= values['white_s_max']) &
                      (hsv[..., 2] >= values['white_v_min']) & ~yellow)
    yellow_view = np.zeros_like(warped)
    yellow_view[yellow] = (30, 220, 250)

    # Show the selected source's actual preprocessing; pixel probes use clean data, not overlays.
    pipeline_key = json.dumps(settings[color_source], sort_keys=True)
    if data.get('_preview_pipeline_key') != pipeline_key:
        data['_preview_pipeline'] = ColorPipeline(color_params(settings[color_source].__getitem__))
        data['_preview_pipeline_key'] = pipeline_key
    pipeline = data['_preview_pipeline']
    processed = pipeline.preprocess(raw)
    color_bev = cv2.warpPerspective(cv2.GaussianBlur(processed, (5, 5), 0), matrix,
                                  (warp.warp_w, warp.warp_h))
    hsv = cv2.cvtColor(color_bev, cv2.COLOR_BGR2HSV)
    baseline_key = json.dumps(data['hsv_baseline']['lane'], sort_keys=True)
    if data.get('_baseline_mask_key') != baseline_key:
        baseline_values = {**data['detector_values'], **data['hsv_baseline']['lane']}
        data['_baseline_masker'] = make_detector(baseline_values.__getitem__)
        data['_baseline_mask_key'] = baseline_key
    if hasattr(data['_baseline_masker'], '_masks'):
        by, bw = data['_baseline_masker']._masks(raw)
    else:
        baseline = data['hsv_baseline']['lane']
        by, bw = ColorPipeline().masks(warped, **{k: baseline[k] for k in
            ('hsv_lower', 'hsv_upper', 'white_h_min', 'white_h_max', 'white_s_max', 'white_v_min')})
    original_mask = np.zeros_like(warped); original_mask[by] = (30, 220, 250); original_mask[bw] = 235
    adjusted_mask = yellow_view.copy(); adjusted_mask[lane_white] = 235
    text(original_mask, 'YAML baseline'); text(adjusted_mask, 'Current preview')
    comparison = np.hstack((original_mask, adjusted_mask))

    lane = (cv2.imdecode(np.frombuffer(frame['lane_jpeg'], np.uint8), cv2.IMREAD_COLOR)
            if frame['lane_jpeg'] else warped.copy())
    white_view = np.zeros_like(warped)
    line_view = warped.copy()
    mission_pixels = warped
    cross_x = None
    bev = data['bev']
    if bev is not None:
        if '_probe' not in data:
            p = data['params']
            probe = LineStopCore(bev, data['finish_params'], p['white_s_max'], p['white_v_min'], p['line_min_run'])
            probe.debug = True
            probe.min_thickness, probe.max_thickness = p['line_min_thickness'], p['line_max_thickness']
            probe.max_gap = p['line_max_gap']
            probe.require_crosswalk = p['line_require_crosswalk']
            data['_probe'] = probe
        probe = data['_probe']
        probe.white_h_min = settings['mission']['white_h_min']
        probe.white_h_max = settings['mission']['white_h_max']
        probe.white_s_max = settings['mission']['white_s_max']
        probe.white_v_min = settings['mission']['white_v_min']
        mission_key = json.dumps(settings['mission'], sort_keys=True)
        if data.get('_probe_color_key') != mission_key:
            probe.color_pipeline = ColorPipeline(color_params(settings['mission'].__getitem__))
            data['_probe_color_key'] = mission_key
        mission_pixels = bev.warp_image(probe.color_pipeline.preprocess(raw))
        line_view = mission_pixels.copy()
        # Preview the current classifier even when mission has not armed the line stop.
        probe.phase = Phase.IDLE
        probe.target = None
        probe._sightings.clear()
        probe._last_sighting = None
        probe.image(frame['t'], raw)
        white_view[probe.white] = (235, 235, 235)
        band = (np.abs(bev.gy) <= data['params']['line_band']) & bev.valid
        line_view[~band] = (line_view[~band]*.35).astype(np.uint8)
        edge = (np.abs(np.abs(bev.gy)-data['params']['line_band']) < .004) & bev.valid
        line_view[edge] = (230, 200, 60)
        white_view[edge] = (110, 100, 30)
        near, far = bev.near_far()
        for distance in np.arange(np.ceil(near*10)/10, min(far, 3.)+.001, .1):
            pixels = (np.abs(bev.gx-distance) < .002) & bev.valid
            line_view[pixels] = (110, 110, 110)
            rows = np.flatnonzero(pixels.any(1))
            if len(rows):
                text(line_view, f'{distance:.1f}m', (3, max(12, int(np.median(rows)))))
        cross_x = probe.cross_x
        remaining = frame['state'].get('remaining')
        committed = (not preview and remaining is not None and
                     frame['state'].get('line_phase') in ('commit', 'arrived'))
        if committed:
            cross_x = remaining+data['finish_params'].front+data['finish_params'].stop_gap
        if cross_x is not None:
            cross = (np.abs(bev.gx-cross_x) < .004) & bev.valid
            line_view[cross] = (220, 80, 250)
            white_view[cross] = (220, 80, 250)
        label = 'stop target' if committed else 'stop candidate'
        text(line_view, label+': --' if cross_x is None else f'{label}: {cross_x:.3f} m', (80, 20))
        entry = frame['state'].get('entry_remaining')
        if not preview and entry is not None and frame['state'].get('entry_active'):
            entry_x = entry+data['params']['front_axle_x']
            pixels = (np.abs(bev.gx-entry_x) < .004) & bev.valid
            line_view[pixels] = (0, 180, 255)
            white_view[pixels] = (0, 180, 255)
            text(line_view, f'entry: {entry:.3f} m to front axle', (8, 40), (0, 180, 255))
    else:
        text(line_view, 'No CameraInfo: metric stop-line unavailable')
        text(white_view, 'No CameraInfo: production mask unavailable')

    if white_source == 'lane':
        white_view = np.zeros_like(warped)
        white_view[lane_white] = (235, 235, 235)

    hue = hsv.copy(); hue[..., 1:] = 255
    h = cv2.cvtColor(hue, cv2.COLOR_HSV2BGR)
    s = cv2.cvtColor(hsv[..., 1], cv2.COLOR_GRAY2BGR)
    v = cv2.cvtColor(hsv[..., 2], cv2.COLOR_GRAY2BGR)
    for image, name in ((h, 'H 0..179'), (s, 'S 0..255'), (v, 'V 0..255')):
        text(image, name)
    channels = np.hstack((h, s, v))
    lane_bev = masker.color_pipeline.warp(raw, matrix, (warp.warp_w, warp.warp_h)) if hasattr(masker, 'color_pipeline') else warped
    return dict(camera=camera, processed=processed, comparison=comparison, lane=lane, line=line_view, white=white_view,
                yellow=yellow_view, hsv=channels, camera_hsv=cv2.cvtColor(raw, cv2.COLOR_BGR2HSV),
                hsv_views=(h, s, v, channels), bev_hsv=hsv, cross_x=cross_x,
                raw_pixels=raw, processed_pixels=processed, color_bev_pixels=color_bev,
                lane_pixels=lane_bev, mission_pixels=mission_pixels,
                yellow_mask=yellow, lane_white_mask=lane_white,
                mission_white_mask=probe.white.copy() if bev is not None else None)
