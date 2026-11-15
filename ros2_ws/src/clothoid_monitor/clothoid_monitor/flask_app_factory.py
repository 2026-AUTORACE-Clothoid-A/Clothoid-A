from pathlib import Path
import threading

try:
    from flask import Flask, Response, jsonify, render_template, request, send_file
    from werkzeug.serving import make_server
    FLASK_IMPORT_ERROR = None
except (ModuleNotFoundError, ImportError) as exc:
    Flask = None
    Response = None
    jsonify = None
    render_template = None
    request = None
    send_file = None
    make_server = None
    FLASK_IMPORT_ERROR = exc

from .image_utils import build_camera_placeholder_svg

PACKAGE_ROOT = Path(__file__).resolve().parent
TEMPLATE_DIR = PACKAGE_ROOT / 'templates'
STATIC_DIR = PACKAGE_ROOT / 'static'


class FlaskServerThread(threading.Thread):
    def __init__(self, app, host, port):
        super().__init__(daemon=True)
        self._server = make_server(host, port, app, threaded=True)

    def run(self):
        self._server.serve_forever()

    def shutdown(self):
        self._server.shutdown()


def create_app(state, page_title,
               battery_topic, image_topic, control_topic, storage_path,
               refresh_interval_ms, image_refresh_interval_ms,
               header_logo_path, telechips_logo_path, topst_logo_path,
               image_display_width, image_display_height,
               debug_image, lane_topic, yolo_topic,
               aruco_topic, control_history_window_sec, graph_snapshot_provider=None,
               topic_echo_setter=None, param_target_nodes=(),
               param_lister=None, param_setter=None, lidar_topic='/scan'):

    app = Flask(__name__, template_folder=str(TEMPLATE_DIR), static_folder=str(STATIC_DIR))
    if hasattr(app, 'json'):
        app.json.sort_keys = False
    else:
        app.config['JSON_SORT_KEYS'] = False

    @app.get('/')
    def index():
        return render_template(
            'index.html',
            page_title=page_title,
            battery_topic=battery_topic,
            image_topic=image_topic,
            control_topic=control_topic,
            storage_path=storage_path,
            refresh_interval_ms=refresh_interval_ms,
            image_refresh_interval_ms=image_refresh_interval_ms,
            placeholder_url='/api/frame/placeholder',
            header_logo_url='/assets/header-logo',
            debug_image=debug_image,
            lane_topic=lane_topic,
            yolo_topic=yolo_topic,
            aruco_topic=aruco_topic,
            lidar_topic=lidar_topic,
            control_history_window_sec=control_history_window_sec,
            param_target_nodes=list(param_target_nodes),
        )

    @app.get('/api/status')
    def api_status():
        return jsonify(state.snapshot())

    @app.get('/api/control_history')
    def api_control_history():
        return jsonify(state.get_control_history())

    @app.get('/api/graph')
    def api_graph():
        if graph_snapshot_provider is None:
            return jsonify({'nodes': [], 'edges': []})
        return jsonify(graph_snapshot_provider())

    @app.get('/api/topic_echo')
    def api_topic_echo_get():
        return jsonify(state.get_topic_echo())

    @app.post('/api/topic_echo')
    def api_topic_echo_start():
        if topic_echo_setter is None:
            return jsonify({'ok': False, 'error': 'topic echo not supported'}), 501

        payload = request.get_json(silent=True) or {}
        topic_name = str(payload.get('topic') or '').strip()
        ok = topic_echo_setter(topic_name)
        status_code = 200 if ok else 404
        return jsonify({'ok': ok, 'topic': topic_name}), status_code

    @app.get('/api/params')
    def api_params_list():
        node_name = str(request.args.get('node') or '').strip()
        if param_lister is None:
            return jsonify({'ok': False, 'error': 'params not supported'}), 501
        if node_name not in param_target_nodes:
            return jsonify({'ok': False, 'error': f'unknown node {node_name!r}'}), 400

        params = param_lister(node_name)
        if params is None:
            return jsonify({'ok': False, 'error': f'{node_name} not reachable'}), 503
        return jsonify({'ok': True, 'node': node_name, 'params': params})

    @app.post('/api/params')
    def api_params_set():
        if param_setter is None:
            return jsonify({'ok': False, 'error': 'params not supported'}), 501

        payload = request.get_json(silent=True) or {}
        node_name = str(payload.get('node') or '').strip()
        param_name = str(payload.get('name') or '').strip()
        if node_name not in param_target_nodes or not param_name:
            return jsonify({'ok': False, 'error': 'missing/unknown node or name'}), 400

        try:
            type_id = int(payload.get('type'))
        except (TypeError, ValueError):
            return jsonify({'ok': False, 'error': 'missing parameter type'}), 400

        ok, reason = param_setter(node_name, param_name, type_id, payload.get('value'))
        return jsonify({'ok': ok, 'node': node_name, 'name': param_name, 'reason': reason}), (
            200 if ok else 400)

    @app.get('/api/frame')
    def api_frame():
        frame_bytes = state.get_latest_frame()
        if frame_bytes is None:
            return Response(
                build_camera_placeholder_svg(image_display_width, image_display_height, image_topic),
                mimetype='image/svg+xml',
            )

        response = Response(frame_bytes, mimetype='image/jpeg')
        response.headers['Cache-Control'] = 'no-store, no-cache, must-revalidate, max-age=0'
        response.headers['Pragma'] = 'no-cache'
        return response

    @app.get('/api/frame/placeholder')
    def api_frame_placeholder():
        return Response(
            build_camera_placeholder_svg(image_display_width, image_display_height, image_topic),
            mimetype='image/svg+xml',
        )

    @app.get('/api/frame/lane')
    def api_frame_lane():
        frame_bytes = state.get_debug_frame('lane')
        if frame_bytes is None:
            return Response(
                build_camera_placeholder_svg(
                    image_display_width, image_display_height, lane_topic
                ),
                mimetype='image/svg+xml',
            )

        response = Response(frame_bytes, mimetype='image/jpeg')
        response.headers['Cache-Control'] = 'no-store, no-cache, must-revalidate, max-age=0'
        response.headers['Pragma'] = 'no-cache'
        return response

    @app.get('/api/frame/yolo')
    def api_frame_yolo():
        frame_bytes = state.get_debug_frame('yolo')
        if frame_bytes is None:
            return Response(
                build_camera_placeholder_svg(
                    image_display_width, image_display_height, yolo_topic
                ),
                mimetype='image/svg+xml',
            )

        response = Response(frame_bytes, mimetype='image/jpeg')
        response.headers['Cache-Control'] = 'no-store, no-cache, must-revalidate, max-age=0'
        response.headers['Pragma'] = 'no-cache'
        return response

    @app.get('/api/frame/lidar')
    def api_frame_lidar():
        frame_bytes = state.get_debug_frame('lidar') or state.get_debug_frame('aruco')
        if frame_bytes is None:
            return Response(
                build_camera_placeholder_svg(
                    image_display_width, image_display_height, lidar_topic
                ),
                mimetype='image/svg+xml',
            )

        response = Response(frame_bytes, mimetype='image/jpeg')
        response.headers['Cache-Control'] = 'no-store, no-cache, must-revalidate, max-age=0'
        response.headers['Pragma'] = 'no-cache'
        return response

    @app.get('/api/frame/aruco')
    def api_frame_aruco():
        frame_bytes = state.get_debug_frame('lidar') or state.get_debug_frame('aruco')
        if frame_bytes is None:
            return Response(
                build_camera_placeholder_svg(
                    image_display_width, image_display_height, aruco_topic
                ),
                mimetype='image/svg+xml',
            )

        response = Response(frame_bytes, mimetype='image/jpeg')
        response.headers['Cache-Control'] = 'no-store, no-cache, must-revalidate, max-age=0'
        response.headers['Pragma'] = 'no-cache'
        return response

    @app.get('/assets/header-logo')
    def header_logo():
        if header_logo_path and Path(header_logo_path).exists():
            return send_file(str(header_logo_path), mimetype='image/png', max_age=0)
        return '', 404

    @app.get('/assets/telechips-logo')
    def telechips_logo():
        if telechips_logo_path and Path(telechips_logo_path).exists():
            return send_file(str(telechips_logo_path), mimetype='image/png', max_age=0)
        return '', 404

    @app.get('/assets/topst-logo')
    def topst_logo():
        if topst_logo_path and Path(topst_logo_path).exists():
            return send_file(str(topst_logo_path), mimetype='image/png', max_age=0)
        return '', 404

    return app
