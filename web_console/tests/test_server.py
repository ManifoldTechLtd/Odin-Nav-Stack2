import array
import json
import threading
import urllib.error
import urllib.request

import pytest

from web_console import core
from web_console.server import make_server


@pytest.fixture
def served():
    state = core.ConsoleState()
    server = make_server("127.0.0.1", 0, state)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    yield state, "http://127.0.0.1:{}".format(server.server_address[1])
    server.shutdown()
    server.server_close()


def get(url):
    try:
        with urllib.request.urlopen(url, timeout=5) as res:
            return res.status, res.headers, res.read()
    except urllib.error.HTTPError as exc:
        return exc.code, exc.headers, exc.read()


def test_index_page_served(served):
    _, base = served
    status, headers, body = get(base + "/")
    assert status == 200
    assert headers["Content-Type"].startswith("text/html")
    assert b"<canvas" in body


def test_state_endpoint_returns_json(served):
    state, base = served
    state.set_pose(0.5, 0.25, 0.0)
    status, _, body = get(base + "/api/state")
    payload = json.loads(body)
    assert status == 200
    assert payload["pose"]["x"] == 0.5
    assert payload["map"] is None


def test_map_png_404_until_map_received(served):
    state, base = served
    status, _, _ = get(base + "/api/map.png")
    assert status == 404
    state.set_map(1, 1, 0.1, 0.0, 0.0, array.array("b", [0]))
    status, headers, body = get(base + "/api/map.png")
    assert status == 200
    assert headers["Content-Type"] == "image/png"
    assert body[:8] == b"\x89PNG\r\n\x1a\n"
    assert headers["ETag"] == '"1"'


def test_map_png_conditional_request_returns_304(served):
    state, base = served
    state.set_map(1, 1, 0.1, 0.0, 0.0, array.array("b", [0]))
    req = urllib.request.Request(base + "/api/map.png", headers={"If-None-Match": '"1"'})
    with pytest.raises(urllib.error.HTTPError) as err:
        urllib.request.urlopen(req, timeout=5)
    assert err.value.code == 304


def test_unknown_path_404(served):
    _, base = served
    status, _, _ = get(base + "/nope")
    assert status == 404
