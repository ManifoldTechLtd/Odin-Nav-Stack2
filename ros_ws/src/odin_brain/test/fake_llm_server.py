# Copyright 2026 Manifold Tech Ltd.(www.manifoldtech.com.co)
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#   http://www.apache.org/licenses/LICENSE-2.0
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Minimal fake OpenAI-compatible HTTP server for local check_llm testing."""

import json
import re
from http.server import BaseHTTPRequestHandler, HTTPServer

RULES = [
    ("回起点", [{"kind": "home"}]),
    ("先去厕所待五秒，然后再去小A工位",
     [{"kind": "goto", "place": "厕所", "dwell_sec": 5},
      {"kind": "goto", "place": "小A工位", "dwell_sec": 0}]),
    ("等3秒", [{"kind": "goto", "place": "厕所", "dwell_sec": 3},
               {"kind": "goto", "place": "会议室", "dwell_sec": 0}]),
    ("卫生间", [{"kind": "goto", "place": "厕所", "dwell_sec": 0}]),
    ("去厕所", [{"kind": "goto", "place": "厕所", "dwell_sec": 0}]),
]


class Handler(BaseHTTPRequestHandler):
    def do_POST(self):
        length = int(self.headers.get("Content-Length", "0"))
        body = json.loads(self.rfile.read(length) or b"{}")
        messages = body.get("messages") or []
        user_text = ""
        for message in reversed(messages):
            if message.get("role") == "user":
                user_text = message.get("content", "")
                break

        steps = [{"kind": "goto", "place": "厕所", "dwell_sec": 0}]
        for pattern, rule_steps in RULES:
            if pattern in user_text:
                steps = rule_steps
                break

        content = json.dumps({"steps": steps, "say": "ok", "unknown": []}, ensure_ascii=False)
        payload = {
            "id": "chatcmpl-fake",
            "object": "chat.completion",
            "model": body.get("model", "fake"),
            "choices": [{"index": 0, "message": {"role": "assistant", "content": content},
                         "finish_reason": "stop"}],
        }
        data = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def log_message(self, *args):
        pass


if __name__ == "__main__":
    HTTPServer(("127.0.0.1", 18765), Handler).serve_forever()
