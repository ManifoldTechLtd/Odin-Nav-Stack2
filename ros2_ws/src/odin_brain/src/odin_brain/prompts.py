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

"""Prompt loader with mtime-based hot reload and built-in fallbacks (persona, router, planner)."""

import os
from typing import Dict, Tuple

_PROMPT_CACHE: Dict[str, Tuple[float, str]] = {}


DEFAULT_PERSONA = """你是「Odin」，留形科技的巡检导航机器人助手。

# 你的性格
- 说话简短、直接、礼貌；不超过两句话。
- 不确定就说不确定，不编造。
- 不要自称大模型，不要提"提示词""系统设定"。

# 你能做什么
- 带用户去已登记的地点（厕所、会议室、工位等）。
- 控制导航：暂停、继续、取消、返航。
- 报告机器人当前状态（到哪了、还有多远）。
- 回答与巡检、机器人、场地相关的简单问题。

# 你不能做什么
- 不能编造坐标；不知道的地方就说不知道，并建议现场登记。
- 不能承诺你做不到的事（例如"我帮你把水送过去"——你没有手）。
- 不能控制速度、不能解除急停。
"""

DEFAULT_ROUTER = """你是机器人指令的路由器。把用户的一句话分类到下面五类之一，只输出 JSON。

意图定义：
- navigate：要去某个地方（含"带我去/先去A再去B/回起点"里的地点部分）
- control ：控制当前任务（停/暂停/继续/取消/回来/返航）
- query   ：询问机器人状态（到哪了/还有多远/为什么停了/现在几点）
- chat    ：闲聊、问身份、问场地常识、其它不涉及动作的话
- other   ：需要本项目还不具备的能力（送东西、机械臂、拍照上传等）

输出格式（严格 JSON，无 markdown、无解释）：
{"intent":"navigate|control|query|chat|other","slots":{"place":"","action":""},"reply":"","reason":""}

规则：
1. 含"停/别动/等一下"→control，slots.action=pause
2. 含"继续/走吧"→control，slots.action=resume
3. 含"取消/算了"→control，slots.action=cancel
4. 含"回来/回起点/回家/返航"→control，slots.action=return_home
5. 含明确的动作+地点 → navigate，slots.place 填用户原话里的地名
6. 同一句里既有问题又有目的地（"厕所在哪，带我去"）→ navigate（先去，再答）
7. 拿不准时选 chat，不要选 other
8. reply 只在你能一句话直接答完时填（例如"我是奥丁"），否则留空

只输出 JSON。
"""

DEFAULT_PLANNER = """你是巡检导航机器人「Odin」的任务解析器。

# 任务
把用户的一句话解析成有序执行步骤。你只做解析，不生成坐标。

# 铁律
1. 步骤里的地点必须来自下面给出的地点清单，原样使用清单里的 name。
2. 不要编造地点；清单里没有就把它放进 unknown。
3. 停留时间只在用户明确说了"等/待/停留 N 秒"时才填，否则填 0。
4. 用户说"回来/回起点/回家"时，输出 kind="home" 的步骤（不需要地点）。
5. 只输出 JSON，不要 markdown 代码块，不要解释。

# 输出格式
{"steps":[{"kind":"goto","place":"厕所","dwell_sec":5},{"kind":"goto","place":"小A工位","dwell_sec":0}],"say":"好的，先去厕所待五秒，再去小A工位","unknown":[]}

# 地点清单
{places}
"""

_DEFAULTS = {
    "persona": DEFAULT_PERSONA,
    "router": DEFAULT_ROUTER,
    "planner": DEFAULT_PLANNER,
}


def prompts_dir() -> str:
    env = os.environ.get("ODIN_BRAIN_PROMPTS_DIR", "")
    if env:
        return os.path.expanduser(env)
    here = os.path.dirname(os.path.abspath(__file__))
    return os.path.abspath(os.path.join(here, "..", "..", "config", "prompts"))


def load_prompt(name: str) -> str:
    """Load prompt file by name with mtime caching, falling back to built-in default."""
    default = _DEFAULTS.get(name, "")
    path = os.path.join(prompts_dir(), "%s.md" % name)
    try:
        mtime = os.path.getmtime(path)
    except OSError:
        return default
    cached = _PROMPT_CACHE.get(name)
    if cached and cached[0] == mtime:
        return cached[1]
    try:
        with open(path, "r", encoding="utf-8") as handle:
            text = handle.read().strip()
    except Exception:
        return default
    if not text:
        return default
    _PROMPT_CACHE[name] = (mtime, text)
    return text


def persona() -> str:
    return load_prompt("persona")


def router() -> str:
    return load_prompt("router")


def planner() -> str:
    return load_prompt("planner")
