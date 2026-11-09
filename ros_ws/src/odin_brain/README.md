# odin_brain（ROS1 文本给点导航）

一句话中文 → 有序航点 → `/waypoint_manager/start_mission`。

- **核心去 ROS 化**：`src/odin_brain/{models,places,planner,llm_client,executor,mock_nav}.py` 不依赖任何 ROS 包。
- **ROS1 接口层**：`src/odin_brain/{ros1_nav,node,place_tool}.py`。
- **快速试用（无需 ROS）**：

```bash
cd ros_ws/src/odin_brain
PYTHONPATH=src python3 -m odin_brain.cli --mock --once "先去厕所待五秒，然后再去小A工位"
python3 -m unittest discover -s test -v      # 26 项单测
```

- **上机使用**：

```bash
cd ~/base_inspection/ros_ws
source /opt/ros/noetic/setup.bash
catkin_make --only-pkg-with-deps odin_brain
source devel/setup.bash
rosrun odin_brain brain_places record 厕所 --aliases 卫生间,洗手间
rosrun odin_brain brain
```

完整说明见 `docs/design/odin大脑开发/Odin大脑第一版使用说明.md`；进度见根目录 `brain_progress.md`。

## 大模型（可选）

```bash
export ODIN_LLM_BASE_URL=http://127.0.0.1:8000/v1
export ODIN_LLM_API_KEY=xxx
export ODIN_LLM_MODEL=hermes-4-70b
```

未配置时自动使用规则解析；解析失败也会回退到规则。
