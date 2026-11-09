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

"""Launch file for odin_vlm_terminal (ROS 2).

Requires a reachable llama-server (OpenAI-compatible endpoint, default port 8080)
serving a vision-language model (default SmolVLM-500M-Instruct).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params = {
        'topic': LaunchConfiguration('topic'),
        'server': LaunchConfiguration('server'),
        'model': LaunchConfiguration('model'),
        'instruction': LaunchConfiguration('instruction'),
        'interval': LaunchConfiguration('interval'),
    }
    return LaunchDescription([
        DeclareLaunchArgument('topic', default_value='/odin1/image/compressed'),
        DeclareLaunchArgument('server', default_value='http://localhost:8080'),
        DeclareLaunchArgument('model', default_value='SmolVLM-500M-Instruct'),
        DeclareLaunchArgument('instruction',
                              default_value='describe what you see and in detailed'),
        DeclareLaunchArgument('interval', default_value='0.0',
                              description='0 = interactive prompt input; >0 = periodic auto inference (seconds)'),
        Node(
            package='odin_vlm_terminal',
            executable='ros_vlm_terminal.py',
            name='odin_vlm_terminal',
            output='screen',
            emulate_tty=True,
            parameters=[params],
        ),
    ])
