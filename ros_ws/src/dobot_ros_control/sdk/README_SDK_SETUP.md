# SDK Setup & Build Notes (`dobot_ros_control`)

Vendor SDK binaries (`libclient_rpc.a`, etc.) are not tracked in git. When building the optional C++ SDK backend, `CMakeLists.txt` searches for the SDK at:

```text
<repo>/third_party/dobot/lumo_sdk/
├── rpc/algs_rpc_client.h
├── thirdparty/nlohmann/json.hpp
└── lib/libclient_rpc.a
```

Or pass an explicit path via CMake:

```bash
catkin_make --pkg dobot_ros_control -DDOBOT_SDK_ROOT=/path/to/lumo_sdk
```

If the SDK is not present, `dobot_cmd_vel_bridge` automatically builds with the built-in `raw` JSON-RPC backend (`DOBOT_USE_SDK=0`), and the default Python velocity bridge (`scripts/dobot_vel_bridge.py`) requires no C++ SDK binary.

## Linking `libclient_rpc.a` on Ubuntu 20.04

The vendor `libclient_rpc.a` archive requires `GLIBCXX_3.4.29+` (GCC 11+) and `__libc_single_threaded` (provided automatically by `src/libc_single_threaded_shim.cpp`):

```bash
sudo add-apt-repository -y ppa:ubuntu-toolchain-r/test
sudo apt install -y g++-11
```
