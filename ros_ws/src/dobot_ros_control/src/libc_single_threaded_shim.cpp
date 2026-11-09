/*
 * Copyright 2026 Manifold Tech Ltd.(www.manifoldtech.com.co)
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *   http://www.apache.org/licenses/LICENSE-2.0
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

// Compatibility shim for linking Dobot Lumo SDK RPC static library (libclient_rpc.a)
// on Ubuntu 20.04 (glibc 2.31), which lacks __libc_single_threaded (introduced in glibc 2.32).
// Setting to 0 forces the thread-safe atomic reference counting path in libstdc++.
extern "C" {
char __libc_single_threaded = 0;
}
