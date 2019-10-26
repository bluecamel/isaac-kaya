"""
Copyright (c) 2019, NVIDIA CORPORATION. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions
are met:
 * Redistributions of source code must retain the above copyright
   notice, this list of conditions and the following disclaimer.
 * Redistributions in binary form must reproduce the above copyright
   notice, this list of conditions and the following disclaimer in the
   documentation and/or other materials provided with the distribution.
 * Neither the name of NVIDIA CORPORATION nor the names of its
   contributors may be used to endorse or promote products derived
   from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
"""

load("@com_nvidia_isaac//engine/build:isaac.bzl", "isaac_app", "isaac_cc_module", "isaac_subgraph")

isaac_app(
  name = "joystick-bkd",
  app_json_file = "joystick.app.json",
  data = [
    "2d_kaya_subgraph",
    "kaya_hardware_subgraph",
    "//packages/kaya/apps/robots",
    "@com_nvidia_isaac//packages/navigation/apps:holonomic_base_commander_subgraph",
    "@com_nvidia_isaac//packages/navigation/apps:holonomic_base_imu_odometry_subgraph"
  ],
  modules = [
    "kaya"
  ],
)

isaac_subgraph(
  name = "kaya_hardware_subgraph",
  modules = [
    "@com_nvidia_isaac//packages/imu",
    "@com_nvidia_isaac//packages/kaya_base",
    "@com_nvidia_isaac//packages/realsense",
    "@com_nvidia_isaac//packages/viewers"
  ],
  subgraph = "kaya_hardware.subgraph.json",
  visibility = ["//visibility:public"],
)

isaac_subgraph(
  name = "2d_kaya_subgraph",
  modules = [
    "@com_nvidia_isaac//packages/rgbd_processing",
  ],
  subgraph = "2d_kaya.subgraph.json",
  visibility = ["//visibility:public"],
)
