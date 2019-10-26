load("@com_nvidia_isaac//engine/build:isaac.bzl", "isaac_cc_library")

isaac_cc_library(
  name = "dynamixel",
  srcs = [
    "driver.cpp"
  ],
  hdrs = [
    "driver_types.hpp",
    "driver_constants.hpp",
    "driver.hpp",
    "util.hpp"
  ],
  visibility = ["//visibility:public"],
  deps = [
    "@robotis//:dynamixel"
  ]
)