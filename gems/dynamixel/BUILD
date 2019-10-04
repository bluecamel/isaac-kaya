load("@com_nvidia_isaac//engine/build:isaac.bzl", "isaac_cc_library")

isaac_cc_library(
    name = "dynamixel",
    srcs = [
        "Driver.cpp"
    ],
    hdrs = [
        "DriverTypes.hpp",
        "DriverConstants.hpp",
        "Driver.hpp"
    ],
    visibility = ["//visibility:public"],
    deps = [
      "@robotis//:dynamixel"
    ]
)