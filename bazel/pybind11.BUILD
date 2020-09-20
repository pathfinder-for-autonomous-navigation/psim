load("@rules_cc//cc:defs.bzl", _cc_library = "cc_library")

_cc_library(
    name = "pybind11",
    hdrs = glob(
        ["include/pybind11/*.h", "include/pybind11/detail/*.h"],
        exclude = ["include/pybind11/common.h"],
    ),
    copts = [
        "-fexceptions",
        "-Xclang-only=-Wno-undefined-inline",
        "-Xclang-only=-Wno-pragma-once-outside-header",
        "-Xgcc-only=-Wno-error",
    ],
    includes = ["include"],
    deps = ["@psim_configuration//:python"],
    visibility = ["//visibility:public"],
)

config_setting(
    name = "osx",
    constraint_values = ["@platforms//os:osx"],
)
