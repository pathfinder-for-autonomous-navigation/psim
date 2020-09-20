load("@rules_python//python:defs.bzl", "py_runtime", "py_binary")
load("@rules_cc//cc:defs.bzl", "cc_library")

load("//:tools/psim.bzl", "psim_autocode")

# Build flight software's gnc code as a library.
cc_library(
    name = "gnc",
    srcs = glob([
        "src/gnc/**/*.hpp", "src/gnc/*.cpp", "src/gnc/**/*.inl",
        "include/gnc/**/*.inl"
    ]),
    hdrs = glob([
        "include/gnc/**/*.hpp", "include/orb/*.h", "include/orb/*.hpp"
    ]),
    includes = ['include'],
    copts = ['-Isrc'],
    linkstatic = True,
    visibility = ["//visibility:public"],
    deps = ["@lin//:lin"],
)

# Let bazel know about our virtual environment.
#
# Also see additional options passed with .bazelrc to ensure we're using the
# virtual environment.
#
# TODO : This is a little hacky and could be cleaned up with toolchains.
py_runtime(
    name = "venv",
    files = glob(["venv/**"], exclude=["venv/**/* *"]),
    interpreter = "venv/bin/python",
    python_version = "PY3",
    visibility = ["//visibility:private"],
)

# Register the autocoder as an executable.
#
# This is later consumed by the autocoder command. See psim.bzl for more
# information.
py_binary(
    name = "autocoder",
    srcs = ["tools/autocoder.py"],
    srcs_version = "PY3",
    visibility = ["//visibility:private"],
)

# Autocode the model interfaces
psim_autocode(
    name = "autocode",
    srcs = glob(["include/psim/**/*.yml"]),
    includes = ["include"],
    tool = "//:autocoder",
    visibility = ["//visibility:private"],
)

# Build all PSim models as a library
#
# This will be linked with executables later on.
cc_library(
    name = "psim",
    srcs = glob([
        "src/psim/**/*.hpp", "src/psim/**/*.inl", "src/psim/**/*.cpp",
        "include/psim/**/*.inl", "include/psim/**/*.yml.hpp",
    ]),
    hdrs = glob(
        ["include/psim/**/*.hpp"],
        exclude = ["include/psim/**/*.yml.hpp"],
    ),
    includes = ["include"],
    copts = ["-Isrc"],
    linkstatic = True,
    visibility = ["//visibility:public"],
    deps = ["@lin//:lin", "//:gnc", "//:autocode"],
)
