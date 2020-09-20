load("@rules_cc//cc:defs.bzl", _cc_library = "cc_library")

_cc_library(
    name = "python",
    hdrs = glob(["include/**/*.h"]),
    includes = ["include"],
    visibility = ["//visibility:public"],
)
