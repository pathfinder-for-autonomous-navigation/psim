cc_library(
    name = "psim",
    srcs = glob(["src/**/*.cpp", "src/**/*.inl", "src/**/*.hpp", "include/**/*.inl"], exclude=["src/targets/*.cpp"]),
    hdrs = glob(["include/**/*.hpp", "include/**/*.h"]),
    copts = ["-Iinclude -Isrc"],
    linkstatic = True,
    visibility = ["//visibility:public"],
    deps = [
        "@lin//:lin",
    ]
)
