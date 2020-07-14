load("@bazel_tools//tools/build_defs/repo:git.bzl", "new_git_repository")
load("@bazel_tools//tools/build_defs/repo:git.bzl", "git_repository")

lin_build = """
cc_library(
    name = "lin",
    srcs = glob(["src/**/*.cpp", "include/**/*.inl"]),
    hdrs = glob(["include/**/*.hpp"]),
    includes = ["include"],
    linkstatic = True,
    visibility = ["//visibility:public"]
)
"""

new_git_repository(
    name = "lin",
    init_submodules = True,
    remote = "https://github.com/pathfinder-for-autonomous-navigation/lin",
    commit = "f8fafaa5a29190663bc313a0b835201749b876b8",
    build_file_content = lin_build
)
