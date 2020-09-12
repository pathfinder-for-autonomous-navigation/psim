load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")
# load("@bazel_tools//tools/build_defs/repo:git.bzl", "git_repository")

# git_repository(
#     name = "rules_python",
#     remote = "https://github.com/bazelbuild/rules_python.git",
#     commit = "3baa2660569a76898d0f520c73b299ea39b6374d",  # (9-8-2020)
# )

http_archive(
    name = "gtest",
    url = "https://github.com/google/googletest/archive/release-1.10.0.zip",
    sha256 = "94c634d499558a76fa649edb13721dce6e98fb1e7018dfaeba3cd7a083945e91",
    strip_prefix = "googletest-release-1.10.0",
)

# http_archive(
#     name = "sofa",
#     url = "https://github.com/pathfinder-for-autonomous-navigation/sofa/archive/v1.0.zip",
#     sha256 = "a5660ff94270934c33b3fdaaf891620c9a454181dad7fb12378e66a5b73c205f",
#     strip_prefix = "sofa-1.0",
# )

local_repository(
    name = "lin",
    path = "lib/lin",
)
