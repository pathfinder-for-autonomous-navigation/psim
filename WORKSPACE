workspace(name = "psim")

load("@bazel_tools//tools/build_defs/repo:git.bzl", "git_repository")
load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

load("//bazel:psim_workspace.bzl", "psim_configure")

# Standard cc rules
git_repository(
    name = "rules_cc",
    remote = "https://github.com/bazelbuild/rules_cc",
    commit = "bea600c25d246a79398a1c1a73e9261aee35a6b1",
)

# Standard Python rules
http_archive(
    name = "rules_python",
    url = "https://github.com/bazelbuild/rules_python/releases/download/0.0.2/rules_python-0.0.2.tar.gz",
    strip_prefix = "rules_python-0.0.2",
    sha256 = "b5668cde8bb6e3515057ef465a35ad712214962f0b3a314e551204266c7be90c",
)

# Pybind11
http_archive(
  name = "pybind11",
  build_file = "//bazel:pybind11.BUILD",
  strip_prefix = "pybind11-2.5.0",
  urls = ["https://github.com/pybind/pybind11/archive/v2.5.0.tar.gz"],
)

# Googletest
http_archive(
    name = "gtest",
    url = "https://github.com/google/googletest/archive/release-1.10.0.zip",
    sha256 = "94c634d499558a76fa649edb13721dce6e98fb1e7018dfaeba3cd7a083945e91",
    strip_prefix = "googletest-release-1.10.0",
)

# Sofa/IAU Coordinate Transformations
http_archive(
    name = "sofa",
    url = "https://github.com/pathfinder-for-autonomous-navigation/sofa/archive/v1.0.zip",
    sha256 = "a5660ff94270934c33b3fdaaf891620c9a454181dad7fb12378e66a5b73c205f",
    strip_prefix = "sofa-1.0",
)

# std::variant alternative for use in C++ 14
http_archive(
    name = "variant",
    build_file = "//bazel:variant.BUILD",
    url = "https://github.com/mapbox/variant/archive/v1.2.0.zip",
    sha256 = "48df02096b954ea35c447edc3e662e7f163a92e1de2ab5f226445d6001a4537d",
    strip_prefix = "variant-1.2.0",
)

# lin
local_repository(
    name = "lin",
    path = "lib/lin",
)

# Grabs the systems Python headers in preparation for building with Pybind11
psim_configure(
    name = "psim_configuration"
)
