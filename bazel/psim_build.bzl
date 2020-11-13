#
# MIT License
#
# Copyright (c) 2020 Pathfinder for Autonomous Navigation (PAN)
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
#

"""Custom rules for PSim.
"""

load("@rules_cc//cc:defs.bzl", _cc_binary = "cc_binary", _cc_library = "cc_library", _cc_test = "cc_test")
load("@rules_python//python:defs.bzl", _py_library = "py_library")


def _psim_autocode_impl(ctx):
    outs = []
    for yml in ctx.attr.ymls:
        for file in yml.files.to_list():
            out = ctx.actions.declare_file(file.path + ".hpp")
            outs.append(out)

            ctx.actions.run(
                inputs = [file],
                outputs = [out],
                executable = ctx.executable.tool,
                arguments = [file.path, out.path],
                progress_message = "Autocoding '%s' into '%s'" % (file.path, out.path),
            )

    includes = []
    includes_base = outs[0].path[:-len(outs[0].short_path)]
    for include in ctx.attr.includes:
        includes.append(includes_base + include)

    return CcInfo(
        compilation_context = cc_common.create_compilation_context(
            headers = depset(outs),
            includes = depset(includes),
        ),
        linking_context = None,
    )


_psim_autocode = rule(
    implementation = _psim_autocode_impl,
    output_to_genfiles = True,
    attrs = {
        "includes": attr.string_list(mandatory = True, allow_empty = False),
        "ymls": attr.label_list(mandatory = True, allow_files = [".yml"]),
        "tool": attr.label(mandatory = True, executable = True, cfg = "host"),
    },
)


def psim_autocoded_cc_library(name, deps = None, local_defines = None, visibility = None):
    """Defines a PSim library with autocoded header files for models.
    """
    _include_dir = "include/psim/" + name

    _psim_autocode(
        name = "psim_" + name + "_autocoded",
        includes = ["include"],
        ymls = native.glob([_include_dir + "/**/*.yml"]),
        tool = "//:autocoder",
        visibility = ["//visibility:private"],
    )

    psim_cc_library(
        name,
        deps = deps + ["psim_" + name + "_autocoded"],
        local_defines = local_defines,
        visibility = visibility
    )


def psim_cc_library(name, deps = None, local_defines = None, visibility = None):
    """Defines a PSim library without autocoded header files for models.
    """
    _include_dir = "include/psim/" + name
    _src_dir = "src/psim/" + name

    _cc_library(
        name = "psim_" + name,
        srcs = native.glob([
            _src_dir + "/**/*.hpp", _src_dir + "/**/*.inl",
            _src_dir + "/**/*.cpp", _include_dir + "/**/*.inl",
        ]),
        hdrs = native.glob([_include_dir + "/**/*.hpp"]),
        includes = ["include"],
        copts = ["-Isrc", "-fvisibility=hidden"],
        linkstatic = True,
        deps = deps,
        local_defines = local_defines,
        visibility = visibility,
    )


def psim_cc_test(name, deps = None, tags = None):
    """Defines a PSim CC test.
    """
    _test_dir = name

    _cc_test(
        name = name + "_test",
        srcs = native.glob([
            _test_dir + "/**/*.hpp", _test_dir + "/**/*.inl",
            _test_dir + "/**/*.cpp",
        ]),
        data = native.glob([_test_dir + "/**/*.txt"]),
        deps = deps + ["@gtest//:gtest_main"],
        tags = tags,
        visibility = ["//visibility:public"],
    )


def psim_py_extension(name, srcs = None, hdrs = None, data = None, deps = None, local_defines = None, visibility = None):
    """Compiles a PSim Python extension written in C++.

    This uses pybind11. Inspiration was taken from the code listed at the link
    below and the pybind11 documentation:
     - https://github.com/blais/oblique/blob/2ac7b3f82b4e3470da94a6b0417935b0eec3eec8/third_party/python/py_extension.bzl#L1
     - https://pybind11.readthedocs.io/en/stable/compiling.html
    """
    _cc_binary(
        name = name + ".so",
        srcs = srcs,
        hdrs = hdrs,
        data = data,
        copts = ["-Isrc", "-fvisibility=hidden"],
        linkopts = select({
            "@pybind11//:osx": ["-undefined", "dynamic_lookup"],
            "//conditions:default": ['-Wl,--export-dynamic-symbol=PyInit_%s' % name],
        }),
        linkshared = True,
        linkstatic = True,
        deps = deps + ["@pybind11//:pybind11"],
        local_defines = local_defines,
        visibility = ["//visibility:private"],
    )

    _py_library(
        name = name,
        data = [name + ".so"],
        visibility = visibility,
    )
