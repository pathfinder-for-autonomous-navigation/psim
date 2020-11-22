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

"""Custom repository rules for PSim.
"""

load("//bazel:psim.bzl", _psim_fail = "psim_fail")


def _psim_get_python_bin(repository_ctx):
    """Retrieves the Python binary that will be used by this repository.

    We are required here that Python 3 is used.
    """
    python_bin = repository_ctx.which("python3")
    if python_bin == None:
        _psim_fail("Failed to detect a python binary. Ensure a 'python3'" +
                   "executable is in your path.")

    return python_bin


def _psim_get_python_inc(repository_ctx, python_bin):
    """Retrieves the include path to the core Python library.

    This requires distutilsl to be installed.
    """
    result = repository_ctx.execute([
        python_bin, "-c", (
            "from distutils import sysconfig\n" +
            "\n" +
            "print(sysconfig.get_python_inc())\n" +
            ""
        )
    ])
    if result.stderr or not result.stdout:
        _psim_fail("Failed to retrive Python include path. Ensure 'distutils' is installed.\n" +
                   result.stderr)

    return result.stdout.strip("\n")


def _psim_configure_impl(repository_ctx):
    """Generates a new repository containing the Python development headers.

    Ensure the Python development packages is installed on your machine.
    """
    python_bin = _psim_get_python_bin(repository_ctx)
    python_inc = _psim_get_python_inc(repository_ctx, python_bin)

    if repository_ctx.name != "psim_configuration":
        _psim_fail("The 'psim_configure' repository name must be 'psim_configuration'.")

    repository_ctx.symlink(python_inc, "include")
    repository_ctx.template(
        "BUILD",
        Label("//bazel:psim_configuration.BUILD.tpl"),
        { },
    )


psim_configure = repository_rule(
    implementation = _psim_configure_impl,
)
