"""Custom rule for PSim.

These include a custom rule for autocoding model interfaces and a custom C++
testing command.
"""

# def _psim_autocode(ctx):
#     outs = []
#     for src in ctx.attr.srcs:
#         for file in src.files.to_list():
#             out = ctx.actions.declare_file(file.path + ".hpp")
#             outs.append(out)


#             ctx.actions.run(
#                 inputs = [file],
#                 outputs = [out],
#                 executable = ctx.executable.tool,
#                 arguments = [file.path, out.path],
#                 progress_message = "Autocoding '%s' into '%s'" % (file.path, out.path),
#             )

#     includes = []
#     includes_base = outs[0].path[:-len(outs[0].short_path)]
#     for include in ctx.attr.includes:
#         includes.append(includes_base + include)

#     return CcInfo(
#         compilation_context = cc_common.create_compilation_context(
#             headers = depset(outs),
#             includes = depset(includes),
#         ),
#         linking_context = None,
#     )


# psim_autocode = rule(
#     implementation = _psim_autocode,
#     output_to_genfiles = True,
#     attrs = {
#         "includes": attr.string_list(mandatory = True, allow_empty = False),
#         "srcs": attr.label_list(mandatory = True, allow_files = [".yml"]),
#         "tool": attr.label(mandatory = True, executable = True, cfg = "host"),
#     },
# )

def psim_cc_test(name, **kwargs):
    native.cc_test(
        name = name + "_test",
        srcs = native.glob([name + "/**/*.hpp", name + "/**/*.cpp"]),
        data = native.glob([name + "/**/*.txt"]),
        deps = ["@gtest//:gtest_main", "//:psim"],
        visibility = ["//visibility:public"],
        **kwargs,
    )
