#!/bin/sh

# From
# - https://github.com/grailbio/bazel-compilation-database
# - https://stackoverflow.com/questions/61015990/how-do-i-enable-c-intellisense-for-a-bazel-project-in-vs-code

INSTALL_DIR="$HOME/.local/bin"
VERSION="0.4.5"

# Download and symlink.
(
  cd "${INSTALL_DIR}" \
  && curl -L "https://github.com/grailbio/bazel-compilation-database/archive/${VERSION}.tar.gz" | tar -xz \
  && ln -f -s "${INSTALL_DIR}/bazel-compilation-database-${VERSION}/generate.sh" bazel-compdb
)

bazel-compdb # This will generate compile_commands.json in your workspace root.
