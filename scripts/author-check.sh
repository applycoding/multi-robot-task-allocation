#!/usr/bin/env bash
# SPDX-License-Identifier: Apache-2.0
set -euo pipefail
expected="Arpan Bom <applycoding@gmail.com>"
author="$(git log -1 --format='%an <%ae>')"
committer="$(git log -1 --format='%cn <%ce>')"
test "$author" = "$expected"
test "$committer" = "$expected"
