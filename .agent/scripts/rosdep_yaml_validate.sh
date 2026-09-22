#!/bin/bash
# .agent/scripts/rosdep_yaml_validate.sh
# Shape gate for a project repo's root rosdep.yaml (#654).
#
# WHY THIS EXISTS: a rosdep.yaml merged in ANY project repo drives a
# root-level `rosdep install -y` on the dev host, inside the ci_local
# container, and at agent-image bake time. rosdep's own format is wider than
# the policy: besides the `{key: {os: [packages]}}` form this workspace
# documents, it accepts installer-keyed rules — `pip`, `npm`, `gem`, and
# `source` (which downloads an rdmanifest and RUNS its install script). Nothing
# validated the file before it reached that install, so a repo-merged rule
# could install outside the documented form, and a `pip:` rule would route
# around ADR-0009's Python-package tiers entirely.
#
# So the workspace accepts ONE shape and rejects everything else:
#
#     <rosdep-key>:            # upstream PR owed: ros/rosdistro#NNNNN
#       <os-name>: [<package>, ...]
#
#   * every OS value is a LIST of plain system package names — never a string,
#     never a nested mapping. The nested mapping is exactly where `pip:`,
#     `npm:`, `gem:` and `source:` live, and also where the codename-keyed
#     (`ubuntu: {noble: [...]}`) form lives; the codename form is collateral,
#     and deliberately not carved out — a local key is a short-lived stand-in
#     for an upstream entry, not a place to express per-release matrices.
#   * package names are plain tokens, so no flags, paths or URLs.
#
# This is a policy gate, not a security boundary: the real defence is that
# project repos are reviewed. It stops the unreviewed-shape case from reaching
# root, and names the rule when it does.
#
# Usage:
#   rosdep_yaml_validate.sh <rosdep.yaml> [<rosdep.yaml> ...]
#
# Exit codes:
#   0  every file conforms (an empty file declares nothing and conforms)
#   1  one or more files rejected — each reason is printed to stderr
#   2  usage error
#   3  cannot validate (no python3, or no yaml module). Callers must FAIL
#      CLOSED on 3: an unvalidated file must not reach a root-level install.

set -uo pipefail

if [[ "${BASH_SOURCE[0]}" != "${0}" ]]; then
    echo "Error: execute this script, don't source it." >&2
    return 1 2>/dev/null || exit 1
fi

if [ "$#" -lt 1 ]; then
    echo "Usage: rosdep_yaml_validate.sh <rosdep.yaml> [<rosdep.yaml> ...]" >&2
    exit 2
fi

if ! command -v python3 >/dev/null 2>&1 \
   || ! python3 -c 'import yaml' >/dev/null 2>&1; then
    echo "rosdep-yaml: cannot validate — python3 with the yaml module is required." >&2
    exit 3
fi

python3 - "$@" <<'PY'
import io
import re
import sys

import yaml

KEY_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._+-]*$")
OS_RE = re.compile(r"^[a-z0-9][a-z0-9._-]*$")
PKG_RE = re.compile(r"^[A-Za-z0-9][A-Za-z0-9._+-]*$")
# Named only to make the message specific; ANY nested mapping is rejected.
INSTALLER_KEYS = {"pip", "npm", "gem", "source", "homebrew", "macports"}

rejected = 0
for path in sys.argv[1:]:
    problems = []
    try:
        raw = io.open(path, encoding="utf-8").read()
    except OSError as exc:
        problems.append("cannot be read: %s" % exc)
        raw = None
    if raw is not None:
        try:
            data = yaml.safe_load(raw)
        except yaml.YAMLError as exc:
            problems.append("not valid YAML: %s" % exc)
            data = None
        else:
            if data is None:
                data = {}
            if not isinstance(data, dict):
                problems.append("top level is not a mapping of rosdep keys")
                data = {}
            for key, rules in data.items():
                if not isinstance(key, str) or not KEY_RE.match(key):
                    problems.append("rosdep key %r is not a plain key name" % (key,))
                    continue
                if not isinstance(rules, dict):
                    problems.append(
                        "key '%s': value must be a mapping of OS name to a package "
                        "list, e.g.  ubuntu: [pkg]" % key)
                    continue
                for osname, pkgs in rules.items():
                    where = "key '%s', OS '%s'" % (key, osname)
                    if not isinstance(osname, str) or not OS_RE.match(osname):
                        problems.append("%s: not a plain OS name" % where)
                        continue
                    if isinstance(pkgs, dict):
                        offenders = sorted(
                            str(k) for k in pkgs if str(k) in INSTALLER_KEYS)
                        detail = (
                            " — %s rule(s) are not accepted here"
                            % ", ".join("'%s'" % o for o in offenders)
                            if offenders else
                            " — nested mappings (installer rules, codename keys)"
                            " are not accepted here")
                        problems.append(
                            "%s: must be a LIST of system package names%s"
                            % (where, detail))
                        continue
                    if not isinstance(pkgs, list):
                        problems.append(
                            "%s: must be a LIST of system package names, "
                            "e.g. [pkg] — got %s"
                            % (where, type(pkgs).__name__))
                        continue
                    for pkg in pkgs:
                        if not isinstance(pkg, str) or not PKG_RE.match(pkg):
                            problems.append(
                                "%s: %r is not a plain package name" % (where, pkg))
    if problems:
        rejected += 1
        sys.stderr.write("❌ REJECTED %s\n" % path)
        for p in problems:
            sys.stderr.write("     %s\n" % p)

if rejected:
    sys.stderr.write(
        "rosdep-yaml: %d file(s) rejected. The workspace accepts exactly\n"
        "     <key>:\n"
        "       <os>: [<package>, ...]\n"
        "   See .agent/knowledge/dependency_policy.md (#654).\n" % rejected)
    sys.exit(1)
sys.exit(0)
PY
