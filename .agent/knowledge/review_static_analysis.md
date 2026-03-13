# Static Analysis Specialist Reference

Tool configurations for the static analysis specialist in `/review-code`.
Configs are split by domain: **ament** (ROS package code) and **workspace**
(infrastructure scripts and configs).

## File Classification

| File location | Domain | Language |
|---|---|---|
| `layers/*/src/**/*.py` | ament | Python |
| `layers/*/src/**/*.cpp`, `*.hpp`, `*.h` | ament | C++ |
| `layers/*/src/**/*.xml` | ament | XML |
| `layers/*/src/**/*.yaml`, `*.yml` | ament | YAML |
| `layers/*/src/**/CMakeLists.txt` | ament | CMake |
| `.agent/scripts/*.py` | workspace | Python |
| `.agent/scripts/*.sh`, `scripts/*.sh` | workspace | Shell |
| Root `*.yaml`, `*.yml` | workspace | YAML |
| Root `*.md` | — | skip (no linting) |

## Python — Ament Profile (ROS packages)

Matches `ament_flake8` defaults from `/opt/ros/jazzy/lib/python3.12/site-packages/ament_flake8/configuration/ament_flake8.ini`.

```bash
flake8 --max-line-length=99 \
  --import-order-style=google \
  --extend-ignore="B902,C816,D100,D101,D102,D103,D104,D105,D106,D107,D203,D212,D404,I202" \
  --show-source --statistics \
  <changed-py-files>
```

Optional deeper check (if mypy is available):

```bash
mypy --ignore-missing-imports --no-error-summary <changed-py-files>
```

## Python — Workspace Profile

Matches pre-commit config (`.pre-commit-config.yaml`).

```bash
flake8 --max-line-length=100 \
  --extend-ignore="E203,W503,E402" \
  <changed-py-files>
```

## C++ — Ament Profile

### cpplint

Matches `ament_cpplint` defaults.

```bash
cpplint --counting=detailed \
  --linelength=100 \
  --filter=-build/c++11,-runtime/references,-whitespace/braces,-whitespace/indent,-whitespace/parens,-whitespace/semicolon \
  <changed-cpp-files>
```

### cppcheck

Matches `ament_cppcheck` defaults.

```bash
cppcheck -f --inline-suppr -q \
  --suppress=internalAstError \
  --suppress=unknownMacro \
  <changed-cpp-files>
```

### clang-tidy (optional, not run by ament by default)

```bash
clang-tidy <changed-cpp-files> -- -std=c++17
```

Note: clang-tidy requires a compilation database (`compile_commands.json`).
Only run if one exists in the build directory. Skip otherwise.

## Shell — Workspace Profile

Matches pre-commit shellcheck config.

```bash
shellcheck --severity=warning <changed-sh-files>
```

## YAML

Matches pre-commit yamllint config.

```bash
yamllint -d '{extends: default, rules: {line-length: {max: 120}, document-start: disable}}' \
  <changed-yaml-files>
```

## XML

```bash
xmllint --noout <changed-xml-files>
```

## CMake — Ament Profile

Matches `ament_lint_cmake` defaults.

```bash
# ament_lint_cmake is a Python tool, not a standalone command.
# If available in the ROS environment:
python3 -m ament_lint_cmake <changed-cmake-files>
```

## Running the Specialist

1. Classify each changed file using the table above
2. Group files by domain + language
3. Run the appropriate tool with the matching config
4. Collect output — each finding should include:
   - **file**: relative path
   - **line**: line number
   - **tool**: which tool found it
   - **message**: the finding text
5. Filter: only report findings on lines **touched by this PR** (added or
   modified lines in the diff). Findings on unchanged context lines are noise.
6. Pass findings to the lead reviewer for deduplication and severity classification

## Key Alignment Notes

- **Python line length**: ament uses 99, workspace uses 100. This is intentional —
  ament_flake8 has used 99 since inception despite the ROS 2 style guide saying
  "up to 100 characters."
- **flake8 ignore lists are complementary**: ament ignores docstring rules (D100-D107)
  and import order (I202); workspace ignores Black compat rules (E203, W503) and
  import ordering (E402). Neither is a superset of the other.
- **C++ linting is ament-only**: no C++ linters in pre-commit (expected — C++ lives
  in project repos, not workspace infrastructure).
- **Black is workspace-only**: ROS 2 does not use Black. The E203/W503 ignores in
  workspace profile compensate for Black's formatting.
