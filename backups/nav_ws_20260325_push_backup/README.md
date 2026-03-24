# nav_ws Backup Snapshot

Created on `2026-03-25` to preserve the important local `nav_ws` changes inside `sentry_planner` before pushing.

## What is included

- `files/`
  - Direct copies of key top-level `nav_ws` files that were locally modified.
- `nav_ws_top_level.diff`
  - Patch for tracked top-level `nav_ws` changes, excluding `build/`, `install/`, and `log/`.
- `subrepo_patches/`
  - `git diff` output for nested repos inside `nav_ws/src/`.
- `untracked_files/`
  - Source-like untracked files copied from `nav_ws` and nested repos.
- `repo_status.txt`
  - Status snapshot showing what was modified and untracked at backup time.

## Notes

- `git diff` patch files only cover tracked file modifications.
- Newly created files are preserved under `untracked_files/`.
- Build artifacts were intentionally excluded where possible.
- This snapshot is meant as a recoverable backup, not a clean importable workspace.
