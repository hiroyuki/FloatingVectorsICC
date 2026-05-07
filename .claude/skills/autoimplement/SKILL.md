---
name: autoimplement
description: Run claude-basics/auto_implement_issues.sh against eligible GitHub issues. Handles the Windows bash process-substitution bug by iterating per-issue, and cleans up stale research worktrees. Use when the user types /autoimplement.
user_invocable: true
---

# autoimplement

Run `claude-basics/auto_implement_issues.sh` to auto-research / auto-implement eligible GitHub issues, then report results.

## Arguments

- No args → process all eligible issues
- `<N>` → process only issue #N

## Steps

### 1. Enumerate target issues

The script's default (no-arg) path uses `<(...)` process substitution, which is broken on Windows Git Bash (`jq: Could not open /proc/<pid>/fd/<n>`). **Always invoke with `--issue N`** — even for multi-issue runs, iterate per-ID.

If the user passed an issue number, that's the target list. Otherwise enumerate via `gh`:

```bash
gh issue list --state open --search "label:auto-ok -label:auto-implemented"      --limit 10 --json number --jq '.[].number'
gh issue list --state open --search "label:research -label:auto-researched"      --limit 10 --json number --jq '.[].number'
gh issue list --state open --search "label:not-fixed"                            --limit 10 --json number --jq '.[].number'
```

Merge and deduplicate the numbers. If empty, say so and stop.

### 2. Handle stale worktrees

For each target N, inspect `.worktrees/issue-N`:

```bash
git -C .worktrees/issue-N status --porcelain 2>/dev/null
```

Decide how to handle dirty state:

- **Only `RESEARCH_FINDINGS.md` is dirty** → auto-delete and proceed without asking. The content is already posted to the issue as a comment, so it's expected leftover from a prior research run.
- **Any other uncommitted/untracked files present** (with or without `RESEARCH_FINDINGS.md`) → quote them to the user and ask whether to discard. Don't auto-touch.
- **Clean** → proceed.

Cleanup (after auto-discard or user confirmation):

```bash
rm -f .worktrees/issue-N/<files>
git worktree remove .worktrees/issue-N
git branch -D auto/issue-N 2>/dev/null || true
```

Use `git worktree remove` (not `rm -rf`) so git's safety checks apply.

### 3. Run the script

For each target N, background the script and arm a Monitor on its output:

```bash
bash claude-basics/auto_implement_issues.sh --issue N
```

Monitor filter (tail the background task's output file):

```
^=== |^  rc=|^  codex:|^  posted |^  labeled |^  WARN:|^  merged |^AUTO_RESULT:|^CODEX_|^===== Summary|^DONE:|^SKIPPED:|^FAILED:|^remaining|Fetching |Found |launching Claude|Error|error:|Traceback
```

Brief updates at key moments — don't narrate every notification. Group related events.

### 4. Handle SKIPPED edge cases

If the summary shows `SKIPPED: N (worktree has local changes)`, that's the stale-worktree case — go back to step 2 for that issue and retry.

### 5. Report

When everything finishes, summarize:
- Per-issue outcome: researched / implemented / skipped / failed
- Commit SHA for implemented issues
- Next steps: review posted comments, `git push` for merged branches

## Notes

- Script never pushes to remote. It auto-merges implemented branches into `main` (unless `--no-auto-merge`).
- Default per-issue budget is $5 (override with `--budget`).
- Full log at `.worktrees/.auto_logs/issue-N.log`.
- Label flow:
  - `auto-ok` → implement → `auto-implemented`
  - `research` → research → `auto-researched`
  - `not-fixed` → re-run; on completion: `-not-fixed +need-confirm`
