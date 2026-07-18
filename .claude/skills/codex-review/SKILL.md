---
name: codex-review
description: Send the current plan or uncommitted code changes to OpenAI Codex for iterative review via the Codex companion runtime (app-server). Claude and Codex go back-and-forth until Codex approves. Max 5 rounds. In code-review mode, after approval auto-commits and merges the feature branch into main (no push).
user_invocable: true
---

# Codex Iterative Review (Plan + Code) with Auto-Commit/Merge

Send the current implementation plan or uncommitted code changes to OpenAI Codex for review. Claude revises based on Codex's feedback and re-submits until Codex approves. Max 5 rounds.

On code-review approval, automatically stage+commit the reviewed changes and merge the feature branch back into main (local only — never pushes).

> **Why the companion runtime (not `codex exec`)**: On a ChatGPT-account login,
> `codex exec` (and any explicit `-m gpt-5.x-codex`) returns
> `400 ... model is not supported when using Codex with a ChatGPT account`.
> The Codex **companion runtime** (`codex-companion.mjs`, the same path
> `/codex:review` uses) talks to `codex app-server` and works with ChatGPT auth
> **as long as no unsupported model is forced**. This skill therefore routes every
> Codex call through `codex-companion.mjs task` with the **default model**
> (no `-m`). Do not reintroduce `-m gpt-5.3-codex` — it will 400.

---

## When to Invoke

- When the user runs `/codex-review` during or after plan mode (plan review)
- When the user runs `/codex-review` after implementing code changes (code review)
- When the user wants a second opinion from a different model

## Agent Instructions

When invoked, perform the following iterative review loop.

### Step 0: Resolve the companion runtime

All Codex calls go through the companion script. Resolve its path once (picks the
newest installed plugin version) and reuse it for every round:

```bash
COMPANION=$(ls -t "$HOME/.claude/plugins/cache/openai-codex/codex/"*/scripts/codex-companion.mjs 2>/dev/null | head -1)
[ -z "$COMPANION" ] && echo "Codex companion not found — is the openai-codex plugin installed?" && exit 1
# Sanity check the runtime + auth are ready:
node "$COMPANION" setup --json | head -40
```

If `setup --json` reports `"ready": false`, stop and surface its `nextSteps`
(usually `!codex login` or `npm install -g @openai/codex`).

### Step 0b: Stale-runtime recovery (do this FIRST if any Codex call model-gates)

**Symptom**: every `task` call — even with **no `-m`** — fails with
`400 ... The '<model>' model is not supported when using Codex with a ChatGPT
account`, or returns `status != 0` with an empty `rawOutput`. This persists across
`codex login`, `npm install` version changes, and workspace re-login.

**Root cause (observed)**: the companion broker (`app-server-broker.mjs`) keeps a
long-lived `codex app-server` process alive and **reuses it**. If that server was
first started under an older/newer codex whose default model (e.g. `gpt-5.3-codex`)
is not entitled for the account's plan/workspace, the bad default is pinned for the
life of the process — upgrading/downgrading the `codex` CLI does **not** restart it,
so the gating error keeps coming back even though the CLI on disk is fine.

**Fix**: kill the stale runtime so the next `task` spawns a fresh app-server:

```bash
# Show what's running (optional):
ps aux | grep -E 'codex.*app-server|app-server-broker' | grep -v grep
# Kill the broker + app-server (portable; ignore "no process" errors):
pkill -f 'app-server-broker\.mjs' 2>/dev/null
pkill -f 'codex app-server'      2>/dev/null
# On Windows/MSYS where pkill is absent, kill by PID from the ps line above:
#   ps aux | grep -E 'codex.*app-server|app-server-broker' | grep -v grep | awk '{print $2}' | xargs -r kill
```

Then re-run the failing `task` command once. The companion prints
`No shared Codex runtime is active yet ... will start one on demand` and the fresh
server picks up the on-disk codex default, which works. Do **not** downgrade the CLI
or switch workspaces to chase this — it is a stale-process issue, not an account or
version issue. (Only if a *fresh* runtime still gates every model is it a real
account/plan/workspace entitlement problem; then fall back to `!codex login`,
default-workspace change, or `!codex login --with-api-key`.)

### Step 1: Generate Session ID

Generate a unique ID to avoid conflicts with other concurrent Claude Code sessions
(`uuidgen` is not available on all platforms, so use a timestamp):

```bash
REVIEW_ID=$(date +%s | tail -c 9)
```

Use this for the temp file paths: `/tmp/claude-review-${REVIEW_ID}.md` (the content
sent to Codex) and `/tmp/codex-review-${REVIEW_ID}.json` (Codex's JSON response).

### Step 2: Detect Review Mode & Capture Content

Determine what to review based on context:

**Mode A — Plan Review** (if a plan exists in the current conversation context):
1. Write the full plan content to `/tmp/claude-review-${REVIEW_ID}.md`
2. Set `REVIEW_MODE=plan`

**Mode B — Code Review** (if no plan, but uncommitted changes exist):
1. Run `git diff` and `git diff --cached` to capture all staged and unstaged changes
2. Write the combined diff output to `/tmp/claude-review-${REVIEW_ID}.md`
3. Set `REVIEW_MODE=code`

**No content**: If there is no plan in context AND no uncommitted changes, ask the user what they want reviewed.

Tell the user which mode was detected: "Detected **plan review** mode" or "Detected **code review** mode — reviewing uncommitted changes".

### Step 3: Initial Review (Round 1)

Run Codex through the companion runtime in the **foreground** with `--fresh` (new
thread) and `--json` (so you can reliably parse the result). Do **not** pass `-m`
unless the user explicitly requested a model (see Notes). The prompt differs by mode.

**For Plan Review (`REVIEW_MODE=plan`):**

```bash
node "$COMPANION" task --fresh --json \
  "Review the implementation plan in /tmp/claude-review-${REVIEW_ID}.md. Read it, then focus on:
1. Correctness - Will this plan achieve the stated goals?
2. Risks - What could go wrong? Edge cases? Data loss?
3. Missing steps - Is anything forgotten?
4. Alternatives - Is there a simpler or better approach?
5. Security - Any security concerns?

Be specific and actionable. If the plan is solid and ready to implement, end your review with exactly: VERDICT: APPROVED
If changes are needed, end with exactly: VERDICT: REVISE" \
  > /tmp/codex-review-${REVIEW_ID}.json 2>/tmp/codex-review-${REVIEW_ID}.err
```

**For Code Review (`REVIEW_MODE=code`):**

```bash
node "$COMPANION" task --fresh --json \
  "Review the code changes (git diff) in /tmp/claude-review-${REVIEW_ID}.md. Also read the relevant source files in the repo for full context. Focus on:
1. Correctness / bug risk - Will this code work as intended?
2. Regression risk - Could this break existing functionality?
3. Edge cases - Are boundary conditions handled?
4. Security - Any security vulnerabilities introduced?

Classify each issue as:
- BLOCKING: Must fix before merge (bugs, security, data loss risk)
- NON_BLOCKING: Improvement suggestions (can be addressed later)

Report only real problems. Skip style nitpicks unless they could cause bugs.

If the code is solid and ready to merge, end your review with exactly: VERDICT: APPROVED
If changes are needed, end with exactly: VERDICT: REVISE" \
  > /tmp/codex-review-${REVIEW_ID}.json 2>/tmp/codex-review-${REVIEW_ID}.err
```

**Notes:**
- The companion `task` runs `read-only` by default (no `--write`), so Codex can read
  the repo for context but cannot modify anything. Do NOT add `--write`.
- **Model**: omit `-m` to use the default (the only thing guaranteed to work on a
  ChatGPT account). If the user explicitly passed a model (`/codex-review <model>`),
  add `-m <model>`, but warn them that explicit `gpt-5.x-codex` models 400 on
  ChatGPT-account logins — they need API-key auth (`!codex login --with-api-key`) for
  those.
- The companion keeps the thread per-repo; subsequent rounds use `--resume-last`
  (see Step 6) instead of a captured session id.

### Step 4: Read Review & Check Verdict

1. Read `/tmp/codex-review-${REVIEW_ID}.json`. The review text is the `rawOutput`
   field. Extract it with the Read tool, or in the same bash shell with
   `python3 -c "import json,sys; print(json.load(open(sys.argv[1]))['rawOutput'])" /tmp/codex-review-${REVIEW_ID}.json`
   or `jq -r .rawOutput /tmp/codex-review-${REVIEW_ID}.json`.
   (Do NOT use `node -e "...'/tmp/...'"` on Windows: bash redirects to the MSYS `/tmp`
   but Node resolves `/tmp` to `C:\tmp`, so they disagree. `python3`/`jq`/cat read the
   same MSYS `/tmp` as the redirect.)
   - If `status` is non-zero or `rawOutput` is empty, the run failed — show the
     `.err` file (often the model-gating 400). If it is the model 400 and you forced
     a model, retry once without `-m`. **If it still model-gates with no `-m`, apply
     Step 0b (stale-runtime recovery: kill the broker + app-server, then retry once)
     before concluding it is an account problem.** Otherwise surface the error and stop.
2. Present Codex's review to the user:

```
## Codex Review — Round N [Plan|Code]

[Codex's rawOutput here]
```

3. Check the verdict (search `rawOutput` for the marker):
   - If **VERDICT: APPROVED** → go to Step 7 (Done)
   - If **VERDICT: REVISE** → go to Step 5 (Revise & Re-submit)
   - If no clear verdict but feedback is all positive / no actionable items → treat as approved
   - If max rounds (5) reached → go to Step 7 with a note that max rounds hit

### Step 5: Revise

Based on Codex's feedback, the revision approach differs by mode:

**Plan Review**: Revise the plan — address each issue Codex raised. Update the plan in conversation context and rewrite `/tmp/claude-review-${REVIEW_ID}.md`.

**Code Review**: Fix the code — apply changes to the actual source files using Edit tool to address BLOCKING issues. Then regenerate the diff and rewrite `/tmp/claude-review-${REVIEW_ID}.md` with the updated diff.

In both modes, briefly summarize what was changed:

```
### Revisions (Round N)
- [What was changed and why, one bullet per Codex issue addressed]
```

Inform the user: "Sending revised [plan|code] back to Codex for re-review..."

### Step 6: Re-submit to Codex (Rounds 2-5)

Resume the **same** Codex thread (so it has full context of the prior rounds) with
`--resume-last`:

```bash
node "$COMPANION" task --resume-last --json \
  "I've revised the [plan|code] based on your feedback. The updated content is in /tmp/claude-review-${REVIEW_ID}.md.

Here's what I changed:
[List the specific changes made]

Please re-review. If it's now solid, end with: VERDICT: APPROVED
If more changes are needed, end with: VERDICT: REVISE" \
  > /tmp/codex-review-${REVIEW_ID}.json 2>/tmp/codex-review-${REVIEW_ID}.err
```

`--resume-last` continues the most recent task thread for this repository. (If you
are running multiple concurrent Codex tasks in the same repo this could grab the
wrong thread; for a normal single review loop it is correct.)

Then go back to **Step 4** (Read Review & Check Verdict).

**Important:** If `--resume-last` fails (e.g., "No previous Codex task thread"),
fall back to a fresh `--fresh` call with context about the prior rounds included in
the prompt.

### Step 7: Present Final Result

Once approved (or max rounds reached):

```
## Codex Review — Final [Plan|Code]

**Status:** Approved after N round(s)

[Final Codex feedback / approval message]

---
**[Plan|Code] has been reviewed and approved by Codex.**
```

If max rounds were reached without approval:

```
## Codex Review — Final [Plan|Code]

**Status:** Max rounds (5) reached — not fully approved

**Remaining concerns:**
[List unresolved issues from last review]

---
**Codex still has concerns. Review the remaining items and decide whether to proceed.**
```

### Step 8: Auto-commit & Auto-merge (code mode only, on APPROVED)

This step runs **only** when all of the following are true:
- `REVIEW_MODE=code`
- Codex verdict is **APPROVED** (or max-rounds reached with code already fixed — treat max-rounds as NOT approved, skip auto-merge)
- The user has NOT said "review only" / "don't commit" / "don't merge" in the current invocation

Skip this step silently for plan-review mode.

**Substep 8a — Generate commit message**

Claude writes a 1–2 line imperative commit message from the diff it already reviewed. Format:

```
<short imperative subject, under 70 chars>

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
```

Do not mention the review process in the subject — describe the code change itself.

**Substep 8b — Stage and commit in the current worktree**

Stage only files that appeared in the reviewed diff (do NOT `git add -A` or `.`). Commit with the message from 8a.

```bash
# From the current worktree (where the changes are)
git add <files-from-diff>
git commit -m "$(cat <<'EOF'
<subject>

Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>
EOF
)"
```

If the pre-commit hook fails: report the failure to the user and stop. Do NOT use `--no-verify`. Do NOT amend. The user fixes the hook issue and re-invokes.

**Substep 8c — Determine merge target**

1. Get the current branch: `git rev-parse --abbrev-ref HEAD`
2. Determine the main branch name: `git symbolic-ref --short refs/remotes/origin/HEAD 2>/dev/null | sed 's@^origin/@@'` — fallback to `main` if not set.
3. If current branch == main branch → already on main, skip merge (commit alone is the result). Go to Step 9.
4. Otherwise, the current branch is a feature branch. Proceed to merge.

**Substep 8d — Locate the main worktree**

`git worktree list --porcelain` lists all worktrees. Find the one whose `branch` is `refs/heads/<main-branch>`. That's the main worktree path.

If no worktree is checked out on main (e.g., main was detached or main branch has no worktree), stop and tell the user: they need to check out main somewhere before auto-merge can run.

**Substep 8e — Merge feature branch into main**

```bash
# From the main worktree directory
cd <main-worktree-path>
git merge --no-ff -m "Merge branch '<feature-branch>'" <feature-branch>
```

Handle outcomes:

- **Clean merge**: report success with the new main HEAD hash. Done.
- **Conflict**: stop immediately. Report which files conflict. Do NOT try to auto-resolve — tell the user to resolve and commit manually, or to abort with `git merge --abort`.
- **Other failure**: report the exact error; do not retry.

**Substep 8f — Safety constraints (must not violate)**

- NEVER run `git push` (push is a separate, explicit user action)
- NEVER use `--force`, `--force-with-lease`, or `--no-verify`
- NEVER delete the worktree or the feature branch (per project memory: worktree deletion crashes running servers)
- NEVER touch the remote in any way

After a successful merge, tell the user:
> Merged `<feature-branch>` into `<main-branch>` (commit `<hash>`). Worktree and feature branch left in place — delete manually when ready (stop any server running from the worktree first).

### Step 9: Cleanup

Remove the session-scoped temporary files:
```bash
rm -f /tmp/claude-review-${REVIEW_ID}.md /tmp/codex-review-${REVIEW_ID}.json /tmp/codex-review-${REVIEW_ID}.err
```

## Loop Summary

```
Round 1: Claude sends [plan|diff] → companion task --fresh → Codex reviews → REVISE?
Round 2: Claude revises [plan|code] → companion task --resume-last → Codex re-reviews → REVISE?
Round 3: Claude revises → companion task --resume-last → APPROVED
  ↓ (code mode only)
Auto-commit in current worktree → Auto-merge feature branch into main (local, no push)
```

Max 5 rounds. Each round preserves Codex's conversation context via `--resume-last`.

## Rules

- Every Codex call goes through `node "$COMPANION" task` (the app-server companion
  runtime), NOT `codex exec`. `codex exec` 400s on ChatGPT-account logins.
- **Default model (no `-m`)**. Only pass `-m <model>` when the user explicitly asked
  for one, and warn that explicit `gpt-5.x-codex` models 400 on ChatGPT accounts
  (API-key auth required for those).
- Claude **actively revises** based on Codex feedback between rounds — this is NOT just passing messages, Claude should make real improvements
- In code review mode, Claude fixes BLOCKING issues only. NON_BLOCKING items are reported but not auto-fixed
- In code review mode, if a fix contradicts the user's explicit requirements or the project's CLAUDE.md rules, skip that fix and note it for the user
- The companion `task` runs read-only (no `--write`) — Codex never modifies files
- Max 5 review rounds to prevent infinite loops
- Show the user each round's feedback and revisions so they can follow along
- If the companion is missing or `setup --json` is not ready, inform the user and suggest `npm install -g @openai/codex` then `!codex login`

### Auto-commit & auto-merge rules (Step 8, code mode only)

- Runs only on **VERDICT: APPROVED** in code-review mode. Never for plan review. Never on max-rounds-reached-without-approval.
- Skip if the user has said "review only", "don't commit", or "don't merge" in the current invocation.
- Stage only files that appeared in the reviewed diff — never `git add -A` / `.` (avoids staging secrets or unrelated untracked files).
- Commit message: Claude writes a 1–2 line imperative subject from the diff it reviewed. Always include `Co-Authored-By: Claude Opus 4.7 (1M context) <noreply@anthropic.com>`.
- On pre-commit hook failure: stop and report. Do NOT `--no-verify`. Do NOT amend.
- Merge target: the branch pointed to by `origin/HEAD` (default `main`), merged with `--no-ff`. If already on main, skip the merge step.
- On merge conflict: stop immediately. Do not attempt to auto-resolve. Tell the user which files conflicted and let them resolve manually.
- NEVER `git push`, `--force`, `--force-with-lease`, or `--no-verify`.
- NEVER delete the worktree or feature branch (worktree deletion can crash a server running from its cwd).
- After a successful merge, explicitly remind the user to stop any server running from the worktree before they delete it.
