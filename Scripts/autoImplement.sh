#!/usr/bin/env bash
# Auto-implement / auto-research GitHub issues via Claude Code (+ /codex-review).
#
# Usage:
#   scripts/auto_implement_issues.sh [--impl-label auto-ok] [--research-label research]
#                                    [--limit 5] [--budget 5] [--dry-run]
#
# Modes (chosen per issue from its labels):
#   * research label → investigate only, post findings as comment, mark "auto-researched"
#   * impl label     → implement, run /codex-review until APPROVED, commit on auto/issue-N
#                      branch, comment on issue, mark "auto-implemented" (no merge, no push)
#   * both labels    → research mode wins (safer: no code changes)
#
# Safety:
#   - Only issues carrying impl/research label are picked up
#   - Already-done issues (auto-implemented / auto-researched) are excluded
#   - Never pushes, never merges to main
#   - --budget caps per-issue Claude spend in USD (default 5)

set -euo pipefail

IMPL_LABEL="auto-ok"
RESEARCH_LABEL="research"
DONE_IMPL_LABEL="auto-implemented"
DONE_RESEARCH_LABEL="auto-researched"
LIMIT=5
BUDGET=5
DRY_RUN=0
TARGET_ISSUE=""
AUTO_MERGE=1

while [[ $# -gt 0 ]]; do
  case "$1" in
    --impl-label)     IMPL_LABEL="$2"; shift 2;;
    --research-label) RESEARCH_LABEL="$2"; shift 2;;
    --limit)          LIMIT="$2"; shift 2;;
    --budget)         BUDGET="$2"; shift 2;;
    --issue)          TARGET_ISSUE="$2"; shift 2;;
    --no-auto-merge)  AUTO_MERGE=0; shift;;
    --dry-run)        DRY_RUN=1; shift;;
    -h|--help)        sed -n '2,22p' "$0"; exit 0;;
    *) echo "unknown arg: $1" >&2; exit 2;;
  esac
done

REPO_ROOT="$(git rev-parse --show-toplevel)"
WT_ROOT="$REPO_ROOT/.worktrees"
LOG_DIR="$WT_ROOT/.auto_logs"
mkdir -p "$LOG_DIR"

gh label create "$DONE_IMPL_LABEL"     --color 0E8A16 \
  --description "Auto-implemented by scripts/auto_implement_issues.sh" >/dev/null 2>&1 || true
gh label create "$DONE_RESEARCH_LABEL" --color 1D76DB \
  --description "Auto-researched by scripts/auto_implement_issues.sh" >/dev/null 2>&1 || true

if [[ -n "$TARGET_ISSUE" ]]; then
  echo "Fetching single issue #$TARGET_ISSUE..."
  RAW=$(gh issue view "$TARGET_ISSUE" --json number,title,url,body,labels)
  HAS_RESEARCH=$(jq --arg L "$RESEARCH_LABEL" '[.labels[].name] | index($L) != null' <<<"$RAW")
  HAS_DONE_RESEARCH=$(jq --arg L "$DONE_RESEARCH_LABEL" '[.labels[].name] | index($L) != null' <<<"$RAW")
  if [[ "$HAS_RESEARCH" == "true" && "$HAS_DONE_RESEARCH" == "false" ]]; then
    MODE_FORCED="research"
  else
    MODE_FORCED="implement"
  fi
  ISSUES=$(jq --arg m "$MODE_FORCED" \
    '[{number, title, url, body, mode:$m}]' <<<"$RAW")
else
  # Fetch both label sets, tag with mode, dedupe (research wins when both present).
  echo "Fetching issues: impl=$IMPL_LABEL, research=$RESEARCH_LABEL (limit=$LIMIT each)..."
  RESEARCH_ISSUES=$(gh issue list --state open --limit "$LIMIT" \
    --search "label:$RESEARCH_LABEL -label:$DONE_RESEARCH_LABEL" \
    --json number,title,url,body \
    | jq '[.[] | . + {mode:"research"}]')
  IMPL_ISSUES=$(gh issue list --state open --limit "$LIMIT" \
    --search "label:$IMPL_LABEL -label:$DONE_IMPL_LABEL" \
    --json number,title,url,body \
    | jq '[.[] | . + {mode:"implement"}]')
  ISSUES=$(printf '%s\n%s\n' "$RESEARCH_ISSUES" "$IMPL_ISSUES" \
    | jq -s --argjson lim "$LIMIT" \
        '(.[0] + .[1]) | unique_by(.number) | .[:$lim]')
fi
COUNT=$(jq 'length' <<<"$ISSUES")
echo "Found $COUNT issue(s)"
[[ "$COUNT" -eq 0 ]] && exit 0

if [[ "$DRY_RUN" == "1" ]]; then
  jq -r '.[] | "[\(.mode)] #\(.number) \(.title)  \(.url)"' <<<"$ISSUES"
  exit 0
fi

SUCCEEDED=()
FAILED=()
SKIPPED=()

for i in $(seq 0 $((COUNT-1))); do
  NUM=$(jq -r ".[$i].number" <<<"$ISSUES")
  TITLE=$(jq -r ".[$i].title" <<<"$ISSUES")
  URL=$(jq -r ".[$i].url" <<<"$ISSUES")
  BODY=$(jq -r ".[$i].body" <<<"$ISSUES")
  MODE=$(jq -r ".[$i].mode" <<<"$ISSUES")

  # Pull comments so impl mode sees prior research findings / user decisions.
  COMMENTS=$(gh issue view "$NUM" --json comments \
    --jq '.comments[] | "--- @\(.author.login) (\(.createdAt)) ---\n\(.body)\n"' 2>/dev/null || true)

  BRANCH="auto/issue-$NUM"
  WT="$WT_ROOT/issue-$NUM"
  LOG="$LOG_DIR/issue-$NUM.log"
  TESTPLAN_PATH="$LOG_DIR/issue-$NUM.testplan.md"
  rm -f "$TESTPLAN_PATH"

  echo ""
  echo "=== [$MODE] Issue #$NUM: $TITLE ==="

  if [[ -d "$WT" ]]; then
    echo "  worktree exists at $WT — skipping"
    SKIPPED+=("#$NUM (worktree exists)")
    continue
  fi
  if git -C "$REPO_ROOT" show-ref --verify --quiet "refs/heads/$BRANCH"; then
    echo "  branch $BRANCH exists — skipping"
    SKIPPED+=("#$NUM (branch exists)")
    continue
  fi

  echo "  creating worktree: $WT (branch $BRANCH from main)"
  git -C "$REPO_ROOT" worktree add -b "$BRANCH" "$WT" main >/dev/null

  if [[ "$MODE" == "research" ]]; then
    FINDINGS_PATH="$WT/RESEARCH_FINDINGS.md"
    PROMPT=$(cat <<EOF
GitHub Issue #$NUM の**調査のみ**をお願いします。コード変更は禁止です。

URL: $URL
Title: $TITLE

Body:
$BODY

手順:
1. issueの質問・調査依頼の意図を把握する。
2. 関連コード・ログ・DBスキーマ等を読んで原因や論点を特定する。
3. **コードは変更しない**（git status がクリーンのまま終了すること）。
4. 調査結果を \`RESEARCH_FINDINGS.md\` に以下の構成で書き出す:
   - ## TL;DR (3行以内)
   - ## 調査範囲 (ファイル/関数を**テーブル**で列挙: | ファイル:行 | 役割 |)
   - ## 所見 (**箇条書き**中心。根拠の行番号を括弧で付ける)
   - ## 対応案 (選択肢を**テーブル**で比較: | 案 | 変更点 | メリット | デメリット |。末尾に「推奨: ...」1行)
   - ## 未解決 (**箇条書き**、1項目1-2行)
5. 完了したら最後に一行 "AUTO_RESULT: RESEARCHED" と出力する。調査不能なら "AUTO_RESULT: SKIP <理由>" で終了。

スタイル制約（厳守）:
- 散文は連続3行以内。3行超えそうなら箇条書きかテーブルに分解する。
- コードブロックは最小限、diff/該当行のみ引用。長い貼り付けは禁止。
- 書きすぎない。読み手はタイトルと TL;DR → テーブル → 推奨の順に見る。

作業ディレクトリは既に worktree ($WT) に入った状態で起動されている。
EOF
)
  else
    PROMPT=$(cat <<EOF
GitHub Issue #$NUM の実装をお願いします。

URL: $URL
Title: $TITLE

Body:
$BODY

Comments (過去の調査結果や方針決定が含まれる — 必ず目を通すこと):
${COMMENTS:-(コメントなし)}

重要な制約（違反は破壊的回帰を招くので厳守すること）:
- 変更範囲は issue と直接関係するコード・ロジックのみに限定する。
- issue に関係しない既存機能・カラム・関数・テスト・migration・UIフィールドを**削除・改名・後退させない**。
- 変更前後で「git diff main..HEAD -- 該当領域外のファイル」に意味のある変更が出るならそれは禁止。
- issue 文面とコメントで言及されていない挙動は一切触らない。疑わしければ触らない。
- 制約を満たせず実装が広がりそうなら、実装せず "AUTO_RESULT: SKIP 範囲を限定できないため" で終了する。

手順:
1. issue内容と過去コメントを読んで変更範囲を特定する。関係ないファイルの変更は一切しない。
2. 要求が曖昧、または制約を満たせない場合は、コード変更せず最後に一行 "AUTO_RESULT: SKIP <理由>" とだけ出力して終了する。
3. 実装したら /codex-review skill を**必ず**呼ぶ。APPROVED が出るまで修正をループする（最大5ラウンド）。
4. /codex-review の各ラウンド終了時に、Codex の verdict 要約を一行 "CODEX_ROUND <n>: <APPROVED|CHANGES_REQUESTED|...> - <要約>" で出力する。
5. 最終的に APPROVED になったら、approval文の最初の一行をそのまま "CODEX_APPROVED: <Codexの一言>" として出力する。APPROVED に到達できなかったら "AUTO_RESULT: NO_APPROVAL <理由>" を出して終了（コミットしない）。
6. APPROVED 後、CLAUDE.md のルールに従って現在のブランチ ($BRANCH) に commit する。
7. commit 後、このIssueの実装を検証するテスト方法を $TESTPLAN_PATH に書き出す（このパスは worktree 外・git 管理対象外なので commit しない）。以下の構成:
   - ## 前提 (必要な機材・ビルド/シーン構成を**箇条書き**で2-4項目)
   - ## 手順 (番号付きで3-7項目、1項目あたり1-2行)
   - ## 期待結果 (**箇条書き**、観察すべき挙動を1項目1行)
   - ## 回帰チェック (**箇条書き**、変更と干渉しそうな既存機能の動作確認を1-3項目)
   スタイル: 散文は連続3行以内、コードブロックは該当コマンドやdiffに限り最小限。長文禁止。
8. main へのマージや push は絶対にしない。ユーザーが後で手動で行う。
9. 完了したら**最終 assistant メッセージに必ず**以下を含めて終了する（-p モードは途中出力を捨てるため、途中で書いても意味がない。必ず最後のメッセージに書く）:
   - "CODEX_ROUND <n>: ..." 行（ラウンド数分）
   - "CODEX_APPROVED: ..." 行
   - "AUTO_RESULT: DONE <commit-sha>" 行

重要: /codex-review を実行せずに commit するのは禁止。最終メッセージに CODEX_APPROVED 行が無い＝失敗扱いでマージされない。$TESTPLAN_PATH が空または未作成の場合、issue コメントのテスト方法欄が欠落する。

作業ディレクトリは既に worktree ($WT / branch $BRANCH) に入った状態で起動されている。
EOF
)
  fi

  echo "  launching Claude (mode=$MODE, budget=\$$BUDGET, log=$LOG)"
  set +e
  ( cd "$WT" && claude -p "$PROMPT" \
      --dangerously-skip-permissions \
      --max-budget-usd "$BUDGET" \
  ) >"$LOG" 2>&1
  RC=$?
  set -e

  RESULT_LINE=$(grep -E '^AUTO_RESULT:' "$LOG" | tail -1 || true)
  echo "  rc=$RC  result=${RESULT_LINE:-<none>}"

  if [[ "$RC" -ne 0 ]]; then
    FAILED+=("#$NUM [$MODE] (rc=$RC)")
    continue
  fi

  if [[ "$MODE" == "research" ]]; then
    if [[ "$RESULT_LINE" == AUTO_RESULT:\ RESEARCHED* && -s "$FINDINGS_PATH" ]]; then
      COMMENT=$(printf '🔍 Auto-researched by `scripts/auto_implement_issues.sh`.\n\n---\n\n%s\n' \
        "$(cat "$FINDINGS_PATH")")
      if gh issue comment "$NUM" --body "$COMMENT" >/dev/null 2>&1; then
        echo "  posted findings to issue #$NUM"
      else
        echo "  WARN: failed to post findings to issue #$NUM"
      fi
      if gh issue edit "$NUM" --add-label "$DONE_RESEARCH_LABEL" >/dev/null 2>&1; then
        echo "  labeled issue #$NUM: $DONE_RESEARCH_LABEL"
      else
        echo "  WARN: failed to add label $DONE_RESEARCH_LABEL to issue #$NUM"
      fi
      SUCCEEDED+=("#$NUM [research] findings posted")
      # research mode leaves no commits — drop the throwaway worktree + branch
      git -C "$REPO_ROOT" worktree remove --force "$WT" >/dev/null 2>&1 || true
      git -C "$REPO_ROOT" branch -D "$BRANCH" >/dev/null 2>&1 || true
    elif [[ "$RESULT_LINE" == AUTO_RESULT:\ SKIP* ]]; then
      SKIPPED+=("#$NUM [research] $RESULT_LINE")
    else
      FAILED+=("#$NUM [research] (no findings file or bad marker)")
    fi
    continue
  fi

  # implement mode
  APPROVED_LINE=$(grep -E '^CODEX_APPROVED:' "$LOG" | tail -1 || true)
  echo "  codex: ${APPROVED_LINE:-<no approval line>}"

  if [[ "$RESULT_LINE" == AUTO_RESULT:\ DONE* ]]; then
    if [[ -z "$APPROVED_LINE" ]]; then
      FAILED+=("#$NUM [impl] (commit without CODEX_APPROVED — review not verified)")
    else
      SHA=$(awk '/^AUTO_RESULT: DONE/ {print $3; exit}' "$LOG")
      CODEX_MSG=${APPROVED_LINE#CODEX_APPROVED: }
      SUBJECT=$(git -C "$WT" log -1 --pretty=%s "$SHA" 2>/dev/null || echo "$TITLE")

      MERGE_STATUS="skipped (--no-auto-merge)"
      if [[ "$AUTO_MERGE" == "1" ]]; then
        CUR=$(git -C "$REPO_ROOT" symbolic-ref --short HEAD 2>/dev/null || echo "")
        DIRTY=$(git -C "$REPO_ROOT" status --porcelain -uno)
        if [[ "$CUR" != "main" ]]; then
          MERGE_STATUS="⚠ skipped (REPO_ROOT HEAD=$CUR, expected main)"
        elif [[ -n "$DIRTY" ]]; then
          MERGE_STATUS="⚠ skipped (uncommitted tracked changes in main worktree)"
        elif git -C "$REPO_ROOT" merge --no-ff "$BRANCH" \
               -m "Merge branch '$BRANCH' (auto issue #$NUM)" >/dev/null 2>&1; then
          MERGE_STATUS="✅ merged to main (no push)"
          echo "  merged $BRANCH into main"
          git -C "$REPO_ROOT" worktree remove "$WT" >/dev/null 2>&1 || true
        else
          git -C "$REPO_ROOT" merge --abort >/dev/null 2>&1 || true
          MERGE_STATUS="⚠ merge failed (conflicts?) — branch left for manual merge"
          echo "  WARN: merge failed for $BRANCH; aborted and left branch"
        fi
      fi

      if [[ -s "$TESTPLAN_PATH" ]]; then
        TEST_SECTION=$(printf '\n## テスト方法\n\n%s\n' "$(cat "$TESTPLAN_PATH")")
      else
        TEST_SECTION=$'\n## テスト方法\n\n(テストプランは生成されませんでした)\n'
      fi

      COMMENT=$(cat <<COMMENT_EOF
🤖 Auto-implemented by \`scripts/auto_implement_issues.sh\`.

- branch: \`$BRANCH\`
- commit: \`${SHA:0:7}\` — $SUBJECT
- codex review: ✅ APPROVED — $CODEX_MSG
- merge: $MERGE_STATUS
$TEST_SECTION
Push は手動で。
COMMENT_EOF
)
      if gh issue comment "$NUM" --body "$COMMENT" >/dev/null 2>&1; then
        echo "  posted comment to issue #$NUM"
      else
        echo "  WARN: failed to post comment to issue #$NUM"
      fi
      if gh issue edit "$NUM" --add-label "$DONE_IMPL_LABEL" >/dev/null 2>&1; then
        echo "  labeled issue #$NUM: $DONE_IMPL_LABEL"
      else
        echo "  WARN: failed to add label $DONE_IMPL_LABEL to issue #$NUM"
      fi
      SUCCEEDED+=("#$NUM [impl] $RESULT_LINE")
    fi
  elif [[ "$RESULT_LINE" == AUTO_RESULT:\ SKIP* ]]; then
    SKIPPED+=("#$NUM [impl] $RESULT_LINE")
  elif [[ "$RESULT_LINE" == AUTO_RESULT:\ NO_APPROVAL* ]]; then
    FAILED+=("#$NUM [impl] $RESULT_LINE")
  else
    FAILED+=("#$NUM [impl] (no AUTO_RESULT line)")
  fi
done

echo ""
echo "===== Summary ====="
printf 'DONE:    %d\n' "${#SUCCEEDED[@]}"
for s in "${SUCCEEDED[@]:-}"; do [[ -n "$s" ]] && echo "  $s"; done
printf 'SKIPPED: %d\n' "${#SKIPPED[@]}"
for s in "${SKIPPED[@]:-}"; do [[ -n "$s" ]] && echo "  $s"; done
printf 'FAILED:  %d\n' "${#FAILED[@]}"
for s in "${FAILED[@]:-}"; do [[ -n "$s" ]] && echo "  $s"; done

echo ""
echo "remaining branches (un-merged):"
git -C "$REPO_ROOT" branch --list 'auto/issue-*' || true
echo ""
echo "next step (manual): git push  (merge はAPPROVED分自動実行済み)"
