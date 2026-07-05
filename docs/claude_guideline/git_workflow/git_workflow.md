# Git 커밋·푸시 워크플로 (Git Commit & Push Workflow)

> **본 파일은 지시용.** Git commit/push 와 다중 작업자 협업 규칙의 self-contained 단일 근원(SSOT / Single Source of Truth).

본 코어는 self-contained 다 — 본문 외 가이드라인·도구·Skill 의존 0.

## 설치

본 번들 폴더(`git_workflow/`)의 `install.sh` 로 설치한다:

```bash
cd git_workflow && ./install.sh <타깃-프로젝트-루트>
```

스크립트가 `git_workflow.md` 를 `docs/claude_guideline/git_workflow/` 로 복사하고 등록 스니펫을 타깃 `CLAUDE.md` 에 append 한다. **활성화 게이트**: 본 파일이 그 경로에 없으면 본 룰 비활성.

## 0. 모드 판정 (solo vs team) — README 기록 우선

push·리뷰 방식이 모드에 갈리므로 **작업 전 먼저 판정**한다. 모드는 **기록된 선언이 권위**이며, 자동 감지는 *제안용*일 뿐 단독으로 확정하지 않는다. 우선순위:

1. **README 선언 (최우선)** — 저장소 `README.md` 의 `git 협업 모드: solo|team` 줄. 있으면 그대로 따른다. 사람이 보는 README 가 1차 기록처다.
2. **CLAUDE.md 선언 (fallback)** — README 에 없고 `CLAUDE.md` 에 `git 협업 모드: solo|team` 이 있으면 따른다(기존 설치 호환).
3. **미선언 시 (필수 절차 — 자동 default 금지)** — 둘 다 없으면 **solo 로 임의 진행하지 않는다.** 사용자에게 solo/team 을 1줄 문의 → 답을 `README.md` 에 `git 협업 모드: <solo|team>` 으로 **기록** → 그 모드로 진행한다. 문의 시 아래 자동 감지를 *제안 근거*로 제시한다.

**자동 감지 (제안 근거 — 확정 아님)** — 하나라도 해당하면 team 후보로 사용자에게 제안:

- **GitHub collaborator 2명 이상**: `gh api repos/<owner>/<repo>/collaborators --jq 'length'`. collaborator 는 "작업 권한이 있는 사람" 전체라 초기 단계에서도 정확. **원격마다 다를 수 있으니 각 원격을 확인**한다.
- `CODEOWNERS` 파일 존재, 또는 `main` 브랜치 보호 활성 (`gh api repos/<owner>/<repo>/branches/main/protection`)
- (gh 불가 시) 최근 50 커밋 author 이메일 2개 이상: `git log -50 --format='%ae' | sort -u | wc -l`
- 사용자가 "팀/회사/공유/여러 명" 맥락 언급

**원격별 모드 (미러 주의)** — 미러 원격마다 collaborator 가 다르면 각 원격은 그 모드를 따른다. 한 원격이라도 team 이면 그 원격 `main` 직접 push 가 팀 PR·리뷰·권한 정책을 우회하지 않는지 확인한다. **단, 관리자가 운영하는 단방향 미러 원격**(팀의 활성 개발 대상이 아님)은 직접 push 를 **문서화된 예외**로 허용할 수 있다 — 누가 관리자이고 어느 원격이 미러인지 명시한다. (예: `origin`=solo, `fito`=collaborator 다수지만 관리자 단방향 미러 → `main` 직접 push 예외.)

모드가 바뀌면(예: solo→team 전환) `README.md` 선언을 갱신한다. 판정이 모호하면 사용자에게 1줄 확인.

## 1. 공통 규칙 (solo·team 둘 다)

- **명시 요청에만 commit/push** — "커밋"/"푸쉬"/"커밋 푸쉬" 자연어 트리거. 그 외 자동 commit/push 금지.
- **작업 단위 = 커밋 = push 단위** — 서로 다른 scope 변경을 한 커밋에 섞지 않는다. dirty tree 는 scope 별 분할 커밋.
- **명시 staging (세션 격리)** — `git add <명시 경로>` 만. `git add -A` / `git add .` **금지**. 작업공간을 여러 세션이 공유하면 working tree 에 **타 세션의 미커밋 변경**이 섞일 수 있으므로 **이번 세션이 만든 파일만** staging 한다. commit 직전 `git diff --cached --name-only` 로 staged 범위가 이번 세션 산출물과 일치하는지 검증. 무관한 dirty 파일은 건드리지 않는다(모호하면 1줄 확인).
- **세션 격리 자동화 (설치 시)** — `hooks/git_workflow-track.py`(PostToolUse) 가 이 세션이 수정한 파일을 `.git/git_workflow/sessions/<session_id>/touched` 에 누적하고, `hooks/git_workflow-reminder.py`(UserPromptSubmit) 가 git 트리거 시 그 목록을 주입한다 → staging 대상이 '이 세션 목록'으로 자동 grounding 된다(`.git` 내부라 비-커밋·세션별 분리). python 부재 시 이 자동화는 생략되고 규칙 텍스트만 생존(수동 식별).
- **세션 격리 강제 (staging 게이트)** — `hooks/git_workflow-stage-gate.py`(PreToolUse·Bash) 가 `git add`/`git commit -a` 를 실행 직전 검사해 **하드 차단(deny)**: 광역 staging(`-A`/`.`/`-u`/`-p`)·`git commit -a`·glob 경로, 그리고 이 세션 touched 목록에 없는 **타 세션/미추적 파일**. 멀티 세션이 working tree 를 공유할 때(예: 한 창의 다중 탭) 타 세션 미커밋 파일 **캡처를 능동 차단**한다. 정당한 예외(Bash 산출물 등)는 명령에 `# gw:allow-foreign` 또는 env `GW_ALLOW_FOREIGN=1` 로 우회. **한계(정직)**: 셸 파싱 휴리스틱(`eval`·`xargs`·git alias·`cd &&` 우회 가능), 훅 미설치 세션은 미보호, Bash 로만 만든 파일은 미추적→override 필요, `git commit <path>` 미검사.
- **커밋 메시지** — `type(scope): subject` (`feat`·`fix`·`docs`·`refactor`·`style`·`chore`·`test`). 한국어 본문 허용. `Co-Authored-By` 푸터.
- **파괴 명령 승인** — `git push --force`·`reset --hard`·`clean -f`·브랜치 삭제는 사용자 명시 승인 후에만.
- **push 전 확인** — secrets(`.env`·키·토큰·사설 IP(Internet Protocol)/MAC(Media Access Control)·운영 endpoint) 미포함, 대상 저장소 정확, vendored read-only 가드 파일 미staged.
- **다중 원격 미러** — 미러 원격이 여럿이면 **모두** push (예: 본 저장소 `origin` + `fito`).

## 2. solo 모드 (혼자)

- `main`(또는 현재 추적 분기) **직접 commit + push**. PR(Pull Request) 미사용.
- 다중 원격이면: `git push origin main && git push fito main`.

### 2-1. 세션별 브랜치 관례 (solo + 다중 세션 공유 워킹트리 — 선택)

여러 세션(예: 한 창의 다중 탭)이 **같은 저장소·워킹트리·`main` 을 공유**하면, 각 세션 커밋이 공유 `main` 위에 교차되어 이력 추적·동시 push 충돌 관리가 어렵다. 이때 아래 경량 관례를 적용해 세션 산출물을 이력상 격리한다. **solo 모드 선언은 유지**(§0 불변)하되, 각 세션은 `main` 에 직접 push 하지 않고 자기 브랜치로 격리하고, `main` 반영(merge)은 **사용자가 수행**한다.

> **⚠ 공유 HEAD 주의 — `git switch` 금지, `git worktree` 사용.** 다중 세션이 **하나의 워킹트리·하나의 HEAD** 를 공유하므로(`git worktree list` 로 단일 트리 확인) `git switch`/`git checkout -b` 로 브랜치를 바꾸면 HEAD 가 **전역 이동**해 동시에 작업 중인 **다른 세션까지 그 브랜치로 끌려간다**. 반드시 별도 링크드 워킹트리(`git worktree add`)를 만들어 그 안에서만 브랜치를 다룬다 — 공유 트리의 HEAD 는 불변.

- **적용 조건** — 다중 세션이 워킹트리를 공유할 때만(`git worktree list` 가 트리 1개 + 다른 세션이 같은 트리를 수정). 단일 세션이면 §2 기본(main 직접 push)으로 충분.
- **브랜치 명명** — `session/<session-id>` (예: `session/6c0ee226`). 식별자는 세션 격리 훅(`git_workflow-reminder.py`)이 주입하는 `<session_id>` 를 쓴다.
- **격리 대상 = 이 세션의 산출물만** — **공유 가변 로그**(`docs/user_instructions/user_instructions.md` 등 모든 세션이 append 하는 파일)는 세션 브랜치에 넣지 않는다(브랜치마다 다른 스냅샷 → merge 충돌). 그런 로그는 `main` 에 직접 커밋(§2)하거나 로그 소유 세션이 일괄 커밋한다. 세션 브랜치에는 **그 세션만 만든 코드·문서 산출물**만 담는다.
- **작업 흐름 (worktree)** —
  1. `git worktree add <경로>/kkw-session-<id> -b session/<id>` (공유 HEAD 불변, main 기준 새 브랜치).
  2. 이 세션 산출물을 그 worktree 로 반영 후 `cd` 하여 §1 규칙대로 커밋(명시 staging·`type(scope): subject`+`Co-Authored-By`).
  3. `git -C <경로> push -u origin session/<id>` — **`main` 직접 push 안 함**(다중 원격이면 각 원격에).
  4. `git worktree remove <경로>` 로 링크드 트리 정리.
- **merge (사용자 소관)** — 세션 브랜치를 `main` 에 반영하는 merge 는 **사용자가 수행**한다. Claude 자동 merge 금지. 사용자가 `git merge --no-ff session/<id>` 또는 원격 UI 로 처리.
- **정리** — merge 후 `git branch -d session/<id>` + `git push origin --delete session/<id>` (브랜치 삭제는 §1 파괴 명령 — 사용자 명시 승인 후).
- **team 과의 구분** — 본 관례는 §3 team 의 PR·리뷰 승인 게이트를 도입하지 **않는다**. 브랜치 목적은 협업 리뷰가 아니라 **세션 이력 격리·동시 push 충돌 방지**뿐. team 의 self-merge 금지(§3)는 여기 해당 없음 — merge 주체가 사용자다.

## 3. team 모드 (여럿)

- **`main` 직접 push 금지.** `<type>/<topic>` 브랜치에서 작업(예: `feat/git-workflow`), 짧게 유지.
- **작업 전 최신화** — `git fetch origin` 후 `git switch -c <branch> origin/main`. 진행 중 `git pull --rebase origin main` 로 main 흡수.
- **PR 워크플로** — 브랜치 push → PR 생성 → **리뷰 ≥1 승인 → merge**. 리뷰는 `code_review` 번들 SOP 적용.
- **작성자 self-approve / self-merge 금지** — 별도 작업자가 승인(code_review 의 self-`APPROVE` 금지와 동일 원칙).
- **충돌·force** — 로컬에서 rebase 로 해결. 공유 브랜치 force-push **금지**, 자기 feature 브랜치만 `--force-with-lease` 허용.
- **merge·정리** — squash 또는 merge commit(팀 합의), merge 후 브랜치 삭제.
- **다중 원격 + 팀** — `origin` 이 협업 기준(PR·리뷰), `fito` 는 미러 — merge 후 `git push fito main` 로 동기화.

## 4. GitHub 정책 강제 (team — 선택)

CLAUDE.md/README 규칙은 **권고**이고, GitHub 설정은 **강제**한다. 사람이 실수로 `main` 에 직접 push 하는 것까지 막으려면 team 저장소에 다음을 건다:

- **브랜치 보호** — `main` 에 PR 필수 + 리뷰 ≥1 승인 + 직접 push 차단. 설정/확인:
  ```bash
  gh api repos/<owner>/<repo>/branches/main/protection            # 현재 상태 확인
  gh api -X PUT repos/<owner>/<repo>/branches/main/protection ...  # PR·리뷰 필수 설정
  ```
- **CODEOWNERS** (`.github/CODEOWNERS`) — 경로별 자동 리뷰어 지정 → PR 에 리뷰 자동 요청.
- **PR 템플릿** (`.github/pull_request_template.md`) — 변경 요약·테스트·관련 이슈 체크리스트.

**관리자 단방향 미러 예외** — 미러 원격(예: `fito`)은 브랜치 보호의 "allow specified actors to bypass"(또는 admin bypass)로 **관리자 직접 push 는 허용**하면서 일반 팀원은 PR 을 강제한다 → §0 미러 예외와 양립.

> 브랜치 보호·CODEOWNERS 설정 변경은 저장소 **admin 권한 + 사용자 명시 승인** 후에만 (공유 팀 워크플로에 영향).

## 룰 (요약)

0. **작업 전 협업 모드 확인** — `README.md` 의 `git 협업 모드: solo|team` 선언 우선(없으면 `CLAUDE.md`), 둘 다 없으면 사용자 문의 후 README 기록(자동 default 금지)
1. 명시 요청에만 commit/push (트리거 "커밋"/"푸쉬")
2. 작업 단위 = 커밋 단위, 명시 staging·세션 격리(`-A`/`.` 금지, 이번 세션 산출물만)
3. `type(scope): subject` + `Co-Authored-By`
4. 다중 원격이면 모두 push
5. 파괴 명령은 명시 승인
6. push 전 secrets·대상·vendored 확인
7. **team 모드: `main` 직접 push 금지 → 브랜치 + PR + 리뷰 승인 + merge**
8. 작성자 self-approve/self-merge 금지

## 자체 점검

```bash
# 모드 선언 확인 — README 우선, CLAUDE.md fallback (미선언 시 문의·기록)
grep -hE "git 협업 모드: (solo|team)" README.md CLAUDE.md 2>/dev/null \
  || echo "(모드 미선언 — 자동 default 금지: 사용자 문의 후 README 에 기록)"

# 모드 자동 감지 — GitHub collaborator 수 (≥2 → team), 원격별 확인
for r in $(git remote); do
  url=$(git remote get-url "$r"); slug=$(echo "$url" | sed -E 's#.*github.com[:/]([^/]+/[^/.]+)(\.git)?#\1#')
  echo "$r: $(gh api "repos/$slug/collaborators" --jq 'length' 2>/dev/null || echo '?') collaborators"
done
# (gh 불가 시) 최근 author 수
git log -50 --format='%ae' 2>/dev/null | sort -u | wc -l

# 커밋 메시지 형식 (마지막 커밋)
git log -1 --format='%s' | grep -E "^(feat|fix|docs|refactor|style|chore|test)(\([^)]+\))?: "
```

---

**VERSION**: 1.5.0 (1.4.0 + §2-1 세션별 브랜치 관례(solo 다중세션 공유 워킹트리 — `session/<id>` 브랜치 커밋·push, main merge 는 사용자 소관, team PR·리뷰 게이트 미도입))
