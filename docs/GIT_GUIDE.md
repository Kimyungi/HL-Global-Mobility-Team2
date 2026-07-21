# 팀원용 Git 가이드 — VS Code 버튼만으로 하기

터미널 명령 없이 VS Code 화면 클릭만으로 개발 → 커밋 → PR까지 가는 순서.
왼쪽 세로 아이콘 바에서 **가지 모양 아이콘 = Source Control(소스 제어)** 이 기본 도구다.

## 0. 최초 1회 — 저장소 받기

1. VS Code 실행 → `Ctrl+Shift+P` → "**Git: Clone**" 입력·선택
2. URL 붙여넣기: `https://github.com/Kimyungi/HL-Global-Mobility-Team2.git`
3. 저장할 폴더 선택 → 우하단 "열기" 클릭 → GitHub 로그인 창이 뜨면 본인 계정으로 로그인
4. 저장소를 열면 우하단에 **"이 저장소의 권장 확장을 설치하시겠습니까?"** 팝업이 뜬다 → **설치** 클릭
   (안 뜨면: 왼쪽 Extensions 아이콘 → "GitHub Pull Requests" 검색 → Install)
5. 빌드가 되는지 확인 (이것만 터미널 — VS Code 메뉴 Terminal → New Terminal):
   ```bash
   source /opt/ros/humble/setup.bash && colcon build --symlink-install
   ```

## 1. 작업 시작 — 내 브랜치 만들기

1. **좌측 하단 상태바의 브랜치 이름(`main`) 클릭**
2. 목록 맨 위 "**+ Create new branch...**" 클릭
3. 이름 입력 — 규칙: `feat/담당-내용` (예: `feat/lane-yolo`, `feat/avoid-ttc`)
4. 좌하단이 새 브랜치 이름으로 바뀌었는지 확인. **이제부터 수정은 전부 이 브랜치에 쌓인다.**

> ⚠️ 시작 전에 좌하단이 `main`인 상태에서 Source Control 패널 우상단 `⋯` → **Pull**을 한 번 눌러 최신을 받고 나서 브랜치를 만들 것.

## 2. 커밋 — 작업 저장하기

1. 코드 수정 후 왼쪽 **Source Control 아이콘** 클릭 — 바뀐 파일 목록(Changes)이 보인다
2. 파일에 마우스 올리면 나오는 **+** 클릭 (전체는 Changes 줄의 +) → Staged Changes로 이동
3. 위 입력창에 한 줄 설명 입력 (예: "차선 검출 YOLO 추론 연결")
4. **✓ Commit** 버튼 클릭

커밋은 하루에 여러 번, 작게 자주 하는 게 좋다. "되는 상태"가 될 때마다 저장한다고 생각하면 된다.

## 3. 올리기 — push

- 커밋 후 Source Control에 나타나는 **"Publish Branch"** (첫 push) 또는 **"Sync Changes ↑"** 버튼 클릭. 끝.

## 4. PR 만들기 — "머지해주세요" 요청

방법 A (제일 쉬움): push 직후 우하단에 뜨는 **"Create Pull Request?"** 알림 클릭
방법 B: 왼쪽 **GitHub 아이콘**(PR 확장) → 위쪽 **Create Pull Request** 버튼

1. base가 `main`, 대상이 내 브랜치인지 확인
2. 제목·설명 쓰고 (뭘 했는지, 어떻게 테스트했는지) → **Create** 클릭
3. 이후는 팀장(김윤기)이 리뷰·머지. 수정 요청이 달리면 코드 고치고 §2~3 반복하면 PR에 자동 반영된다.

## 5. 머지된 뒤 — 최신 main 받기

1. 좌하단 브랜치 클릭 → `main` 선택
2. Source Control `⋯` → **Pull**
3. 다음 작업은 다시 §1부터 (브랜치는 매 작업마다 새로 만든다)

## 팀 규칙 3줄

1. **main에 직접 push 금지** — 모든 변경은 브랜치 → PR → 팀장 머지
2. **fma_interfaces·CLAUDE.md·adas_mgm 수정은 사전에 팀장과 상의** — 전원 영향
3. **`colcon build` 통과 상태로만 PR** — 깨진 코드가 main에 오면 전원이 멈춘다

## 팀장(김윤기) 전용 — PR 리뷰·머지

1. 이메일 알림 또는 GitHub 웹 "Pull requests" 탭 / VS Code GitHub 아이콘에서 PR 열기
2. "Files changed"에서 diff 확인 — 체크 3가지: ① 자기 폴더만 건드렸나 ② 빌드 되나 ③ REQUIREMENTS.md 계약 지켰나
3. 문제없으면 웹의 초록 **Merge pull request** 버튼 (VS Code에서도 가능) → 브랜치 삭제(Delete branch) 버튼도 눌러 정리
4. 애매하면 Claude Code에 "PR N번 검토해줘"라고 시키기
