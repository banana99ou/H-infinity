# 논문 작성 프로젝트

## 프로젝트 개요

- **논문 제목**: Curvature-Velocity Scheduled LPV H∞ Path Following via VFG
- **대상 저널**: IJAT (International Journal of Automotive Technology, KSAE/Springer, SCIE Q2)
- **컨트롤러**: PID-FF, MPC, LPV-Hinf (3종)
- **주요 키워드**: Path following, VFG, LPV control, H∞ synthesis, Gain scheduling, Autonomous vehicles
- **디렉토리**: `latex-ijat/` (영문), `latex-ko-ijat/` (한국어 번역)

## 디렉토리 구조

```
.
├── latex-ijat/               # 영문 원고
│   ├── paper_ijat.tex
│   ├── sections/             # 6개 섹션
│   ├── figures/              # 그림 (fig_*.pdf) — scripts/plot/ 출력 경로
│   ├── tables/               # LaTeX 표 (.tex) — DuckDB 쿼리 출력 경로
│   └── references.bib
├── latex-ko-ijat/            # 한국어 번역본
│   ├── paper_ijat_ko.tex
│   ├── sections/
│   ├── figures → ../latex-ijat/figures   (심볼릭 링크)
│   ├── tables  → ../latex-ijat/tables    (심볼릭 링크)
│   └── references.bib → ../latex-ijat/references.bib
├── scripts/
│   ├── analysis/
│   │   ├── matlab/           # MATLAB H∞ 합성 스크립트
│   │   └── python/           # Python 시뮬레이션·분석
│   │       ├── models/
│   │       ├── guidance/
│   │       ├── controllers/  # pid_ff, mpc_linear, hinf_lpv
│   │       ├── paths/        # step_curvature, slalom
│   │       ├── simulation/
│   │       ├── analysis/
│   │       ├── database/     # DuckDB 파이프라인
│   │       ├── version_registry.py
│   │       ├── run_phase2.py
│   │       └── run_phase3.py
│   └── plot/                 # 그래프 생성 (fig_*.py)
├── data/
│   ├── raw/
│   │   ├── phase2/           # 결정론적 시뮬레이션 (.npz, git 미추적)
│   │   └── phase3/           # Monte Carlo (.npz, git 미추적)
│   └── controllers/          # v3 컨트롤러 JSON
├── plans/                    # 계획서 (YAML front matter)
├── submission/               # 투고 스냅샷
├── traceability.md           # 그림·표·수치 추적관리
├── environment.yml           # conda 의존성
├── Makefile
└── README.md
```

## 그림 출력 경로 규칙

- `scripts/plot/fig_*.py --paper a` → `latex-ijat/figures/fig_*.pdf`
- `scripts/plot/config.py`의 `PAPER_SCOPES['paper_a']`에서 `output_dir` 확인.
- 데이터 경로는 상대 경로를 사용하라.

## 추적관리 문서 (traceability.md)

프로젝트 루트에 `traceability.md`를 유지하라. 논문 내 모든 그림, 표, 본문 수치가 어디서 생성되었는지 추적한다.

### 관리 대상

- **그림(Figure)**: 그림 파일 경로, 생성 스크립트 경로, 사용된 데이터 소스
- **표(Table)**: 수치의 데이터 소스, 생성 스크립트 (있는 경우)
- **본문 수치**: 본문에 등장하는 정량적 수치와 해당 데이터 소스

### 문서 형식

```markdown
# Traceability

## Figures

| Figure | 파일 경로 | 생성 스크립트 | 데이터 소스 | 비고 |
|---|---|---|---|---|
| Fig. 1 | latex-ijat/figures/fig1.pdf | scripts/plot/fig1.py | data/raw/phase2/ | |

## Tables

| Table | 위치 (섹션) | 생성 스크립트 | 데이터 소스 | 비고 |
|---|---|---|---|---|

## In-text Values

| 수치 | 위치 (섹션/문장) | 데이터 소스 | 산출 방법 | 비고 |
|---|---|---|---|---|
```

### 갱신 규칙

- 그림, 표, 본문 수치를 생성하거나 수정할 때 반드시 `traceability.md`를 함께 업데이트하라.
- 기존 항목을 삭제할 때도 문서에서 제거하라.
- 스크립트가 없이 수동으로 생성된 항목은 비고에 "수동 생성"이라고 기재하라.

## 작성 규칙

### 영어 표현

- 한국어 화자가 읽기에 자연스러운 간결한 학술 영어를 사용하라.
- 짧은 문장을 선호하라. 한 문장에 절이 3개 이상 이어지면 분리하라.
- 불필요한 수동태, 형식적 표현(it is worth noting that 등)을 피하라.

### LaTeX 규칙

- 벡터: `\mathbf{x}` (볼드 소문자)
- 행렬: `\mathbf{A}` (볼드 대문자)
- 스칼라: 이탤릭 (기본)
- 집합: `\mathcal{S}` (캘리그래피)
- 수식 번호는 참조되는 수식에만 부여하라.
- `\label`, `\ref`, `\cite` 키는 일관된 네이밍을 유지하라.
  - 수식: `eq:이름` (예: `eq:dynamics`)
  - 그림: `fig:이름` (예: `fig:trajectory`)
  - 표: `tab:이름` (예: `tab:results`)
  - 섹션: `sec:이름` (예: `sec:method`)

### 참고문헌

- BibTeX cite key: 저자연도 형식 (예: Kim2023)
- 모든 참고문헌은 `latex-ijat/references.bib`에서 관리하라.
- 본문에 없는 논문을 인용하지 마라.

## 주의사항

- 본문에 없는 내용을 추가하지 마라.
- 정량적 수치는 데이터/원고의 값을 정확히 인용하라.
- 파일 수정 전 변경 사항을 사용자에게 보고하라.

## Git Commit Guidelines

커밋 메시지 작성 규칙:
- **한국어 개조식**으로 간결하게 작성
- "Claude Code로 작성" 등 AI 도구 관련 문구 **포함하지 않음**
- Co-Authored-By 태그 **사용하지 않음**

예시:
```
IJAT 시뮬레이션 수치 업데이트

- Phase 2/3 데이터 재생성
- tab:phase2_step_curv, tab:phase3_mc 갱신
```

## Python 환경 설정

- 모든 Python 스크립트는 conda 가상환경에서 실행
- 가상환경 디렉토리: `./env`
- Python 실행: `./env/bin/python`
- 의존성 관리: `environment.yml`

## 프레임워크 버전 관리

- 중앙 레지스트리: `scripts/analysis/python/version_registry.py`
- `VERSIONS` dict에 v1 (legacy), v3 (active) 2개 버전 정의
- `ACTIVE_VERSION = 'v3'` — Paper A 최종 버전
- 컨트롤러 JSON: `data/controllers/phase2v3_K_vertices.json`

## 데이터 관리 방침

- `.npz` 파일은 git에서 제외 (`.gitignore`)
- 로컬 보관 + DuckDB로 쿼리 관리
- DB 파일: `data/sim.duckdb` (git 미추적)
- 추후 연구실 MinIO/NocoDB 서버로 이전 예정
