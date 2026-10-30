# physical-AI-pro

MuJoCo로 단일 관절 로봇 팔의 PD 피드백 제어를 구현하고, 입력·상태·물리 계산·평가를
연결해 이해하는 Physical AI 입문 프로젝트입니다. **작업 기준일은 2026-10-29,
GitHub 등록일은 2026-10-30(Asia/Seoul)**입니다.

## 먼저 읽기

- [수행 과정과 실제 결과](01_금일수행결과.md)
- [소스코드 쉬운 해설](02_소스코드_쉬운해설.md)
- [프로젝트 실행·학습 가이드](physical-ai-lab/README.md)
- [전체 실험 보고서](physical-ai-lab/RESULTS.md)
- [10월 29일 작업 과정 기록](docs/2026-10-29-worklog.md)
- [파일 목록과 SHA-256](03_파일목록.csv)

## 실행

Windows PowerShell에서 저장소를 받은 후:

```powershell
cd physical-ai-lab
python -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
.\.venv\Scripts\python.exe main.py --target 45 --viewer --output results\practice45
.\.venv\Scripts\python.exe run_experiments.py
.\.venv\Scripts\python.exe -m pytest -q tests
```

GUI 없이 실행하려면 `--viewer`를 생략합니다. 직접 실험할 때는 기존 기록을 보존하도록
별도 `--output` 경로를 지정하세요. `run_experiments.py`는 A~E 기록을 다시 생성합니다.

## 확인한 결과

3초·1500스텝, 초기 각도/속도 0, 중력 0, 입력 토크 ±10 N·m에서 수행했습니다.

| 실험 | 조건 | 실제 최종 각도 | 평가 |
| --- | --- | ---: | --- |
| A | 목표45°, Kp15/Kd3 | 44.999879° | 통과 |
| B | 목표90°, Kp15/Kd3 | 89.999739° | 통과 |
| C | 목표45°, Kp5/Kd3 | 44.057763° | 마지막 구간 오차 기준 미달 |
| D | 목표45°, Kp15/Kd1 | 45.000000° | 통과, 약1.61° 오버슈트 |
| E | 제어 입력0 | 0° | 기준 방법: 목표 미도달 |

통과 기준은 ‘3초 완료 + 마지막 0.5초 전체에서 오차 ±1도 이내’라는 교육용 기준입니다.
테스트 13개 통과, 45° GUI의 초기화 및 1500스텝 완료 기록을 보관했습니다.
이는 2026-10-29의 로컬 실행 기록이며 GitHub Actions 실행 결과가 아닙니다.

![실제 MuJoCo 실험 비교](physical-ai-lab/results/comparison.png)

## 구성과 범위

- XML: 링크0.6m·질량1kg·회전관절·토크 모터.
- Python: 관측 → PD → 토크 제한 → `mj_step` → 로그·평가.
- 결과: 실험별 CSV, 수치 JSON, 그래프 및 3D 렌더링.
- 근거: 사용자 Physical AI 온톨로지 팩의 상태/입력 구분·재현성·실패 기록 원칙.

사람이 이득을 정한 전통적 제어 실습입니다. 정책 학습, VLA, 실제 로봇 전이는 수행하지
않았습니다. 공개 등록본에는 개인 PC 절대경로, 비공개 팩 내부 식별자, 가상환경과
캐시를 포함하지 않습니다. 소스코드와 실험 로그는 원래 실행본의 바이트를 보존했습니다.
