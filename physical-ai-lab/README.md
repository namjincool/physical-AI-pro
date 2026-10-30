# 단일 관절 로봇 팔 PD 제어 실습

첨부한 「MuJoCo 세부 실습 가이드 — 가상 로봇 팔 제어 프로젝트」를 실행 가능한
Python 프로젝트로 보완했다. 사용자 소유 OpenCrab 팩의 근거를 조회하여 실험 설계에 반영했다.
이 모델은 교육용으로 작성된 단일 링크이며 실제 로봇의 복제 모델이 아니다.
정책을 학습하지 않는 전통적인 피드백 제어 실습이다.

## 실행

프로젝트 폴더에서 PowerShell을 연다. 새 환경을 만드는 경우:

```powershell
python -m venv .venv
.\.venv\Scripts\python.exe -m pip install -r requirements.txt
.\.venv\Scripts\python.exe main.py --target 45
.\.venv\Scripts\python.exe main.py --target 45 --viewer --output results\viewer45
.\.venv\Scripts\python.exe run_experiments.py
.\.venv\Scripts\python.exe -m pytest -q tests
```

이 폴더는 원 작업공간에서 날짜별 전달용으로 복사한 프로젝트다.
현재 위치에서 `.venv`를 새로 만들어 위 명령을 사용한다.
상위 폴더에 이미 있는 가상환경이나 원 작업공간의 환경은 변경하지 않는다.

가상환경 활성화나 PowerShell 실행 정책 변경은 필요하지 않다.
`requirements.txt`는 이번 실행의 직접 의존성 버전이고,
`requirements-lock.txt`는 전이 의존성까지 포함한 설치 기록이다.
프로젝트를 다른 위치로 복사하면 새 가상환경을 만들고 설치한다.

## 원본에서 보완한 내용

- 끊어진 `main.py`의 import, 제어 루프, 뷰어, 저장·분석 코드를 완성했다.
- 모델의 링크 길이 0.6 m, 질량 1 kg, 관절 감쇠 1 N·m·s/rad,
  중력 0, timestep 0.002 s, 입력 범위 ±10 N·m를 유지했다.
- 베이스 구는 시각적 피벗이므로 충돌 마스크를 0으로 설정했다.
  원점에서 겹치는 베이스와 링크의 접촉이 제어 실험에 개입하지 않도록 한 설계 수정이다.
- 목표각은 CLI에서 degree로 받고 내부에서는 radian으로 변환한다.
- 목표가 관절 한계 [-3.14, 3.14] rad를 벗어나거나 인자가 NaN/무한대이면 거부한다.
- 실험별 폴더로 로그가 섞이는 것을 방지했다. 같은 출력 경로를 다시 지정하면 덮어쓴다.
- GUI를 중간에 닫은 실험은 `completed=false`로 기록하며 완료 실험으로 판정하지 않는다.

## 제어 흐름과 단위

1. **State:** `MjData`의 현재 관절 위치 `qpos`와 각속도 `qvel`.
2. **Observation:** 매 스텝 관측하는 `joint1`의 각도(rad), 각속도(rad/s).
   이번 실습은 센서 잡음·지연이 없는 시뮬레이터 상태 관측이다.
3. **목표:** 입력한 degree 값을 radian으로 변환한 고정 목표 각도. 목표 속도는 0.
4. **PD Control:** `tau_raw = kp * (target - q) - kd * qvel`.
   Kp 단위는 N·m/rad, Kd 단위는 N·m·s/rad.
5. **Action:** ±10 N·m로 제한한 토크를 `data.ctrl`에 입력.
   이 모델은 hinge에 `motor`를 gear=1로 연결했으므로 입력을 관절 토크로 해석한다.
6. **Physics:** `mujoco.mj_step(model, data)`가 실제 MuJoCo 동역학을 계산한다.
7. **Feedback:** 계산 뒤 상태를 기록하고, 다음 스텝에서 새 관측으로 토크를 다시 결정한다.

제어 루프에서 `qpos`를 목표값으로 대입하지 않는다. `data.ctrl`을 설정하는 것만으로
관절이 목표각에 순간 이동하지 않는다. 관절 자체의 감쇠도 있으므로 Kd=0이
전체 감쇠가 0이라는 뜻은 아니다. 중력이 0이라 실물의 중력 보상 성능은 검증하지 않는다.

## 실험과 판정 기준

각 실험은 초기 각도·각속도 0에서 시작하고 3초 동안 1500스텝 계산한다.
GUI 없는 실행은 시뮬레이션 시간 기준이며, 실제 경과 시간 3초를 기다릴 필요가 없다.
GUI 모드는 타이밍을 근사적으로 맞추지만 실시간 실행을 보장하지 않는다.

| 실험 | 목표(deg) | Kp | Kd | 비교 목적 |
| --- | ---: | ---: | ---: | --- |
| A_target45 | 45 | 15 | 3 | 기준 PD 제어 |
| B_target90 | 90 | 15 | 3 | 목표 변경 및 토크 포화 |
| C_kp5 | 45 | 5 | 3 | P 이득 변경 |
| D_kd1 | 45 | 15 | 1 | D 이득 변경 |
| E_no_control | 45 | 0 | 0 | 입력 없는 기준 방법 |

**교육용 판정 기준:** 지정된 시간 전체를 계산하고, 마지막 0.5초 구간의 최대 절대 각도
오차가 1도 이내이면 `passed=true`다. 원본·공식 문서의 인증 기준이 아니라 이번 실습의
비교용 기준이다. 실행 전에 코드에 정의했으며, 판정 실패도 결과로 보관한다.
사용자가 짧은 duration을 지정하면 마지막 구간은 실제 계산한 구간으로 한정된다.

- 최종 오차: 목표 - 최종 관측 각도. 부호를 보존한다.
- 마지막 구간 오차: 마지막 0.5초의 최대 절대 오차.
- 정착 시간: 관측된 실행 구간에서 마지막으로 ±1도 밖이었던 표본의 다음 표본 시각.
  실행 종료 이후까지 안정함을 증명하는 값이 아니다. 끝까지 범위 밖이면 null.
- 오버슈트: 목표 방향으로 목표를 초과한 최대 각도(deg).
- 포화 비율: 원래 PD 토크와 제한 후 입력이 달랐던 스텝 비율.
- RMSE: 물리 스텝 이후 상태들의 각도 오차 RMS이며 초기 표본은 제외한다.

실제 수행 결과는 `RESULTS.md`, `results/summary.csv`, 각 실험의 `metrics.json`에서 확인한다.

## 결과 파일 읽기

`main.py`의 기본 출력은 `results/joint_log.csv`, `joint_angle.png`, `metrics.json`이다.
`run_experiments.py`는 실험별 폴더와 종합 그래프 `results/comparison.png`를 생성한다.

CSV 한 행은 **한 물리 스텝**이다. `time_start_s`에서 관측한 각도·속도·오차로
`torque_applied_nm`를 결정하고, `time_end_s`에서 그 입력의 물리 계산 결과를 기록한다.
`angle_before_deg`와 `angle_after_deg`를 구분하므로 상태와 입력의 시점이 혼동되지 않는다.
JSON에는 환경 버전, 초기 상태, 중력, timestep, 입력 한계, 모델·코드 SHA-256을 남긴다.
관측값은 숫자로 복사해 저장하므로 MuJoCo 배열의 공유 메모리 참조를 누적하지 않는다.

## 단계별 학습 과제

아래 예상 질문은 학습 과제이며 실행 결과를 미리 확정한 문장이 아니다.

| 단계 | 직접 바꿀 변수·수행 | 실행 전 예상 질문 | 확인 방법 |
| --- | --- | --- | --- |
| 환경 | 사용할 Python 실행 경로 | 다른 Python으로 실행하면 같은 패키지가 보일까? | `--version`, `pip show`, import 확인 |
| 모델 | 링크 질량 또는 길이 하나 | 관성이 바뀌면 응답은 어떻게 바뀔까? | 다른 조건을 고정하고 별도 폴더로 실행 |
| 시간 진행 | timestep 하나 | 같은 3초에서 스텝 수는 어떻게 바뀔까? | JSON의 시간·steps 확인 |
| 피드백 | Kp 또는 Kd 하나 | 응답 속도·오버슈트가 어떻게 바뀔까? | A/C/D 그래프와 수치 비교 |
| 목표 | 45도에서 90도 | 토크 제한이 동작에 영향을 줄까? | B의 raw/applied 토크·포화 비율 |
| 평가 | 초기 조건 또는 오차 허용치 하나 | 성공 판정의 범위가 달라질까? | 기존 결과와 구분해 새 실험 기록 |

모든 변경은 원본을 복사하거나 버전 관리 후 수행한다. 모델을 변경했다면 해시가 바뀐
실험끼리 동일 모델 결과로 섞지 않는다. 학습용 코드가 실행된 것과 개념을 이해한 것은
구분하고, 변경 이유·예상·관찰·실패를 자기 말로 기록한다.

### 확인 문제 해설

- `data.ctrl[0]`에 명령을 넣은 다음 물리 계산을 진행하는 함수는 **`mujoco.mj_step()`**이다.
- XML에서 `joint`만 삭제하면, 남아 있는 motor가 없는 관절을 참조해 모델 컴파일 오류가
  발생한다. motor도 제거하면 이 모델의 팔은 world에 고정되고 회전 자유도가 사라진다.
  이 두 경우를 실제 MuJoCo 테스트로 구분한다.
- P/D 값을 사람이 정한 것이므로 이 실습은 정책 학습을 수행한 AI 모델이 아니다.

## 온톨로지 팩 근거와 적용 범위

OpenCrab에서 2026-10-29 조회한 사용자 팩:

- 이름: Physical AI — 근거 기반 개념·모델·데이터·도구, v1.0.0
- package_id: `[内部 ID omitted]`
- `01_Physical_AI_개념과기술구조.md` / 「무엇을 기록해야 검증 가능한가」
  evidence `[内部 ID omitted]`: 초기 조건·환경·단위·평가·실패 기록.
- 같은 문서 / 「먼저 구별해야 할 표현」
  evidence `[内部 ID omitted]`: 상태 직접 변경과 제어 입력의 차이.
- `03_공식도구와실습경로.md` / 「첫 실습: MuJoCo 공식 예제로 물리 루프 확인」
  evidence `[内部 ID omitted]`: 변수 하나 변경·예상과 관찰 비교·재현 절차.

조회 자료의 공개용 원칙 요약은 `evidence/pack_evidence.json`에 있다. 팩을 수정하거나 실험 결과를
OpenCrab에 업로드하지 않았다. 팩은 실습 원칙의 근거이며 이 코드·이득·성공 결과를
제공한 출처가 아니다. XML은 첨부 예제를 보완했고, 제어 코드와 평가 기준은 이번에
구현했다. 이미지 프록시 링크·출처 이름만 있는 표시는 API 검증 근거로 사용하지 않았다.

## 공식 API 근거

- [MuJoCo Python: 모델·상태·뷰어](https://mujoco.readthedocs.io/en/stable/python.html)
- [MJCF XML Reference: joint, motor, option](https://mujoco.readthedocs.io/en/stable/XMLreference.html)
- [Simulation loop: 입력과 mj_step](https://mujoco.readthedocs.io/en/stable/programming/simulation.html#simulation-loop)

## 실패 원인과 한계

- 원본 Python 코드가 잘려 있고 줄바꿈이 사라짐: 실행 가능한 파일로 재작성했다.
- 설치 때 sandbox 임시 폴더 쓰기 제한: 임시 경로를 허용된 작업 폴더로 지정했다.
- 초기 패키지 다운로드 네트워크 제한: 사용자에게 네트워크 권한을 요청한 후 설치했다.
- OS는 Windows 11, Python은 3.13.7로 확인했다. GPU 드라이버 정보의 CIM 조회는
  제한되어 GPU 모델·드라이버 버전을 확인했다고 주장하지 않는다.
- 실물 장비, 센서 노이즈, 지연, 중력 하중, 다관절 제어, 강화학습·VLA 성능은 검증하지 않는다.
- GUI는 headless 물리 계산과 별도 확인한다. 실제 GUI 확인 범위는 `RESULTS.md`에 기록한다.
