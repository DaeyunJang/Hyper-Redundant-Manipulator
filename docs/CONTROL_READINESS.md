# Position / admittance 실기 준비 검토

## 2026-10-31: 현재 힘 좌표 정의에 맞춘 순응 부호 수정 (현재 구현)

- 사용자가 발산을 보고하고 정지를 확인했다. `f_env=-prediction`을 HRM에
  작용하는 힘으로 정의하고 `x_t=nominal+offset`을 사용하므로, 현재 물리적
  순응식은 `Mδ̈+Bδ̇+Kδ=F_env-F_target`이다. 아래의 direct legacy 복원은
  이 정의와 맞지 않으므로 대체했다. 필터 자체의 6x6 desired-env 계산은
  유지하고 wrapper가 `(-f_desired,-f_env)`로 전달해 필요한 우변을 만든다.
  공개 desired/env 메시지와 노드 입력의 -1은 그대로이며 delta_force는 우변이다.
- PID, M/B/K 소스 값, 이미지 dt, 초기화, 모터 매핑/보호는 변경하지 않는다.
  현재 사용자가 편집한 소스 기본값 M=.05/B1/K20을 보존한다. 직전 실행에서
  읽힌 .1/15/50과 다르므로 재시작 후 Read로 확인해야 한다. 게인 추천이 아니다.
- 빌드 중 사용자가 FORCE_TIMEOUT_SEC를1.0으로 편집했고, 현재 허용범위
  <=.25와 충돌했다. 사용자 승인으로 이 값만 .25로 복원했다. 함께 편집한
  MAX_OFFSET_M=.002/MAX_VELOCITY_M_S=.001은 유지하지만 현재 미사용 상수여서
  보정 제한으로 작동하지 않는다. 제한 기능을 재도입한 것은 아니다.
- 사용자의 동시 편집으로 위치 KP25.55, 각속도25deg/s, 위치 피드백 timeout
  .25s, 모터 프로파일25가 소스에 들어갔으며 그대로 보존/빌드한다. PID 로직은
  수정하지 않았지만 전체 실행 조건이 직전 실험과 부호만 다른 것은 아니다.
- 목표/외력/상태가 정확히0이면 nominal을 유지하지만 50mN은0이 아니다.
  50mN의 일정한 힘은 K20에서2.5mm, K50에서1mm 정적 보정을 만든다.
  무부하 라벨로 힘을0으로 만들거나 deadband/보정 한계를 추가하지 않았다.
- 이전 테스트의 한계는 외력을 독립적으로 주입해 수식 일치만 확인한 점이다.
  이번에는 이상적 내부 위치추종과 수동 스프링 환경의 폐루프 회귀도 추가한다.
  이는 실물 센서/지연/마찰/연성 접촉/축결합까지 검증하는 실험이 아니다.
  특히 부호를 고쳐도 .05/1/20과 환경100N/m의 단순 샘플 모델은30Hz에서
  수렴하지만20Hz 또는1프레임 힘 지연에서는 불안정할 수 있다. 실제 시스템의
  측정 결과가 아닌 한계 예시이며 내부 적분 분할이 피드백 지연을 없애지는 않는다.
- 변경 직전 백업: `/tmp/hrm_admittance_pre_compliance_20261031.vsgYEo`.
  이 백업은 문제가 된 legacy 부호이므로 안전한 실행본으로 간주하지 않는다.
  격리 빌드에서 C++6개 타깃(어드미턴스11개 테스트, 폐루프108조합 및 초기
  변위12조건 포함)과 Python229개가 통과했다. domain79 ROS 가상검증은
  진단391쌍/모터 명령0건이었다. 일반 빌드 적용과 C++6개 타깃 재검증도
  완료했다. 실제 제어기는 정지 상태이며 자동 재시작이나 모터 구동은 하지 않았다.

## 2026-10-31: legacy 힘 오차 부호 복원 (과거 기록, 위 수정으로 대체)

- 사용자 요청으로 `AdmittanceController`에서 두 힘 입력을 함께 음수로 바꾸던
  처리를 제거했다. 이제 원본처럼 `computeAdmittance(f_desired, f_env, dt)`를 호출한다.
  `Mδ̈+Bδ̇+Kδ = F_target-F_env`, `x_t=x_desired+δ`이며 `delta_force`도 이 부호다.
  아래 과거 기록의 `F_env-F_target` 설명은 이번 변경으로 대체된다.
- 모델 출력에 곱하는 `force_sign=-1`, hrm_base/N 단위, Y/Z 확장, 위치 PID,
  현재 M/B/K 기본값, 이미지 dt, 모드 진입 초기화 및 모터/장력 보호는 유지한다.
  특이 행렬 처리와 수치 적분 분할도 유지하므로 전체 2D legacy 코드 롤백은 아니다.
- 기존 실행과 동일한 힘 숫자에 대한 보정 방향이 반대로 바뀐다. 모터 출력 중지 후
  빌드/재시작이 필요하고 실물 접촉 안정성은 가상 테스트로 보장할 수 없다.
  무부하 분류는 힘을 0으로 만드는 기능이 아니며, 두 힘이 0이어도 이전 필터 상태가
  즉시 0으로 초기화되는 것은 아니다. PID는 이번 변경 대상이 아니다.
- 격리 Release 빌드와 C++ 테스트 타깃 6개(컨트롤러 부호 회귀 9개 포함),
  GUI/진단 회귀 229개가 통과했다. domain 79 가상 센서 통합 테스트에서
  395개 진단 쌍, 모터 명령 0건을 확인했다. 양·음 Y/Z, 부호 변환 1회,
  힘/상태 0일 때 nominal 유지, 잔류 상태 복원 및 기존 보호를 검증했다.
  실물 접촉 안정성이나 외력 모델 정확도를 검증한 것은 아니다.
- 실제 제어기 프로세스 종료를 확인하고 일반 실행 경로 빌드를 완료했다.
  일반 빌드에서도 C++ 테스트 타깃 6개가 모두 통과했다. 검증 후에도 제어기
  프로세스는 정지 상태이며 실제 노드 자동 재시작이나 모터 구동은 하지 않았다.

## 2026-10-29: M/B/K 후보 선정 및 읽기 전용 실측 진단

- GUI의 기존 M/B/K 영역에 `Read → Tune`을 추가했다. 확인된 현재값을 기준으로
  힘 추종(M–B, K=0)과 스프링 순응(M–B–K)을 구분한다. 입력한 가정값으로
  `B=힘오차/보정속도`, `M=B×속도시정수`, `K=허용스프링힘/보정거리`를 계산한다.
  mN와 mm의 배율이 소거되어 B는 N·s/m, K는 N/m가 된다. 물리적 속도 제한이나
  힘 오차 보장값이 아니다. 환경 강성/내부 PD/영상·모터 지연을 포함한 자동 튜닝은 아니다.
- `Use candidate`는 입력창에 후보를 넣기만 한다. ROS 적용은 기존 Apply로
  별도 수행하며 Admittance 모드에서는 계속 금지된다. 자동 모드전환/Enable/
  실시간 게인 적용/모터 구동은 하지 않는다. 기존 영역 높이 48 px 유지.
- `M/B`와 자유모델의 느린 감쇠시간을 구분한다. M=.15/B30/K5이면 각각
  .005 s와 약 5.995 s다. 실제 접촉 수렴시간은 어느 값과도 같다고 보장할 수 없다.
  K>0이면 정상 힘 오차가 남을 수 있고, K=0이면 bias에 의한 누적 이동이 가능함을 표시한다.
- `scripts/inspect_admittance_tuning.py --duration 15`는 3개 진단 토픽을 읽기만 하며,
  동일 원본 시각끼리 짝지어 힘 오차/Kδ/속도/내부 위치 오차를 출력한다. 목표/행렬 게인/
  기준 위치/출력 gate/메시지상 pan PD 변경이나 시간 공백은 별도 구간으로 나눈다.
  legacy 메시지에 tilt PD 게인은 없으므로 측정 중 양축 PD를 고정해야 한다.
  마지막 최대 2초를 요약하고 짧은 구간/비활성 출력/누락 쌍을 명시한다. 수렴 판정 허용값은
  보고용일 뿐 제어기 deadband가 아니다. 출력 force는 부호 변환 후 hrm_base 작용력이다.
- 6×6 제어식, 현재 게인, 힘부호, 속도·장력·모터 보호는 변경하지 않았다.
  Y/Z가 둘 다 활성이라는 구조도 그대로다. 목표 Fz=0은 Z 어드미턴스 OFF가 아니다.
  이번 보조 도구만으로 실물 접촉 안정성이나 목표 힘 달성을 보장하지 않는다.
- 검증: 관련/회귀 테스트 265개 통과. 격리 ROS 환경의 가상 진단 메시지 연결과
  스프링 평형 힌트도 확인했다. 실제 환경의 8초 읽기 전용 관찰에서는 진단 메시지가
  없었으므로 실물 수렴은 미검증이다. 설치된 symlink 경로의 import도 확인했으며,
  별도 빌드 없이 사용자가 GUI를 재시작하면 Tune 버튼을 볼 수 있다.

## 2026-10-29: Position PD 각속도 출력 × dt 복원 (현재 구현)

- 사용자 자유운동 진동 보고를 읽기 전용으로 관찰했다. 당시 Position 모드,
  양축 P20.55/I0/D10, 약30Hz/33.34ms의 일정한 프레임 주기였다. 12초 동안
  IK 명령각 변화율이 ±40deg/s에 근접한 비율은 Y71.4%, Z58.9%였다.
  이 비율은 실제 모터 속도 측정이 아니라 명령각 변화율이다.
- 사용자 요청으로10/27의 위치제어 각도증분 변경만 되돌렸다.
  `u=Kp*e+Kd*LPF(de/dt)`를 각속도[rad/s]로 해석하고,
  `q_next=clamp(q_previous+clamp(u,±speed)*image_dt,±bend_limit)`로 갱신한다.
  PID 내부의 미분/적분 dt와 출력 각속도의 시간 적분은 서로 다른 역할이다.
- P 단위 rad/(m*s), D 단위 rad/m. 원래 소스 기본 P20.55/I0/D5와
  D필터0.05s, 각속도40deg/s, motor profile35는 숫자를 변경하지 않았다.
  GUI의 실시간 게인은 자동 저장되지 않으므로 재시작 후 Read로 확인한다.
- Y→tilt/Z→pan, X 비제어, 모드진입 원점 설정, 프레임 누락 복귀와
  모든 하드웨어 보호는 그대로다. 어드미턴스6x6 M/B/K/힘 부호도 그대로다.
  단 Admittance도 같은 내부 위치제어기를 쓰므로 그 PD 출력은 함께 복원된다.
- 아래10/27 항목의 rad/update 부분은 과거 기록이며 현재 구현이 아니다.
- 검증: 격리 빌드 및 일반 워크스페이스 빌드 성공, 각각 C++6개 테스트 타깃
  (Position23개 사례 포함)과 GUI mock21개 통과. domain78 가상 피드백 통합검증도
  통과했으며 실제 모터 명령0건. 15/30/60Hz에서 같은 시간 동안 같은 오차를
  주었을 때 누적 P 보정량이 같은지, 프레임 누락 복귀와 기존 보호도 확인했다.
  사용자가 종료를 확인한 후 일반install을 갱신했다. 실제 노드 자동 재시작이나
  실기 구동은 하지 않았으므로 물리적 진동 해소 여부는 별도로 확인해야 한다.
- domain79 어드미턴스 가상 통합검증도 통과: 진단369쌍, 실제 모터 명령0건.
  서로 다른 토픽의 콜백 도착 순서를 가정하던 테스트 보조함수는 동일 시각의
  진단이 도착할 때까지 최대3초 대기하도록 수정했다. 실제 제어 코드는 그대로다.

## 2026-10-27: legacy 어드미턴스/각도 증분 복원 (과거 기록)

- 사용자 요청으로6x6 M/B/K 행렬 계산 및 legacy 분기/적분 순서를 복원했다.
  ±5mm 위치보정 제한과2mm/s 보정속도 제한 및 해당 ROS 파라미터는 제거했다.
  아래의 속도형 PD/어드미턴스 제한 설명은 과거 구현 기록이다.
- 필터 내부 식은 `desired-env`이다. 현재 공개 입력은 hrm_base의 로봇에 작용하는
  힘이므로 controller에서 두 입력을 함께 부호 변환해 넘긴다. 결과적으로
  `Mδddot+Bδdot+Kδ = F_on_robot-F_target`, `goal=nominal+δ`는 유지한다.
  추정힘의 훈련 부호→작용력 변환 `force_sign=-1`과 GUI 목표 부호는 바꾸지 않았다.
- 전축 행렬과 결합항을 처리하며 비활성 영행은 안전하게 제외한다. M/B가0인 경우
  legacy B+K/B/K 분기도 유지한다. 현재 높은 B/M에서30Hz 단일 legacy 적분은
  수치발산할 수 있어 내부 하위 시간간격으로 나누며, 힘/변위/속도를 자르지는 않는다.
- 위치 PD는 출력[rad/update]을 바로 이전 명령각에 더한다. **추가 ×dt를 제거**했다.
  Y→tilt/Z→pan, I=0, D필터,40deg/s 각도명령 속도와 최대굽힘각 제한은 유지한다.
  이 각도속도 제한은 제거한2mm/s 어드미턴스 제한과 다르다.
- 사용자 소스 설정 P20.55/I0/D5, M.05/B5/K2, 모터profile35를 그대로 남겼다.
  같은 수치의 게인도 이전 속도형 구현과 반응이 다르므로 재튜닝이 필요하다.
  모터 enable/실행노드/모드/실시간 파라미터는 바꾸지 않는다.
- 힘/영상 갱신 대기와 복귀, 모터·로드셀·과장력·모터범위 보호는 유지한다.
  무부하 ID 판정은 힘을0으로 만드는 deadband가 아니다. 잔류 힘이나 bias는
  여전히 δ를 만들며, K>0일 때 정상상태 δ=(F_on_robot-F_target)/K이다.
- 검증: `/tmp/hrm_legacy_control_20261027.cgUNli` 격리 빌드 성공, C++6개 테스트
  타깃과 GUI78개 테스트 통과. domain78/79 가상 센서 통합검증도 통과했으며
  실제 모터 명령0건. 기본install/현재 실행노드는 교체하지 않았으므로 실사용에는
  모터 출력 중지 후 정상 워크스페이스 재빌드 및 컨트롤러 재시작이 필요하다.


## 2026-10-26: Y/Z 사인파 목표 속도 제한 10 mm/s

- 사용자 요청으로 `motion.position_sine.max_speed_m_s` 기본값을0.005에서0.010m/s로
  변경했다. 위치 허용오차나 실제 모터 속도가 아니라 축별 목표 궤적의 최고속도 및
  첫 위상으로 접근하는 속도의 상한이다. PID/모터 profile/어드미턴스 offset 제한은 그대로다.
- 조건은 `2*pi*A/T <= 0.010`. T=20s에서 최대 진폭 약31.83mm이며,
  A=20mm/T=20s는 통과한다. 진폭을 줄이거나 주기를 늘리면 천천히 움직이는 목표가 된다.
- 실행 중인 사인파는 먼저 Stop 후 파라미터를 바꿔야 한다. 재빌드 없이 현재 노드만
  바꾸려면 `ros2 param set /robot_control motion.position_sine.max_speed_m_s 0.01`.
  이 작업에서는 실제 노드/모터 설정을 변경하거나 사인파를 시작하지 않았다.

## 2026-10-26: 속도 프로파일과 피드백 복귀 처리 (최신)

- 느린 속도의 확인된 차이: Kinematics 모터 profile50, Position/Admittance10.
  후자를50으로 맞추고 `position_control/motor_velocity_profile_percent` 정수1–100
  파라미터로 분리했다. software hold의10은 그대로다. 실제 이송시간5배 보장은 아니다.
- 사용자 수정값인 각속도 제한20deg/s, 피드백 freshness1.0s는 유지한다.
  P=5에서는10mm오차의 P항도2.86deg/s이므로 제한값을 높인다고 자동으로 빨라지지 않는다.
  느린 모터보다 명령각이 먼저 누적되면 I=0이어도 overshoot가 가능하다. 실제 encoder와
  target 비교 없이 게인을 자동으로 높이거나 새로운 anti-windup을 추가하지 않았다.
- freshness와 제어 적분 시간은 분리한다. 영상 dt>0.25s 또는 힘/영상 대기 복귀에서는
  첫 정상 프레임으로 시각과 D이력만 재설정하고 다음 정상 프레임부터 재개한다.
  기존 목표/IK명령/모터 원점/사인 위상은 보존하며 공백 시간을 몰아서 적분하지 않는다.
  계속0.25s보다 긴 주기로만 들어오는 경우 명령을 갱신하지 않는다.
- 힘의 수신/원본시각 지연 및 producer의 warming_up, stale_input,
  missing_or_invalid_input, input_time_skew는 대기 후 자동 복귀한다. 잘못된 힘을
  0이나 이전값으로 대체하지 않는다. 모터·로드셀 누락, 과장력, 과힘, 모델 변경,
  frame/scope 오류, valid=true의 NaN 등은 계속 fault다.
- 실제 경고1792998445.553 직전 원인은 인식 오류가 아닌 모터/로드셀 freshness fault였다.
  과거 로그는 두 센서를 합쳐 기록해 어느 쪽인지는 확정 불가. 이제 누락 센서명을 분리하고
  목표 거절 로그에 실제 controller status를 포함한다.
- 힘 부호는 기존 `admittance/force_sign=-1`에서 이미1회 적용된다.
  `F_env=-F_pred`, `M*x_ddot+B*x_dot+K*x=F_env-F_target`; Y/Z만 제어.
  추정 토픽/GUI/학습 라벨 자체를 뒤집거나 제어기에서 다시 -1을 곱하지 않는다.
- 아래 과거 기록의5deg/s, profile10, gap에서0.25s분 명령을 누적한다는 설명은
  이 최신 항목으로 대체된다. 사인파 reference10mm/s와 admittance offset2mm/s는 별도다.

## 2026-10-26: 추정 힘 무부하 범위와 ID 시각화 전용 운용

- 사용자가 명시적으로 선택한 현재 정책: 실제 tip에 힘을 주는 실험에서 ID가9 등으로
  잘못 예측돼도 어드미턴스를 멈추지 않는다. `admittance/use_contact_id_gate=false`가
  기본이며, 현재의 유효한 Fy/Fz를 계속 사용한다. X는 제어하지 않는다.
- 이는 모델이 출력하는 XYZ 하나를 tip 하중 실험의 힘으로 사용하는 것이지, 18개 힘
  중 ID18 힘을 선택/분리하는 기능이 아니다. 실제 body contact가 동시에 존재하는
  상황의 tip-force 분리나 안전성은 보장하지 않는다. 불확실하면 구동을 중지한다.
- 지원 모델 scope는 실제 번들의 `tip` 또는 `all_single_contact`. 현재 기본 선택
  KalmanNet/Transformer의 번들은 후자다. 옆의 과거학습 JSON에 적힌 tip은 신뢰하지 않는다.
- ID/load/`tip_force_valid`는 이 기본모드에서 제어허가가 아닌 진단이다. 반면 valid,
  frame, 모델, 시각/입력창, 유한값, 힘 상한, 모터·장력 보호는 그대로 적용된다.
- 선택적으로 `use_contact_id_gate=true`로 바꾸면 ID18/무부하18에만 적용하고 body나
  unknown localization에서는 보류한다. 이 boolean은 Admittance 밖에서만 바꿀 수 있다.
- 추정기의 `id_localization_config.json`은 예측 XYZ의 mN 범위를 정의한다. 세축 모두
  경계포함 범위 안=>UNLOADED/운용ID18, 하나라도 밖=>LOADED/양수 후보ID.
  원래 classifierID, 확률, 힘은 보존한다. 무부하라고 힘을0으로 만들지 않는다.
  설정은 시작시1회 로드, 파일수정후 Force/contact 재시작; 잘못된설정은 조용히무시하지 않는다.
- 학습 calibration에서 가져온 기본 범위는 실시간 판정 정확도를 입증한 값이 아니다.
  별도 실기 검증 및 필요시 문턱 조정이 필요하며, 자동 hysteresis/debounce는 없다.

## 2026-10-26: GUI Y/Z 사인파 기준 위치

- 기존 IK 각도 사인파와 별도로 Position/Admittance용 m 단위 사인파를 추가했다.
  GUI에서 모드에 따라 같은 한 줄이 전환되며, 위 Move Tip Y/Z가 절대 중심 좌표다.
  `reference = centre + amplitude*sin(2*pi*t/period + phase)`이고 X는 제어하지 않는다.
- 기본 진폭 양축0.010m, 주기20s, Y위상0도/Z위상90도. 한 축 진폭0이면 그 축의
  지정 중심을 유지한다. 시작 시 마지막 nominal 목표에서 첫 위상까지 점진 접근한다.
- 새 `/position/sine_motion` Start/Stop, `/position/sine_status`,
  `/position/sine_reference`를 제공한다. source-image 프레임 기준으로 진행하며
  카메라 gap에서는 위상을 멈추고 새 프레임에서 재개한다. 모드/출력gate 변경,
  모터·로드셀·장력·외력 등 제어 fault에서는 사인파를 종료한다.
- Stop은 마지막 nominal 목표 유지이지 PD 종료/모터 disable/비상정지가 아니다.
  Admittance에서는 사인파 기준에 외력 변위가 더해지며 기존 힘 eligibility는 유지한다.
- 새 per-axis nominal 기준 속도 제한0.005m/s는 기존 PD5deg/s와 다르다.
  `2*pi*A/T` 검사 및 전체길이 반경 검사는 완전한 가동영역/추종/안정성 보장이 아니다.
  힘 목표나 PID gain, angle/FK, force gating을 이번 기능에서 바꾸지 않는다.
- 실제 모터 이동 없이 별도 ROS domain의 가상 피드백으로 검증한다. 소프트웨어
  검증과 실기 폐루프 성능 검증은 별개다. 아래 PD 변경 기록의 임시빌드 상태는
  당시 기록이며, 최신 배포 결과는 CODEX_HANDOFF.md의 이번 항목을 따른다.

## 2026-10-26: 절대 Y/Z 위치오차 기반 PD 속도 명령 (현재 구현)

실기에서 Z 목표 `0.010 m`에 FK 피드백이 약 `0.00316 m`로 머무는 현상을
확인했다. GUI/service의 m 단위는 유지됐고, 문제의 핵심은 진입각에 고정된
작은 PD 각도 보정만 더하던 구조였다. 아래 9월의 제어식은 **과거 구현**이다.

현재는 기존 scalar `PIDController::compute_output()`의 오차 기반 PID를 다시
사용한다. Position에서는 I=0을 유지하므로 PD이며, 출력의 의미를 각속도로
명시하여 영상 프레임 수에 무관하게 시간 기준으로 명령각을 갱신한다.

```text
e_tilt[k] = y_desired[k] - y_FK[k]     # hrm_base, metres
e_pan[k]  = z_desired[k] - z_FK[k]     # X is passive
d[k] = LPF((e[k] - e[k-1]) / image_dt, tau=0.05 s)
u[k] = Kp * e[k] + Ki * integral(e dt) + Kd * d[k]   # rad/s; Ki=0
q_command[k] = clamp(q_command[k-1]
  + clamp(u[k], -max_speed, +max_speed) * image_dt, -60deg, +60deg)
# 정상 프레임 dt<=0.25s만 적분; 대기 복귀/긴 gap은 건너뛰고 D/시간만 재설정
motor_command = motor_entry + count_per_mm * (IK(q_command) - wire_entry)
```

- 목표 Y/Z와 FK 현재 Y/Z는 모두 `hrm_base`의 **절대 좌표**이다. Absolute의
  Z=0.01은 해당 프레임의 Z=10 mm이다. Relative service 의미는 이전 목표에 더함.
- `q_entry + PD` 고정 기준은 제거했다. 현재 명령각을 초기화할 때만 진입 시
  관절각을 사용한다. 모터 count/케이블 길이 진입 offset은 모터 영점으로의
  갑작스러운 이동을 막는 별개 기준이므로 유지한다.
- PID I항은 0이고 오차 적분 상태도 0으로 유지한다. **각속도를 dt로 적분해
  명령각을 만드는 것**은 PID I항과 다르다. 오차가 남으면 계속 보정하며,
  e와 오차 미분이 0이면 도달한 명령각을 유지하고 진입각으로 돌아가지 않는다.
- D는 측정값 미분이 아니라 **오차 미분**이다. 목표 계단 입력도 D에 반영되며,
  기존 저역통과 필터와 각속도 제한을 적용한다. `derivative_filter_sec=0`은
  필터 없는 오차 차분이다. 기존 scalar 3-argument 호출은 이 기본값을 쓴다.
- 기본 수치는 P=5, I=0, D=0.05 그대로지만 단위는 이제
  **P=rad/(m·s), I=rad/(m·s²), D=rad/m**이다. 기존 각도보정 게인과 같은 물리적
  의미가 아니므로 예전의 큰 게인을 그대로 복구하면 안 된다. 실기 튜닝 전 값이다.
- 프레임 timestamp dt/신선도/중복 거부, 카메라 gap 자동 재개, 모터·로드셀·장력
  보호, ±60도와 사용자 설정20deg/s 제한을 유지한다. gap 뒤 D 이력을 초기화하고
  공백의 명령 적분을 건너뛴다. 각도 포화 밖의 누적은 막지만 실제 모터 추종 지연에
  의한 명령 선행까지 제한하는 것은 아니다.
- Admittance도 이 PositionController를 사용하므로 동일한 내부 PD가 적용된다.
  외력 토픽/힘 부호/어드미턴스 식은 이번에 수정하지 않았다.

비구동 검증: 별도 `/tmp/hrm_position_pd_20261026.n76Nlu` build/install에 패키지를
컴파일했다. 본 워크스페이스 install과 실행 중인 컨트롤러는 교체하지 않았다.
실물 시스템의 모터 명령/런타임 게인 변경/노드 재시작은 하지 않았다.

- C++ 38개 통과: Position PD19, FK/IK5, IK sine5, Admittance/force eligibility9.
  실제 SurgicalTool FK를 이상 로봇으로 사용하여 직선/비영점 시작의 양축 ±10mm,
  두 축 동시 목표, 10/30/50Hz, 일정 위치 외란, 작은 진폭·긴 주기 사인파를 검증했다.
  단계 목표는 가상60초 후 양축 오차 <1µm 조건으로 회귀검증했으며 실물 정확도가 아니다.
- ROS Domain78/localhost 가상 센서 통합 시험 통과. 별도 빌드 바이너리,
  motor_output=false 및 motor_command remap을 함께 사용했고 구동 메시지0건이었다.
  지속 오차의 dt 비례 명령, 실제 source dt, 모드 진입, 영상 지연/복귀,
  motor/loadcell 누락/과장력과 runtime gain 검증을 확인했다.
  로그: `/tmp/hrm_position_smoke_nmnv4scz/control.log`.
- 기본 게인은 응답이 느린 초기값이다. 이상 FK 목표 수렴 시험은 실물의
  지연·마찰·백래시·축간 결합 하에서의 안정성이나 빠른 사인파 추종 보장이 아니다.

적용은 사용자가 모터 출력을 끄고 기존 robot_control 노드를 종료한 뒤 일반
workspace에 다시 빌드하고 재실행해야 한다. 첫 실기에서는 작은 Y/Z 목표로
부호와 시간응답을 확인하고, 그 다음 사인파/하중/어드미턴스를 검증한다.

## 2026-09-15: Position PD 수정 결과 (과거 구현, 위 절로 대체)

Position 경로는 사용자 승인에 따라 수정했다. **비구동 소프트웨어 검증 완료이며,
실제 모터 폐루프 안정성 검증은 아직 아니다.** Admittance의 힘 변환과 Y-only 모델,
Dynamics, TCP 드라이버 안전 계층은 이번에 재설계하지 않았다.

### 제어식과 좌표계

```text
e_tilt = y_goal - y_actual     # hrm_base, m
e_pan  = z_goal - z_actual     # X는 제어하지 않음
u = Kp*e - Kd*LPF(measured_position_rate)
q_target = clamp(q_entry + u, -60deg, +60deg)
q_command = slew_limit(q_previous, q_target, 5deg/s * min(actual_frame_dt, 0.25s))
motor_command = motor_entry + count_per_mm * (IK(q_command) - IK(q_entry))
```

`q_entry`는 모드 진입 때 활성 pan/tilt 관절각을 각각 합산해 얻은 케이블 IK 기준각이다.
끝단 Euler orientation이 아니다. **이전 명령에 매번 전체 PD 출력을 더하지 않는다.**
제한이 없는 경우 PD 출력의 차이 `u[k]-u[k-1]`만 더하는 것과 동등하다.
같은 오차가 반복되어도 각도는 계속 증가하지 않는다. 대신 **고정 진입 기준각을
사용하는 I=0 PD는 정적 목표 오차가 남을 수 있다.** 목표의 정확한 추종이나
큰 굽힘에서의 안정성을 보장하지 않으며, 필요하면 다음 단계에서 feedforward/IK
operating point/적분 또는 Jacobian 기반 설계를 검토한다. 축간 결합 보상은 보류했다.

기본값과 ROS 파라미터 (`control_parameters.hpp`, `control_node.cpp`):

| 항목 | 기본값 | ROS parameter |
|---|---|---|
| 양축 P | 5.0 rad/m | `position_control/pid_controller_{pan,tilt}/p_gain` |
| 양축 I | 0.0 (현재 nonzero 거부) | 같은 경로의 `i_gain` |
| 양축 D | 0.05 rad·s/m | 같은 경로의 `d_gain` |
| D 측정속도 저역통과 | 0.05 s | `position_control/derivative_filter_sec` |
| 각도 변화율 | 축당 5 deg/s | `position_control/max_angular_speed_deg_s` |
| 피드백 timeout | 0.25 s | `position_control/feedback_timeout_sec` (0보다 크고 0.25 이하) |

수치는 **초기 시험 설정**이다. 장치의 안전 허용치를 인증한 값이 아니다.
목표 계단 입력으로 D kick이 생기지 않도록 오차 미분 대신 측정값 미분을 사용한다.
1 mm 정적 오차의 P 보정은 0.005 rad≈0.2865°이고, dt=1/30 첫 프레임에서는
변화율 제한에 의해 약 0.1667°만 변한다. 0.1 mm 오차를 10번 반복해도 약
0.02865°에서 유지된다. 모터 velocity profile은 양축 공통 10으로 시작하며,
별도의 케이블 속도/가속도·실제 모터 추종 시험은 필요하다.

### 영상 주기, 초기화와 누락 처리

- `/estimated_segment_angle.header.stamp`의 새 값마다 최대 1회 계산한다.
  30 Hz 고정 적분/재발행을 없앴고, dt는 연속으로 사용한 영상의 timestamp 차이다.
  처리 중 새 프레임이 여러 개 쌓이면 최신 프레임을 사용하고 지나간 프레임을 몰아서
  재실행하지 않는다. 20 ms 대기는 watchdog 점검용이지 제어 dt가 아니다.
- 사용자 요청으로 **영상 지연 때문에 수동 복귀가 필요했던 정책을 제거했다.**
  유효한 영상이 없거나 오래됐으면 WAITING으로 계산만 건너뛰고, 새 유효 영상이 오면
  기존 목표/진입 기준각/모터 기준을 유지한 채 자동 재개한다. 영상 중단 자체로
  현재 모터 위치 hold 명령을 새로 보내지 않으며 마지막 모터 목표가 유지된다.
  따라서 그 목표에 도달할 때까지 모터가 움직일 수 있고, 카메라 대기가 물리 정지는 아니다.
- 긴 영상 간격(>0.25 s)도 오류로 종료하지 않는다. 복귀 프레임에서 D 이력만 초기화하고
  실제 dt로 PD를 계산하되, 명령 변화량에는 최대 0.25 s분의 각속도만 허용한다.
  목표를 현재 위치로 덮어쓰거나 진입 기준각을 재설정하지 않는다. Admittance는 긴
  간격의 힘 적분을 건너뛰어 공백 시간을 한꺼번에 적분하지 않는다. 신선한 영상이
  0.35 s마다 들어오는 경우에도 Position 계산은 계속 가능하다.
- 배열 8개×18/finite/`hrm_base`를 검사하고 중복·역행 stamp를 거부한다.
  모터 위치/속도와 로드셀 stress는 4채널이어야 한다. 영상·모터·로드셀은 수신
  경과시간과 source timestamp를 모두 검사한다. 이는 서로 정확한 시각 동기화나
  하드웨어 실제 샘플링 확인을 뜻하지는 않는다. 시계가 역행/리셋되면 노드 재시작과
  센서 시계 점검이 필요할 수 있다.
- 모드 변경 또는 output 상태 변경 후 **새 영상**에서 한 번만 현재 FK 위치를
  목표로 설정하고 PD/admittance 상태를 초기화한다. 같은 모드를 다시 설정하는 것만으로
  목표를 덮어쓰지는 않는다. 기존 demo 타이머도 모드/enable 전환 때 취소한다.
- 현재 모터 count 4개를 함께 저장한다. 첫 명령은 현재 count 그대로이고 이후
  케이블 IK의 변화분만 더하므로 모드 진입에 의해 모터 영점으로 뛰지 않는다.
  Position의 진입 offset은 absolute IK/Direct의 새로운 영점으로 전파되지 않는다.
- 모터/로드셀(Admittance에서는 외력도)이 아직 없으면 WAITING, 진입 후 이 센서들의 누락·과장력·잘못된
  dt(0 이하/비유한 값)·모터 목표 범위 초과는 FAULT로 latch한다. 영상 대기 중에도
  이 보호는 검사한다. **영상만의 지연과 달리** 이 오류들은 자동 재개하지 않는다.
  모드를 나갔다 재진입하거나 output을 전환하면 **새 현재위치**부터 준비한다.
- 유효하고 최근이며 범위 내인 모터 피드백이 있으면 fault/모드 종료/output OFF 시
  현재 모터 위치 유지 명령을 한 번 보낸다. **모터 피드백 자체가 없으면 안전한
  hold를 만들 수 없다.** 새 명령만 중단되며 TCP는 이전 목표를 계속 보낼 수 있으므로
  드라이버 watchdog/Disable/물리 E-stop이 반드시 필요하다.

### 토픽과 비구동 검증

- `/position/target_motor_command`: MotorCommand 미리보기. output=false에서도 발행.
  TCP가 소비하는 `/motor_command`와 다르다. 첫 진입 dt=0은 reset/hold 진단이다.
- `/position/control_status`: transient-local String, WAITING / READY (dry-run) /
  ACTIVE / FAULT / INACTIVE. recorder 기본 motor/control 그룹에 둘 다 추가했다.
- `/position_controller`, `/admittance_controller`: 사용한 영상 stamp와 `hrm_base`.
  PositionControl의 scalar gain과 theta 배열은 여전히 pan-only legacy 항목이다.
  tilt gain은 ROS 파라미터, 전체 각도는 SegmentAngle에서 확인한다.
- 목표는 `/position/set_goal_position`, 단위 m, Y/Z만 적용. `relative`는 이전 목표에
  더한다. 진입 준비 전/FAULT에서는 목표 요청을 거부한다. `surgical_tool_pose` 각도는
  IK 의미와 맞춰 angular.z=tilt, angular.y=pan으로 정리했다 (Euler pose 아님).

검증: 영상 정책 수정 후 `robot_control_pkg` 재빌드, PD 12개 + FK/IK 5개 gtest 통과.
격리 ROS 가상 시험에서 무센서 시작, 비영점 진입, 같은 오차의 비누적,
10→20 Hz 영상 dt, 중복/NaN/다른 frame/오래된 영상, 빈 로드셀, 모드 재진입 확인.
0.7 s 영상 누락/중복 및 0.35 s 주기에서도 목표를 유지하고 자동 재개하며, 영상
공백 중 추가 명령을 발행하지 않는 것을 확인했다. 모터/로드셀 오류는 영상 대기 중에도
FAULT로 유지되고, 과장력·비정상 게인 거부 및 양축 startup/runtime gain도 검증했다.
직전 구현에서 `record_pkg` 빌드와 recorder/GUI 회귀시험 20개 통과.
**실기/카메라는 실행하지 않았고 구동 명령 0건.**

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select robot_control_pkg record_pkg
source install/setup.bash
ctest --test-dir build/robot_control_pkg \
  -R 'test_(position_controller|surgical_tool_fk)$' --output-on-failure
# 비어 있는 별도 도메인만 사용. 테스트 자체도 output=false/구동 토픽 remap을 강제한다.
ROS_DOMAIN_ID=78 ROS_LOCALHOST_ONLY=1 ROS_LOG_DIR=/tmp/hrm_position_ros_logs \
  /usr/bin/python3 src/robot_control_pkg/test/position_frame_smoke.py
```

실기 전에는 케이블 방향/프리텐션/홈 기준과 travel·장력·속도 제한, 작은 Y/Z 목표의
응답 및 정적 오차, 영상 필터/수신 지연, driver stop을 확인해야 한다. **Admittance는
여전히 옛 `(Fy,-Fx,0)*0.001`와 Y-only 모델이라 바로 사용할 수 없다.**

## 아래는 2026-09-14 수정 전 감사 기록

아래의 과거 코드라인·게인·SIGSEGV·고정 dt 문제는 당시 재현 결과다. 현재 Position
수정 결과는 위 절이 우선한다. `scripts/audit_control_math.cpp`는 현재 클래스를 사용하도록
reset 호출을 추가했으므로 지금 실행하면 아래 과거 Position 수치와는 달라진다.

## 결론

**현재 코드는 FK/IK 검증 단계이며, Position/Admittance 폐루프를 실제 모터에
바로 적용할 준비는 안 됨.** 빌드 성공은 폐루프 안전성/안정성 검증이 아님.
이번 작업은 제어 코드 검토와 비구동 시험만 수행했으며, 제어 수식·게인·출력
Enable은 변경하지 않았다. recorder와 GUI 녹화 상태 처리는 수정했다.

## 입력과 좌표계

현재 위치 입력 경로:

`/estimated_segment_angle → pan_relative/tilt_relative → 3D DH FK → x_actual[0:3]`

- `x_actual`은 **필터링된 관절각으로 구한 FK tip 위치(m)**이다.
  `hrm_fk_tip` TF를 다시 구독하거나, 영상으로 구한 마지막 boundary를 직접
  사용하는 구현은 아니다. 두 위치는 측정 오차/곡선과 강체 링크 근사 때문에 다를 수 있다.
- FK와 목표 위치는 `hrm_base` 기준. X는 계산값만 유지하고 구동하지 않음.
  직선 자세 부근에서는 Y 오차 → tilt(q1 local +Z), Z 오차 → pan(q2).
  19 segment = 고정 os + 18 회전 관절, 전체 82.27 mm.
- 입력 배열은 각 축마다 길이 18, 순서는 tilt-pan-tilt-pan…이며 비활성 축은 0.
  18개의 pan 관절 + 18개의 tilt 관절이라는 뜻이 아님.
- 목표 서비스 `/position/set_goal_position` 단위는 **m**. GUI Position/Admittance
  입력은 숫자를 그대로 보내므로 1 mm는 `0.001`이다.
  `relative`는 **현재 측정 위치가 아니라 이전 목표 위치에 더한다**.
- `hrm_base`의 Camera 기준 Ry(-90)Rz(-45) 회전 설정은 이번에 변경하지 않았다.

현재 힘 입력 경로는 `/fts_data`가 아니라 `/estimated_external_force` (Vector3)이다.
외력 추정 노드는 아직 새 학습 모델로 준비되지 않았고 기본 launch에서도 비활성화되어 있다.
따라서 실제 F/T 센서를 지금 켠다고 admittance 입력에 자동 연결되지 않는다.

권장 통합 계약(아직 구현되지 않음):

`source-stamped force in hrm_base [N] → Y/Z admittance → 목표 Y/Z 보정 [m] → position feedback → cable IK → 4 motor targets`

`f_actual`이라는 이름 대신 현재 구현에서는 `f_env_`를 사용한다. 힘이 **로봇에
가해진 힘**인지 센서가 보는 반력인지 명시하고 부호를 확인해야 한다. 현재 수식은
`F_desired - F_env`이므로 로봇에 가해진 +힘을 넣었을 때 같은 방향으로 순응하는지
입력 정의와 함께 검증해야 한다. 이름만 바꾸거나 모델 축만 맞추면 끝나는 문제가 아니다.

## 실기 전 반드시 고칠 사항

| 우선 | 위치(소스 기준) | 확인된 문제와 영향 |
|---|---|---|
| P0 | `control_node.cpp:1270,1413`, `:851` | 센서 유효성 확인 전에 제어 계산/`stress[0..3]` 접근. loadcell 배열은 초기 resize도 없다. 센서 없이 Position 모드로 시작하면 dry-run에서도 SIGSEGV 재현. |
| P0 | `position_controller.cpp:47,57`, `PID_controller.cpp:23` | 기본 게인 250/5/100에서 1 mm 계단 오차의 첫 보정은 3.25017 rad ≈186.2°. IK가 ±60°로 자르므로 시작부터 포화. PID 자체 anti-windup/각속도 제한 없음. |
| P0 | `control_node.cpp:178,1310` | 외력 구독은 x,y만 복사하고 z는 버림. 이후 `f_env=(Fy,-Fx,0)*0.001`라는 예전 2D 축/단위 변환. 이미 Base/N인 모델 출력을 넣으면 재변환·1000배 축소 오류. |
| P0 | `control_parameters.hpp:104`, `admittance_filter.cpp:84` | M/B/K는 6×6 중 Y 하나만 활성화. singular 6×6 solve 사용. 이 환경의 Eigen에서는 유한값이지만 Z 힘에 대한 응답은 0. Y/Z 활성 부분공간을 명시해야 함. |
| P0 | `control_node.cpp:1316` | 원하는 힘 `(0.02,0.02,0,…)`이 하드코딩. 외력이 없어도 Y 보정 목표가 1 mm로 수렴. 원하는 동작인지 확정 후 설정화/기본값 수정 필요. |
| P0 | `control_node.cpp:1275,1345,1468` | 새 각도 플래그는 진단 발행 조건에만 쓰임. 데이터가 안 와도 마지막 값으로 제어를 반복하며 dt는 고정 1/30 s. 각도/힘/모터/로드셀 source age와 누락 시 safe hold/disable 필요. |
| P1 | `position_controller.cpp:57`, `control_node.cpp:619,1000` | 위치형 PID 출력을 이전 IK 명령 각도에 계속 더한다. 의도한 증분 제어인지 재설계 필요. 모드 전환/Enable 시 목표·PID 적분·이전 오차·명령각 동기화와 bumpless 전환 없음. 출력 OFF 동안 쌓인 명령으로 재활성화하면 점프할 위험. |
| P1 | `control_node.cpp:53,1041,1055` | 런타임 위치 PID 파라미터가 pan만 갱신하고 tilt는 그대로. launch override도 선언 후 제어 객체에 명시적으로 적용하지 않음. admittance 파라미터도 Y만 바꿈. |
| P1 | `control_node.cpp:1449` | 속도 프로파일이 Y 오차만 사용. **최소값 10이 있어 Z-only가 0속도인 것은 아님**. 다만 Z 오차 크기와 무관하게 10으로 고정되고 Y/Z 동시 이동 크기 반영이 안 됨. |
| P1 | `control_node.cpp:134,161,388` | 각도 배열 크기는 검사하지만 finite/frame/age는 검사하지 않음. motor/loadcell 배열 크기도 검사 없음. 모터·로드셀·목표·PID 파라미터를 callback/작업 스레드가 공유하는 부분의 동기화 점검 필요. |

별도 실행 계층 확인사항:

- `tcp_pkg/src/tcp_node.cpp:133` 제한은 `min(target, limit)`뿐이라 음수 하한이 없다.
  `recv()` partial read를 완전한 프레임처럼 해석하고 `send()` partial write도
  처리하지 않는다. TCP 메시지 조립과 재연결 시 유효 데이터 확인을 검토할 것.
- 컨트롤러 Enable OFF는 **새 ROS 명령 발행을 막는 것**이다. TCP는 마지막 목표를
  계속 송신할 수 있다. 물리 E-stop/드라이버 Disable/명령 timeout은 별도 확인해야 한다.
- 장력 상한 2000 g 확인은 위치 루프에 있지만 IK/direct 경로와 동일한 공통 안전
  게이트가 아니다. 초기 프리텐션, slack, 하한, 실제 모터 travel/count limit도 검증 필요.
- Y↔tilt, Z↔pan 독립 PID는 큰 굽힘에서는 결합이 생긴다. 작은 범위에서 Jacobian
  부호/감도를 확인하고 필요하면 2×2 Jacobian 또는 MIMO 제어를 적용한다.

## 재현한 비구동 시험

수치 시험은 ROS/모터 연결 없이 실제 C++ 제어 클래스를 컴파일해 호출한다.

```bash
g++ -std=c++17 -O2 -I/usr/include/eigen3 \
  -Isrc/robot_control_pkg/include/robot_control_pkg \
  scripts/audit_control_math.cpp \
  src/robot_control_pkg/src/{position_controller,PID_controller,surgical_tool,admittance_filter}.cpp \
  -o /tmp/hrm_control_math_audit
/tmp/hrm_control_math_audit
```

| 시험 | 결과 |
|---|---|
| Y 또는 Z 목표 1 mm step, dt=1/30 | 해당 축 delta=3.250166667 rad, IK 명령 60° 포화 |
| 같은 0.1 mm Y 오차로 10회 반복, 실제 위치 고정 | 명령 tilt 31.5652°까지 증가 |
| Y 힘 +0.1 N, F_des=0, 1 step | δY=-2.22222 mm (현재 부호 정의) |
| Z 힘 +0.1 N, F_des=0, 1 step | δZ=0 |
| 외력 0 + 현재 하드코딩 F_des, 10 s | δY=1 mm |
| PID dt=0 | `inf` 출력 (실제 루프는 고정 양수 dt지만 유효성 방어 없음) |

기존 `test_surgical_tool_fk` CTest 대상(5개 FK/IK gtest) 통과.
별도 `ROS_DOMAIN_ID=77`, localhost, `motor_output_enabled=false`, 센서/모터 노드 없이
Position 모드 시작: exit 139/SIGSEGV. gdb stack top은
`ControlNode::run_position_with_admittance_control_thread()`였다.
이것은 테스트 **통과가 아니라 readiness 실패 재현**이다.

## 수정 후 검증 순서

1. 입력 없음/짧은 배열/NaN/잘못된 frame/오래된 stamp/역행 시간 입력에서도
   크래시 및 모터 명령 없이 안전 대기. 제어 중 끊겨도 안전 hold/disable.
2. 축·단위·force 의미를 고정. x는 m, q는 rad, IK 입출력은 deg/mm,
   모터는 counts. force는 경계에서 N으로 **한 번만** 변환.
3. 가상 피드백에서 제어 진입 시 현재 위치 유지 → 아주 작은 Y 목표 → Z 목표 →
   결합 목표. P-only 낮은 게인부터 시작, 포화/속도/anti-windup/reset 검증.
   실제 허용 이동량은 사용자가 기구/장력 조건을 보고 정해야 한다.
4. 실제 모터의 +명령 부호, 케이블 순서 E/W/S/N, 홈/프리텐션을 supervised IK로
   확인한 다음 제한된 Position 시험. 목표/실제 위치/4개 cable/장력/age/포화를 함께 기록.
5. 외력 0이면 목표 변화 0, +Fy/+Fz 각각 원하는 방향의 순응만 생성되는지 모의 force로
   검증. 과대 힘, 끊김, 바이어스, 접촉 해제, mode 변경도 시험.
6. 실시간 외력 모델의 frame/unit/sign/지연/학습 범위 검증 후 낮은 힘부터 접촉 시험.
   F/T raw와 Kalman 신호는 선택적으로 비교하되 학습/추론 전처리를 일치시킨다.
7. 전체 camera+estimation+GUI+control+record 부하에서 source stamp 간격,
   latency, 누락, 장력 한계, E-stop 및 재시작 절차 검증.

## 다음에 사용자와 확정할 것

- 의도한 힘은 자유 순응(`F_des=0`)인가, 일정 접촉력 추종인가?
- F/T 수치 단위 및 로봇에 작용하는 힘/반력 부호, 장착 자세가 실험 중 고정인지?
  제시된 매핑은 `Fb=[Fsx,-Fsz,Fsy]`. 장착이 상대 회전하면 고정행렬로는 부족함.
- 허용 tip 이동범위, 축별 각속도/cable step/모터 travel/장력 상·하한.
- 초기 위치 목표와 모드 변경 시 현재 위치 hold 정책.
- raw force(`/fts_data`) 또는 별도 Kalman force를 학습 label로 택할지.
  둘 다 수집하므로 지금 삭제/확정할 필요는 없다.

## 진단 메시지의 남은 한계

`PositionControl`은 scalar PID gain 한 세트와 pan만 담긴 legacy angle 배열을 유지한다.
`header.frame_id`도 `position_controller`/`admittance_controller`로 되어 있어
`hrm_base`와 일치하도록 정리해야 한다. 위치/admittance 경로의 `surgical_tool_pose`
pan/tilt 필드는 IK 경로와 의미가 뒤바뀌어 있다. 신규 recorder는 원문을 저장하지만
이를 자동으로 올바른 기구학 데이터로 고치지는 않는다. 완전한 pan/tilt는 별도
`/estimated_segment_angle`을 사용하고, 제어 진단 메시지 개정은 다음 작업에 포함한다.
