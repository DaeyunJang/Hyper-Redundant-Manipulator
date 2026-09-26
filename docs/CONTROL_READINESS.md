# Position / admittance 실기 준비 검토

## 2026-09-15: Position PD 수정 결과 (현재 구현)

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
