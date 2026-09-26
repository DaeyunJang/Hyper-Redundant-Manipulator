# IK 기반 tilt/pan 주기 운동

`/kinematics/sine_motion` (`std_srvs/srv/SetBool`)가 새 Start/Stop 서비스다.
기존의 pan 전용 `/motion/move_sine_wave` 및 legacy 원운동 서비스 대신 사용한다.
GUI에서 Robot control을 시작한 뒤 아래 버튼 또는 서비스 명령으로 제어한다.
다른 robot_control 인스턴스를 중복 실행하지 않는다.

## GUI (2026-09-26)

Motor control의 Move Tip 바로 아래 **IK sine** 줄:

- Pan/Tilt 각각 **A**(진폭 °), **φ**(위상 °), 공통 **T**(주기 s)를 설정한다.
  기본값은 Pan 45°/90°, Tilt 45°/0°, 주기 20초다. 중심각은 두 축 모두 0°다.
- 한 축만 운동하려면 다른 축 A=0, φ=0으로 둔다. 이는 현재 각도를 유지하는 것이
  아니라 0° 목표 고정이므로, 처음에 그 축이 굽혀져 있으면 0°로 이동할 수 있다.
- Kinematics 모드에서 **Start**: 파라미터를 한 번에 적용하고 성공한 후에만 시작한다.
  Move Tip의 Absolute/Relative 선택과 독립적이다. 진폭은 0~45°, 주기는 1초 이상이며
  속도 제한 검증을 통과해야 한다. 기존 서버의 더 낮은 속도 제한도 임의로 높이지 않는다.
- **Stop**: 마지막 목표 유지, 원점 복귀 없음. 비상정지 기능이 아니다.
  Start/Stop 결과와 보호 정지는 옆 상태에 표시하며 마우스를 올리면 상세 이유를 본다.
- 실행/요청 중에는 설정을 잠근다. 응답 지연 시 Unknown/Timeout으로 표시하며
  늦은 설정 응답이 Start로 이어지지 않는다. Start 결과가 불명확하면 Stop을 요청하고
  늦은 Start 응답 이후 다시 Stop하여 정지 요청을 추월하지 못하게 한다.
- 물리적 모터 Enable과 제어 모드를 자동으로 변경하지 않는다. 실제 검증은 작은
  진폭부터 수행한다. 외부에서 실행한 제어기는 GUI 종료로 정지되지 않으므로 먼저 Stop한다.

## 각도와 파형

각도는 각 관절 q_i가 아니라 기존 `surgical_tool` IK에 전달하는 전체 굽힘 명령(deg)이다.

```text
tilt(t) = tilt_center + tilt_amplitude * sin(2*pi*t/period + tilt_phase)
pan(t)  = pan_center  + pan_amplitude  * sin(2*pi*t/period + pan_phase)
```

각 축 파라미터는 `[중심각(deg), 진폭(deg), 위상(deg)]`이며 주기는 두 축 공통이다.
진폭 0은 해당 축을 중심각에 유지한다. 0~45°는 중심 22.5°, 진폭 22.5°이고,
-45~45°는 중심 0°, 진폭 45°로 서로 다르다. 위상 -90°는 최소각에서 시작한다.
두 축에 동일 진폭과 90° 위상차를 주면 **각도 평면에서** 원이지만 실제 tip의
XYZ 경로가 정확한 원인 것은 아니다. 외력 중 개루프 IK는 실제 각도/위치를 보장하지 않는다.

## 먼저 드라이런

기존 Robot control이 실행 중이면 재시작하여 새 코드를 반영한다.
아래 `/robot_control`은 launcher가 붙인 노드 이름이다.

```bash
source install/setup.bash
ros2 param set /robot_control motor_output_enabled false
ros2 param set /robot_control control_mode 1

# 안전한 초기 확인: tilt 0~5°, pan 0° 고정, 30초 주기
ros2 param set /robot_control motion.sine.tilt '[2.5, 2.5, -90.0]'
ros2 param set /robot_control motion.sine.pan '[0.0, 0.0, 0.0]'
ros2 param set /robot_control motion.sine.period_sec 30.0
ros2 param set /robot_control motion.sine.max_speed_deg_s 5.0
ros2 service call /kinematics/sine_motion std_srvs/srv/SetBool '{data: true}'

# 다른 터미널에서 상태/목표 케이블 길이 확인 (East,West,South,North, mm)
ros2 topic echo /kinematics/sine_status
ros2 topic echo /kinematics/target_wire_length

# Stop: 새 목표 생성 중단. 자동으로 영점 복귀하지 않는다.
ros2 service call /kinematics/sine_motion std_srvs/srv/SetBool '{data: false}'
```

## 실험 파형 예시

아래 설정은 **Stop 후** 적용한다. 진폭/위상 변경 중에는 운동하지 않는다.
설정이 거절되면 응답 이유를 확인한다. 각 파라미터 변경은 전체 후보 설정을 검증한다.

```bash
# tilt만 0~45°
ros2 param set /robot_control motion.sine.tilt '[22.5, 22.5, -90.0]'
ros2 param set /robot_control motion.sine.pan '[0.0, 0.0, 0.0]'
ros2 param set /robot_control motion.sine.period_sec 30.0

# pan만 0~45° (위 tilt 설정 대신)
ros2 param set /robot_control motion.sine.tilt '[0.0, 0.0, 0.0]'
ros2 param set /robot_control motion.sine.pan '[22.5, 22.5, -90.0]'

# 두 축 모두 0~45°, 동일 위상
ros2 param set /robot_control motion.sine.tilt '[22.5, 22.5, -90.0]'
ros2 param set /robot_control motion.sine.pan '[22.5, 22.5, -90.0]'

# 두 축 모두 0~45°, pan이 tilt보다 90° 앞섬
ros2 param set /robot_control motion.sine.tilt '[22.5, 22.5, -90.0]'
ros2 param set /robot_control motion.sine.pan '[22.5, 22.5, 0.0]'

# 설정 후 시작 / 정지
ros2 service call /kinematics/sine_motion std_srvs/srv/SetBool '{data: true}'
ros2 service call /kinematics/sine_motion std_srvs/srv/SetBool '{data: false}'
```

실제 출력은 기존 GUI의 Motor output Enable 또는 해당 파라미터를 사용자가 명시적으로
켜야 한다. 서비스가 Enable/제어모드를 자동으로 바꾸지 않는다. 출력 gate를 바꾸면
진행 중 파형은 취소되므로 원하는 상태를 확인하고 다시 Start해야 한다.

## 동작·보호 범위

- 시작 기본값: tilt ±45°(위상 0°), pan ±45°(위상 +90°), 주기 20s,
  각 축 최대 명령 속도 15°/s. pan이 tilt보다 90° 앞선다.
  별도 ROS 파라미터 override가 없는 노드 재시작 시 적용되며 자동 시작/출력 Enable은 없다.
  최대 사인파 속도는 약 14.14°/s다. 이전 5°/s는 사용자가 지정한 값이 아닌
  초기 보호 설정이었으며, 요청한 20초 주기에 맞춰 15°/s로 변경했다.
- 축별 `|중심각|+진폭 ≤45°`, 진폭≥0, 주기≥1s. 속도는 `(0,15]°/s`에서 설정 가능.
  사인파 자체의 최대 속도 `2*pi*진폭/주기`가 설정 제한 이하인 경우만 허용한다.
- 시작 시 마지막 IK 명령각에서 최초 위상각으로 속도 제한하여 접근한 뒤 주기가 시작된다.
  이것은 측정 자세가 아니다. 실출력 Start는 신선한 실제 모터 위치가 해당 IK 시작각의
  케이블 목표에서 각 모터 1000 count 이내인지 확인한다. 드라이런/직접 모터 이동 후
  불일치하면 거절한다. 모터 영점·장력 세팅·IK 초기 자세를 먼저 확인해야 한다.
- 실제 출력 중 4모터/4로드셀 피드백 수신·source stamp가 0.25초 이내이고,
  모든 장력이 기존 `TENSION_LIMIT` 미만이어야 한다. 현재 상수는 2000g이지만
  이것이 해당 하드웨어에서 검증된 안전 장력이라는 뜻은 아니다.
- 타이머 지연>0.25s, 피드백/장력 검사 실패, 모터 소프트웨어 위치 한계 초과,
  모드 또는 motor output gate 변경 시 정지하며 자동 재개하지 않는다.
- **Stop/보호 정지는 새 목표 발행 중단**이며 자동 감김/풀림/영점복귀는 없다.
  드라이브는 이미 받은 마지막 목표까지 움직일 수 있다. 하드웨어 비상정지가 아니다.
- 실행 중 수동 IK/직접 모터 명령과 다른 legacy 모션 Start는 거절한다.
  legacy 모션 Stop으로도 새 사인파를 정지할 수 있다.
- 최대각 45°는 소프트웨어 범위일 뿐 물리적 안전성/정확성 보장이 아니다.
  먼저 작은 진폭에서 케이블 방향·장력·실제 움직임을 사용자 감독하에 검증할 것.

## 기록과 검증

기존 `/surgical_tool_pose`는 **명령** pan/tilt를 rad로 표시하고,
`/estimated_segment_angle`은 이미지로 측정한 관절각이다. 두 값을 혼동하지 않는다.
`/kinematics/sine_status`를 선택 기록 목록에 추가했다(필수 토픽 아님).
파형 설정은 기존 recorder의 `/robot_control` 파라미터 snapshot 및 `/parameter_events`로
남는다. Record 상태가 recording인지 확인한 후 파형 Start를 권장한다.
contact_segment_id는 힘 변화와 관계없이 해당 기록 동안 고정된다.

실제 모터를 구동하지 않는 C++ 수학 단위 테스트와 별도 도메인의 통합 테스트:

```bash
ctest --test-dir build/robot_control_pkg -R test_ik_sine_motion --output-on-failure
ROS_DOMAIN_ID=79 ROS_LOCALHOST_ONLY=1 ROS_LOG_DIR=/tmp/hrm_ik_sine_test_logs \
  /usr/bin/python3 src/robot_control_pkg/test/ik_sine_smoke.py
```

통합 테스트는 TCP/카메라를 실행하지 않고 모터 명령을 테스트 전용 토픽으로 재매핑한다.
