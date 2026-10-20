# 실시간 카메라·각도 추정 실행 가이드

검증일: 2026-09-13. 현재 하드웨어 정의는 고정 os 포함 19개 세그먼트,
18개 tilt–pan 회전관절, 4개 모터입니다.

## 내일 실행

조명을 켜고 HRM이 ROI 안에 보이도록 놓은 후 워크스페이스에서:

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch launcher system.launch.py
```

GUI가 CUDA 카메라 → serial → TCP → robot_control → estimation을 기존
순서/지연으로 실행합니다. Camera와 Estimation의 자동 실행은 유지했습니다.
Record는 아직 기존 2D 토픽/저장 구조이므로 자동 실행을 해제했으며 수동
Start는 가능합니다. 데이터 수집 구조 수정은 실제 모터 동작 확인 후 진행합니다.

초기 Base 평균을 고정하므로 조명·ROI를 준비한 뒤 estimation을 시작하세요.
`No HRM body detected`는 검출 실패를 뜻합니다. 이때 crop은 계속 나오지만
새로운 각도/FK 입력은 만들어지지 않습니다.

모터 출력 게이트는 시작 시 OFF입니다. GUI의 네 케이블 목표량
`[East, West, South, North]`(mm)을 먼저 확인한 뒤, 실제 장비 준비와 방향을
확인하면서 기존 Enable physical motor output 절차로 모터 시험을 진행합니다.
이번 검증에서는 실제 모터를 구동하지 않았습니다.

카메라·추정만 별도 실행하려면 다음 중 하나를 사용합니다. 동시에 중복 실행하지
말고, 기존 실행은 Ctrl+C로 종료합니다(Ctrl+Z는 종료가 아닙니다).

```bash
# 카메라 + estimation
ros2 launch launcher cuda_estimation.launch.py

# 또는 각각의 터미널에서
./scripts/run_realsense_cuda.sh
ros2 launch estimation_pkg _launch.py
```

이번 변경은 이미 `estimation_pkg`, `gui_py_pkg`, `launcher`, `image_rate_probe`를
`--symlink-install`로 빌드했습니다. 실행 중인 프로세스에는 새 설정이 소급
적용되지 않으므로 카메라와 estimation을 모두 재시작해야 합니다.

## 주기와 지연 확인

```bash
source install/setup.bash
ros2 run image_rate_probe pipeline_rate_probe
```

C++ 구독기가 color, aligned_depth, crop, skeleton, angles, markers의 수신 Hz와
촬영 타임스탬프부터 수신까지의 지연(p50/p95), 중복/역순 타임스탬프, 30 Hz
기준 누락을 5초마다 표시합니다. 정상 영상에서는 각 항목이 약 30 Hz이고
`repeat=0`, `backwards=0`인지 확인합니다. 최초 수 초는 발견/초기화 과정으로
Hz가 낮을 수 있습니다. `missing_30hz`는 수신한 메시지 타임스탬프 기준 추정치로,
센서 누락과 전송/처리 누락을 단독으로 구분하지는 못합니다.

이 명령은 시각화 토픽을 실제 구독해 시각화 작업도 활성화합니다. 시각화 없이
카메라·각도만 점검하려면 `--ros-args -p visualization:=false`를 붙입니다.
동일 PC의 같은 ROS 시계를 사용해야 source age를 비교할 수 있습니다.

처리 시간 로그는 `src/estimation_pkg/estimation_pkg/config.json`의
`performance.debug_timing_enabled`, `debug_rate_enabled`로 켭니다.
기본값은 모두 false이며, 노드 재시작 후 적용됩니다. 직접 실행 시에는:

```bash
ros2 run estimation_pkg segment_angle_estimator --ros-args \
  -p debug_timing_enabled:=true -p debug_rate_enabled:=true
```

RViz의 Image/MarkerArray 디스플레이는 Best Effort로 설정합니다. 화살표는
기존 ARROW 타입과 namespace를 유지합니다.

## 수정한 원인과 코드

Fast DDS의 기본 SHM 영역은 512 KiB인데 848×480 RGB 한 장은 약 1.16 MiB,
depth 한 장은 약 0.78 MiB입니다. 작은 공유 메모리에서 큰 메시지를 연속
전달하면 읽기 전 데이터가 덮여 누락될 수 있습니다. 카메라의 CUDA align
계산 이외에 이 전송 단계가 이번 프레임 누락에 영향을 주었습니다.
[Fast DDS 공식 설명](https://fast-dds.docs.eprosima.com/en/2.x/fastdds/transport/shared_memory/shared_memory.html)
역시 메시지보다 작은 segment의 데이터 손실 위험을 명시합니다.

- `estimation_pkg/config/fastdds_images.xml`: SHM 64 MiB, SHM 메시지 최대 4 MiB.
  UDPv4를 함께 유지하므로 ROS 발견/다른 PC와의 통신을 막지 않습니다. 이 설정은
  로컬 이미지 전송용이며 원격 대용량 영상 전송 성능을 보장하지 않습니다.
- `launcher/cuda_realsense.launch.py`: GUI 버튼과 기존 셸 스크립트에서 동일하게
  카메라 프로세스에 프로파일을 적용합니다.
- `estimation_pkg/runtime.py`: `ros2 run`/launch 모두에서 `rclpy.init()` 전에
  같은 프로파일을 적용합니다. 사용자가 지정한 FASTRTPS/FASTDDS 프로파일과
  다른 RMW 선택은 덮어쓰지 않습니다.
- `postprocess.py`: Lee skeleton을 몸통의 bounding box 안에서 계산한 뒤 원래
  ROI 픽셀 좌표에 복원합니다. extrapolation → body mask AND → depth → 3D
  fitting → DH projection/filter 계산과 하드웨어 길이는 유지합니다.
- `segment_angle_estimation.py`: RBSC 계산 중 시각화 잠금을 잡지 않습니다.
  완성된 한 프레임의 복사본을 최신 결과 한 칸에 전달하고, 느린 시각화 때문에
  과거 결과를 쌓지 않습니다. crop 발행도 이미지 수신 콜백에서 분리했습니다.
- ARROW마다 안정적인 `(ns,id)`를 갱신합니다. 매 프레임 DELETEALL로 약 100개
  그래픽 객체를 지우고 다시 만드는 작업을 없앴습니다. 시작 시 초기화와
  실제로 사라진 ID의 DELETE만 수행합니다.
- OpenCV/BLAS worker 수는 기본 1입니다. `blas_num_threads`로 작은 행렬 연산의
  과도한 스레드 생성을 제한합니다.
- 검출 실패를 프레임마다 print하지 않고 2초 간격 경고로 알립니다. 입력 Hz
  진단은 재구성 실패 중에도 출력됩니다.
- `max_depth_age_sec=0.1`: 최신 depth와 color의 타임스탬프 간격이 100 ms를
  넘으면 그 프레임의 각도를 만들지 않습니다. 정확한 RGB-D 동기화 기능은
  아니며, 끊긴 depth를 계속 재사용하지 않기 위한 제한입니다. 0으로 끌 수
  있지만 실제 피드백 시험에는 기본값을 권장합니다.

QoS는 변경하지 않았습니다. 영상/마커는 Best Effort + Keep Last(1),
제어용 SegmentAngle은 Reliable + Keep Last(1)입니다. SHM 크기는 ROS 큐의
depth와 별개의 설정입니다. 전역 OS 설정이나 시스템 librealsense는 바꾸지 않았습니다.

## 검증 결과와 범위

같은 최적화 코드에 DDS 프로파일만 바꾼 대조 실험에서 512 KiB는 depth
17.6–26.0 Hz, color 18.2–29.4 Hz, skeleton 14.4–25.6 Hz로 흔들렸고
64 MiB에서는 안정 구간의
입력·각도·skeleton·마커가 약 30 Hz로 회복됐습니다.

90초 합성 RGB-D 시험에서는 움직이는 입력 2,700프레임을 공급하고 estimator,
dry-run robot_control/FK, C++ 시각화 구독기를 함께 실행했습니다. 안정 구간에서
약 30 Hz, 타임스탬프 중복/역순 없이 전달되는 것을 확인했습니다. 이 시험의
source-to-angle 지연은 대체로 p50 15–20 ms, 구간별 p95 20.2–36.4 ms였습니다.
각도 정확도나 실제 depth 노이즈에 대한 검증을 대신하지는 않습니다.

실제 D405 + GUI(offscreen Qt 렌더링) + estimation + robot_control + 기존
RViz 조건에서도 RGB/aligned depth/crop이 30 Hz로 유지됐습니다. 실제 영상은
당시 조명이 없어 ROI 최댓값이 19(검출 임계값 120)였으므로, 실제 HRM 영상의
최종 각도·skeleton 성공률은 다음 출근 시 조명을 켜고 확인해야 합니다.

동일 합성 ROI의 단일 프레임 core 측정에서는 몸통 두께에 따라 중앙값이
10.0/13.1/19.2 ms에서 7.7/9.1/12.3 ms로 줄었습니다. Lee skeleton 픽셀 동일성,
19-segment/18-joint 기구학, snapshot 독립성, 마커 ID 갱신을 회귀 테스트했습니다.

카메라 없이 연속 시험을 재현하려면:

```bash
python3 scripts/benchmark_estimation_pipeline.py --duration 90
```

스크립트는 별도 ROS_DOMAIN_ID=73에서 합성 카메라·추정·dry-run FK·C++ probe만
실행하며, 종료 시 자신이 만든 프로세스를 정리합니다. 실제 카메라/TCP bridge는
실행하지 않습니다. 결과 로그는 출력되는 `/tmp/hrm-pipeline-*`에 있습니다.
합성 몸통은 현재 ROI `(100, 0, 640, 480)`를 기준으로 하므로 ROI를 바꿨다면
입력 형상과 ROI가 겹치는지 먼저 확인합니다.
