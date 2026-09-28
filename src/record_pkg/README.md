# HRM recorder

현재 3D 추정/4모터/4로드셀의 **수치 데이터는 원본 ROS 메시지와 CSV**로,
**HRM crop 컬러 + 같은 ROI의 수치 depth는 별도 이미지 파일**로 저장한다.
GUI 서비스 `/data/record`는 유지한다. **Record component 시작과 녹화 시작은 별개**다.
recorder는 모터 명령을 발행하지 않으며 물리 출력 Enable을 변경하지 않는다.

## 이미지 저장 선택 (2026-09-27)

GUI Recording 설정 줄의 **Save crop RGB/depth**로 세션별 저장 여부를 선택한다.
기본 체크 ON은 기존 crop 컬러·depth 저장을 유지한다. OFF이면 두 이미지 아카이브를
만들지 않으며, 수치 bag·CSV·summary.csv·메타데이터는 그대로 저장한다.
CameraInfo/depth calibration 등의 작은 메타데이터는 유지될 수 있다.
이미지 소스가 없어도 OFF 세션은 이미지 관련 필수 검사나 후처리 오류로 막히지 않는다.
실시간 카메라/추정/GUI 표시에는 영향을 주지 않는다. OFF 세션은 영상 재처리를 할 수 없다.

- `/record`의 bool parameter `save_images` 기본값은 true다. GUI는 힘 정렬·접촉 ID와
  함께 원자적으로 설정하고 성공 응답 후에만 Record를 요청한다.
- 녹화/종료/이미지 flush/CSV export 중 변경을 거부한다. 다음 Record 전에 선택한다.
- `session.json`의 `snapshot.save_images` 및 `snapshot.*_archive_settings`,
  세션의 `recording_config.json`에 실제 적용값을 저장한다. 후처리는 이 설정을 읽어
  OFF 아카이브의 상태를 `disabled`로 처리한다. `/data/record_status.save_images`도 제공한다.
- ON으로 되돌리면 노드 시작 시 설정 파일의 archive.enabled를 복원한다. 원래 설정에서
  꺼둔 아카이브까지 강제로 활성화하지 않는다. 숫자 토픽 선택이나 CSV 스키마는 바꾸지 않는다.
- 이 선택은 crop 파일 아카이브를 제어한다. 기본 bag에는 이미지가 없지만, 사용자 지정
  프로필이 별도로 이미지를 bag 토픽에 추가했다면 해당 토픽 선택은 별도로 수정해야 한다.
- 기존 기록 폴더는 변경하지 않는다. 업데이트 후 녹화/변환이 끝난 상태에서 GUI와
  Data recorder를 다시 실행하면 사용할 수 있다. 기록 중 노드 재시작은 하지 않는다.

## 경량 이미지 저장 프로필

기본 영상 저장은 `/estimated_segment_crop_image`와
`/estimated_segment_crop_depth/image_raw`만 남긴다.
`images/hrm_crop/`에 무손실 PNG를 저장하며 **같은 이미지를 bag에 중복 저장하지 않는다**.
depth는 `images/hrm_crop_depth/`에 16-bit PNG(16UC1/mono16), 또는 float32
NPY(32FC1)로 저장한다. 0·NaN 등 무효 depth도 원값 그대로 두고 색상화/정규화하지 않는다.
전체 HRM 컬러/depth, 태그 카메라 영상, body binary/skeleton 영상과
3D 시각화(point cloud/markers)는 기본 기록 대상에서 제외했다.
GUI/RViz 실시간 표시와 추정·태그 검출 계산은 변경하지 않는다.

- 기존 `csv/summary.csv`, 토픽별 수치 CSV, 메타데이터와 수치 bag은 유지한다.
  **bag 전체를 없앤 것은 아니며, 큰 이미지 payload를 bag에서 뺀 구성**이다.
- PNG와 `images/hrm_crop/index.csv`에 원본 이미지 source timestamp와 frame을
  연결한다. 목록에는 파일명, 수신 시각, 크기, encoding, 접촉 라벨과
  촬영 시각을 남긴다. Depth index는 프레임별 ROI, crop 기준 K/P/R/D,
  원본 크기, `depth_scale_m_per_unit`도 보관한다. `depth_m = raw * scale`이다.
- 이미지 콜백은 유한 큐에 전달하고 별도 작업에서 PNG 압축·파일 쓰기를 한다.
  과부하로 생긴 큐 누락이나 저장 오류는 상태/manifest에 남긴다. 무손실 PNG라는
  말은 저장된 픽셀을 보존한다는 뜻이며, 모든 카메라 프레임 수신을 보장하지 않는다.
- **crop만으로 2D mask/skeleton을 다시 계산할 수는 있지만**, 이미 crop된 영상을
  원본처럼 다시 ROI crop하면 안 된다. 당시 ROI와 설정을 참조해 처리해야 하며,
  crop 밖의 픽셀은 복구할 수 없다. 함께 저장한 depth와 calibration으로 ROI의
  3D 복원 재처리가 가능하다. RGB/depth는 독립 source stamp를 보존하므로 같은
  세션 안에서 시각을 맞춰야 한다. 같은 노출이나 온라인 처리 당시의 정확한
  RGB/depth 쌍을 보장하지는 않는다. 태그 영상 재검출은 불가능하다.
- 기존 세션의 bag/PNG/CSV는 삭제·변경하지 않는다. 기본 프로필 변경은
  **estimator와 Data recorder를 재시작한 뒤 새 세션부터** 적용된다.

## 센서 이동 시 별도 녹화 (2026-09-26)

사용자 최종 선택에 따라 **기존 Stop → 새 Record 방식**을 유지한다.
일시정지·재개 버튼이나 `interval_id` 컬럼을 추가하지 않는다. Stop/export 완료 후
센서를 이동하고 새 Record를 시작하면 별도 날짜/시간 세션 폴더에 저장된다.
Record Stop도 모터 정지는 아니므로 필요한 운동 정지는 별도로 수행한다.

LSTM/TCN 학습은 각 파일 안에서 연속 프레임 window를 만든 뒤, 완성된 window들을
합쳐 학습한다. CSV 행부터 전부 연결해 window를 만들면 파일 경계의 시간 단절이
가짜 연속 운동으로 들어간다. 실험/세션 단위 train/validation 분리 후 window를
만들고, window 안 순서는 유지한다. 비시계열 모델도 ID·timestamp를 자동으로
무시하지 않는다. 학습 입력 열은 학습 코드에서 명시적으로 선택해야 한다.
이번 작업에는 학습 코드나 데이터셋 분할 구현은 포함하지 않는다.

## 접촉 실험 라벨 및 추가 상대각 (2026-09-25)

- GUI Recording 영역의 `contact_segment_id`는 0~18 정수다. 0은 무하중
  자유운동 실험, 1~18은 사용자가 지정한 접촉 segment 실험을 뜻한다.
- Record 시작 전에 지정한다. recorder가 설정을 승인한 뒤에만 수집을 시작하며,
  수집/종료/CSV 변환 중에는 고정된다. ID 변경은 Stop 및 export 완료 후 한다.
  힘을 가했다 풀어도 라벨은 바뀌지 않는다. 힘 임계값 판정은 없다.
- `session.json`의 `snapshot.contact_segment_id`에 고정값을 보관하고,
  모든 토픽별 시계열 CSV 및 `summary.csv`의 각 행에 같은 컬럼을 추가한다.
  key/value 형식의 `session_metadata.csv`에는 snapshot 항목으로 보관한다.
  라벨이 없는 예전 기록은 빈칸이며 0으로 추측하지 않는다.
- 각도 토픽 CSV 및 summary에는 `relative_angle_1`~`relative_angle_18`을
  추가한다. 확정된 고정 규칙은 홀수=해당 인덱스 `tilt_relative`, 짝수=
  `pan_relative`다. 동일한 메시지에서 선택하며 rad, 부호, 0을 유지한다.
  다른 축으로 대체하거나 절댓값/합산/새로운 각도 계산을 하지 않는다.
- summary 마지막에는 라벨 및 상대각 총 19개 열을 둔다. 2026-09-25 사용자
  선택 반영 후 전체는 268열이다. 원본 pan/tilt relative/absolute는 그대로다.
- 사용자 재확인으로 tilt-first를 확정했다. estimator의 홀수 tilt / 짝수 pan
  활성 축과 동일하다. 추정/FK 계산은 변경하지 않았다.
- 기존 bag/CSV를 덮어쓰지 않는다. 학습, one-hot/soft-label, 네트워크는 미구현.

## 실행

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select estimation_pkg record_pkg gui_py_pkg
source install/setup.bash
ros2 launch record_pkg _launch.py
```

기존 GUI의 System → **Data recorder → Start**, 그 다음 **Record** 버튼으로도
동일하게 사용할 수 있다. 자동 녹화는 하지 않는다.

GUI 상태는 `starting → recording → stopping → exporting → stopped`. `recording`이 되기 전에는
데이터 수집용 동작을 시작하지 말고, **`stopped`를 확인한 후 종료/전원 차단**한다.
`exporting`은 bag은 닫혔고 CSV/누락 검사 중이라는 뜻이며 새 Record는 잠시 차단한다.
주황색 `INCOMPLETE`는 저장 종료는 됐지만 필수 수치 데이터 또는 이미지 기록에
문제가 있다는 뜻이다.
저장 경로는 상태 문구의 tooltip과 노드 로그에 표시한다.
노드가 죽으면 GUI는 stale로 표시한다. 실패한 서비스 요청은 성공으로 표시하지 않는다.

터미널 제어/확인:

```bash
ros2 service call /data/record std_srvs/srv/SetBool '{data: true}'
ros2 topic echo /data/record_status
# 동작/수집이 끝나면:
ros2 service call /data/record std_srvs/srv/SetBool '{data: false}'
# 위 상태가 stopped가 된 후, 실제 폴더명을 넣어 확인:
ros2 bag info record/<session>/bag
```

기본 출력은 **실행한 작업 디렉터리의 `record/`**다. 다른 디스크를 사용하려면
`ros2 launch record_pkg _launch.py output_root:=/absolute/dataset/path`.
각 녹화는 새 날짜/시간/마이크로초 폴더를 만들며 기존 세션을 덮어쓰지 않는다.
형식은 `YYYYMMDD_HHMMSS_ffffff` (PC의 로컬 시간, 마지막 6자리는 마이크로초)다.
예: `record/20260915_200630_017213/`. `session.json`에 같은 `session_id`와
UTC/로컬 시작 시각(시간대 오프셋 포함)을 함께 저장한다.

## 저장 내용과 설정

[config/recording.json](config/recording.json)의 `enabled_groups`, `extra_topics`,
`required_topics`로 선택한다. config는 recorder 시작 시 읽으므로 변경 후 **recorder
노드를 재시작**한다. 소스/설치 config의 symlink 여부는 실제 설치 경로를 확인할 것.
다른 config는 launch `config_file:=/absolute/recording.json`으로 지정 가능하다.

저장 대상으로 **선택한 토픽**과 녹화 시작에 **필수인 토픽**은 서로 다르다.
`required_topics`에서만 빼면 해당 토픽은 선택 목록에 남아 있을 때 실제 들어오는
메시지를 저장하지만, 없어도 시작을 막지 않는다. 아예 bag에 저장하지 않으려면 그룹 또는
`topic_groups`/`extra_topics`의 목록에서도 제외한다. 필수 목록은 저장 목록의 부분집합이어야 한다.
예를 들어 센서 그룹만 선택하고 `/fts_data`, `/loadcell_state`만 필수로 지정하면
나머지 센서 그룹 토픽은 발행될 때만 함께 저장된다. 현재 GUI에는 이 목록을 고르는
체크박스가 없으므로 config 변경 후 Data recorder를 재시작해야 한다.
PNG 보관은 아래 독립 설정을 사용하며 `required_topics`(bag 전용)에 crop을
추가하지 않는다. `required: true`이므로 기본 Record에는 crop publisher도 필요하다.

```json
"image_archive": {
  "enabled": true,
  "topic": "/estimated_segment_crop_image",
  "queue_size": 16,
  "required": true
}
```

depth 저장은 `depth_archive`의 `enabled/topic/metadata_topic/queue_size/required`로
선택한다. 기본 `enabled: true, required: false`로, 예전 estimator에 새 depth 토픽이
없어도 기존 녹화 시작을 막지 않는다. **미저장은 상태 및 depth_integrity에 경고**한다.
3D 재처리용 실험 전 반드시 depth 저장 수를 확인할 것. crop depth publisher는 같은
ROI와 원래 header를 사용하며, calibration은 exact depth stamp/frame으로 연결한다.
calibration이 없거나 다르면 잘못된 3D 복원용 파일을 만들지 않고 오류 수를 남긴다.

| 그룹 | 저장 |
|---|---|
| camera | D405 color/aligned depth 및 crop depth CameraInfo와 ROI/scale metadata. 이미지 payload는 bag에 저장하지 않음 |
| sensors | `/fts_data`, `/fts_data_kalman_filter`, 두 센서 offset, 4채널 `/loadcell_state`, `/data_filter_setting` |
| motor_and_control | 4모터 상태/명령, 실제 cable 변위/속도, `/kinematics/target_wire_length`, `/kinematics/fk_tip_position`, control mode, pose, position/admittance 진단, 기존 dynamics 진단, 외력 추정 |
| geometry | 새 `SegmentAngle`(pan/tilt 및 속도 배열 전체), `/estimated_tip_position`, `/estimated_tip_position_unscaled`, `/tf`, `/tf_static` |
| apriltag (선택 저장) | 태그 카메라 `color/camera_info`, `/detections`, 세 `/apriltag/.../pose` 토픽. 발행될 때 저장하며 미검출/카메라 미실행이어도 녹화 가능. 태그 영상은 저장하지 않음 |
| visualization (기본 OFF) | 선택 항목 목록만 유지. 명시적으로 켜면 crop/body/skeleton 이미지와 centerline point cloud, reconstruction markers가 bag에 추가됨 |
| configuration | `/parameter_events` |

카메라 calibration 토픽을 변경했다면 **해당 camera/apriltag 그룹과 required_topics를
함께 수정**한다. crop PNG 토픽 변경은 `image_archive.topic`에 반영한다.
기본 bag 필수 토픽은 color CameraInfo, motor_state, loadcell_state,
raw/Kalman F/T, wire_length, estimated_segment_angle, tool_endeffector_pose다. crop PNG는 별도
`image_archive.required` 조건으로 확인한다.
publisher가 없거나 작은 필수 메시지의 최근 유효한
시각 진행이 없으면 시작을 거부한다. **태그 카메라 CameraInfo, detections와 세 pose는
선택 저장**이므로 없어도 시작/계속 기록할 수 있고, 태그 누락만으로 필수 데이터
`INCOMPLETE` 경고를 만들지 않는다. 저장 목록과 CSV 열은 유지하며 검출되는 데이터는
그대로 저장한다. 대응되는 태그 자세가 없으면 summary의 해당 칸은 빈칸/unmatched다.
`/detections`의 빈 배열이나 `/tf`의 다른 프레임은 태그 인식으로 판정하지 않는다.
출력 명령/모드 변경/각 제어모드 진단/AI 외력은 이벤트 또는 모드
의존성이 있으므로 선택 저장하되 전역 필수 주기 토픽으로 강제하지 않는다.

기본 bag에는 이미지가 없으며, crop 컬러와 depth만 파일 보관용으로 별도
구독한다. 저장 목록과 큐 누락/쓰기 오류는 각 archive에서 확인한다.
작은 수치 데이터는 BEST_EFFORT keep-last1
보조 구독으로 관측하므로 이 상태도 bag 저장 성공 자체를 보장하지 않는다.
`required_max_gap_sec` 기본 2초는 데이터 품질 경고 기준이며 제어기의 보호 timeout과
다르다. 수집 중 끊기면 GUI `DATA WARNING`, `live_health_events`에 기록하되 **남은
데이터 수집은 계속하며 모터를 제어하지 않는다**. 시작 준비가 `startup_timeout_sec`
(기본 10초)를 넘으면 자신의 녹화만 중지한다.

종료 후 실제 SQLite 수신 시각을 검사해 필수 토픽 0건/중간 및 끝부분의 긴 공백을
`postprocess.json`과 `session.json.data_quality`에 남긴다. recording-ready부터 최초
Stop 요청까지를 검사하여 준비/flush 시간을 공백으로 오인하지 않는다. `complete`는
이 수신 연속성 검사 통과이지 센서 정확도·소스 시각 동기화·매 프레임 무손실 보장이
아니다. `allow_incomplete:=true`는 센서 없는 개발 시험 전용이며 누락이 그대로 남는다.
태그 없는 실험도 기본 설정 그대로 가능하다. 태그 데이터를 아예 저장 대상에서도
제외하려는 경우에만 `enabled_groups`에서 apriltag을 빼면 된다. 사용자 지정 config에
태그 required_topics를 추가해 둔 경우에는 해당 필수 설정도 별도로 확인한다.

raw/Kalman force와 기존 dynamics 진단 등 수치 저장 항목은 유지했다.
`visualization` 그룹은 설정의 선택지로 남기되 기본 비활성화했다. 다시 켜면 큰
메시지와 crop의 중복 저장이 생기므로 필요한 실험에서만 명시적으로 선택한다.
2D 마찰/동역학 파생 CSV의 기존 계산 코드는
[legacy/record_node_2d.py](legacy/record_node_2d.py)에 보존했다. 예전 AprilTag 상대 자세는
`ID0` 기준 `ID1`이었고 이번 pose/CSV로 복원했다. 예전 cached 최신값을 이미지 시각에
붙이던 방식이나 미검출 시 0/이전 값 저장은 사용하지 않는다.
**기존 실험 파일은 삭제하거나 변경하지 않았다.**

### 한 파일로 확인하는 summary.csv (선택 갱신: 2026-09-25)

Record Stop 후 기존 토픽별 CSV에 더해 **`csv/summary.csv`**를 자동 생성한다.

F/T 센서 `/fts_data`, `/fts_data_kalman_filter`의 토크 `tx, ty, tz`는
토픽별 CSV와 summary 모두 소수점 한 자리로 저장한다. 힘, 모터 토크,
좌표, 각도 등 다른 열의 정밀도는 바꾸지 않으며 rosbag 원본도 유지한다.
기존에 저장된 CSV는 자동으로 덮어쓰지 않는다.
원본은 바꾸지 않으며, 이 표를 **학습 입력 원본 및 확인용**으로 사용한다.
학습용 X/Y 열 선택, 중간 누락 처리, 보간·필터링·학습/평가 분리는 별도
학습 Git 레포지토리에서 한다. CSV 생성 성공만으로 학습 준비가 끝난 것은 아니다.

#### 학습 핵심 신호가 확보된 시점부터 시작 (2026-09-26)

새 기본 프로필은 `summary_csv.startup_required_groups`를
`["motor", "loadcell", "fts", "wire", "angles"]`로 사용한다. 같은 각도 행에서
기존 시간 매칭 기준을 통과하고 아래 값이 모두 유한한 첫 행부터 summary를 쓴다.

- 모터 위치 4개, 로드셀 장력 4개, raw F/T 힘 XYZ, 케이블 길이 4개,
  실제 축 relative 각도 18개(홀수 tilt / 짝수 pan).
- 세션의 힘 축 정렬이 활성화됐으면 aligned 힘 XYZ와 변환 유효성도 확인한다.
  정렬 기능이 꺼져 있으면 aligned 열을 요구하지 않는다.
- 태그, tip/FK, 모터 가속도·토크, 케이블 속도, 칼만 힘은 기본 시작 조건이 아니다.
  필요하면 `fts_kalman`/`wire_velocity` 그룹도 설정할 수 있다. 특정 센서만 모으는
  별도 프로필은 실제 필요한 그룹을 명시한다. `[]`는 기존 시작 구간 보존 방식이다.
- 0과 음수는 정상 숫자로 인정한다. 매칭되지 않은 값이나 NaN/잘못된 배열을
  다른 시각의 값·이전 값·0으로 채우지 않는다. 시간 허용차 ±25ms는 그대로다.
- **처음 준비 구간만 제외**한다. 이후 누락 행은 삭제하지 않으며 빈칸/matched 정보와
  실제 시간 간격을 유지한다. 원본 bag/토픽별 CSV와 268열 구조는 바뀌지 않는다.
- 원래 타임스탬프와 `elapsed_s` 기준점을 유지하므로 첫 출력 elapsed_s는 0보다
  클 수 있다. `summary.schema.json.startup`과 CSV manifest에 제외 행 수,
  그룹별 미충족 횟수, 첫 출력 시각, 시작 후 미충족 행 수를 남긴다.
- 끝까지 동시에 확보되지 않으면 헤더만 있는 CSV와 `no_common_start`를 남기고,
  export는 `partial`로 보고한다. 원본은 보존하며 정상적인 학습 표가 생겼다고
  표시하지 않는다. 각도 자체가 없을 때의 `no_angle_samples`와 구분한다.

Data recorder 재시작 후 새 세션부터 기본값이 적용된다. 예전 세션의 설정 사본에
이 옵션이 없으면 기존 방식으로 내보내며, 기존 summary를 자동 덮어쓰지 않는다.
수동 `export_summary`는 세션의 설정 사본을 읽고, 명시적으로 다르게 처리하려면
`--startup-required-groups motor loadcell fts wire angles`를 사용할 수 있다.
이 옵션 역시 기존 summary가 없는 **새 CSV 내보내기 폴더**에서만 실행한다.

한 행은 `/estimated_segment_angle`의 유효한 원본 시각 한 개에 대응한다.
현재 268열이며, 기존 331열에서 unscaled tip의 위치/시간 연결 정보 9열을
빼고 72개 각속도 열을 실제 회전축의 relative 각속도 18열로 정리했다.
이 선택은 summary만 바꾸며 원본 토픽 저장 목록과 토픽별 CSV는 유지한다.

| 구분 | 통합 파일 열 예시 / 의미 |
|---|---|
| 모터 4개 | `motor.Motor #1 position [count]` … `#4`; 속도·가속도·토크도 개별 열 |
| 로드셀 4개 | `loadcell.Loadcell #1 tension` … `#4`; voltage도 개별 열 |
| F/T | `fts.fx/fy/fz/tx/ty/tz`, `fts_kalman.*`; 단위는 원본 그대로 |
| 케이블 | `wire.Cable #1 length` … `#4`, `wire_velocity.*`; 모터 피드백 환산 변위/속도 |
| 관절각 | `pan_relative_01_rad` … `18`, `tilt_relative_*`, `pan_absolute_*`, `tilt_absolute_*` |
| 실제 축 상대각 | `relative_angle_1` … `relative_angle_18`; tilt1, pan2, …, pan18 (rad) |
| 실제 축 상대 각속도 | `relative_angular_velocity_1_rad_s` … `relative_angular_velocity_18_rad_s`; 같은 tilt-pan 순서 |
| depth 피팅 끝점 | `estimated_tip.x_m/y_m/z_m`; unscaled는 summary에서만 제외 |
| FK tip | `fk_tip.x_m/y_m/z_m`(PointStamped), `fk_tip_tf.*`(TF 위치+회전) |
| 추정 끝 링크 자세 | `estimated_tip_orientation.qx/qy/qz/qw`, `roll_deg/pitch_deg/yaw_deg` |
| 태그 | `tag_id0_to_id1.x_m/y_m/z_m`, `qx/qy/qz/qw`, `roll_deg/pitch_deg/yaw_deg` |

pan/tilt **각 배열은 실제 관절 번호 기준 18칸**을 유지한다. relative는 홀수
관절 tilt, 짝수 관절 pan이며 비활성 축의 원래 0을 그대로 저장한다. 총 36개의
회전관절이라는 뜻이 아니다. absolute는 hrm_base에서 본 각 세그먼트의 방향각이며
relative 각도를 단순히 더한 값이 아니다. 두 absolute 축 모두 값이 있을 수 있다.
18개 각속도는 같은 각도 메시지의 `tilt_angular_velocity_relative`(홀수)와
`pan_angular_velocity_relative`(짝수)를 선택한다. 0, 부호, rad/s를 유지하며
새 미분/필터링은 하지 않는다. 원래 relative/absolute 각속도 4배열은 토픽별
CSV와 bag에 모두 남는다.

명령 흐름은 목표 tilt/pan → IK 목표 케이블 길이 → 모터 명령이다. 그러나
위 `wire.*`는 **실제 모터 위치/속도의 환산값**이며 IK 목표값과 다르다.
`/kinematics/target_wire_length`는 기존처럼 bag/토픽별 CSV에 따로 보존한다.

태그는 `ID0_T_ID1`의 translation과 rotation을 열로 분리한 것이다. 동일 정보를
4×4 행렬 16개 열로 중복하지는 않는다. 회전은 원본 quaternion xyzw와 파생
extrinsic XYZ Euler(deg), 즉 `Rz(yaw) Ry(pitch) Rx(roll)`를 함께 남긴다.
**태그 장착 위치/회전 보상은 아직 적용하지 않았다.** 후에 `base_T_ID0`와
`ID1_T_tip` 보상을 넣을 때 원본 tag 열을 덮어쓰지 않고 별도 파생 열로 추가할 것.

`estimated_tip`은 피팅 곡선의 마지막 **경계점**이고 `fk_tip_tf`는 기구학적 tip이다.
`estimated_tip_orientation`은 마지막 링크 중심 TF의 **회전만** 가져온다. 그 중심
위치를 tip으로 사용하지 않으며, 이 회전은 필터링된 DH 각도에서 만들어진 것으로
독립적인 depth roll 관측은 아니다. 같은 각도를 쓰는 FK 회전과 구별해서 해석한다.

#### 시간 연결과 빈칸

- 센서와 태그는 기본 ±25 ms 이내의 가장 가까운 원본 시각을 연결한다. 과거/미래
  표본 모두 가능하고 가까운 표본이 여러 행에 재사용될 수 있다. 학습용 동기화가 아니다.
- 추정/FK 위치와 회전은 해당 각도와 **원본 시각이 정확히 같을 때만** 연결한다.
- 각 그룹의 `matched`, `source_time_ns`, `bag_receive_time_ns`, `frame_id`,
  `time_basis`, `time_difference_ms`를 함께 남긴다. `matched`는 시간 연결 성공만
  뜻하고, 센서 건강·힘 보정·카메라 동기화를 보장하지 않는다.
- header가 없는 케이블 토픽은 각도 메시지의 **수신 시각** 기준으로 연결하고
  `time_basis=receive`로 명시한다. source timestamp를 만들어 붙이지 않는다.
- `/detections` CSV가 있으면 가장 가까운 검출 프레임에 ID0·ID1이 모두 있어야
  하며, 그 프레임과 정확히 같은 시각의 상대 자세만 사용한다. 명시적인 미검출을
  주변 유효 자세로 채우지 않는다. detections가 없으면 `detection_guard=unavailable`.
- 오래된/미수신 값, 잘못된 숫자/배열 길이는 빈칸이다. 실제 0은 0으로 보존한다.
- 위 시작 조건을 통과한 이후에는 빈칸 때문에 행 전체를 삭제하지 않는다.
  단, 각도 anchor의 시각이 0/누락이거나
  중복/역행하면 그 anchor는 summary에서 제외하고 schema에 건수를 기록한다.
  토픽별 CSV/bag의 수신 원본은 그대로이며 미발행/미수신 메시지를 만들어내지 않는다.
- 시간은 실제 표본 간격과 데이터 연결을 확인하는 데 필요하다. 행 순서만으로
  드롭 프레임이나 처리 지연을 알 수 없다. 파일별 세션 경계도 학습 쪽에서 유지한다.
- `matched=False`는 학습에서 자동 무시된다는 뜻이 아니다. 잡음이 있는 유한한
  값과 누락/잘못 연결된 ground truth는 구별하고, 결측 처리/학습 마스크는 별도
  학습 레포지토리에서 결정한다. 현재 코드에 새 품질 임계값 필터는 추가하지 않았다.
- 각도 표본이 없으면 헤더만 생성하고 `no_angle_samples`로 명시한다.
- 중간 TF, 이벤트성 명령·설정·진단은 선택된 bag/토픽별 CSV에 보존하며 이 보기용
  표에 반복하지 않는다. crop PNG는 `images/hrm_crop/`와 해당 `index.csv`에 남고,
  다른 이미지/클라우드/marker는 기본 기록 대상이 아니다.

연결 정책과 열 정의는 `summary.schema.json`에 저장된다. 설정:

```json
"summary_csv": {"enabled": true, "max_time_difference_ms": 25.0}
```

기존에 토픽별 CSV까지만 만들어진 기록에는 다음 명령으로 요약만 추가할 수 있다.
이미 summary가 있으면 덮어쓰지 않는다.

```bash
ros2 run record_pkg export_summary /path/to/session/csv
```

### 추정 끝점과 FK 끝점 CSV (2026-09-21)

아래 세 토픽은 `geometry_msgs/PointStamped`로, **`hrm_base` 기준 XYZ(m)**와
원본 source stamp를 각각 독립 CSV에 저장한다. 추가 토픽은 선택 저장이며 기존
`required_topics`를 늘리지 않는다. 이 데이터가 실험에 반드시 필요하면 해당 토픽을
필수 목록에도 명시한 뒤 Data recorder를 재시작한다.

| 토픽 | 의미 |
|---|---|
| `/estimated_tip_position` | depth 기반 3D 피팅 곡선의 마지막 경계점. 설정된 하드웨어 길이 보정이 적용된 최종 추정점으로, 마지막 segment의 **중심점이 아님** |
| `/estimated_tip_position_unscaled` | 같은 피팅 곡선에서 **하드웨어 전체 길이로 배율 보정하기 전**의 끝점. 피팅·base 변환은 적용되어 있으므로 raw depth 픽셀 또는 무필터 값이라는 뜻은 아님 |
| `/kinematics/fk_tip_position` | 추정 관절각과 하드웨어 링크 길이로 계산한 FK 끝점. 기존 `/tool_endeffector_pose` 위치를 source stamp/frame이 있는 형식으로 함께 기록 |

세 CSV의 주요 열은 `point.x`, `point.y`, `point.z`, `header.frame_id`,
`source_time_ns`, `bag_receive_time_ns`다. 파일명은 기존 규칙대로 토픽명과 해시를
사용하며 `csv/manifest.json`에서 정확한 토픽↔파일 대응을 확인한다.
기존 `/tool_endeffector_pose`와 각도·TF 기록도 유지한다.

각 토픽은 **발행된 시점의 행만** 저장한다. 추정 실패/미수신 프레임에 과거 값을
복사하지 않으며, 아예 발행되지 않은 선택 토픽은 manifest에 `not_recorded` 또는
`no_messages`로 남는다. 토픽별 주기가 달라도 공통 행으로 강제하지 않는다.
비교·학습 시에는 원본 source stamp와 frame을 기준으로 오프라인 정렬할 것.
`unscaled`는 배율에 의해 오차가 숨겨지지 않도록 남기는 비교용 데이터이며,
독립적인 ground truth는 아니다. 두 카메라 간 보정 및 ID1↔실제 tip의 장착 오프셋은
별도로 확인해야 한다.

## AprilTag 실행과 좌표계

설치된 `apriltag_ros`를 유지한다(Isaac로 교체하지 않음). GUI에서 AprilTag용
두 번째 카메라를 먼저 켜고 AprilTag를 실행한다. 기존 전체 자동실행 ON/OFF 선택은 유지한다.
단독 실행:

```bash
ros2 launch launcher apriltag.launch.py
```

기본 입력은 D435i의 `/tag_camera/tag_camera/color/image_raw`와
`/tag_camera/tag_camera/color/camera_info`다. 먼저 `image_proc`가
`/tag_camera/tag_camera/color/image_rect`로 보정한 뒤 검출하며, 이미지 시각과
`tag_camera_color_optical_frame`은 유지한다. 다른 카메라는 `image_topic:=...`,
`camera_info_topic:=...` 인수로 지정한다. 이미 보정된 D405 이미지를 사용하려면
`rectify:=false`와 기존 D405 image/CameraInfo 토픽을 함께 지정하고 기록 설정도 맞춘다.
intrinsic은 CameraInfo에서 가져오며 pose scale은 태그 실제 변 길이에 의존한다. 저장소의
`launcher/config/apriltag.yaml`은 **tag36h11, ID 0/1, 20mm**로 설정되어 있다.
크기는 흰 여백을 제외한 검정 테두리 바깥쪽 한 변 기준이며, 실물의 측정 길이와 일치하는지 확인한다.

| 토픽 | 자세 표현 |
|---|---|
| `/apriltag/tag0/pose` | 검출기가 사용하는 카메라 optical frame 기준 ID0 |
| `/apriltag/tag1/pose` | 같은 카메라 frame 기준 ID1 |
| `/apriltag/tag1_in_tag0/pose` | ID0 기준 ID1: `inverse(camera_T_ID0) @ camera_T_ID1` |

모두 PoseStamped(m, quaternion xyzw), 원본 이미지 stamp를 보존한다. 상대 자세는
두 TF의 부모/시각이 **정확히 같을 때만** 계산한다. tag TF가 가려지거나 오래되면
출력하지 않으며 `hrm_base` 좌표계는 바꾸지 않는다. 단일 태그만 검출되면 해당
camera pose만 기록된다. `/tf` 원문 및 `/detections`도 별도로 보존한다.
설정 YAML 원문/해시와 detector/pose 노드 공개 파라미터도 session snapshot에 들어간다.

두 번째 카메라의 **CameraInfo와 검출·자세 결과만 저장**한다. 기본 프로필은
태그 원본/보정 RGB 모두 저장하지 않으므로 나중에 태그 검출을 다시 실행할 수 없다.
`/tag_camera/tag_camera`의 공개 파라미터와
`launcher/config/realsense_apriltag.yaml` 원문/해시도 녹화 시작 시 함께 보존한다.
D405도 CameraInfo를 유지하되 영상은 crop 컬러 및 crop depth만 보관한다.

두 카메라 사이의 외부 보정값은 아직 정의하지 않는다. 따라서 ID0 기준 ID1 상대
자세는 사용할 수 있지만 D435i camera pose를 그대로 `hrm_base` 좌표로 해석하면 안 된다.
두 카메라의 source header 시각과 bag 수신 시각을 각각 보존하며, **자동 동기화나
시간 보간은 하지 않는다**. 후처리 정렬 전에 카메라별 clock 관계와 지연을 검증한다.

## 저장 형식

```text
record/<session>/
  bag/                  수치 ROS 메시지 + metadata.yaml (이미지 없음; 2 GiB 단위 분할)
  images/hrm_crop/
    *.png               HRM crop 컬러만 무손실 압축 저장
    index.csv           파일명 ↔ 원본 source stamp/frame, 수신 시각, 크기, 접촉 라벨
    manifest.json       이미지 보관 설정, 수신/저장/누락 수 및 오류
  images/hrm_crop_depth/
    *.png 또는 *.npy    원래 uint16/float32 depth, 색상화하지 않음
    index.csv           시간/라벨 + ROI/scale/crop intrinsics
    manifest.json       depth 저장 수와 calibration/쓰기 오류
  session.json          선택/누락 토픽, 시작/종료, 실제 저장 수량, 설정 snapshot
  recording_config.json 이 실험에 사용한 recorder 설정
  qos_overrides.yaml    bag 구독 QoS
  recorder.log          rosbag 프로세스 로그
  postprocess.json      종료 후 실제 bag 누락·수신 간격 검사 및 CSV 작업 결과
  export.log            종료 후 검사/CSV 프로세스 로그
  csv/
    manifest.json       토픽↔파일 대응, 저장 행 수/누락/변환 오류/무결성 결과
    session_metadata.csv 하드웨어·설정 메타데이터(변환 시작 시 snapshot)
    summary.csv         기존 268열 통합 수치 데이터
    summary.schema.json 통합 열/시간 연결 정책
    <topic>__<hash>.csv 토픽별 수치/자세/메시지 시각
```

`auto_export_csv: true`가 기본이므로 GUI **Record Stop** 후 수치 CSV가 자동 생성된다.
수치 CSV를 실시간으로 합치지 않고, 닫힌 bag을 낮은 우선순위의 별도 프로세스로
읽는다. `false`이면 CSV는 생략하지만 종료 후 bag 누락 검사는 수행한다.
CSV에 배열은 JSON 셀로 저장하므로 모터/로드셀 4개 및 각도 18개가 모두 보존된다.
AprilTag는 XYZ, 원래 quaternion 및 파생 RPY(rad)를 함께 저장한다. 각 행에는
`bag_receive_time_ns`, `source_time_ns`가 있고 headerless 메시지의 source는 공란이다.
TF는 transform마다 한 행으로 parent/child와 해당 stamp를 보존한다.
기본 프로필은 이미지/depth/cloud/MarkerArray를 bag에 넣지 않는다. crop PNG와
image index는 수집 중 별도 쓰기 작업에서 기록한다. 예전 영상 포함 bag이나 명시적으로
선택한 시각화 프로필은 기존 exporter가 계속 지원하며, 이미지/클라우드는 CSV에
시각·크기만, MarkerArray는 생략 사실을 manifest에 남긴다.

녹화 노드/GUI를 먼저 종료한 경우 bag은 flush하지만 CSV 처리는 `pending_or_interrupted`
일 수 있다. 나중에 **bag을 재생하지 않고** 다음과 같이 변환한다:

```bash
ros2 run record_pkg export_csv record/<session>
# 기존 CSV 또는 중단된 결과는 덮어쓰지 않음. 다른 폴더로 재변환:
ros2 run record_pkg export_csv record/<session> --output record/<session>/csv_retry
```

자동/수동 모두 분할 db3 전체를 metadata.yaml을 통해 읽는다. 개별 db3가 아니라
세션 또는 bag 폴더를 인수로 준다. 변환 성공과 데이터 품질은 별개다. 필수 데이터가
없어도 있는 메시지는 내보내며, 없는 값은 0으로 채우지 않고 manifest에 명시한다.

### 세션별 힘 축 정렬 CSV (2026-09-18)

GUI Record 버튼 위 **Force axes for CSV (sensor → hrm_base)**에서
**Add aligned_fx / aligned_fy / aligned_fz**를 켜고 축을 선택한다(초기 OFF).
설정은 **출력 Base 축 = 부호 × 입력 센서 축**이다. 예를 들어:

| GUI 설정 | 같은 CSV 행에 추가되는 값 |
|---|---|
| Base Fx = + sensor Fx | `aligned_fx = +wrench.force.x` |
| Base Fy = − sensor Fy | `aligned_fy = -wrench.force.y` |
| Base Fz = − sensor Fz | `aligned_fz = -wrench.force.z` |

Y/Z 교환 예: `Base Fx=+sensor Fx, Base Fy=-sensor Fz, Base Fz=+sensor Fy`.
각 입력 축을 정확히 한 번 사용해야 하며, 두 오른손 좌표계 사이의 회전이므로
행렬식은 +1이어야 한다. 축 중복이나 반사(-1) 조합은 설명과 함께 거부한다.
현재 UI는 축 교환/부호 반전(90° 단위 장착)용이며 임의 각도 캘리브레이션은 아니다.

GUI는 `/record/set_parameters_atomically`로 `force_alignment_enabled`와
`force_alignment_axes`(예: `['+x', '-y', '-z']`)를 함께 전달하고, 성공 응답을 받은
뒤에만 Record 시작을 요청한다. 잘못된/구버전 설정을 무시하고 녹화하지 않는다.
녹화 준비·녹화·flush·CSV 변환 중에는 GUI 설정과 recorder의 해당 파라미터 변경을
막는다. 변경은 다음 Record 세션부터 적용한다. 수집을 지시하는 GUI는 하나만 사용한다.

설정은 녹화 시작 시 `session.json → snapshot.force_alignment`에 부호축, 실제
`matrix_base_from_sensor`, 대상 frame `hrm_base`, 단위 유지 등의 설명과 함께 고정된다.
CSV 변환은 **이 세션 사본만** 사용한다. 나중에 GUI/config를 바꿔도 기존 세션의
해석은 바뀌지 않는다. `csv/manifest.json`과 `session_metadata.csv`에도 출처가 남는다.

- `/fts_data`와 `/fts_data_kalman_filter` 각각의 CSV에만 `aligned_fx`, `aligned_fy`,
  `aligned_fz`, `aligned_frame_id`, `aligned_force_valid`를 추가한다. 원본 열
  `wrench.force.x/y/z`, 토크, 원래 `header.frame_id`, 시각, 단위는 그대로 보존한다.
- 원본 rosbag·실시간 센서 토픽·제어기·모터/로드셀 보호는 바꾸지 않는다. 별도의
  aligned 토픽이나 실시간 force 구독을 추가하지 않고, 종료 후 CSV에서 계산한다.
- 활성화된 경우 원본이 NaN/Inf 등일 때 원본 행은 보존하고 변환값만 공란/false로
  남긴다. `aligned_force_valid`는 계산 가능한 숫자라는 뜻이지 CAN 연결/센서 정상
  판정이 아니다. CAN 미연결의 0을 실제 외력 학습 라벨로 사용하면 안 된다.
- 단위·gain·영점·작용/반작용 부호는 보정하지 않는다. 토크 회전이나 토크 기준점
  이동도 하지 않는다. 명칭은 `calibrated_`가 아닌 `aligned_`로 구분한다.
- 한 세션 내 센서 장착 방향은 고정이라고 가정한다. 센서가 수집 중 회전한다면
  시각별 자세를 이용하는 별도의 동적 변환이 필요하다.

OFF이거나 이전 세션에 축 설정 snapshot이 없으면 기존 원본-only CSV를 유지한다.
과거 문서의 고정 회전 예시를 임의로 적용하지 않으며 기존 bag/CSV는 덮어쓰지 않는다.
`/fts_data` 원본은 기존 센서 영점 및 선택한 LPF/MAF 처리가 반영된 토픽이며, 이 기능이
필터 이전 ADC 원본을 새로 만드는 것은 아니다. Kalman 토픽 역시 별도로 보존한다.

### 하드웨어·설정 메타데이터

매번 **Record 시작 시** `session.json`의 `snapshot`에 다음을 남긴다(schema version 2).

| 필드 | 내용과 출처 |
|---|---|
| `hardware_constants.sources.<node>` | 실행 중 controller의 읽기 전용 `hardware.*` 파라미터. `OP_MODE`(8/CSP), 모터/관절/세그먼트 수, 기어비/엔코더, os/l/le/전체 길이, ARC/직경/WIRE_DISTANCE, 각도/장력/모터 제한 등 **해당 바이너리에 컴파일된 값** |
| `runtime_parameters.nodes.<node>` | 녹화 시작 직후 파라미터 서비스로 조회한 이름/타입/값, 요청·응답 시각, 성공/누락 상태. control mode, motor output enable, pan/tilt PD 게인, 카메라 설정 등 **노드가 파라미터로 공개하는 값** |
| `package_configs` | estimation `config.json`/ROI 및 GUI system component 설정의 JSON 사본 |
| `reference_files` | 위 설정 파일과 `hw_definition.hpp`, `control_parameters.hpp`, AprilTag 및 두 번째 카메라 YAML의 원문/경로/SHA-256/복사 시각. 녹화 시작 당시 디스크 파일이며 **실행 중 적용값이라고 단정하지 않음** |
| `last_observed_filter_setting` | recorder가 마지막으로 수신한 `/data_filter_setting`; 미수신이면 `null` |

`recording_config.json`에는 이 세션에서 사용한 recorder 설정/토픽 선택/단위 및
좌표계 메모가 보존된다. 그 안의 `notes.hardware`는 설명용 메모이지 실행값의 근거가
아니다. 실행 중 값의 출처는 위 필드에서 구분한다.

`metadata_nodes` 기본값은 현재 GUI/launch의 노드 이름을 따른다. 직접 실행하여
`/ControlNode`, `/segment_estimation_node`, 다른 namespace 등을 사용하면 이 목록도
맞춘다. 조회는 비동기로 수행하며 기본 최대 3초(`metadata_timeout_sec`) 후 마친다.
bag 수집은 먼저 시작하지만 GUI `recording` 상태는 조회 종료/시간 초과 및 bag 구독
준비 후 표시된다. 꺼진 노드/파라미터 서비스 차단은 `unavailable`, 무응답은 `timeout`,
중복된 노드 이름은 `ambiguous`로 남긴다. 비필수 메타데이터 누락만으로 수집을
거부하지 않으며 전체 snapshot은 `partial`로 표시한다. 빠르게 Stop하면 완료되지
않은 조회는 `cancelled`로 보존한다.

**OP_MODE는 드라이브 실측 모드가 아니라 C++ 상수**다. `/control_mode`(제어기 모드),
GUI `op_mode` 파라미터와 별개로 출처를 남긴다. controller의 `hardware.*`는 읽기 전용이고
launch/CLI override도 무시하므로 값을 바꾸려면 실제 하드웨어 정의를 수정·재빌드해야
한다. 이번 업데이트 후 **robot_control과 recorder를 재시작**해야 새 metadata를 사용할
수 있다(동작 중이면 안전하게 정지한 뒤 재시작). 구버전 controller는 값이 `unavailable`로
남으며 참조 헤더로 실행 중 값을 추정하지 않는다.

조회는 여러 노드에 걸친 원자적 snapshot이 아니다. 수집 준비 중 설정을 바꾸지 말고,
이후 변경은 `/parameter_events`, `/control_mode`, `/data_filter_setting` 기록과 함께
검토한다. 파라미터 조회만으로 드라이버 실제 상태나 파라미터로 공개하지 않은 내부
필터 상태를 알 수는 없다. 특히 기존 dynamics/admittance의 선언 파라미터가 내부 계산에
그대로 반영되는지는 별도 검증 대상이다. 설정 파일을 노드 실행 후 수정했다면 사본과
노드가 시작 때 읽은 설정이 다를 수 있다.

실시간 수치 CSV 결합이나 PNG 압축·파일 쓰기를 이미지 콜백에서 하지 않는다.
crop은 유한 큐(기본 16개)를 통해 별도 이미지 쓰기 작업으로 넘긴다.
C++ rosbag 엔진은 선택한 수치 메시지를 저장하고 64 MiB double-buffer cache를
사용한다(최대 약 128 MiB).
SQLite `resilient` preset, 압축 없음. 기본 2 GiB 여유 공간 미만이면 시작 거부/자동 Stop.
crop 구독은 best-effort로 수집이 영상 발행자를 막지 않게 하며,
**전체 프레임의 무손실 수신을 보장하지 않는다**. `/tf_static`은 transient-local로 기존 TF도 수신한다.
설치된 estimation_pkg의 FastDDS 대용량 SHM profile을 자동 사용하되 사용자 환경설정을
덮어쓰지 않는다. 기존 기본 RMW 설정 자체는 변경하지 않는다.

이전 무압축 RGB+depth만 해도 848×480×(3+2)×30 ≈ **61 MB/s, 3.66 GB/min**이었다.
새 기본값은 crop 컬러+depth만 압축 저장하며 실제 용량은 ROI 크기와 이미지 내용에 따라 달라진다.
`session.json`의 bag counts와 이미지 manifest의 counts는 각각 저장/수신의 의미를
확인해야 하며, 카메라 실제 fps와 동일한 의미는 아니다.
예상치 않은 종료/미완성 bag은 `failed`, 필수 토픽이 0개면 `INCOMPLETE`로 표시한다.

## 학습용 후처리 원칙

- 현재 기본값은 **crop 컬러/depth와 온라인 각도·수치 데이터**를 저장한다.
  depth 수량·calibration·RGB/depth 시간 대응을 확인한 뒤 3D 재처리에 사용할 수 있다.
  2D mask/skeleton 재처리 시 원래 ROI를 다시 잘라내지 않도록 crop 입력을 구별한다.
- Header stamp와 frame_id, CameraInfo K/D, TF를 원문 그대로 보존한다.
  bag timestamp는 수신 시각이며 **노출 시각으로 바꿔 쓰면 안 됨**.
- 3D 재처리는 같은 세션 안에서 시간순 시퀀스를 같은 causal filter/skeleton/depth/curvefit
  파이프라인에 통과시킨 뒤 **소스 이미지 시각**에 해당하는 센서/모터 데이터를 결합한다.
  최대 시간차/센서 지연/clock 일치/보간 정책을 검증하고 오래된 샘플을 배제한다.
- `fx`는 `/fts_data.wrench.force.x`에 해당하며 별도의 Kalman 신호와 구분한다.
  이것도 F/T zero-offset 및 선택적 LPF/MAF 이후 값이지 ADC 원시값은 아니다.
- 힘 축 정렬은 위 세션별 설정으로 선택한다. `Fb=[Fsx,-Fsz,Fsy]`는 한 장착 예시일
  뿐, 모든 실험의 기본값으로 적용하지 않는다.
  raw sensor-frame 값은 남긴다. N/mN 및 반력 부호는 실제 보정으로 확정해야 한다.
- 당시 config/ROI와 마지막으로 관측한 filter 설정을 snapshot에 남긴다.
  recorder 시작 이전의 volatile filter 변경은 알 수 없으므로 `null`이면 **미확인**이다.
  녹화 시작 후 GUI에서 의도한 필터 설정을 다시 전달해 기록으로 남길 것.
  초기 공개 ROS 파라미터는 별도 조회하지만 이벤트/조회만으로 미공개 내부 설정까지
  복원할 수 있는 것은 아니다.
- base 평균 고정/칼만 state는 warm-up 이력에 의존한다. 학습용 녹화는 초기 warm-up을
  포함하고, 온라인/오프라인 초기 조건과 필터 지연을 맞춘다. 저장된 TF는 온라인
  base origin 확인에 쓸 수 있다. 현재 소스 snapshot은 실제 실행 중 config와 다른
  파일을 뒤늦게 수정한 상황까지 보장하지 않는다.
  미기록 구간의 필터 상태는 오프라인에서 동일하게 복원할 수 없다.
  세션별 초기화/warm-up 정책을 별도로 정해야 한다.
- 토픽별 CSV와 crop PNG 저장은 구현되었지만, **이미지 시각 기준 학습 데이터
  결합/보간·오프라인 각도 재추정**은 후속 작업이다. 현재 CSV는 각 토픽의 독립적인
  샘플 시각을 유지하며, 예전처럼 최신 센서값을 하나의 이미지 시각으로 꾸미지 않는다.

**bag에 `/motor_command`가 들어 있다. 연결된 로봇의 ROS domain에 전체 bag을
play하지 말 것.** 오프라인 학습은 reader로 읽거나 격리 domain에서 허용된 센서
토픽만 재생해야 한다.

## 테스트

crop RGB+depth 전용 실제 ROS 합성 테스트(장비 구동 없음):

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ROS_DOMAIN_ID=184 ROS_LOCALHOST_ONLY=1 \
  python3 src/record_pkg/test/crop_depth_record_roundtrip.py
```

격리 domain의 기존 노드가 있으면 실행을 거부한다. Record/Stop 두 번으로 별도
폴더, 원본 depth/RGB 픽셀·시각·보정정보, 수치 bag/268열 summary, 이전 파일 비변경을 검증한다.

단위 테스트: `test/test_capture.py`, `test/test_metadata.py`,
`test/test_topic_health.py`, `test/test_bag_integrity.py`, `test/test_export_csv.py`,
AprilTag 기하 테스트, GUI 상태 테스트: `gui_py_pkg/test/test_record_status.py`.

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ROS_DOMAIN_ID=76 ROS_LOCALHOST_ONLY=1 \
  FASTRTPS_DEFAULT_PROFILES_FILE="$PWD/src/estimation_pkg/config/fastdds_images.xml" \
  python3 src/record_pkg/test/rosbag_roundtrip.py
```

실제 카메라/TCP/robot_control 없이 가짜 848×480 RGB/depth 30 Hz, 4모터/4로드셀 및
raw/Kalman force 100 Hz, 18축 angle, recorder 시작 전 정적 TF로 실제 launch/service/bag
저장 및 read-only 복원을 검사한다. 테스트 파일은 `/tmp/hrm_record_roundtrip_*`에 남는다.
실제 카메라+estimator+GUI+모터+디스크 전체 부하의 무손실 보장은 별도의 supervised
시험으로 확인해야 한다.

메타데이터 실통신/저장 검증은 같은 격리 환경에서
`python3 src/record_pkg/test/metadata_roundtrip.py`로 실행한다. 이 테스트만은 실제
robot_control 바이너리를 **output=false + actuator 토픽 remap**으로 실행하며
카메라/TCP는 실행하지 않는다. 컴파일 상수/override 방지/실행 게인/연속 세션 갱신/
부재 노드 timeout/즉시 Stop의 partial snapshot 및 실제 bag 저장을 확인한다.
결과는 `/tmp/hrm_metadata_roundtrip_*`에 보존한다. 두 통합 테스트를 동시에 실행하지 않는다.

추가 검증(실제 카메라/모터 없이 실행):

```bash
# 실제 설치된 apriltag_ros에 합성 ID0/ID1 이미지, 자세/스케일/시각/재시작 확인(domain79)
python3 src/record_pkg/test/apriltag_pose_smoke.py
# 필수 샘플 없는 시작 거절, 중도 누락 경고, 자동 CSV와 INCOMPLETE 확인(domain80)
python3 src/record_pkg/test/record_required_smoke.py
```

`test_export_csv.py`는 실제 분할 db3를 만들고 motor/loadcell/angles/force/AprilTag/TF/
image metadata CSV를 읽어 검사한다. `test_bag_integrity.py`는 SQLite 손상·누락·
수신 공백·정상 recording 구간 및 읽기 전용 동작을 검증한다. 이 합성 검증 결과는
실제 조명·태그 가림·렌즈 calibration·전체 시스템 부하를 대신하지 않는다.
