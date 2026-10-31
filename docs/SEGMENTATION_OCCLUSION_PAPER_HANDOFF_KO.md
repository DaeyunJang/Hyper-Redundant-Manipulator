# HRM 부분 가림: visible-body segmentation 및 관절각·FK tip 변화량 검증

작성일: 2026-10-27 (Asia/Seoul)  
실험 결과 생성일: 2026-10-24  
목적: 논문을 함께 작성하는 사람/AI가 실제 수행한 방법, 결과, 평가 범위를 정확하게 이해하도록 제공하는 기술 인계 문서.

## 1. 먼저 확정해야 하는 실험의 의미

이번 실험은 **HRM 몸체 중간을 합성 가림으로 여러 조각으로 보이게 만들었을 때, 보이는 몸체의 segmentation과 관측 가능한 depth만으로 계산한 관절각 및 FK tip pose가 무가림 추정값에서 얼마나 달라지는지** 평가한 개발 단계 실험이다.

핵심 처리 흐름은 다음과 같다.

```text
동일한 원본 RGB-D 프레임
├─ 무가림 RGB → U-Net V2 → clean mask → 관측 depth → 3D 곡선/관절각/FK
└─ 중간 가림 합성 RGB → 같은 U-Net V2 → disconnected visible mask
   → 가림 영역 depth 제외 → 보이는 조각들의 skeleton/depth
   → 같은 3D 곡선/관절각/FK 계산 → clean 결과와 paired 비교
```

중요한 구분:

- **기준값은 같은 U-Net으로 같은 프레임을 무가림 처리한 추정값이다.** 저장된 OpenCV 원본 마스크로 얻은 형상을 기준으로 비교한 결과가 아니다.
- AprilTag라는 이름의 데이터 폴더를 사용한 것은 맞지만, **이번 비교에서 AprilTag pose를 ground truth로 사용하지 않았다.**
- 따라서 아래 수치는 `occlusion-induced deviation`, `무가림 추정값 대비 변화량`이다. 실제 관절각/실제 tip pose에 대한 절대 정확도를 뜻하지 않는다.
- U-Net은 **보이는 몸체만** 예측한다. 가려진 몸체까지 마스크로 채우는 amodal segmentation을 학습한 것이 아니다. 끊어진 관측 사이의 연속 형상은 후단의 매끄러운 곡선 모델로 추정한다.
- 후단의 3D 피팅·관절각·FK 계산 핵심은 재사용했지만, **production의 전체 영상처리 경로를 수정 없이 그대로 실행한 실험은 아니다.** 여러 마스크 조각을 유지하는 실험용 전처리를 사용했다.

## 2. 사용 데이터와 출처

### 2.1 RGB-D 형상/pose 비교 데이터

| 구분 | 결과 파일의 dataset 이름 | 사용 프레임 | 내용 |
|---|---|---:|---|
| 신규 직선 영상 | `live_black` | 40 | 검정 배경에서 대체로 직선·정지 상태의 HRM을 새로 촬영한 RGB-D |
| 저장된 굽힘 영상 | `archive_45deg_1` | 64 | `shape_estimation_AprilTag_45deg_1`에 저장된 굽힘/동적 움직임 RGB-D |

아카이브의 실험 당시 경로:

```text
/home/daeyun/Hyper-Redundant-Manipulator/record/
  20260927_HRM_shape_estimation_apriltag/shape_estimation_AprilTag_45deg_1/
```

2026-10-27 확인한 현재 데이터 위치는 다음과 같다. 위의 과거 `record/` 경로는 현재 존재하지 않지만, 저장 프로토콜에는 당시 경로가 남아 있다.

```text
/home/daeyun/Hyper-Redundant-Manipulator-Segmentation/datasets/
  20260927_HRM_shape_estimation_apriltag/shape_estimation_AprilTag_45deg_1/
```

신규 직선 영상은 segmentation 프로젝트의 다음 위치에 있다.

```text
runs/occlusion_pose_20261024/live_capture_black/
  hrm_crop/
  hrm_crop_depth/
  manifest.json
```

신규 RGB-D 80쌍 중 40프레임을 선택했다. 아카이브에는 RGB 1,670장, depth 1,669장이 있었으며, **`source_time_ns`가 정확히 같은 1,669쌍을 구성한 뒤 64프레임을 선택했다.** RGB와 depth의 파일 순번이 어긋나 있으므로 같은 파일 인덱스를 직접 연결하면 안 된다.

각 원본에 clean을 포함한 7조건을 적용했다. 따라서 U-Net V2는 `104 × 7 = 728`건이며, clean을 제외한 가림 비교는 `104 × 6 = 624`건이다. 이는 **104개의 원본 프레임을 반복 사용한 624개 프레임–조건 쌍**이지, 624개의 독립 촬영 표본이 아니다.

### 2.2 Segmentation 학습과 별도 마스크 평가

학습 RGB와 정답 역할의 마스크는 기존 HRM crop 이미지와 OpenCV 기반 몸체 마스크다. 마스크는 **pseudo-label**이며 독립적인 수작업 정답이 아니다.

V2 학습 설정:

| 항목 | 설정 |
|---|---|
| 모델 | U-Net, ImageNet 사전학습 ResNet18 encoder |
| Decoder channels | `(128, 64, 32, 16, 16)` |
| 입력 | RGB, 384 × 384 letterbox |
| 출력 | 보이는 HRM 몸체의 binary mask, sigmoid threshold 0.5 |
| 학습 원본 | 1,600장: `session_01` 671장 + `session_02` 929장 |
| 검증 원본 | `session_03` 96장, 각각 clean/세 줄 30%/복합 배경·원근·가림 조건 |
| 학습 | 16 epochs, batch 8, seed 173 |
| Optimizer | AdamW, learning rate 3e-4, weight decay 1e-4, cosine scheduler |
| Loss | `0.5 × BCE + 0.5 × Dice loss` |
| 체크포인트 선택 | 검증 micro foreground Dice 최대 |
| 실행 환경 | RTX 3060, PyTorch 2.7.1+cu126, AMP |

마스크 성능 표의 평가는 별도 원본 128장 × 10조건 × 3방법(OpenCV/V1/V2)이며, 아래 RGB-D 40/64프레임 pose 평가와 표본 구성이 다르다. V1/V2는 학습량과 epoch도 달라 **증강 방식 하나만 바꾼 통제된 ablation**이라고 쓰면 안 된다. 평가 촬영 구간은 이전 개발에서도 살펴보았으므로 독립 blind test가 아니다.

## 3. 합성 가림과 증강의 정의

### 3.1 RGB-D 비교에 사용한 가림 조건

| 조건 이름 | 가림 형태 | 기준 중심선 길이에 대한 가림 비율 |
|---|---|---:|
| `clean` | 없음 | 0% |
| `thin` | 한 개의 얇은 띠 | 약 2% |
| `thick10` | 한 개의 띠 | 약 10% |
| `thick20` | 한 개의 띠 | 약 20% |
| `thick30` | 한 개의 띠 | 약 30% |
| `multi3_15` | 서로 떨어진 세 개의 띠 | 합계 약 15% |
| `multi3_30` | 서로 떨어진 세 개의 띠 | 합계 약 30% |

비율은 **이미지 면적이나 몸체 마스크 면적이 아니라 기준 skeleton의 영상상 길이 중 가려진 비율**이다. 목표 비율과 실제 생성 비율은 조금 다를 수 있으며 실제 비율도 저장했다.

- 단일 띠의 중심은 전체 경로의 약 37–63% 사이에서 선택한다.
- 세 띠는 대략 24%, 50%, 76% 부근에 위치를 변화시켜 배치한다.
- Base/tip 쪽 중심선 양 끝 7% 구간은 가리지 않도록 보호한다.
- 띠는 해당 위치의 몸체 방향을 가로지르도록 만든다. 따라서 예시에서 수평선처럼 보여도 모든 영상에서 고정된 수평선을 사용하는 것은 아니다.
- 검정·회색·흰색·유색의 불투명 띠와 texture를 사용한다.
- 라벨은 원래 몸체 마스크에서 실제 가린 부분을 제거한 **visible mask**다. U-Net이 가림을 만드는 것이 아니라, 합성 단계에서 가림을 만들고 U-Net이 남은 몸체를 예측한다.

### 3.2 학습용 외관·기하 증강과 RGB-D 실험의 분리

학습에는 배경 교체, 밝은 방해물, 위치 이동, 회전, 크기 변화, 평면 projective homography, 조명/색상 변화 및 단일·다중 가림을 포함했다.

그러나 **RGB의 위치·원근을 바꾼 후 원래 depth를 그대로 결합하지 않았다.** RGB-D 관절각/FK 실험은 원본 영상 좌표와 depth 정합을 유지하는 가림 조건만 사용했다.

후속 V3의 base-near/tip-near 25°/45°/60° 증강은 별도의 **segmentation-only 평가**다. 여기의 각도는 가상 평면의 투영 기울기이며 실제 HRM 관절각이나 새 3D 관측 시점이 아니다. V3 결과 폴더 안의 `paper_pose_summary`는 V2 RGB-D 결과를 재집계한 파일이므로, V3 원근 영상에서 얻은 pose 결과로 혼동하면 안 된다.

## 4. Visible mask에서 3D 관측점을 얻는 방법

### 4.1 여러 몸체 조각을 보존하는 skeleton 전처리

단일 최대 연결 성분만 선택하면 가림으로 분리된 몸체 일부가 버려진다. 실험용 전처리에서는 다음 방법을 사용했다.

1. 예측 binary mask에서 면적 16 pixel 이상인 여러 연결 성분을 유지한다.
2. 각 연결 성분을 Lee skeletonization한다.
3. 8-neighbour graph의 가중 경로 탐색으로 각 조각의 주요 skeleton 경로를 얻는다.
4. 영상의 아래쪽, 즉 image-v가 가장 큰 끝점을 base 쪽으로 정한다.
5. 현재 경로 끝점과 남은 조각의 끝점 중 가까운 것을 순차적으로 선택해 base→tip 순서를 구성한다.

이 단계의 '연결'은 **관측 픽셀의 배열 순서를 정한다는 뜻**이다. 가림 내부에 가짜 skeleton 픽셀이나 숨겨진 XYZ를 삽입하지 않는다. 원래 production에 있던 2D polynomial fitting/extrapolation은 이 실험용 전처리에서는 사용하지 않았다.

초기 구현은 길이 8 pixel 미만의 skeleton 경로를 버렸으나, 가림 후 남은 짧은 base 조각을 놓치는 문제가 있었다. 최종 결과인 `pose_evaluation_short_fragments`에서는 **`minimum_path_points=1`**로 바꾸어 해당 연결 성분의 짧은 경로도 보존했다. 면적 16 pixel 기준은 유지했다. 이는 초기 결과를 확인한 뒤 수정한 개발 단계 변경이며, 초기 결과를 지우지 않고 별도 폴더에 남겼다.

2026-10-29에 [저장 skeleton 비교 이미지](figures/segmentation_occlusion_20261028/B_visible_body_skeleton.png)를 추가했다. 예시 프레임 822에서 무가림은 146픽셀, Three gaps는 네 조각의 총 31픽셀이며 base→tip 조각별 점 수는 2/8/3/18이다. 색과 P1–P4는 보이는 경로 순서로, 실제 18개 관절 번호와 구분해야 한다. 짧은 마스크 조각의 skeleton이 몇 픽셀만 남더라도 길게 연장하거나 가림을 연결하는 픽셀을 추가하지 않았다. 이 프레임에서는 모든 skeleton 픽셀이 유효 depth 관측점이지만 다른 프레임에서는 depth 제외로 점 수가 줄 수 있다. 오검출/자기 겹침에서는 가까운 끝점 기준 정렬이 실패할 수 있다.

### 4.2 가림 위치의 depth는 사용하지 않음

예측 마스크를 \(\hat M\), 알려진 합성 가림 영역을 \(O\)라 하면 depth 조회에 사용할 영역은

\[
M_D=\hat M\cap\neg O
\]

이다. Skeleton 조회 픽셀 자체가 이 영역에 있어야 하고, 5 × 5 주변에서 **같은 영역 안의 양수·유효 depth**만 모아 median을 사용한다. 최소 3개 유효 이웃이 필요하며, 마스크 밖이나 가려진 픽셀의 depth로 대체하지 않는다.

**합성 가림 지도를 예측 마스크 자체에 적용해 segmentation 결과를 정답처럼 고치지는 않았다.** 알려진 가림 정보는 depth의 관측 불가 영역을 정의하는 데만 사용했다. 따라서 이것은 `missing-depth simulation`이며, 실제 가림 물체 표면에서 읽히는 정상적인 depth를 자동으로 식별·제거한 성능 검증은 아니다.

최소 5개 유효 3D 점을 요구하며, 끝점에서 depth가 없는 후보를 너무 많이 건너뛰거나, 인접 관측점 거리가 비정상적으로 크거나, 첫 관측점이 고정 base에서 지나치게 멀면 실패로 기록한다. 구체적으로 끝점 skip 최대 5개, 인접 점 거리 최대 전체 하드웨어 길이의 1.5배, 첫 관측점–고정 base 거리 최대 전체 길이의 10%를 사용했다.

### 4.3 역투영과 base 좌표계

Crop 기준 내부 파라미터 \(K_{\mathrm{crop}}\)와 depth \(Z\)를 이용해

\[
{}^C\mathbf p=ZK_{\mathrm{crop}}^{-1}[u,v,1]^\top
\]

로 카메라 좌표계의 점을 계산한다. 이후

\[
{}^B\mathbf p=({}^CR_B)^\top({}^C\mathbf p-{}^C\mathbf o_B),
\qquad {}^CR_B=R_y(-90^\circ)R_z(-45^\circ)
\]

로 `hrm_base`에 표현한다. 단위는 m이다.

Paired 평가에서는 **각 방법·각 원본 프레임의 clean 처리에서 얻은 카메라상 base 원점**을 그 프레임의 모든 가림 조건에 동일하게 제공했다. 가림 때문에 남은 첫 조각을 새 base로 재설정하지 않는다. 이는 base 위치까지 독립적으로 추정하는 완전한 가림 인식 시험과 구분해야 한다.

## 5. 3D 곡선, 경계점, 18개 관절각 및 FK

### 5.1 공유한 production 계산부

실험 당시 `postprocess.py`에서 다음 함수를 AST로 추출해 재사용했다.

```text
fit_quartic_through_origin
wrap_angle
update_joint_kalman              # 이번 실험에서는 필터 OFF
project_segment_directions_to_joint_planes
reconstruct_3d_segments
```

관절 Kalman과 초기 base 평균은 끄고 각 프레임을 독립 처리했다. 따라서 이번 변화량은 시간 필터가 가림 효과를 완화한 결과가 아니다.

### 5.2 매개변수와 원점제약 4차 곡선

순서가 정해진 관측점 \(\mathbf p_k\)에 대해

\[
s_k=\frac{\sum_{j=1}^{k}\|\mathbf p_j-\mathbf p_{j-1}\|}
{\sum_{j=1}^{m-1}\|\mathbf p_j-\mathbf p_{j-1}\|},\qquad s\in[0,1]
\]

을 만든다. 여기의 \(s\)는 실제 가려진 몸체 길이를 아는 매개변수가 아니라 **관측점 사이의 누적 chord 길이로 만든 매개변수**다. 특히 서로 다른 visible 조각 사이에서는 두 관측 끝점의 직선거리로 간격을 근사한다.

하드웨어 전체 길이 \(L\)로 좌표를 나눈 뒤 각 좌표에 대해 최소제곱 피팅한다.

\[
\bar{\mathbf r}(s)=\mathbf a_1s+\mathbf a_2s^2+\mathbf a_3s^3+\mathbf a_4s^4,
\qquad \bar{\mathbf r}(0)=\mathbf0.
\]

상수항은 없으며 production 계산부는 입력의 첫 점도 정확한 원점으로 고정한다. 피팅 곡선을 1,001개 지점에서 조밀하게 평가하여 길이를 구하고, base를 중심으로 등방 스케일링하여 최종 전체 arc length를 알려진 하드웨어 길이에 맞춘다.

따라서 최종 곡선은 **가시 depth뿐 아니라 고정 길이와 매끄러운 4차 곡선이라는 사전 조건을 사용한 추정**이다. 숨은 부분의 실제 형상을 직접 측정한 것이 아니다.

### 5.3 경계점과 중심점, 실제 각도 입력의 구분

실험 설정은 고정 proximal link를 포함한 **19개 기하학적 segment, 18개 굽힘 관절**이다. 각 길이 4.33 mm, 전체 길이 82.27 mm를 사용했다.

- 곡선 arc length를 기준으로 경계점 \(P_0,\ldots,P_{19}\), 총 20개를 얻는다.
- 각 구간의 arc-length 중심점은 19개다.
- 경계점과 중심점에서 미분한 단위 접선도 계산한다.
- **그러나 이번 관절각 계산에 입력한 것은 접선 미분값이나 중심점–중심점 벡터가 아니라, 인접 경계점 사이의 chord 방향이다.**

\[
\mathbf d_j=\frac{P_{j+1}-P_j}{\|P_{j+1}-P_j\|},\qquad j=0,\ldots,18.
\]

첫 방향 \(\mathbf d_0\)는 고정 base link에 해당하므로 관절각 계산에서 제외한다. 나머지 \(\mathbf d_1,\ldots,\mathbf d_{18}\)으로 18개의 상대 관절각을 구한다. 따라서 '18개 관절각'과 '19개 물리적 길이 구간'을 같은 개수로 표현하면 안 된다.

### 5.4 실제 관절 회전평면으로 투영하고 상대각 계산

각 관절은 로컬 Z축 회전이며, 관절 사이 X축 twist를 번갈아 적용한다. 첫 관절은 tilt, 다음은 pan이다.

\[
\alpha_{i-1}=[0,+90^\circ,-90^\circ,+90^\circ,\ldots]_i,
\qquad i=1,\ldots,18.
\]

이전까지 누적한 회전을 \(R_{i-1}\)라 하면

\[
Q_i=R_{i-1}R_x(\alpha_{i-1}),\quad
\mathbf a_i=Q_i\mathbf e_z,\quad \mathbf b_i=Q_i\mathbf e_x.
\]

관측 방향에서 관절축 성분을 제거하여 회전평면에 투영한다.

\[
\hat{\mathbf d}_i=
\frac{\mathbf d_i-(\mathbf a_i^\top\mathbf d_i)\mathbf a_i}
{\|\mathbf d_i-(\mathbf a_i^\top\mathbf d_i)\mathbf a_i\|}.
\]

부호 있는 상대각과 다음 좌표계는

\[
q_i=\operatorname{atan2}\left(
\mathbf a_i^\top(\mathbf b_i\times\hat{\mathbf d}_i),
\mathbf b_i^\top\hat{\mathbf d}_i\right),\qquad
R_i=Q_iR_z(q_i)
\]

로 계산한다. 이는 전역 기울기를 단순히 서로 빼는 방식이 아니다. 앞 관절의 회전을 반영한 **누적 관절 좌표계 기준 상대각**이다.

홀수 \(q_1,q_3,\ldots,q_{17}\)은 tilt, 짝수 \(q_2,q_4,\ldots,q_{18}\)은 pan이다. 첫 관절의 양의 회전은 base +Z축 기준으로 +X 방향을 +Y 방향으로 회전시킨다. 저장 각도는 rad이고 아래 결과 표는 degree로 변환했다.

### 5.5 FK tip pose

복원한 \(q_1,\ldots,q_{18}\)에 modified-DH chain을 적용한다.

\[
{}^{i-1}T_i=R_x(\alpha_{i-1})T_x(l_{i-1})R_z(q_i),
\qquad
{}^BT_{\mathrm{tip}}=
\left(\prod_{i=1}^{18}{}^{i-1}T_i\right)T_x(l_e).
\]

첫 이동은 proximal offset, 중간 이동은 관절 간 간격, 마지막 이동은 distal offset이다. Python 구현은 `surgical_tool.cpp`의 실제 계산 함수와 수치 비교하는 테스트로 확인했다.

**FK tip은 피팅 곡선의 마지막 점과 별개다.** `curve_tip`은 마지막 경계점 \(P_{19}\), `fk_tip`은 관절각과 하드웨어 기구학으로 계산한 pose다. 아래 주 결과는 `fk_tip` 비교이며, curve tip 차이도 원본 결과 CSV에 별도로 남아 있다.

## 6. 변화량 지표의 정의

### 6.1 관절각

프레임 \(n\), 관절 \(i\)에 대해 clean/occluded 상대각의 차이를 wrap한다.

\[
\Delta q_{n,i}=\operatorname{atan2}\bigl(
\sin(q^{\mathrm{occ}}_{n,i}-q^{\mathrm{clean}}_{n,i}),
\cos(q^{\mathrm{occ}}_{n,i}-q^{\mathrm{clean}}_{n,i})\bigr).
\]

아래 식은 degree로 변환한 \(\Delta q\)에 적용한다.

\[
E_{q,n}=\sqrt{\frac1{18}\sum_{i=1}^{18}\Delta q_{n,i}^2},
\qquad \overline E_q=\frac1N\sum_{n=1}^N E_{q,n}.
\]

조건별 표의 'q18 RMSE 평균'은 **프레임별 18관절 RMSE를 구한 뒤 프레임 평균한 값**이다. 아래 pooled RMSE와는 다르다.

\[
E_{q,\mathrm{pooled}}=\sqrt{\frac1{18N}\sum_{n=1}^N\sum_{i=1}^{18}\Delta q_{n,i}^2}.
\]

개별 관절 표에는 다음 값을 사용한다.

\[
\mathrm{RMSE}_i=\sqrt{\frac1N\sum_n\Delta q_{n,i}^2},
\qquad \mathrm{MAE}_i=\frac1N\sum_n|\Delta q_{n,i}|.
\]

이는 18개 실제 관절축의 상대각 비교이지, pan 18개와 tilt 18개를 모두 합친 36차원 비교가 아니다.

### 6.2 FK tip 위치와 자세

\[
E_{p,n}=1000\|\mathbf p_n^{\mathrm{occ}}-\mathbf p_n^{\mathrm{clean}}\|_2
\quad[\mathrm{mm}],
\]

\[
E_{R,n}=\frac{180}{\pi}\cos^{-1}\left[
\operatorname{clip}\left(
\frac{\operatorname{tr}((R_n^{\mathrm{clean}})^\top R_n^{\mathrm{occ}})-1}{2},-1,1
\right)\right]\quad[\mathrm{deg}].
\]

위치 표의 평균은 **3D 거리 차이의 평균**이며, XYZ 좌표별 RMSE나 전체 위치 RMSE라고 바꿔 부르면 안 된다. 자세는 yaw/pitch/roll 차이가 아니라 SO(3) 회전 차이다. p95는 표본 95백분위이며 95% 신뢰구간이 아니다.

### 6.3 실패 및 분모

출력 실패도 요청 행에 남기며, 변화량은 clean과 occluded가 모두 유효한 paired subset에서 계산한다. 반드시 `유효 paired 수 / 요청 수`를 함께 보고한다. 최종 U-Net V2의 728/728 출력은 **계산 가능 비율 100%**이지 정확도 100%가 아니다. clean 자기 비교의 0들은 가림 전체 통계에서 제외한다.

## 7. U-Net V2의 관절각·FK 변화 결과

원본: `runs/perspective_pose_followup_20261024/paper_pose_summary.csv`. 이 파일은 최종 `pose_evaluation_short_fragments` 결과를 재집계한다. 단위는 위치 mm, 각도 degree이며 표는 반올림했다.

### 7.1 신규 직선/검정 배경: 각 조건 40프레임

| 조건 | 유효/요청 | FK 위치 평균 | 위치 p95 | 위치 최대 | 위치 ≤1 mm | q18 RMSE 평균 | FK 회전 평균 |
|---|---:|---:|---:|---:|---:|---:|---:|
| 한 띠 약 2% | 40/40 | 0.434 | 1.058 | 1.108 | 87.50% | 1.556 | 5.402 |
| 한 띠 10% | 40/40 | 0.169 | 0.344 | 0.364 | 100.00% | 0.767 | 1.983 |
| 한 띠 20% | 40/40 | 0.219 | 0.416 | 0.977 | 100.00% | 0.948 | 2.400 |
| 한 띠 30% | 40/40 | 0.242 | 0.584 | 0.758 | 100.00% | 0.997 | 2.257 |
| 세 띠 합계 15% | 40/40 | 0.359 | 0.717 | 0.739 | 100.00% | 1.118 | 2.663 |
| 세 띠 합계 30% | 40/40 | 0.413 | 0.804 | 2.280 | 95.00% | 1.265 | 3.305 |

### 7.2 저장된 굽힘/동적 영상: 각 조건 64프레임

| 조건 | 유효/요청 | FK 위치 평균 | 위치 p95 | 위치 최대 | 위치 ≤1 mm | q18 RMSE 평균 | FK 회전 평균 |
|---|---:|---:|---:|---:|---:|---:|---:|
| 한 띠 약 2% | 64/64 | 0.375 | 1.027 | 3.716 | 93.75% | 1.080 | 3.204 |
| 한 띠 10% | 64/64 | 0.165 | 0.355 | 0.426 | 100.00% | 0.870 | 2.403 |
| 한 띠 20% | 64/64 | 0.273 | 0.795 | 1.297 | 96.88% | 1.203 | 3.003 |
| 한 띠 30% | 64/64 | 0.353 | 0.822 | 1.346 | 96.88% | 1.504 | 3.255 |
| 세 띠 합계 15% | 64/64 | 0.445 | 0.800 | 1.854 | 98.44% | 1.270 | 3.484 |
| 세 띠 합계 30% | 64/64 | 0.554 | 1.390 | 2.332 | 90.63% | 1.477 | 3.603 |

### 7.3 여섯 가림 조건 전체 집계: clean 제외

| 지표 | 직선 240쌍 | 굽힘 384쌍 |
|---|---:|---:|
| 유효 paired 수 | 240/240 | 384/384 |
| FK 위치 차이 평균 [mm] | 0.306 | 0.361 |
| FK 위치 차이 p95 [mm] | 0.800 | 0.934 |
| FK 위치 차이 최대 [mm] | 2.280 | 3.716 |
| FK 위치 차이 ≤1 mm | 233/240 = 97.08% | 369/384 = 96.09% |
| 프레임별 q18 RMSE 평균 [deg] | 1.108 | 1.234 |
| 모든 프레임·관절 pooled RMSE [deg] | 1.289 | 1.401 |
| FK 회전 차이 평균 [deg] | 3.002 | 3.158 |

**해석:** 평균 tip 위치 변화는 sub-millimetre였지만 모든 결과가 1 mm 이내인 것은 아니다. 얇은 가림이 일부 두꺼운 가림보다 큰 변화를 보였으므로, 가림 비율이 증가할수록 오차가 단조 증가한다거나 모든 가림 형태에서 동일하게 강건하다고 결론 내리지 않는다.

## 8. 실제 18개 관절 각각의 변화량

대표적인 다중 분리 조건인 **`multi3_30`(세 띠 합계 30%)**의 관절별 통계다. 직선은 관절마다 N=40, 굽힘은 관절마다 N=64이다. 각 관절의 clean 상대각과 occluded 상대각을 짝지어 계산했다. 모든 값은 degree다.

| 관절 | 실제 축 | 직선 RMSE | 직선 MAE | 굽힘 RMSE | 굽힘 MAE |
|---|---|---:|---:|---:|---:|
| q1 | tilt | 1.794 | 1.405 | 1.582 | 1.274 |
| q2 | pan | 0.929 | 0.377 | 0.582 | 0.418 |
| q3 | tilt | 2.757 | 2.267 | 3.032 | 2.532 |
| q4 | pan | 1.853 | 0.807 | 1.274 | 0.734 |
| q5 | tilt | 0.944 | 0.766 | 0.836 | 0.637 |
| q6 | pan | 0.570 | 0.259 | 0.550 | 0.375 |
| q7 | tilt | 0.613 | 0.532 | 1.076 | 0.913 |
| q8 | pan | 0.876 | 0.418 | 0.651 | 0.447 |
| q9 | tilt | 0.941 | 0.744 | 1.524 | 1.288 |
| q10 | pan | 0.922 | 0.445 | 0.810 | 0.467 |
| q11 | tilt | 1.039 | 0.846 | 1.470 | 1.150 |
| q12 | pan | 0.746 | 0.335 | 0.909 | 0.517 |
| q13 | tilt | 0.949 | 0.803 | 1.073 | 0.806 |
| q14 | pan | 0.500 | 0.297 | 0.904 | 0.539 |
| q15 | tilt | 1.079 | 0.897 | 1.449 | 1.173 |
| q16 | pan | 0.769 | 0.630 | 1.023 | 0.719 |
| q17 | tilt | 2.517 | 1.985 | 3.903 | 3.233 |
| q18 | pan | 3.075 | 1.974 | 2.002 | 1.500 |

전체 관절별 원자료는 `pose_evaluation_short_fragments/per_joint_delta.csv`에 있으며 `clean_q_rad`, `occluded_q_rad`, `wrapped_delta_deg`를 포함한다. 다른 조건이나 전체 여섯 가림 조건으로도 관절별 통계를 다시 계산할 수 있다.

Tip 위치 변화가 작아도 q3/q17/q18 등 특정 관절의 변화가 작다고 보장되지는 않는다. **Tip 위치, tip 회전, 18개 관절각을 함께 제시해야 몸체 형상 변화에 대한 설명이 가능하다.**

## 9. Segmentation 자체의 마스크 성능

아래는 별도의 128프레임 마스크 평가에서 visible pseudo-mask와 비교한 **이미지별 foreground Dice 평균(%)**이다. 기준은 수동 정답이 아니다.

\[
\mathrm{Dice}(\hat M,M_{\mathrm{vis}})=
\frac{2|\hat M\cap M_{\mathrm{vis}}|}{|\hat M|+|M_{\mathrm{vis}}|}.
\]

| 조건 | OpenCV | U-Net V1 | U-Net V2 |
|---|---:|---:|---:|
| clean | 100.000 | 98.101 | 98.092 |
| 한 띠 약 2% | 78.504 | 97.550 | 97.710 |
| 한 띠 10% | 67.901 | 97.570 | 97.728 |
| 한 띠 20% | 62.015 | 97.707 | 97.649 |
| 한 띠 30% | 58.884 | 97.638 | 97.534 |
| 세 띠 합계 15% | 42.451 | 95.597 | 96.779 |
| 세 띠 합계 30% | 31.686 | 95.615 | 96.500 |
| 배경 + 원근 + 세 띠 | 20.327 | 84.486 | 96.030 |

OpenCV clean의 100%는 기준 pseudo-label을 생성한 처리를 재현한 결과다. V2가 V1보다 모든 조건에서 높지는 않다. 복합 배경+원근 결과는 **마스크 성능에만 해당하며 위 pose 표와 결합하지 않는다.** MobileSAM은 이번 V2 angle/FK 핵심 비교에 사용하지 않았다.

## 10. 논문에서 명시할 가정과 제한

1. **Base와 tip이 보이는 중간 가림 조건**이다. 시야 밖으로 나가거나 끝점이 가려지는 상황은 검증하지 않았다.
2. 가림 물체는 합성했다. 알려진 가림 영역의 depth를 제외했으므로, 실제 물체의 유효 depth가 섞이는 문제를 해결했다고 주장할 수 없다.
3. 비교 기준은 무가림 모델 추정값이다. AprilTag/수동 3D 형상/실제 관절각을 이용한 absolute accuracy 검증과 구분한다.
4. Clean base 원점을 고정해 사용했다. Base 검출 실패까지 포함한 완전한 실시간 가림 처리 성능은 아니다.
5. 여러 visible 조각을 정렬하는 규칙은 몸체가 크게 자기 중첩하지 않고 base가 영상 아래쪽에 있다는 가정에 의존한다.
6. 가림 내부의 실제 arc length나 급격한 굽힘을 직접 관측하지 못한다. 4차 곡선과 길이 제약은 보간에 도움이 되지만 실제 숨은 형상과의 일치를 보장하지 않는다.
7. 학습 마스크는 OpenCV pseudo-label이다. 독립 수동 segmentation GT 검증은 별도로 필요하다.
8. 최종 short-fragment 처리는 초기 실패를 본 뒤 개선했다. 초기 `minimum_path_points=8`과 최종 1을 섞어서 하나의 고정된 사전 검증 실험처럼 쓰지 않는다.
9. 같은 원본 프레임을 여러 조건에 재사용했고 영상 내 프레임은 시간적으로도 연관되어 있다. 독립 표본 개수를 과장하거나 이 자료만으로 일반적인 임상 성능을 주장하지 않는다.
10. 추정된 외력/접촉 위치나 폐루프 제어까지 가림 상태에서 검증한 실험은 아니다. 이번 지표는 segmentation과 기하학적 상태 추정 단계에 한정한다.
11. Skeleton 픽셀의 depth는 카메라에서 보이는 몸체 표면의 깊이다. 원통 반지름을 이용해 실제 공간 중심축으로 이동시키는 보정을 수행한 것은 아니므로, 이를 독립적인 실제 중심축 ground truth로 취급하지 않는다.

참고로 최종 minpath=1의 세 띠 30% 조건에서 U-Net V2의 유효 paired 수는 직선 40/40, 굽힘 64/64였다. 초기 minpath=8에서는 각각 15/40, 31/64였다. 이 차이는 네트워크 가중치를 새로 바꾼 효과가 아니라 **짧은 visible 조각을 보존하도록 후단 전처리를 수정한 효과**다.

## 11. 논문 Results에 사용할 수 있는 서술 예시

> 부분 가림이 영상 기반 형상 추정에 미치는 영향을 평가하기 위해, 몸체의 양 끝이 관측되는 상태에서 중간 영역에 단일 또는 다중 띠 형태의 합성 가림을 적용하였다. Visible-body segmentation 모델로 분리된 몸체 영역을 추출하고, 가림 영역을 제외한 관측 depth를 이용하여 3차원 곡선과 관절각을 계산하였다. 동일한 원본 프레임의 무가림 추정값을 기준으로 관절각 및 순기구학 기반 끝단 pose의 변화를 비교하였다. 세 군데에 합계 30%의 중심선 가림을 적용한 조건에서 평균 끝단 위치 차이는 직선 영상과 굽힘 영상에서 각각 0.413 및 0.554 mm였으며, 1 mm 이내의 위치 차이를 보인 비율은 각각 95.0%와 90.6%였다. 프레임별 18관절 RMSE의 평균은 각각 1.265°와 1.477°, 평균 끝단 회전 차이는 각각 3.305°와 3.603°였다. 이 결과는 관측 가능한 몸체 조각을 유지한 상태에서 매끄러운 곡선 모델을 이용한 재구성이 중간 관측 누락에 대한 형상 추정의 변화를 제한할 수 있음을 보여준다. 다만 이는 무가림 추정값에 대한 상대적인 민감도 평가이며, 가려진 형상의 절대 정확도 또는 실제 가림 물체의 depth 제거 성능을 검증한 결과는 아니다.

위 문단을 줄이더라도 **비교 기준이 무가림 추정값이라는 점**과 **합성 가림/depth 제외 조건**은 유지한다. '1 mm 이하 정확도', '가려진 몸체를 정확히 복원', '모든 조건에서 1 mm 이내'로 바꾸지 않는다.

## 12. 권장 그림 구성과 이미 저장된 자료

- A: 같은 프레임의 clean RGB / 단일 띠 / 세 띠 RGB, visible prediction, skeleton/관측점.
- B: Clean과 occluded 3D curve 및 FK chain을 같은 base에서 중첩. 관측된 점과 추정된 곡선을 구별한다.
- C: 조건별 FK 위치 차이 분포와 1 mm 기준선. 평균만 표시하지 않고 p95/최대 또는 boxplot도 표시한다.
- D: 조건 × q1–q18의 MAE/RMSE heatmap. 직선/굽힘을 분리하고 tilt/pan 순서를 명시한다.
- E: FK 회전 차이 분포. Tip 위치가 비슷해도 orientation/관절각 차이는 남는다는 점을 보여준다.

기존 그림은 segmentation 프로젝트의 다음 위치에 있다.

```text
runs/occlusion_pose_20261024/plots/joint_deltas.png
runs/occlusion_pose_20261024/plots/pose_live_black.png
runs/occlusion_pose_20261024/plots/pose_archive_45deg_1.png
runs/occlusion_pose_20261024/plots/v2_shape_examples.png
runs/occlusion_pose_20261024/pose_evaluation_short_fragments/examples/
```

`joint_deltas.png`는 조건 × 18관절의 **평균 절대 차이(MAE)** heatmap이므로 RMSE 그림이라고 표기하지 않는다. 대표 이미지는 임의의 가장 좋은 사례만 고르지 말고 동일 원본 프레임으로 조건을 비교하며, 논문에 사용할 때 원본 ID와 조건을 함께 보존한다.

## 13. 다른 AI/연구자를 위한 근거 파일 지도

아래 상대경로의 기준은 모두 `/home/daeyun/Hyper-Redundant-Manipulator-Segmentation`이다.

| 파일 | 확인할 내용 |
|---|---|
| `README.md` | V1/V2/V3 실험 구분, 전체 요약 및 제한 |
| `runs/occlusion_pose_20261024/unet_v2/config.json` | 실제 학습 설정 |
| `runs/occlusion_pose_20261024/unet_v2/split.json` | 학습/검증 원본 목록 |
| `runs/occlusion_pose_20261024/unet_v2/best.pt` | RGB-D 평가에 사용한 V2 체크포인트 |
| `runs/occlusion_pose_20261024/pose_evaluation_short_fragments/protocol.json` | 촬영 출처, timestamps/K, 가림 조건, minpath=1, 모델/코드 정보 |
| `runs/occlusion_pose_20261024/pose_evaluation_short_fragments/per_frame.csv` | 방법·조건·프레임별 성공/실패와 변화량 |
| `runs/occlusion_pose_20261024/pose_evaluation_short_fragments/per_joint_delta.csv` | 각 관절의 clean/occluded 상대각과 signed wrapped 차이 |
| `runs/occlusion_pose_20261024/pose_evaluation_short_fragments/shape_arrays/` | 관측점·곡선·경계·중심·q18·FK pose의 NPZ 배열 |
| `runs/perspective_pose_followup_20261024/paper_pose_summary.csv` | 이 문서의 조건별 표에 사용한 평균/p95/최대/≤1mm 집계 |
| `runs/perspective_pose_followup_20261024/paper_pose_frames.csv` | 논문용 V2 paired frame 결과 |
| `runs/perspective_pose_followup_20261024/paper_pose_source_index.json` | RGB/depth/metadata/shape 배열의 원본 연결 정보 |
| `runs/occlusion_pose_20261024/segmentation_evaluation/summary.csv` | 별도 128프레임 마스크 평가의 Dice/IoU 등 |
| `feasibility_augmentation_v2.py` | 실제 가림 생성, 비율, 끝점 보호 및 augmentation |
| `run_occlusion_v2.py`, `hrm.py` | 학습·네트워크·손실 및 segmentation 평가 |
| `feasibility_shape.py` | 실험용 multicomponent front-end, production core 추출, FK |
| `feasibility_depth_preview.py` | mask-aware depth median 및 역투영 |
| `evaluate_pose_occlusion_v2.py` | 같은 방법의 clean 기준, paired 평가, 저장 지표 |
| `summarize_pose_for_paper.py` | V2 pose 결과의 논문용 재집계 |
| `report_occlusion_v2.py` | 각도 heatmap/pose 그림 생성과 캡션 의미 |

재검증 기록: 2026-10-27에 저장된 U-Net V2의 728개 유효 shape NPZ를 읽어 관절각 RMSE, FK 위치 차이, curve tip 차이 및 SO(3) 회전 차이를 다시 계산했고, 저장된 per-frame CSV와의 최대 차이는 0이었다. 위 관절별 표 역시 저장 배열/관절 CSV를 재집계한 값이다. 새 학습이나 카메라·로봇 구동은 수행하지 않았다.

재현성 주의: 실험 당시 production `postprocess.py` 전체 SHA256은 `protocol.json`에 `93a4432bb6e2dc505b0a3e5362c851c91501f0948a6e6a691e4e213d4d5c33d9`로 기록되어 있다. 현재 파일은 이후 변경되어 전체 파일 해시가 다르므로, 오늘의 코드 전체가 당시와 byte-identical이라고 기술하지 않는다. 이 문서는 **저장된 실험 프로토콜과 배열/CSV 결과**를 기준으로 정리했다. 재실험 시에는 production 계산부까지 포함한 코드 스냅샷과 새 출력 폴더를 함께 보존하는 것이 필요하다.

## 14. Results Figure A–D 자료 묶음 (2026-10-28 추가)

사용자가 제시한 2×2 배치에 맞춰 실제 저장된 예시와 원자료를 정리했다.

- [패널별 이미지·데이터 안내 및 캡션](figures/segmentation_occlusion_20261028/README_KO.md)
- [전체 A–D 미리보기](figures/segmentation_occlusion_20261028/figure_segmentation_ABCD.png)
- [편집용 SVG](figures/segmentation_occlusion_20261028/figure_segmentation_ABCD.svg)
- [삽입용 PDF](figures/segmentation_occlusion_20261028/figure_segmentation_ABCD.pdf)

A–C는 같은 굽힘 프레임을 사용하며, D는 U-Net V2 / multi3_30의 전체 104프레임이다. 본문 C의 길이 정규화된 곡선·FK와 실제 depth 관측점을 혼동하지 않도록, 원래 depth 점 + 정규화 전 곡선은 별도 보충 그림으로 제공했다. 18관절별 RMSE/MAE 보충 그래프와 전체 프레임·관절별 CSV, 원본 RGB/짝 depth/저장된 예측 mask/NPZ도 포함했다. 새로운 학습이나 추론을 수행한 것이 아니다.

최신 B는 **무가림(Unoccluded) 마스크 / 세 군데 가림(Three gaps) 마스크**를 나란히 표시한다. RGB는 A에서 비교한다. 이 변경은 PNG와 MATLAB 묶음에 반영했으며, PDF/SVG는 더 필요 없다는 요청에 따라 이전 버전을 그대로 남겼다.

별도 **B_skeleton** 그림은 동일 프레임의 mask/RGB 위에 분리된 skeleton을 표시한다. 세 조건의 원래 해상도 skeleton PNG와 245개 픽셀 CSV/Excel, 색/경로 순서/검증 manifest도 포함했다. 기존 A–D의 수치나 곡선을 변경하지 않았다.

### MATLAB에서 다시 그리기

[MATLAB 코드·Excel·이미지 독립 묶음](figures/segmentation_occlusion_matlab_20261028/README_KO.md)을 별도로 추가했다. 폴더 전체를 MATLAB PC로 복사하고 단일 파일 `make_segmentation_figures.m`을 실행하면 개별 패널·전체 배치·18관절별 RMSE/MAE를 그린다. C·D는 원자료로 그린 편집 가능한 MATLAB plot이며, PNG/FIG만 저장한다. `make_segmentation_figures('C', [], false)`처럼 세 번째 인자를 false로 지정하면 파일 저장 없이 plot 창만 열린다. `data/segmentation_figure_data.xlsx`에 104프레임과 1,872개 관절차, C의 3D 좌표 등을 포함한다. 실행 시 다른 저장소나 모델은 필요 없다. 제작 환경에는 MATLAB이 없어 수치 대조와 정적 검사까지만 수행했으며, 포함된 미리보기는 기존 Python 그림이다.

Skeleton 비교만 열려면 `make_segmentation_figures('B_skeleton', [], false)`를 사용한다. 이 보충 그림을 포함해 기본 실행은 총 10개 figure를 만든다.
