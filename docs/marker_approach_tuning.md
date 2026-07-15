# marker_approach 튜닝 가이드

`marker_approach_node` 의 접근 동작을 현장에서 조정하기 위한 파라미터 설명과 **증상별 처방**.
모든 값은 `config/marker_approach.yaml` 에서 수정한다(런치가 덮어쓰지 않음).

---

## 1. 동작 개요

```
IDLE ─start▶ CENTER ─|e_center|<center_first_tol▶ YAW ─|e_axis|<yaw_tol▶ FINE ─|e_center|<lat_tol▶ APPROACH ─z=final▶ ARRIVED
                                                                                        │
                                                     마커 소실 시 정지(근접이면 OPENLOOP) ◀┘
```

정렬은 **축을 순차 분리**한다(각 단계 전진 없음, 해당 축만):
- **CENTER**: `vy`만으로 마커를 화면 중앙에 **거칠게**(`center_first_tolerance`) 맞춤. yaw 아직 안 씀.
- **YAW**: `wz`만으로 마커 정면(z축) 정렬(`yaw_tolerance_deg`). 회전하면 중앙이 조금 틀어짐 → 다음 단계에서 보정.
- **FINE**: `vy`만으로 중앙을 **미세**(`lateral_tolerance`) 재정렬(yaw로 틀어진 중앙 복구).
- **APPROACH**: 전진 `vx` + 정렬 유지 보정(`vy`,`wz` 데드밴드).
- **APPROACH**: 정면을 유지한 채 `vx`로 전진, **카메라 z거리(`tvec[2]`)가 `final_distance`** 가 되면 정지.
- **OPENLOOP**: 마커가 너무 가까워 화면을 벗어나면(=근접 소실), 마지막 남은 거리를 무피드백 저속 직진으로 마무리(odom 이동거리 or 시간 기반).

제어 = **P제어 → 속도 포화(`max_linear_speed`) → 최소속도 floor(`min_linear_speed`) → 가속 제한(slew)**.
제어 오차 3개: `z_cam`(전진거리), `e_center`(좌우 중앙정렬), `e_axis`(정면 z축 정렬).

---

## 2. 파라미터 전체 표

| 파라미터 | 기본 | 역할 |
|---|---|---|
| `final_distance` | 0.25 m | **최종 정지 거리**(카메라 z). "마커 앞 몇 m에서 설까". |
| `distance_tolerance` | 0.02 m | 도착 판정 밴드. `|z-final|`이 이내면 ARRIVED. |
| `kp_forward` | 0.8 | 전진 P게인. `vx = kp_forward·(z-final)`. |
| `kp_lateral` | 1.0 | 중앙정렬 P게인. `vy = kp_lateral·e_center`. |
| `kp_yaw` | 1.2 | z축 정렬 P게인. `wz = kp_yaw·e_axis`. |
| `lateral_tolerance` | 0.03 m | 중앙정렬 만족 밴드(ALIGN 완료·APPROACH 유지). |
| `yaw_tolerance_deg` | 5.0° | z축 정렬 만족 밴드. |
| `max_linear_speed` | 0.15 m/s | 선속도(vx,vy 합성) 상한. |
| `max_yaw_rate` | 0.6 rad/s | 각속도 상한. |
| `max_linear_accel` | 0.5 m/s² | 선속도 slew(가속) 제한 → 부드러움. |
| `max_yaw_accel` | 1.0 rad/s² | 각속도 slew 제한. |
| `min_vx` / `min_vy` / `min_yaw_rate` | 0.05 / 0.1 / 0.02 | **하드웨어 축별 최소속도(데드존 하한)**. 0 아닌 명령이 이보다 작으면 로봇이 무시하므로 이 값까지 끌어올림. 정렬완료로 0 인 축은 그대로 0(정지 수렴). |
| `center_first_tolerance` | 0.05 m | CENTER 단계 종료 기준: `|e_center|`가 이 이내면 ALIGN(yaw 투입)으로 전환. |
| `obs_filter_alpha` | 0.5 | 관측 EMA 계수(0~1). ↑ 반응빠름/노이즈↑, ↓ 부드럽/지연↑. |
| `max_pose_age_sec` | 0.5 s | 관측이 이보다 오래되면 "마커 소실"로 정지. |
| `openloop_final_approach` | true | 근접 소실 시 무피드백 직진 마무리 on/off. |
| `openloop_enter_distance` | 0.35 m | 이 거리 이내에서 소실해야 OPENLOOP 진입. |
| `openloop_speed` | 0.1 m/s | OPENLOOP 직진 속도. |
| `openloop_max_distance` | 0.3 m | OPENLOOP 최대 직진 거리(과주행 안전상한). |
| `control_rate_hz` | 20 | (참고) 제어는 이미지 콜백에서 수행 → 실제 주기는 카메라 FPS. |

---

## 3. 증상별 처방

### 🔴 "목표보다 너무 가까이 붙는다" (지금 이슈)

원인은 보통 둘 중 하나:

**(A) 정지 목표가 그냥 가깝다** — 10cm는 꽤 바짝임.
- `final_distance` **↑** (예: `0.15` ~ `0.20`). 제일 직접적.

**(B) 근접에서 마커가 화면을 벗어나 OPENLOOP가 더 밀고 들어간다** — 마커(0.185m)는 10~20cm에서 화면을 넘겨 검출이 끊기고, 그때 열린루프가 남은 거리를 무피드백으로 직진하다 과주행.
- 가장 확실: `final_distance` **↑** 로 올려 **마커가 계속 보이는 거리에서 멈추게** 한다(열린루프 자체가 안 걸림).
- OPENLOOP를 보수적으로: `openloop_max_distance` **↓**(예: `0.15`), `openloop_speed` **↓**(예: `0.05`).
- 아예 끄기: `openloop_final_approach: false` → 마커 놓치면 그냥 정지(그 지점에서 멈춤, 과주행 없음).

> 판별법: 로그에서 `열린루프` WARN 이 뜨고 그 뒤 더 전진하면 (B). 안 뜨는데 가까우면 (A).

### 🔴 목표를 지나쳐 오버슛(가까이 갔다가 되돌아옴/더 붙음)
- `max_linear_speed` **↓**(예: 0.2), `max_linear_accel` **↓**(예: 0.3) — 감속 여유 확보.
- `kp_forward` **↓**(예: 0.5) — 접근을 완만하게.
- `distance_tolerance` **↑**(예: 0.03) — 도착을 조금 일찍 인정(과도한 미세 접근 방지).
- `min_linear_speed` **↓**(예: 0.01) — floor가 목표 근처에서 계속 미는 것 완화.

### 🔴 너무 멀리서 멈춘다
- `final_distance` **↓**.
- `distance_tolerance` **↓**(너무 크면 일찍 ARRIVED). 단 `min_linear_speed`와의 stall 관계 주의(아래).

### 🔴 도착 판정이 안 되고 목표 앞에서 계속 꿈틀댄다(stall/limit-cycle)
- `min_linear_speed`가 커서 floor가 `distance_tolerance`를 넘겨 왕복. `min_linear_speed` **↓**(0.01) 또는 `distance_tolerance` **↑**.
- 규칙: `kp_forward·distance_tolerance` 가 `min_linear_speed` 보다 너무 작지 않게.

### 🟠 좌우로 흔들린다(중앙정렬 진동)
- `kp_lateral` **↓**(예: 0.6). `obs_filter_alpha` **↓**(0.4)로 관측 안정화. `max_linear_accel` **↓**.

### 🟠 중앙정렬이 안 맞는다/느슨하다
- **가장 흔한 원인**: `min_vy`가 실제 하드웨어 최소보다 작으면(예 0.02인데 실제 0.1) 작은 보정이 무시돼 중앙이 안 맞음 → **`min_vy`를 로봇 실제 최소(0.1)로 맞출 것**.
- `kp_lateral` **↑**(1.5~2.5), `lateral_tolerance` **↓**(0.02~0.025).
- **한계**: `min_vy`가 0.1이면 최소 횡이동 한 스텝이 커서, `lateral_tolerance`를 그보다 작게 잡으면 좌우로 미세 진동. → `lateral_tolerance`는 `min_vy` 한 스텝 오버슛(대략 1.5~2.5cm) 이상으로.
- **CENTER 단계**(`center_first_tolerance`)로 yaw 전에 y축 정렬을 먼저 끝내면 정렬 품질이 좋아짐.

### 🟠 정면(회전)이 흔들린다/안 맞는다
- 흔들림: `kp_yaw` **↓**(0.8), `obs_filter_alpha` **↓**. 근본적으로 near-frontal에서 마커 법선 추정이 노이즈가 큼.
- 느슨함: `kp_yaw` **↑**, `yaw_tolerance_deg` **↓**(3°).

### 🟠 전체적으로 느리다/굼뜨다
- `max_linear_speed` **↑**(스펙 상한 0.3), 게인 소폭 **↑**, `max_linear_accel` **↑**, `obs_filter_alpha` **↑**(0.65).

### 🟠 급출발/급정지(안 부드럽다)
- `max_linear_accel`·`max_yaw_accel` **↓**(예: 0.3 / 0.6).

### 🟠 마커를 자주 놓쳐 멈춘다
- `max_pose_age_sec` **↑**(예: 1.0) — 짧은 미검출을 견딤. 단 너무 키우면 실제 소실에도 늦게 정지(안전 저하).

---

## 4. 안전
- `max_linear_speed` 는 스펙 상한 **0.3** 유지 권장. 실주행 첫 테스트는 0.1로 낮춰 방향(`linear.y`·`angular.z` 부호)부터 확인.
- 접근 목표 10cm 는 카메라 기준. **로봇 몸체가 벽/거치대에 먼저 닿지 않는지** 물리 여유 확인 후 `final_distance` 설정.
- 언제든 `ros2 service call /marker_approach/stop std_srvs/srv/Trigger` 로 비상 정지.

---

## 5. 권장 튜닝 순서
1. 낮은 속도(`max_linear_speed: 0.1`)로 `/cmd_vel` echo 하며 **부호/방향** 확인.
2. **정지 거리**부터: `final_distance` 로 원하는 지점에서 서게. (이번 "너무 가까움"은 여기 + OPENLOOP)
3. **오버슛** 잡기: `max_linear_accel`·`kp_forward`.
4. **정렬 품질**: `kp_lateral`/`lateral_tolerance`, `kp_yaw`/`yaw_tolerance_deg`.
5. 마지막에 속도 상한을 목표치로 올려 재확인.
