//! 自然轨迹生成器。
//!
//! 画像驱动：打卡点拟合闭合环（Catmull-Rom，每段 18 采样）→ 弧长表；
//! 采样间隔主 5s（80%）；速度曲线 = ramp × 疲劳 × 三正弦 × 余弦凹陷 × 微噪；
//! 正常点按速度曲线分配位移并归一到精确总距离；所有采样点保持在跑步区域环线上，避免被服务端标成无效灰段；
//! 哨兵点（首 type∈{0,7}、索引1 type=5、末 type=6）；结尾断崖；点位吸附。
#![allow(non_snake_case)]

use super::geom::{fmt_gain_time, make_point_ring, ring_point_at, round_to, to_bd, Rng, MET_PER_DEG_LAT, MET_PER_DEG_LNG};
use super::geom::PointRing;
use super::model::{GenPoint, Segment, TenWindow, Track};
use super::postfix::apply_post_fixes;
use super::stadium::{make_stadium_ring, TrackSpec};
use crate::api::model::TrackColorMode;

/// 有效配速窗口（判定规则 2'21"-10'00"/km ≈ 1.667-7.092 m/s），硬边界留余量。
pub const SPEED_FLOOR: f64 = 1.90;
pub const SPEED_CEIL: f64 = 6.30;
pub const MAX_EXCEPTION_GPS_SPEED_KMH: f64 = 6.0;
/// Maximum coordinate drift for a protocol exception point.
///
/// These points may still carry their original type marker, but they must not
/// create a map-scale jump that the server interprets as an invalid route.
pub const MAX_EXCEPTION_DRIFT_M: f64 = 6.0;

/// 全绿色模式的 GPS 抖动软上限（米，合位移）。
///
/// 详情页着色只看 `type`/`locType`/`state`/`bdG`，抖动不参与判定。普通模式
/// 的横向漂移不超过一条车道（1.22m），既明显不是"完美线条"，又不会飘出
/// 道路；固定跑道（绕圈）模式另见 [`LOOP_TRACK_OFFSET_M`]。
pub const GREEN_JITTER_CLAMP_M: f64 = 0.6;
/// 全绿色模式的 AR(1) 相干漂移标准差（米）与测量噪声标准差（米）。
/// 稳态 σ = 0.12/√(1-0.72²) ≈ 0.17m/轴，叠加测量噪声后 3σ ≈ 0.86m，再经
/// [`GREEN_JITTER_CLAMP_M`] 截断，合位移落在 0.1-0.6m。
pub const GREEN_JITTER_SIGMA_CORR: f64 = 0.12;
pub const GREEN_JITTER_SIGMA_MEAS: f64 = 0.08;

/// 标准田径场跑道数（8 条）。
pub const TRACK_LANE_COUNT: f64 = 8.0;
/// 单条跑道的半宽（米）。标准道宽 1.22m，取 0.6m 作为每条道的偏移单位。
pub const TRACK_LANE_HALF_WIDTH_M: f64 = 0.6;
/// 固定跑道（绕圈）模式的横向漂移半径（米）。
///
/// 操场有 8 条跑道，跑者可以在整条跑道的道次间换道，因此轨迹偏离跑道
/// 中心线的幅度应覆盖 ±(0.6m × 8) = ±4.8m，而不是贴着中心线跑一条细线。
pub const LOOP_TRACK_OFFSET_M: f64 = TRACK_LANE_HALF_WIDTH_M * TRACK_LANE_COUNT;
/// 全绿色模式下建筑 SDF 对抖动的放大倍数上限（近墙本来可放大到 3×）。
pub const GREEN_JITTER_SDF_CAP: f64 = 1.2;

/// 把米制抖动截断到 `limit` 合位移以内（方向不变）。
pub fn clamp_jitter(dx: f64, dy: f64, limit: f64) -> (f64, f64) {
    let len = dx.hypot(dy);
    if len > limit && len > 0.0 {
        let k = limit / len;
        (dx * k, dy * k)
    } else {
        (dx, dy)
    }
}

/// 把米制抖动截断到 [`GREEN_JITTER_CLAMP_M`] 合位移以内（方向不变）。
pub fn clamp_green_jitter(dx: f64, dy: f64) -> (f64, f64) {
    clamp_jitter(dx, dy, GREEN_JITTER_CLAMP_M)
}

/// 等比缩放逐点速度至目标总距：越界点钳在窗口边界，剩余差量由未饱和点分摊（迭代收敛）。
/// 与整体等比缩放的区别：任何一点的瞬时配速都不会越出有效窗口。
fn fit_speeds(w: &mut [f64], dts: &[f64], target: f64) {
    for _ in 0..24 {
        let cur: f64 = w.iter().zip(dts).map(|(x, dt)| x * dt).sum();
        if (cur - target).abs() <= 1.0 {
            break;
        }
        let k = target / cur;
        for x in w.iter_mut() {
            *x = (*x * k).clamp(SPEED_FLOOR, SPEED_CEIL);
        }
    }
    for x in w.iter_mut() {
        *x = x.clamp(SPEED_FLOOR, SPEED_CEIL);
    }
}

/// 经典速度剖面：5s 主采样时间网格 + ramp×疲劳×三正弦×余弦凹陷×微噪速度曲线，
/// 逐点速度约束在有效配速窗口内并精确命中目标距离。返回 (times, dts, speeds)。
/// 自定义路线（track::custom）复用同一剖面，保证协议字段与经典模式一致。
pub(crate) fn speed_profile(
    dist: f64,
    dur: i64,
    rng: &mut Rng,
) -> (Vec<f64>, Vec<f64>, Vec<f64>) {
    let dur_f = dur as f64;
    let phase_v = rng.uniform(0.0, std::f64::consts::TAU);
    let mut times = Vec::new();
    let mut t = 0.0;
    while t < dur_f {
        times.push(t);
        t += if rng.random() < 0.80 {
            5.0
        } else {
            rng.choice(&[1.0, 2.0, 3.0, 4.0, 6.0, 7.0, 8.0])
        };
    }
    let n = times.len();

    let n_dips = rng.choice(&[1, 1, 2]);
    let mut dips = Vec::new();
    for _ in 0..n_dips {
        dips.push((
            rng.uniform(0.15, 0.75) * dur_f,
            rng.uniform(25.0, 55.0),
            rng.uniform(0.08, 0.16),
        ));
    }
    let dip_factor = |t: f64| -> f64 {
        let mut f = 1.0;
        for &(c, hw, d) in &dips {
            if (t - c).abs() < hw {
                f *= 1.0 - d * 0.5 * (1.0 + std::f64::consts::PI * (t - c) / hw).cos();
            }
        }
        f
    };
    let base = dist / dur_f;
    let mut w = Vec::with_capacity(n);
    for &tt in &times {
        let ramp = if tt < 8.0 {
            (0.85 + 0.15 * (tt / 1.0f64.max(8.0f64.min(dur_f / 20.0)))).min(1.0)
        } else {
            1.0
        } * if tt > dur_f - 8.0 {
            1.0 + 0.03 * (tt - (dur_f - 8.0)) / 8.0
        } else {
            1.0
        };
        let km_done = (tt / dur_f) * dist / 1000.0;
        let fatigue = if km_done <= 0.5 {
            1.03
        } else {
            (1.03 - 0.06 * (km_done - 0.5)).max(0.80)
        };
        let wave = (1.0
            + 0.010 * (std::f64::consts::TAU * tt / 115.0 + phase_v).sin()
            + 0.035 * (std::f64::consts::TAU * tt / 47.0 + phase_v * 2.3).sin()
            + 0.015 * (std::f64::consts::TAU * tt / 19.0 + phase_v * 3.7).sin())
            * dip_factor(tt);
        let noise = 1.0 + rng.gauss(0.0, 0.008);
        w.push(base * ramp * fatigue * wave * noise);
    }
    let mut dts: Vec<f64> = (0..n - 1).map(|i| times[i + 1] - times[i]).collect();
    dts.push(1.0f64.max(dur_f - times[n - 1]));
    // 逐点速度全部约束在有效配速窗口内，并精确命中目标距离
    fit_speeds(&mut w, &dts, dist);
    (times, dts, w)
}

/// 轨迹生成主入口。points_bd 为 BD 系打卡点。
pub fn build(
    dist: f64,
    dur: i64,
    seed: u64,
    _center: (f64, f64),
    start_ms: i64,
    points_bd: &[(f64, f64)],
) -> Track {
    build_with_color_mode(
        dist,
        dur,
        seed,
        _center,
        start_ms,
        points_bd,
        TrackColorMode::FullGreen,
    )
}

/// Generate a route along a selected standard athletics track.
pub fn build_stadium(
    dist: f64,
    dur: i64,
    seed: u64,
    center: (f64, f64),
    start_ms: i64,
    points_bd: &[(f64, f64)],
    spec: TrackSpec,
) -> Result<Track, String> {
    // 固定跑道（绕圈）用**打卡点连成的环**，而不是跑到每个点再折返。
    //
    // 打卡点在这里是"范围标记"，不是"必须逐一拜访的点"：把它们按环序串成
    // 一条闭合路径，然后一直绕这条环即可。这样轨迹是一条干净的环，不会像
    // 逐点最短路径那样岔出去再回来。
    let ring = if spec.is_fixed_loop() {
        if points_bd.len() >= 3 {
            // 拟合一条跑道形状的环：尺寸随点位自适应，而不是直接连点位
            // （直接连点位会得到任意多边形，看起来不像跑道）。
            let lap = match spec {
                TrackSpec::Loop { total_m } => total_m as f64,
                _ => 400.0,
            };
            super::stadium::make_fitted_track_ring(points_bd, lap)?
        } else {
            super::stadium::make_fixed_loop_ring(center, spec)?
        }
    } else {
        make_stadium_ring(points_bd, spec)?
    };
    // 固定跑道（绕圈）的横向漂移要覆盖 8 条跑道；其他操场规格仍按单车道抖动。
    let jitter_limit = if spec.is_fixed_loop() {
        LOOP_TRACK_OFFSET_M
    } else {
        GREEN_JITTER_CLAMP_M
    };
    Ok(build_on_ring(
        dist,
        dur,
        seed,
        center,
        start_ms,
        points_bd,
        Some(ring),
        jitter_limit,
        TrackColorMode::FullGreen,
    ))
}

/// 生成轨迹并选择详情页路线颜色模式。
pub fn build_with_color_mode(
    dist: f64,
    dur: i64,
    seed: u64,
    _center: (f64, f64),
    start_ms: i64,
    points_bd: &[(f64, f64)],
    color_mode: TrackColorMode,
) -> Track {
    build_on_ring(
        dist,
        dur,
        seed,
        _center,
        start_ms,
        points_bd,
        None,
        GREEN_JITTER_CLAMP_M,
        color_mode,
    )
}

fn build_on_ring(
    dist: f64,
    dur: i64,
    seed: u64,
    _center: (f64, f64),
    start_ms: i64,
    points_bd: &[(f64, f64)],
    supplied_ring: Option<PointRing>,
    jitter_limit: f64,
    color_mode: TrackColorMode,
) -> Track {
    let mut rng = Rng::new(seed);
    let dur_f = dur as f64;
    let (dense, arcs, pc) = supplied_ring.unwrap_or_else(|| make_point_ring(points_bd));
    let (c_lat, c_lng) = pc;
    // 田径场与学校体测统一按**逆时针**（左手弯道）跑。环序由 `make_point_ring`
    // 的 atan2 升序给出，本身即逆时针，因此固定为 +1；旧版随机取 ±1 会有一半
    // 概率跑成顺时针，方向看起来不对。
    let direction: f64 = 1.0;
    let s0 = rng.uniform(0.0, *arcs.last().unwrap_or(&400.0));
    let phase_l = rng.uniform(0.0, std::f64::consts::TAU);

    let (times, dts, w) = speed_profile(dist, dur, &mut rng);
    let n = times.len();
    let seg_dist: Vec<f64> = (0..n).map(|i| w[i] * dts[i]).collect();
    let speeds: Vec<f64> = w.clone();
    // 步频波动相位（速度剖面内部相位已由 speed_profile 消费）
    let phase_v = rng.uniform(0.0, std::f64::consts::TAU);

    let kinds: Vec<(i64, i64)> = match color_mode {
        TrackColorMode::FullGreen => vec![(0, 1); n],
        TrackColorMode::HalfGreenGray => {
            let tl_w = [((-1i64, 4i64), 54u32), ((-1, 1), 27), ((-1, 12), 13), ((-1, 5), 3), ((-1, 6), 2)];
            let mut kinds: Vec<(i64, i64)> = Vec::with_capacity(n);
            for _ in 0..n {
                let u = rng.random();
                if u < 0.39 {
                    kinds.push((3, 1));
                } else if u < 0.93 {
                    kinds.push((0, 1));
                } else if u < 0.96 {
                    kinds.push(rng.weighted(&tl_w));
                } else {
                    kinds.push((rng.choice(&[1, 1, 1, 1, 2, 2]), 1));
                }
            }
            for i in 1..kinds.len() {
                let prev_drift = (-1 == kinds[i - 1].0) || kinds[i - 1].0 == 5 || kinds[i - 1].0 == 6;
                if kinds[i].0 != -1 && prev_drift && rng.random() < 0.08 {
                    kinds[i] = (if rng.random() < 0.75 { 7 } else { 8 }, 1);
                }
            }
            for i in 1..n {
                if kinds[i].0 == -1 && kinds[i - 1].0 == -1 {
                    kinds[i] = (rng.choice(&[3, 0]), 1);
                }
            }
            kinds
        }
    };
    let normal_idx: Vec<usize> =
        (0..n).filter(|&i| kinds[i].0 != -1 && i > 1).collect();
    let share: f64 = normal_idx.iter().map(|&i| seg_dist[i]).sum();
    let share = if share == 0.0 { 1.0 } else { share };
    let mut disp_of = vec![0.0f64; n];
    for &i in &normal_idx {
        disp_of[i] = seg_dist[i] * (dist / share);
    }

    let mut locs: Vec<GenPoint> = Vec::with_capacity(n);
    let mut s = s0;
    let mut t_acc = 0.0f64;
    let mut dist_acc = 0.0f64;
    let mut steps_acc = 0.0f64;
    let mut jx = 0.0f64;
    let mut jy = 0.0f64;
    let mut alt = 82.0 + rng.uniform(-1.0, 1.0);
    let n_est = 1.max((dur_f / 5.0) as i64);
    let alt_sigma = rng.uniform(3.8, 6.2) / (0.40 * n_est as f64);
    let mut ten_t = 0.0f64;
    let mut ten_d = 0.0f64;
    let mut ten_st = 0.0f64;
    let mut ten_speed: Vec<TenWindow> = Vec::new();
    let mut ten_steps: Vec<TenWindow> = Vec::new();
    // 每点相对环上的米制抖动：吸附时保留，避免出现"完美贴合点"。
    let mut jitter_m: Vec<(f64, f64)> = Vec::with_capacity(n);

    for i in 0..n {
        let dt = dts[i];
        let (typ, lt) = kinds[i];
        t_acc += dt;
        let pos = |ss: f64| ring_point_at(&dense, &arcs, ss);
        let mut d_step = 0.0f64;
        let px;
        let py;
        let x;
        let y;
        let rad;
        let state;
        if typ != -1 {
            d_step = disp_of[i];
            s += direction * d_step;
            let (bx, by) = pos(s);
            x = bx;
            y = by;
            if color_mode == TrackColorMode::FullGreen {
                // 小抖动：AR(1) 相干漂移 + 独立测量噪声，合位移截断到
                // `jitter_limit` 以内（普通模式一条车道，绕圈模式整条跑道）。
                // 着色四字段稍后无条件置绿，抖动不影响详情页颜色。
                let jitter_scale = (jitter_limit / GREEN_JITTER_CLAMP_M).max(1.0);
                jx = 0.72 * jx + rng.gauss(0.0, GREEN_JITTER_SIGMA_CORR * jitter_scale);
                jy = 0.72 * jy + rng.gauss(0.0, GREEN_JITTER_SIGMA_CORR * jitter_scale);
                let (cx, cy) = clamp_jitter(
                    jx + rng.gauss(0.0, GREEN_JITTER_SIGMA_MEAS * jitter_scale),
                    jy + rng.gauss(0.0, GREEN_JITTER_SIGMA_MEAS * jitter_scale),
                    jitter_limit,
                );
                px = bx + cx;
                py = by + cy;
                jitter_m.push((cx, cy));
            } else {
                jx = 0.72 * jx + rng.gauss(0.0, 0.75);
                jy = 0.72 * jy + rng.gauss(0.0, 0.75);
                px = bx + jx;
                py = by + jy;
                jitter_m.push((jx, jy));
            }
            rad = round_to(
                if typ == 3 {
                    rng.uniform(1.4, 5.1)
                } else {
                    rng.uniform(1.4, 2.4)
                },
                2,
            );
            state = if color_mode == TrackColorMode::FullGreen {
                1
            } else if typ == 0 {
                rng.weighted(&[(1, 145), (2, 45), (3, 164)])
            } else {
                rng.weighted(&[(1, 145), (2, 256), (3, 151)])
            };
        } else {
            // Keep the exception marker, but bound its coordinate drift. Large
            // random offsets produce red route segments and false fastest paces.
            if rng.random() >= 0.20 {
                d_step = rng.uniform(0.0, MAX_EXCEPTION_DRIFT_M);
            }
            let (bx, by) = pos(s);
            x = bx;
            y = by;
            if d_step > 0.0 {
                let ang = rng.uniform(0.0, std::f64::consts::TAU);
                px = x + d_step * ang.sin();
                py = y + d_step * ang.cos();
            } else if let Some(last) = locs.last() {
                // 零位移：精确复制上一点坐标（BD→平面）
                py = (last.gLat - c_lat) * MET_PER_DEG_LAT;
                px = (last.gLng - c_lng) * MET_PER_DEG_LNG;
            } else {
                px = x + jx;
                py = y + jy;
            }
            // 吸附保留的是相对环上的偏移；异常点没有同源抖动，按 0 处理。
            jitter_m.push((px - x, py - y));
            rad = if typ == -1 {
                round_to(rng.uniform(3.0, 10.0), 2)
            } else if lt == 4 {
                round_to(if rng.random() < 0.75 { rng.uniform(30.0, 100.0) } else { rng.uniform(100.0, 550.0) }, 2)
            } else if lt == 1 {
                round_to(
                    if rng.random() < 0.75 {
                        rng.uniform(1.6, 12.0)
                    } else {
                        rng.uniform(12.0, 95.0)
                    },
                    2,
                )
            } else if lt == 12 {
                round_to(rng.uniform(30.0, 125.0), 2)
            } else if lt == 5 {
                round_to(rng.uniform(25.0, 300.0), 2)
            } else {
                550.0
            };
            state = rng.weighted(&[(1, 102), (2, 124), (3, 136)]);
        }
        if typ != -1 {
            dist_acc += d_step; // 轨迹点位移计入累计距离
        }
        let (lat, lng) = to_bd(px, py, c_lat, c_lng);
        alt += 0.04 * (82.0 - alt) + rng.gauss(0.0, alt_sigma);
        let v_now = dist / dur_f;
        let stride_target = 0.62
            + 0.17 * v_now
            + 0.03 * (std::f64::consts::TAU * t_acc / 200.0 + phase_l).sin()
            + rng.gauss(0.0, 0.008);
        let v_cad = v_now * (1.0 + 0.03 * (std::f64::consts::TAU * t_acc / 70.0 + phase_v).sin());
        let cad = (v_cad / stride_target * 60.0).clamp(100.0, 200.0);
        steps_acc += cad / 60.0 * dt;
        let nxt = pos(s + direction * 2.0);
        let brg = ((nxt.0 - x).atan2(nxt.1 - y).to_degrees() + rng.gauss(0.0, 35.0)).rem_euclid(360.0);
        // 异常点（-1）：avgSpeed 为累计均值（真人与此一致，不为 0）；
        // GPS 瞬时速度保持低速，避免尖峰污染最快配速。
        // 保留累计均值与 GPS 瞬时速度的轻微波动，但不生成越出跑步区域的漂移点。
        let (avg_sp, gps_speed) = if typ == -1 {
            let avg = round_to(dist_acc / t_acc.max(1.0), 4);
            let gps = rng.uniform(0.5, MAX_EXCEPTION_GPS_SPEED_KMH);
            (avg, round_to(gps, 4))
        } else {
            let avg = round_to(d_step / dt, 4);
            let kmh = avg * 3.6;
            let sigma = (kmh * 0.08).max(0.05);
            let gps = if rng.random() < 0.20 {
                0.0
            } else {
                round_to((kmh + rng.gauss(0.0, sigma)).max(0.0), 4)
            };
            (avg, gps)
        };

        ten_t += dt;
        ten_d += speeds[i] * dt;
        ten_st += cad / 60.0 * dt;
        while ten_t >= 10.0 {
            let k = 10.0 / ten_t; // 只取前 10s 的量，余量留给下一窗
            let (out_d, out_st) = (ten_d * k, ten_st * k);
            ten_speed.push(TenWindow {
                time: 10,
                value: round_to(out_d, 2),
            });
            ten_steps.push(TenWindow {
                time: 10,
                value: round_to(out_st, 0),
            });
            ten_t -= 10.0;
            ten_d -= out_d;
            ten_st -= out_st;
        }
        locs.push(GenPoint {
            id: i as i64 + 1,
            flag: start_ms,
            lat: -1.0,
            lng: -1.0,
            gLat: round_to(lat, 7),
            gLng: round_to(lng, 7),
            speed: round_to(gps_speed, 4),
            avgSpeed: avg_sp,
            radius: rad,
            accuracy: rad,
            ptype: typ,
            locType: lt,
            hasAltitude: true,
            totalTime: round_to(t_acc, 0) as i64,
            totalDis: round_to(dist_acc, 4),
            validDis: round_to(dist_acc, 4),
            validTime: round_to(t_acc, 0) as i64,
            steps: steps_acc as i64,
            stepDistance: 0.0,
            gainTime: fmt_gain_time(start_ms + (t_acc * 1000.0) as i64),
            gainTimeMs: start_ms + (t_acc * 1000.0) as i64,
            queueNum: 0,
            coorType: "gcj02".into(),
            bdA: round_to(alt, 2),
            bdD: round_to(brg, 2),
            bdS: round_to((avg_sp * rng.uniform(0.6, 0.95)).max(0.0), 3),
            bdG: if color_mode == TrackColorMode::FullGreen {
                1
            } else {
                rng.choice(&[1, 1, 1, -1])
            },
            count: rng.randint(20, 88),
            dtr: 0.0,
            state,
            locationId: String::new(),
        });
    }
    if ten_t > 1.0 {
        // 尾窗：限制放大倍数 ≤1.4
        let k = (10.0 / ten_t).min(1.4);
        ten_speed.push(TenWindow {
            time: 10,
            value: round_to(ten_d * k, 2),
        });
        ten_steps.push(TenWindow {
            time: 10,
            value: round_to(ten_st * k, 0),
        });
    }

    let mut segments: Vec<Segment> = Vec::new();
    let (mut seg_t, mut seg_d, mut seg_n) = (0.0f64, 0.0f64, 0i64);
    let mut seg_v: Vec<f64> = Vec::new();
    for i in 0..n {
        seg_t += dts[i];
        seg_d += seg_dist[i];
        seg_v.push(speeds[i]);
        seg_n += 1;
        if seg_t >= 60.0 || i == n - 1 {
            segments.push(Segment {
                totalTime: round_to(seg_t, 0) as i64,
                distance: round_to(seg_d, 0) as i64,
                startTime: round_to((times[i] - seg_t) * 1000.0, 0) as i64,
                endTime: round_to((times[i] + dts[i]) * 1000.0, 0) as i64,
                avgSpeed: round_to(seg_v.iter().sum::<f64>() / seg_v.len() as f64, 3),
                avgStep: round_to(steps_acc / 1.0f64.max(t_acc) * 60.0, 0) as i64,
                state: 0,
            });
            seg_t = 0.0;
            seg_d = 0.0;
            seg_v.clear();
            seg_n = 0;
        }
    }
    let _ = seg_n;

    apply_post_fixes(&mut locs, &mut rng, start_ms);

    if color_mode == TrackColorMode::FullGreen {
        // 详情页会根据这些字段给轨迹分段着色。统一普通点协议，保留
        // 起点/终点哨兵，避免出现灰色段。
        let last_index = locs.len().saturating_sub(1);
        for (i, point) in locs.iter_mut().enumerate() {
            point.ptype = if i == 1 {
                5
            } else if i == last_index {
                6
            } else {
                0
            };
            point.locType = 1;
            point.state = 1;
            point.bdG = 1;
        }
    }

    // 点位吸附：<40m 落位到打卡点，但保留该点原有抖动，避免地图上出现
    // 「完美贴合点」。全绿色模式把抖动截断在允许的横向漂移内（绕圈模式为
    // 整条 8 道跑道）；混合模式保持原有的精确落位（其抖动本来更大，吸附点
    // 很少触发）。
    for pl in points_bd {
        let mut best_i = None;
        let mut best_d = 1e18f64;
        for (i, q) in locs.iter().enumerate() {
            let dd = ((q.gLat - pl.0) * MET_PER_DEG_LAT).powi(2)
                + ((q.gLng - pl.1) * MET_PER_DEG_LNG).powi(2);
            if dd < best_d {
                best_d = dd;
                best_i = Some(i);
            }
        }
        if let Some(i) = best_i {
            if best_d < 40.0 * 40.0 {
                if color_mode == TrackColorMode::FullGreen {
                    let (jx, jy) = clamp_jitter(jitter_m[i].0, jitter_m[i].1, jitter_limit);
                    locs[i].gLat = round_to(pl.0 + jy / MET_PER_DEG_LAT, 7);
                    locs[i].gLng = round_to(pl.1 + jx / MET_PER_DEG_LNG, 7);
                } else {
                    locs[i].gLat = round_to(pl.0, 7);
                    locs[i].gLng = round_to(pl.1, 7);
                }
            }
        }
    }

    let total_dis = round_to(dist, 3);
    Track {
        totalTime: round_to(t_acc, 0) as i64,
        totalDistance: total_dis,
        validDistance: total_dis,
        validTime: round_to(t_acc, 0) as i64,
        startTime: start_ms,
        startLatitude: locs[0].gLat,
        startLongitude: locs[0].gLng,
        totalSteps: steps_acc as i64,
        locations: locs,
        speedPerTenSec: ten_speed,
        stepsPerTenSec: ten_steps,
        segments,
    }
}

#[cfg(test)]
mod tests {
    use super::{
        build, build_with_color_mode, clamp_green_jitter, MAX_EXCEPTION_DRIFT_M,
        MAX_EXCEPTION_GPS_SPEED_KMH, GREEN_JITTER_CLAMP_M,
    };
    use crate::api::model::TrackColorMode;
    use crate::track::geom::{MET_PER_DEG_LAT, MET_PER_DEG_LNG};

    /// 全绿色模式：着色四字段固定为绿，且坐标带小抖动而不是完美环线。
    #[test]
    fn full_green_keeps_colors_and_applies_small_jitter() {
        let points = vec![
            (39.9000, 116.4000),
            (39.9000, 116.4020),
            (39.9020, 116.4020),
            (39.9020, 116.4000),
        ];
        let track = build_with_color_mode(
            3300.0, 1220, 42, points[0], 1_788_958_186_123, &points,
            TrackColorMode::FullGreen,
        );
        let last = track.locations.len() - 1;
        for (i, p) in track.locations.iter().enumerate() {
            let expected_type = if i == 1 { 5 } else if i == last { 6 } else { 0 };
            assert_eq!(p.ptype, expected_type, "点 {i} 类型被抖动影响");
            assert_eq!(p.locType, 1, "点 {i} locType");
            assert_eq!(p.state, 1, "点 {i} state");
            assert_eq!(p.bdG, 1, "点 {i} bdG");
        }

        // 该模式以 4 个打卡点的 Catmull-Rom 环为几何：抖动被截断后，每个点到
        // 环线的距离都不应超过车道宽度（截断值 + 折线采样误差）。
        let (dense, _arcs, _) = crate::track::geom::make_point_ring(&points);
        let dist_to_ring = |lat: f64, lng: f64| -> f64 {
            // 转到与 make_point_ring 相同的局部米制平面
            let (cx, cy) = (
                points.iter().map(|q| q.1).sum::<f64>() / points.len() as f64,
                points.iter().map(|q| q.0).sum::<f64>() / points.len() as f64,
            );
            let p = ((lng - cx) * MET_PER_DEG_LNG, (lat - cy) * MET_PER_DEG_LAT);
            (0..dense.len() - 1)
                .map(|k| {
                    let (a, b) = (dense[k], dense[k + 1]);
                    let (abx, aby) = (b.0 - a.0, b.1 - a.1);
                    let len2 = abx * abx + aby * aby;
                    let t = if len2 > 0.0 {
                        (((p.0 - a.0) * abx + (p.1 - a.1) * aby) / len2).clamp(0.0, 1.0)
                    } else {
                        0.0
                    };
                    ((p.0 - a.0 - abx * t).powi(2) + (p.1 - a.1 - aby * t).powi(2)).sqrt()
                })
                .fold(f64::INFINITY, f64::min)
        };
        let off: Vec<f64> = track.locations.iter().map(|p| dist_to_ring(p.gLat, p.gLng)).collect();
        let moved = off.iter().filter(|&&d| d > 0.03).count();
        assert!(moved * 2 > off.len(), "全绿轨迹几乎没有抖动：{moved} / {}", off.len());
        for &d in &off {
            assert!(
                d <= GREEN_JITTER_CLAMP_M + 0.05,
                "抖动超过截断上限：{d:.2}m（上限 {GREEN_JITTER_CLAMP_M}m）"
            );
        }

        // 混合模式仍保留大抖动（防止小抖动参数被误用到另一分支）。
        let mixed = build_with_color_mode(
            3300.0, 1220, 42, points[0], 1_788_958_186_123, &points,
            TrackColorMode::HalfGreenGray,
        );
        let mixed_off: Vec<f64> = mixed.locations.iter().map(|p| dist_to_ring(p.gLat, p.gLng)).collect();
        assert!(
            mixed_off.iter().any(|&d| d > GREEN_JITTER_CLAMP_M + 0.2),
            "混合模式抖动被误改成小抖动"
        );
    }

    #[test]
    fn clamp_green_jitter_bounds_combined_displacement() {
        let (x, y) = clamp_green_jitter(10.0, 0.0);
        assert!((x.hypot(y) - GREEN_JITTER_CLAMP_M).abs() < 1e-9);
        let (x, y) = clamp_green_jitter(0.1, -0.2);
        assert!((x - 0.1).abs() < 1e-12 && (y + 0.2).abs() < 1e-12);
    }

    #[test]
    fn exception_points_cannot_create_map_scale_jumps_or_speed_spikes() {
        let points = vec![
            (39.9000, 116.4000),
            (39.9000, 116.4020),
            (39.9020, 116.4020),
            (39.9020, 116.4000),
        ];
        for seed in 0..32 {
            let track = build(2_000.0, 800, seed, points[0], 1_700_000_000_000, &points);
            let mut exception_count = 0;
            for (index, point) in track.locations.iter().enumerate() {
                if point.ptype != -1 {
                    continue;
                }
                exception_count += 1;
                assert!(point.radius <= 10.0);
                assert!(point.speed <= MAX_EXCEPTION_GPS_SPEED_KMH + 1e-9);
                if let Some(previous) = index.checked_sub(1).and_then(|i| track.locations.get(i)) {
                    let dx = (point.gLng - previous.gLng) * MET_PER_DEG_LNG;
                    let dy = (point.gLat - previous.gLat) * MET_PER_DEG_LAT;
                    assert!(
                        (dx * dx + dy * dy).sqrt() <= 80.0,
                        "exception jump at seed {seed}: {}m",
                        (dx * dx + dy * dy).sqrt()
                    );
                }
            }
            // Campus routes no longer emit invalid (-1) drift points after the
            // optional green-fence integration; if one appears it must still
            // satisfy the bounds checked above.
            assert_eq!(exception_count, 0, "seed {seed} emitted an invalid drift point");
        }
        assert!(MAX_EXCEPTION_DRIFT_M < 10.0);
    }
}
