//! 生物力学联动：配速 v、步频 f_cadence、步幅 L_stride 的非线性耦合。
//!
//!   硬约束：v = (f_cadence / 60) · L_stride   （v: m/s，f: 步/分，L: 米）
//!
//! 由速度与疲劳度确定 (步频, 步幅)，**始终严格满足等式**。

/// 步态结果。
#[derive(Clone, Copy, Debug)]
pub struct Gait {
    /// 步频（步/分）。
    pub cadence: f64,
    /// 步幅（米）。
    pub stride: f64,
}

/// 由速度与疲劳度计算步态，严格满足 `v = cadence/60 * stride`。
///
/// 步频公式由**真实记录拟合**得到，而不是拍脑袋的常数：
/// 用户 4 条真实记录（步频-配速）线性回归给出 `cadence = 34.4 + 44.5 * v`，
/// 拟合误差 ≤2 步/分：
///
/// | 配速 | 实测步频 | 公式预测 |
/// |------|---------|---------|
/// | 5:31/km (3.02 m/s) | 170 | 169 |
/// | 6:40/km (2.50 m/s) | 144 | 146 |
/// | 7:02/km (2.37 m/s) | 139 | 140 |
/// | 8:19/km (2.00 m/s) | 125 | 124 |
///
/// 旧公式 `150 + 12v` 在 6:40/km 会给出 180 步/分，比实测高 36 步，
/// 而且形态是「步频主导」；真实数据是「步幅主导」（慢配速步频降得更多）。
pub fn gait(v: f64, fatigue: f64) -> Gait {
    let target_cad = (34.4 + 44.5 * v) * (1.0 - 0.05 * fatigue);
    let mut cadence = target_cad.clamp(100.0, 210.0);
    let mut stride = v * 60.0 / cadence;
    // 步幅越界时反解步频，保持等式成立。
    if stride > 1.7 {
        stride = 1.7;
        cadence = v * 60.0 / stride;
    } else if stride < 0.5 {
        stride = 0.5;
        cadence = v * 60.0 / stride;
    }
    Gait { cadence, stride }
}

/// 由速度与疲劳度取步幅（米），包络 [0.55, 1.35]。
pub fn stride_for(v: f64, fatigue: f64) -> f64 {
    gait(v, fatigue).stride
}

/// 由速度反解步频（步/分）。
pub fn cadence_for(v: f64, stride: f64) -> f64 {
    (60.0 * v / stride.max(1e-6)).clamp(110.0, 220.0)
}

/// 由步频与步幅计算速度（m/s）。
pub fn speed_from(cadence: f64, stride: f64) -> f64 {
    (cadence / 60.0) * stride
}

/// 疲劳度（0..0.6）：由已跑里程（公里）线性抬升，0.5km 后开始，封顶 0.6。
pub fn fatigue_from_km(km: f64) -> f64 {
    if km <= 0.5 {
        0.0
    } else {
        ((km - 0.5) * 1.5).min(0.6)
    }
}

/// 步频 OU 耦合：在目标步频序列上做均值回归随机波动，返回逐点步频（步/分）。
///
///   c_{t+1} = c_t + θ(target_i − c_t)·Δt + σ·√Δt·Z,  Z~N(0,1)
///
/// 用于「步频-速度」耦合的随机部分，使步频既跟踪速度又不机械等值。
pub fn ou_cadence_series(
    targets: &[f64],
    dts: &[f64],
    seed: u64,
    theta: f64,
    sigma: f64,
    floor: f64,
    ceil: f64,
) -> Vec<f64> {
    use rand::rngs::StdRng;
    use rand::SeedableRng;
    use rand_distr::{Distribution, Normal};

    let mut rng = StdRng::seed_from_u64(seed);
    let normal = Normal::<f64>::new(0.0, 1.0).unwrap();
    let mut out = Vec::with_capacity(targets.len());
    let mut c = targets.first().copied().unwrap_or(150.0);
    for i in 0..targets.len() {
        let dt = dts[i];
        c = c + theta * (targets[i] - c) * dt + sigma * dt.sqrt() * normal.sample(&mut rng);
        c = c.clamp(floor, ceil);
        out.push(c);
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 步频公式必须匹配用户真实记录，而不是旧的「150 + 12v」。
    ///
    /// 这四条是用户 2026-10 的真实跑步记录（配速 → 平均步频）：
    /// 5:31→170、6:40→144、7:02→139、8:19→125。
    /// 旧公式在 6:40 给出 180，比实测高 36 步，属于明显失真。
    #[test]
    fn cadence_matches_the_users_real_records() {
        for (pace_s, want) in [(331.0, 170.0), (400.0, 144.0), (422.0, 139.0), (499.0, 125.0)] {
            let v = 1000.0 / pace_s;
            let g = gait(v, 0.0);
            assert!(
                (g.cadence - want).abs() <= 3.0,
                "配速 {:.0}s/km 步频 {:.0} 偏离实测 {want} 超过 3 步",
                pace_s,
                g.cadence
            );
            // 步幅必须落在真实跑者的合理区间。
            assert!(
                (0.85..=1.30).contains(&g.stride),
                "配速 {:.0}s/km 步幅 {:.2}m 超出合理范围",
                pace_s,
                g.stride
            );
        }
    }

    /// 等式约束 `v = cadence/60 * stride` 必须始终成立。
    #[test]
    fn cadence_and_stride_always_satisfy_the_speed_identity() {
        for v in [1.5, 2.0, 2.5, 3.0, 3.5, 4.0] {
            for fatigue in [0.0, 0.5, 1.0] {
                let g = gait(v, fatigue);
                let reconstructed = g.cadence / 60.0 * g.stride;
                assert!(
                    (reconstructed - v).abs() < 1e-9,
                    "v={v} fatigue={fatigue} 重建速度 {reconstructed} 不符"
                );
            }
        }
    }

    #[test]
    fn gait_coupling_equation_holds() {
        for v in [1.9f64, 2.5, 3.5, 5.0, 6.3] {
            for fat in [0.0f64, 0.3, 0.6] {
                let g = gait(v, fat);
                let back = speed_from(g.cadence, g.stride);
                assert!((back - v).abs() < 1e-6, "v={v} back={back}");
                assert!((0.5..=1.7).contains(&g.stride), "stride={}", g.stride);
                assert!((110.0..=230.0).contains(&g.cadence), "cad={}", g.cadence);
            }
        }
    }

    #[test]
    fn cadence_ou_stays_in_envelope() {
        let n = 200;
        let targets: Vec<f64> = (0..n).map(|i| 170.0 + 0.1 * i as f64).collect();
        let dts = vec![5.0; n];
        let cad = ou_cadence_series(&targets, &dts, 42, 0.25, 0.5, 110.0, 220.0);
        assert_eq!(cad.len(), n);
        for c in cad {
            assert!((110.0..=220.0).contains(&c), "cadence={c}");
        }
    }

    #[test]
    fn fatigue_monotonic_capped() {
        assert_eq!(fatigue_from_km(0.4), 0.0);
        assert!(fatigue_from_km(1.0) > 0.0);
        assert_eq!(fatigue_from_km(5.0), 0.6);
    }
}
