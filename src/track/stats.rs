//! 派生统计的唯一来源：最终累计采样点。
//!
//! 十秒窗的步数分配是这里的关键：每个窗按距离分摊总步数时，简单四舍五入会
//! 让各窗之和偏离总步数（几秒内凭空多出或丢掉几十步）。这里用**最大余数法**
//! 分配，保证所有窗之和严格等于总步数，并且不为零距离窗口编造步数。

use super::model::TenWindow;

/// 在给定累计时间处线性插值累计距离与步数。
///
/// 先插值再取整（而不是逐窗取整），末窗步数才能与总和一致。
pub fn counters_at_time(
    samples: &[(i64, f64, i64)],
    time: i64,
) -> (f64, i64) {
    let Some(&(t0, d0, s0)) = samples.first() else {
        return (0.0, 0);
    };
    if time <= t0 {
        return (d0, s0);
    }
    for pair in samples.windows(2) {
        let (a, b) = (pair[0], pair[1]);
        if b.0 >= time && b.0 > a.0 {
            let k = (time - a.0) as f64 / (b.0 - a.0) as f64;
            return (a.1 + (b.1 - a.1) * k, (a.2 as f64 + (b.2 - a.2) as f64 * k).round() as i64);
        }
    }
    let (_, d, s) = *samples.last().unwrap();
    (d, s)
}

/// 最大余数法：把 `total` 步按窗口距离比例分配到各窗。
///
/// 每个有距离的窗口先保底 1 步，剩余步数按距离比例分配，小数部分按
/// 余数从大到小补足。这样无距离窗口为 0 步，总和精确等于 `total`。
pub fn allocate_steps(windows: &[TenWindow], total: i64) -> Result<Vec<i64>, &'static str> {
    if total < 0 || windows.iter().any(|w| !w.value.is_finite() || w.value < 0.0) {
        return Err("窗口距离或总步数无效");
    }
    let mut out = vec![0i64; windows.len()];
    let active: Vec<usize> = (0..windows.len()).filter(|&i| windows[i].value > 0.0).collect();
    if active.is_empty() {
        return if total == 0 { Ok(out) } else { Err("无移动距离，不能按距离分配步数") };
    }
    if total < active.len() as i64 {
        return Err("总步数不足以分配给每个移动窗口");
    }
    let distance: f64 = active.iter().map(|&i| windows[i].value).sum();
    if !distance.is_finite() || distance <= 0.0 {
        return Err("窗口距离总量无效");
    }
    let remaining = total - active.len() as i64;
    let mut remainders: Vec<(usize, f64)> = Vec::with_capacity(active.len());
    for &i in &active {
        let quota = remaining as f64 * (windows[i].value / distance);
        let base = quota.floor() as i64;
        out[i] = 1 + base;
        remainders.push((i, quota - base as f64));
    }
    // 余数大的先补；余数相同按窗口顺序，保证结果可复现。
    remainders.sort_by(|a, b| b.1.total_cmp(&a.1).then(a.0.cmp(&b.0)));
    let allocated: i64 = out.iter().sum();
    let mut left = total - allocated;
    if left < 0 || left as usize > remainders.len() {
        return Err("步数分配超出精度范围");
    }
    for &(i, _) in remainders.iter() {
        if left == 0 {
            break;
        }
        out[i] += 1;
        left -= 1;
    }
    Ok(out)
}

/// 按距离比例在时间轴上定位 `distance` 米对应的时刻（末窗用）。
pub fn time_at_distance(samples: &[(i64, f64, i64)], distance: f64) -> i64 {
    let Some(&(t0, d0, _)) = samples.first() else {
        return 0;
    };
    if distance <= d0 {
        return t0;
    }
    for pair in samples.windows(2) {
        let (a, b) = (pair[0], pair[1]);
        if b.1 >= distance && b.1 > a.1 {
            let k = (distance - a.1) / (b.1 - a.1);
            return a.0 + ((b.0 - a.0) as f64 * k).round() as i64;
        }
    }
    samples.last().map(|s| s.0).unwrap_or(0)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn windows(values: &[f64]) -> Vec<TenWindow> {
        values
            .iter()
            .map(|&value| TenWindow { time: 10, value })
            .collect()
    }

    #[test]
    fn allocation_sums_exactly_to_total() {
        let ws = windows(&[12.0, 11.0, 13.5, 9.0, 14.0]);
        let out = allocate_steps(&ws, 1700).unwrap();
        assert_eq!(out.iter().sum::<i64>(), 1700);
        assert!(out.iter().all(|&n| n > 0));
    }

    #[test]
    fn zero_distance_windows_get_zero_steps() {
        let ws = windows(&[10.0, 0.0, 10.0]);
        let out = allocate_steps(&ws, 1000).unwrap();
        assert_eq!(out[1], 0);
        assert_eq!(out.iter().sum::<i64>(), 1000);
    }

    #[test]
    fn impossible_allocations_are_rejected_not_rounded() {
        assert!(allocate_steps(&windows(&[10.0, 10.0]), 1).is_err());
        assert!(allocate_steps(&windows(&[0.0, 0.0]), 5).is_err());
        assert!(allocate_steps(&windows(&[10.0]), -1).is_err());
    }

    #[test]
    fn counters_interpolate_between_samples() {
        let samples = [(0i64, 0.0f64, 0i64), (600, 1000.0, 1200)];
        assert_eq!(counters_at_time(&samples, 300), (500.0, 600));
        assert_eq!(counters_at_time(&samples, 0), (0.0, 0));
        assert_eq!(counters_at_time(&samples, 999), (1000.0, 1200));
        assert_eq!(time_at_distance(&samples, 500.0), 300);
    }
}
