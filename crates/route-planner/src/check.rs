//! 生成前预检：在真正规划路线之前，把"注定失败"的原因说清楚。
//!
//! 规划失败时用户只拿到一句"路由失败"，无法判断是路网没连通、点位离路太远、
//! 还是目标距离超过校内可用道路长度。这里逐项预检，并给出**问题点坐标**，
//! 界面可以在地图上标红。

use crate::{boundary, project, route, Coord, RoadGraph};

/// 点位离路的判定上限（米）。
pub const MAX_SNAP_M: f64 = 40.0;

#[derive(Clone, Debug)]
pub struct Issue {
    pub message: String,
    /// 出问题的点位序号（对应传入的 checkpoints）。
    pub point: Option<usize>,
    /// 在地图上标红的位置。
    pub position: Option<Coord>,
}

#[derive(Clone, Debug, Default)]
pub struct Check {
    pub issues: Vec<Issue>,
    /// 按点位顺序走完并闭合回起点的最短距离（米）。
    pub minimum_m: Option<f64>,
    /// 路网连通分量数量（>1 说明围栏裁剪或数据把路网切开了）。
    pub components: usize,
}

impl Check {
    pub fn ok(&self) -> bool {
        self.issues.is_empty()
    }

    /// 首个问题（UI 直接展示）。
    pub fn error(&self) -> Option<String> {
        self.issues.first().map(|i| i.message.clone())
    }
}

/// 检查点位可达性与最短闭环距离。
///
/// `checkpoints[0]` 为起点；围栏传入时同时检查点位是否在围栏内。
pub fn inspect(net: &RoadGraph, checkpoints: &[Coord], fences: &[Vec<Coord>]) -> Check {
    let mut out = Check {
        components: connected_components(net),
        ..Check::default()
    };
    if checkpoints.is_empty() {
        out.issues.push(Issue {
            message: "没有起点".into(),
            point: None,
            position: None,
        });
        return out;
    }
    if net.graph.node_count() == 0 {
        out.issues.push(Issue {
            message: "路网为空：没有可用道路，请更新或手动导入 OSM".into(),
            point: None,
            position: None,
        });
        return out;
    }

    // 1) 围栏：点位越界时直接给出坐标，避免把越界误报成"不可达"。
    if !fences.is_empty() {
        for (i, p) in checkpoints.iter().enumerate() {
            if !boundary::contains(fences, *p) {
                out.issues.push(Issue {
                    message: format!(
                        "打卡点[{i}] ({:.6},{:.6}) 不在已确认的学校围栏内，请核对校区选择",
                        p.lat, p.lon
                    ),
                    point: Some(i),
                    position: Some(*p),
                });
            }
        }
        if !out.issues.is_empty() {
            return out;
        }
    }

    // 2) 离路距离：超过 40 米视为不可达（投影总会给出最近的边，不能只看有没有边）。
    for (i, p) in checkpoints.iter().enumerate() {
        let dist = net
            .nearest_edge(net.to_m(*p))
            .map(|(_, _, _, d)| d)
            .unwrap_or(f64::INFINITY);
        if !(dist <= MAX_SNAP_M) {
            out.issues.push(Issue {
                message: format!(
                    "打卡点[{i}] ({:.6},{:.6}) 最近道路 {:.0} 米，超过 {MAX_SNAP_M:.0} 米判定范围",
                    p.lat, p.lon, dist
                ),
                point: Some(i),
                position: Some(*p),
            });
        }
    }
    if !out.issues.is_empty() {
        return out;
    }

    // 3) 投影到路网（与实际规划走同一入口，投影点一致）。
    let (working, nodes) = match project_points(net, checkpoints) {
        Ok(v) => v,
        Err(e) => {
            out.issues.push(Issue {
                message: e,
                point: None,
                position: None,
            });
            return out;
        }
    };

    // 4) 依次连通性：任何一跳不可达都说明路网被围栏裁开。
    let mut total = 0.0f64;
    let mut cursor = nodes[0];
    for (i, &target) in nodes.iter().enumerate().skip(1) {
        match route::shortest_path(&working, cursor, target) {
            Some(path) => {
                // 必须用切分后的 working 算长度：虚拟节点只存在于该图，
                // 拿原图索引会越界。
                total += path_length(&working, &path);
                cursor = target;
            }
            None => out.issues.push(Issue {
                message: format!(
                    "打卡点[{i}] ({:.6},{:.6}) 与上一点之间道路不连通",
                    checkpoints[i].lat, checkpoints[i].lon
                ),
                point: Some(i),
                position: Some(checkpoints[i]),
            }),
        }
    }

    // 5) 闭合：末点必须能回到起点。
    if out.issues.is_empty() {
        match route::shortest_path(&working, nodes[nodes.len() - 1], nodes[0]) {
            Some(back) => out.minimum_m = Some(total + path_length(&working, &back)),
            None => out.issues.push(Issue {
                message: "路线无法闭合回起点：末点与起点之间道路不连通".into(),
                point: Some(0),
                position: Some(checkpoints[0]),
            }),
        }
    }
    out
}

/// 把点位集合投影到路网（切分后的新图 + 虚拟节点）。
///
/// 虚拟节点属于新图，因此后面的可达性判断必须用返回的 `working`，不能再用原图。
fn project_points(
    net: &RoadGraph,
    checkpoints: &[Coord],
) -> Result<(RoadGraph, Vec<petgraph::graph::NodeIndex>), String> {
    let mut working = net.clone();
    let nodes = project::add_virtual_nodes(&mut working, checkpoints)?;
    Ok((working, nodes))
}

/// 「目标距离不足以覆盖必经点最短游览」错误消息的固定前缀。
///
/// 上层（UI / 生成流程）用 [`is_target_below_min_tour`] 或这个前缀把这类错误
/// 翻译成友好提示，而不是当成崩溃。目标距离足够时不会出现这个前缀。
pub const TARGET_BELOW_MIN_TOUR_PREFIX: &str = "目标距离不足";

/// 判定目标距离时只容忍浮点噪声：目标恰好等于最短游览时放行，少 1 米即报错。
const TARGET_EPS_M: f64 = 1e-6;

/// 目标距离是否至少能覆盖「按序拜访必经点并回到起点」的最短游览。
///
/// 目标更短时规划器只能产出比目标更长的路线（长度由必经点游览决定），
/// 多出来的里程用户毫不知情，因此直接报错而不是静默超长。
pub fn check_target(check: &Check, target_len_m: f64) -> Result<(), String> {
    if !check.ok() {
        return Err(check.error().unwrap_or_else(|| "路线预检未通过".into()));
    }
    if let Some(min) = check.minimum_m {
        if target_len_m + TARGET_EPS_M < min {
            return Err(format!(
                "{TARGET_BELOW_MIN_TOUR_PREFIX}：目标距离 {:.0} 米小于必经点最短游览 {:.0} 米；\
                 请把目标里程提高到 {:.0} 米以上，或减少打卡点",
                target_len_m, min, min
            ));
        }
    }
    Ok(())
}

/// `check_target` 返回的错误是否为「目标距离不足」，是则给出最短游览长度（米）。
///
/// 供上层做友好提示：`if let Some(min) = is_target_below_min_tour(&err) { ... }`。
pub fn is_target_below_min_tour(err: &str) -> Option<f64> {
    let rest = err.strip_prefix(TARGET_BELOW_MIN_TOUR_PREFIX)?;
    let start = rest.find("最短游览 ")? + "最短游览 ".len();
    let rest = &rest[start..];
    let end = rest.find(" 米")?;
    rest[..end].trim().parse().ok()
}

fn path_length(net: &RoadGraph, path: &[petgraph::graph::NodeIndex]) -> f64 {
    path.windows(2)
        .map(|w| {
            let a = net.to_m(net.node_coord(w[0]));
            let b = net.to_m(net.node_coord(w[1]));
            (a[0] - b[0]).hypot(a[1] - b[1])
        })
        .sum()
}

fn connected_components(net: &RoadGraph) -> usize {
    use petgraph::Direction;
    let mut seen = vec![false; net.graph.node_count()];
    let mut count = 0;
    for node in net.graph.node_indices() {
        if seen[node.index()] {
            continue;
        }
        count += 1;
        let mut stack = vec![node];
        seen[node.index()] = true;
        while let Some(cur) = stack.pop() {
            for next in net
                .graph
                .neighbors_directed(cur, Direction::Outgoing)
                .chain(net.graph.neighbors_directed(cur, Direction::Incoming))
            {
                if !seen[next.index()] {
                    seen[next.index()] = true;
                    stack.push(next);
                }
            }
        }
    }
    count
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::graph::EdgeData;

    fn line_net() -> RoadGraph {
        let mut g = RoadGraph::new();
        let a = g.add_node(Coord::new(0.0, 0.0));
        let b = g.add_node(Coord::new(0.001, 0.0));
        let c = g.add_node(Coord::new(0.002, 0.0));
        let edge = |way_id, length_m| EdgeData {
            way_id,
            oneway: false,
            highway: "footway".into(),
            maxspeed_kmh: None,
            length_m,
        };
        // 双向：闭环判定要求末点能回到起点，单向夹具无法闭合。
        g.add_edge(a, b, edge(1, 90.0));
        g.add_edge(b, a, edge(1, 90.0));
        g.add_edge(b, c, edge(2, 90.0));
        g.add_edge(c, b, edge(2, 90.0));
        g.compute_anchor();
        g.rebuild_index();
        g
    }

    #[test]
    fn reachable_checkpoints_report_a_minimum_loop() {
        let net = line_net();
        let check = inspect(&net, &[Coord::new(0.0, 0.0), Coord::new(0.002, 0.0)], &[]);
        assert!(check.ok(), "{:?}", check.issues);
        let min = check.minimum_m.expect("可闭合时应给出最短闭环");
        // 0 → 0.002° ≈ 222m 单程，往返 ≈ 445m（米制投影含纬度校正）。
        assert!(min > 400.0 && min < 500.0, "最短闭环 {min}");
        assert_eq!(check.components, 1);
    }

    /// 两端离路的中间点：必须用切分后的图判断可达，否则会误报"不连通"。
    #[test]
    fn midpoint_projection_is_reachable_through_the_split_graph() {
        let net = line_net();
        let check = inspect(&net, &[Coord::new(0.0, 0.0), Coord::new(0.001, 0.00005)], &[]);
        assert!(check.ok(), "中途投影点被误判为不连通: {:?}", check.issues);
        assert!(check.minimum_m.is_some());
    }

    #[test]
    fn disconnected_checkpoint_names_the_point_and_position() {
        let mut net = line_net();
        net.add_node(Coord::new(0.5, 0.5));
        net.rebuild_index();
        let check = inspect(&net, &[Coord::new(0.0, 0.0), Coord::new(0.5, 0.5)], &[]);
        let issue = check.issues.first().expect("应当报告问题");
        assert_eq!(issue.point, Some(1));
        assert_eq!(issue.position, Some(Coord::new(0.5, 0.5)));
        assert!(check.components >= 1);
    }

    #[test]
    fn target_below_minimum_loop_is_rejected() {
        let net = line_net();
        let check = inspect(&net, &[Coord::new(0.0, 0.0), Coord::new(0.002, 0.0)], &[]);
        assert!(check_target(&check, 100.0).is_err());
        assert!(check_target(&check, 1000.0).is_ok());
    }

    /// 边界：恰好等于最短游览放行，少 1 米即拒绝；错误消息带固定前缀，
    /// 且能解析出最短游览数值（上层据此做友好提示）。
    #[test]
    fn target_check_boundary_and_message_round_trip() {
        let net = line_net();
        let check = inspect(&net, &[Coord::new(0.0, 0.0), Coord::new(0.002, 0.0)], &[]);
        let min = check.minimum_m.expect("应给出最短闭环");
        assert!(check_target(&check, min).is_ok(), "目标 = 最短游览应放行");
        assert!(check_target(&check, min + 500.0).is_ok());

        let err = check_target(&check, min - 1.0).expect_err("目标 −1m 应拒绝");
        assert!(
            err.starts_with(TARGET_BELOW_MIN_TOUR_PREFIX),
            "错误消息前缀不符：{err}"
        );
        let parsed = is_target_below_min_tour(&err).expect("应能解析出最短游览");
        assert!(
            (parsed - min).abs() < 1.0,
            "解析值 {parsed} 与真实最短游览 {min} 不一致（消息：{err}）"
        );
        assert!(is_target_below_min_tour("其它错误").is_none());
    }

    #[test]
    fn point_outside_fence_is_reported_before_routing() {
        let net = line_net();
        let fence = vec![
            Coord::new(-0.01, -0.01),
            Coord::new(0.0015, -0.01),
            Coord::new(0.0015, 0.01),
            Coord::new(-0.01, 0.01),
        ];
        let check = inspect(&net, &[Coord::new(0.0, 0.0), Coord::new(0.002, 0.0)], &[fence]);
        let issue = check.issues.first().expect("越界点应被报告");
        assert_eq!(issue.point, Some(1));
        assert!(issue.message.contains("围栏"));
    }
}
