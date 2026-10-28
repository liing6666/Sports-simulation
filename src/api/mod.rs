//! API 层：HTTP 客户端 + 各业务接口 + GT4 验证 + 全链编排。

pub mod ai;
/// 跑步数据本地存档（轨迹 / 快照 / 诊断包）。
pub mod archive;
pub mod cheat;
pub mod client;
pub mod fence;
pub mod flow;
pub mod gt4;
pub mod gt4image;
pub mod login;
pub mod map_boundary;
pub mod map_search;
pub mod model;
pub mod obs;
pub mod points;
pub mod policy;
/// 多套配置档位。
pub mod profiles;
pub mod rank;
pub mod records;
pub mod school_roads;
pub mod semester;
/// 运动项目（校园跑/户外跑/室外锻炼）参数。
pub mod sport;
pub mod submission_guard;
pub mod submit;
pub mod user;
