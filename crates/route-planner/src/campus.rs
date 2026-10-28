//! Campus areas are independent of roads and never inferred from a POI or building.
use crate::Coord;
use geo::{Area, Contains, Intersects, Line, LineString, Polygon, TriangulateEarcut};
use quick_xml::{events::Event, Reader};
use std::collections::HashMap;

#[derive(Clone, Debug, serde::Serialize, serde::Deserialize)]
pub struct Campus {
    pub id: String,
    pub name: String,
    pub outer: Vec<Vec<Coord>>,
    pub inner: Vec<Vec<Coord>>,
}

fn line(r: &[Coord]) -> LineString<f64> {
    let mut p: Vec<_> = r.iter().map(|c| (c.lon, c.lat)).collect();
    if p.first() != p.last() {
        p.push(p[0]);
    }
    LineString::from(p)
}

fn valid_ring(r: &[Coord]) -> bool {
    if r.len() < 4
        || r.first() != r.last()
        || r.len() > 10000
        || r.iter().any(|c| {
            !c.lat.is_finite() || !c.lon.is_finite() || c.lat.abs() > 85. || c.lon.abs() > 180.
        })
    {
        return false;
    }
    let ls = line(r);
    let edges: Vec<Line<f64>> = ls.lines().collect();
    for (i, a) in edges.iter().enumerate() {
        if a.start == a.end {
            return false;
        }
        for (j, b) in edges.iter().enumerate().skip(i + 2) {
            if i == 0 && j == edges.len() - 1 {
                continue;
            }
            if a.intersects(b) {
                return false;
            }
        }
    }
    Polygon::new(ls, vec![]).unsigned_area() > 1e-12
}

impl Campus {
    /// Triangulation preserves holes while reusing whole-segment union checks.
    pub fn triangles(&self) -> Result<Vec<Vec<Coord>>, String> {
        if self.outer.is_empty() || self.outer.iter().chain(&self.inner).any(|r| !valid_ring(r)) {
            return Err("校园轮廓不闭合或几何无效".into());
        }
        let mut polys: Vec<_> = self
            .outer
            .iter()
            .map(|r| Polygon::new(line(r), vec![]))
            .collect();
        for i in 0..polys.len() {
            for j in i + 1..polys.len() {
                if polys[i].intersects(&polys[j]) {
                    return Err("校园外环重叠".into());
                }
            }
        }
        let holes: Vec<_> = self
            .inner
            .iter()
            .map(|r| Polygon::new(line(r), vec![]))
            .collect();
        for i in 0..holes.len() {
            for j in i + 1..holes.len() {
                if holes[i].intersects(&holes[j]) {
                    return Err("校园排除区域重叠".into());
                }
            }
        }
        for hole in holes {
            let Some(parent) = polys
                .iter_mut()
                .find(|p| p.contains(&hole) && !p.exterior().intersects(hole.exterior()))
            else {
                return Err("校园排除区域不在外环内".into());
            };
            parent.interiors_push(hole.exterior().clone());
        }
        let expected: f64 = polys.iter().map(Area::unsigned_area).sum();
        let triangles: Vec<_> = polys.iter().flat_map(|p| p.earcut_triangles()).collect();
        let actual: f64 = triangles
            .iter()
            .map(|t| {
                let [a, b, c] = t.to_array();
                ((b.x - a.x) * (c.y - a.y) - (c.x - a.x) * (b.y - a.y)).abs() * 0.5
            })
            .sum();
        if (expected - actual).abs() > expected * 1e-7 {
            return Err("校园区域分解失败".into());
        }
        Ok(triangles
            .iter()
            .map(|t| t.to_array().iter().map(|p| Coord::new(p.x, p.y)).collect())
            .collect())
    }
}

/// 多边形面积（平方米，鞋带公式 + 米制近似）。
fn polygon_area(ring: &[Coord]) -> f64 {
    if ring.len() < 3 {
        return 0.0;
    }
    let lat0 = ring[0].lat;
    let m_lat = 111_132.0;
    let m_lon = 111_320.0 * lat0.to_radians().cos();
    let mut acc = 0.0;
    for i in 0..ring.len() {
        let a = ring[i];
        let b = ring[(i + 1) % ring.len()];
        acc += (a.lon * m_lon) * (b.lat * m_lat) - (b.lon * m_lon) * (a.lat * m_lat);
    }
    (acc / 2.0).abs()
}

fn rings(
    ids: &[u64],
    ways: &HashMap<u64, Vec<u64>>,
    nodes: &HashMap<u64, Coord>,
) -> Option<Vec<Vec<Coord>>> {
    let mut parts: Vec<Vec<u64>> = ids
        .iter()
        .map(|id| ways.get(id).cloned())
        .collect::<Option<_>>()?;
    let mut result = Vec::new();
    while let Some(mut ring) = parts.pop() {
        while ring.first() != ring.last() {
            let tail = *ring.last()?;
            let i = parts
                .iter()
                .position(|p| p.first() == Some(&tail) || p.last() == Some(&tail))?;
            let mut next = parts.remove(i);
            if next.first() != Some(&tail) {
                next.reverse();
            }
            ring.extend(next.into_iter().skip(1));
        }
        result.push(
            ring.iter()
                .map(|id| nodes.get(id).copied())
                .collect::<Option<_>>()?,
        );
    }
    Some(result)
}

pub fn parse(bytes: &[u8]) -> Result<Vec<Campus>, String> {
    if bytes.len() > 24 * 1024 * 1024 {
        return Err("校园地图超过 24 MiB".into());
    }
    let text = std::str::from_utf8(bytes).map_err(|_| "地图编码无效")?;
    if !text.contains("</osm>") || text.contains("<remark") {
        return Err("校园地图响应不完整".into());
    }
    let mut reader = Reader::from_reader(bytes);
    let mut nodes = HashMap::new();
    let mut ways = HashMap::new();
    let mut candidates = Vec::new();
    let mut kind = String::new();
    let mut id = 0;
    let mut refs = Vec::new();
    let mut members = Vec::new();
    let mut tags = HashMap::new();
    loop {
        match reader.read_event().map_err(|e| e.to_string())? {
            Event::Start(e) | Event::Empty(e) => {
                let mut a = HashMap::new();
                for attr in e.attributes() {
                    let attr = attr.map_err(|e| e.to_string())?;
                    a.insert(
                        String::from_utf8_lossy(attr.key.as_ref()).into_owned(),
                        attr.decode_and_unescape_value(reader.decoder())
                            .map_err(|e| e.to_string())?
                            .into_owned(),
                    );
                }
                let get = |k: &str| a.get(k).map(String::as_str).unwrap_or("");
                match e.name().as_ref() {
                    b"node" => {
                        if let (Ok(id), Ok(lat), Ok(lon)) = (
                            get("id").parse::<u64>(),
                            get("lat").parse::<f64>(),
                            get("lon").parse::<f64>(),
                        ) {
                            nodes.insert(id, Coord::new(lon, lat));
                        }
                    }
                    b"way" | b"relation" => {
                        kind = String::from_utf8_lossy(e.name().as_ref()).into_owned();
                        id = get("id").parse().unwrap_or(0);
                        refs.clear();
                        members.clear();
                        tags.clear();
                    }
                    b"nd" if kind == "way" => {
                        refs.push(get("ref").parse::<u64>().unwrap_or(0));
                    }
                    b"member" if kind == "relation" => {
                        members.push((
                            get("type").to_owned(),
                            get("ref").parse::<u64>().unwrap_or(0),
                            get("role").to_owned(),
                        ));
                    }
                    b"tag" if !kind.is_empty() => {
                        tags.insert(get("k").to_owned(), get("v").to_owned());
                    }
                    _ => {}
                }
            }
            Event::End(e) if e.name().as_ref() == b"way" || e.name().as_ref() == b"relation" => {
                if kind == "way" {
                    ways.insert(id, refs.clone());
                }
                let educational = tags
                    .get("amenity")
                    .is_some_and(|v| matches!(v.as_str(), "university" | "college" | "school"))
                    || tags.get("landuse").is_some_and(|v| v == "education");
                let building = tags.get("building").is_some_and(|v| v != "no");
                if educational && !building && id > 0 {
                    let name = tags
                        .get("name:zh")
                        .or_else(|| tags.get("name"))
                        .or_else(|| tags.get("operator"))
                        .cloned()
                        .unwrap_or_else(|| "未命名校园".into());
                    if kind == "way" {
                        candidates.push((format!("way/{id}"), name, vec![id], vec![]));
                    } else if tags.get("type").is_some_and(|v| v == "multipolygon")
                        && members
                            .iter()
                            .all(|m| m.0 == "way" && matches!(m.2.as_str(), "outer" | "inner" | ""))
                    {
                        candidates.push((
                            format!("relation/{id}"),
                            name,
                            members
                                .iter()
                                .filter(|m| m.2 != "inner")
                                .map(|m| m.1)
                                .collect(),
                            members
                                .iter()
                                .filter(|m| m.2 == "inner")
                                .map(|m| m.1)
                                .collect(),
                        ));
                    }
                }
                kind.clear();
            }
            Event::Eof => break,
            _ => {}
        }
    }
    let mut out = Vec::new();
    for (id, name, outer, inner) in candidates {
        if let (Some(outer), Some(inner)) =
            (rings(&outer, &ways, &nodes), rings(&inner, &ways, &nodes))
        {
            let c = Campus {
                id,
                name,
                outer,
                inner,
            };
            if c.triangles().is_ok() {
                out.push(c);
            }
        }
    }
    out.sort_by(|a, b| a.id.cmp(&b.id));
    Ok(out)
}

/// 宽松解析：**不检查标签**，把范围内每个闭合多边形都当成候选校园边界。
///
/// 用于「按坐标范围查找」——用户在 OSM 上新画了边界但没打 `amenity`/
/// `landuse=education` 标签时，严格的 [`parse`] 找不到它。这里只看几何：
/// 闭合的 way、或 outer 全部闭合的 multipolygon relation。
///
/// 命名优先取 `name`/`name:zh`，没有就用 `OSM way/<id>`，方便用户在列表里区分。
pub fn parse_loose(bytes: &[u8], focus: Coord, max_dist_m: f64) -> Result<Vec<Campus>, String> {
    if bytes.len() > 24 * 1024 * 1024 {
        return Err("校园地图超过 24 MiB".into());
    }
    let text = std::str::from_utf8(bytes).map_err(|_| "地图编码无效")?;
    if !text.contains("</osm>") || text.contains("<remark") {
        return Err("校园地图响应不完整".into());
    }
    let mut reader = Reader::from_reader(bytes);
    let mut nodes = HashMap::new();
    let mut ways: HashMap<u64, Vec<u64>> = HashMap::new();
    let mut candidates: Vec<(String, String, Vec<u64>, Vec<u64>)> = Vec::new();
    let mut kind = String::new();
    let mut id = 0u64;
    let mut refs: Vec<u64> = Vec::new();
    let mut members: Vec<(String, u64, String)> = Vec::new();
    let mut tags = HashMap::new();
    loop {
        match reader.read_event().map_err(|e| e.to_string())? {
            Event::Start(e) | Event::Empty(e) => {
                let mut a = HashMap::new();
                for attr in e.attributes() {
                    let attr = attr.map_err(|e| e.to_string())?;
                    a.insert(
                        String::from_utf8_lossy(attr.key.as_ref()).into_owned(),
                        attr.decode_and_unescape_value(reader.decoder())
                            .map_err(|e| e.to_string())?
                            .into_owned(),
                    );
                }
                let get = |k: &str| a.get(k).map(String::as_str).unwrap_or("");
                match e.name().as_ref() {
                    b"node" => {
                        if let (Ok(id), Ok(lat), Ok(lon)) = (
                            get("id").parse::<u64>(),
                            get("lat").parse::<f64>(),
                            get("lon").parse::<f64>(),
                        ) {
                            nodes.insert(id, Coord::new(lon, lat));
                        }
                    }
                    b"way" | b"relation" => {
                        kind = String::from_utf8_lossy(e.name().as_ref()).into_owned();
                        id = get("id").parse().unwrap_or(0);
                        refs.clear();
                        members.clear();
                        tags.clear();
                    }
                    b"nd" if kind == "way" => {
                        refs.push(get("ref").parse::<u64>().unwrap_or(0));
                    }
                    b"member" if kind == "relation" => {
                        members.push((
                            get("type").to_owned(),
                            get("ref").parse::<u64>().unwrap_or(0),
                            get("role").to_owned(),
                        ));
                    }
                    b"tag" if !kind.is_empty() => {
                        tags.insert(get("k").to_owned(), get("v").to_owned());
                    }
                    _ => {}
                }
            }
            Event::End(e) if e.name().as_ref() == b"way" || e.name().as_ref() == b"relation" => {
                if kind == "way" {
                    ways.insert(id, refs.clone());
                }
                if id > 0 {
                    let name = tags
                        .get("name:zh")
                        .or_else(|| tags.get("name"))
                        .or_else(|| tags.get("official_name"))
                        .or_else(|| tags.get("operator"))
                        .cloned()
                        .unwrap_or_else(|| format!("OSM {kind}/{id}"));
                    if kind == "way" && refs.len() >= 4 && refs.first() == refs.last() {
                        candidates.push((format!("way/{id}"), name, vec![id], vec![]));
                    } else if kind == "relation" {
                        // 只接受 outer 成员齐全、且能拼成闭合环的 multipolygon。
                        let outers: Vec<u64> = members
                            .iter()
                            .filter(|m| m.0 == "way" && m.2 != "inner")
                            .map(|m| m.1)
                            .collect();
                        let inners: Vec<u64> = members
                            .iter()
                            .filter(|m| m.0 == "way" && m.2 == "inner")
                            .map(|m| m.1)
                            .collect();
                        if !outers.is_empty() {
                            candidates.push((
                                format!("relation/{id}"),
                                name,
                                outers,
                                inners,
                            ));
                        }
                    }
                }
                kind.clear();
            }
            Event::Eof => break,
            _ => {}
        }
    }

    let mut out = Vec::new();
    for (id, name, outer, inner) in candidates {
        let Some(outer) = rings(&outer, &ways, &nodes) else {
            continue;
        };
        let inner = rings(&inner, &ways, &nodes).unwrap_or_default();
        let c = Campus {
            id,
            name,
            outer,
            inner,
        };
        // 只保留「几何上像校园」的：能三角化、面积不太小、且离焦点不太远。
        if c.triangles().is_err() {
            continue;
        }
        let Some(first) = c.outer.first().and_then(|r| r.first()).copied() else {
            continue;
        };
        // 以焦点为原点做平面近似，判断包围盒中心距离。
        let m_lat = 111_132.0;
        let m_lon = 111_320.0 * focus.lat.to_radians().cos();
        let (mut minx, mut maxx, mut miny, mut maxy) =
            (f64::INFINITY, f64::NEG_INFINITY, f64::INFINITY, f64::NEG_INFINITY);
        for ring in c.outer.iter() {
            for p in ring {
                let x = (p.lon - focus.lon) * m_lon;
                let y = (p.lat - focus.lat) * m_lat;
                minx = minx.min(x);
                maxx = maxx.max(x);
                miny = miny.min(y);
                maxy = maxy.max(y);
            }
        }
        if !minx.is_finite() {
            continue;
        }
        let dist = (((minx + maxx) / 2.0).hypot((miny + maxy) / 2.0)).max(
            // 焦点若落在多边形内部，距离记为 0。
            0.0,
        );
        let near = dist <= max_dist_m
            || crate::boundary::contains(&c.outer, focus);
        if !near {
            continue;
        }
        let _ = first;
        out.push(c);
    }
    // 面积大的排前面：校园边界通常比单个建筑大，用户更容易一眼认出。
    let area = |c: &Campus| -> f64 {
        let mut total = 0.0;
        for ring in &c.outer {
            total += polygon_area(ring);
        }
        total
    };
    out.sort_by(|a, b| {
        area(b)
            .partial_cmp(&area(a))
            .unwrap_or(std::cmp::Ordering::Equal)
            .then(a.id.cmp(&b.id))
    });
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;
    fn square(a: f64, b: f64) -> Vec<Coord> {
        vec![
            Coord::new(a, a),
            Coord::new(b, a),
            Coord::new(b, b),
            Coord::new(a, b),
            Coord::new(a, a),
        ]
    }
    /// 宽松解析：没有 amenity/landuse 标签的闭合多边形也必须被识别。
    #[test]
    fn loose_parse_accepts_untagged_closed_polygon() {
        // 一个没有教育标签的方形，中心在焦点附近。
        let osm = r#"<?xml version="1.0"?>
<osm version="0.6">
  <node id="1" lat="39.9000" lon="116.4000"/>
  <node id="2" lat="39.9000" lon="116.4100"/>
  <node id="3" lat="39.9100" lon="116.4100"/>
  <node id="4" lat="39.9100" lon="116.4000"/>
  <way id="100"><nd ref="1"/><nd ref="2"/><nd ref="3"/><nd ref="4"/><nd ref="1"/>
    <tag k="barrier" v="wall"/></way>
</osm>"#;
        let focus = Coord::new(116.405, 39.905);
        let items = parse_loose(osm.as_bytes(), focus, 800.0).expect("宽松解析应成功");
        assert_eq!(items.len(), 1, "未打教育标签的闭合多边形应被识别");
        assert_eq!(items[0].id, "way/100");
        assert!(items[0].triangles().is_ok());
    }

    /// 距离焦点太远的多边形不应被收进来。
    #[test]
    fn loose_parse_rejects_far_away_polygons() {
        let osm = r#"<?xml version="1.0"?>
<osm version="0.6">
  <node id="1" lat="39.9000" lon="116.4000"/>
  <node id="2" lat="39.9000" lon="116.4100"/>
  <node id="3" lat="39.9100" lon="116.4100"/>
  <node id="4" lat="39.9100" lon="116.4000"/>
  <way id="100"><nd ref="1"/><nd ref="2"/><nd ref="3"/><nd ref="4"/><nd ref="1"/></way>
</osm>"#;
        // 焦点在 10 公里外。
        let focus = Coord::new(116.500, 39.905);
        let items = parse_loose(osm.as_bytes(), focus, 800.0).unwrap();
        assert!(items.is_empty(), "远处多边形不应入选");
    }

    /// 未闭合的 way 不应被当成边界。
    #[test]
    fn loose_parse_ignores_open_ways() {
        let osm = r#"<?xml version="1.0"?>
<osm version="0.6">
  <node id="1" lat="39.9000" lon="116.4000"/>
  <node id="2" lat="39.9000" lon="116.4100"/>
  <node id="3" lat="39.9100" lon="116.4100"/>
  <way id="100"><nd ref="1"/><nd ref="2"/><nd ref="3"/></way>
</osm>"#;
        let focus = Coord::new(116.405, 39.905);
        let items = parse_loose(osm.as_bytes(), focus, 800.0).unwrap();
        assert!(items.is_empty(), "未闭合的 way 不应入选");
    }

    #[test]
    fn holes_and_crossings_survive_triangulation() {
        let c = Campus {
            id: "x".into(),
            name: "x".into(),
            outer: vec![square(0., 4.)],
            inner: vec![square(1., 2.)],
        };
        let t = c.triangles().unwrap();
        assert!(crate::boundary::contains(&t, Coord::new(0.5, 1.5)));
        assert!(!crate::boundary::contains(&t, Coord::new(1.5, 1.5)));
        assert!(!crate::boundary::segment_inside_union(
            &t,
            Coord::new(0.5, 1.5),
            Coord::new(3., 1.5)
        ));
        assert!(crate::boundary::segment_inside_union(
            &t,
            Coord::new(0.5, 0.5),
            Coord::new(3., 0.5)
        ));
    }
    #[test]
    fn invalid_regions_are_rejected() {
        let mut c = Campus {
            id: "x".into(),
            name: "x".into(),
            outer: vec![square(0., 4.)],
            inner: vec![square(3., 5.)],
        };
        assert!(c.triangles().is_err());
        c.inner.clear();
        c.outer[0].pop();
        assert!(c.triangles().is_err());
    }
}

#[cfg(test)]
mod parser_tests {
    use super::*;
    const XML: &str = r#"<osm><node id="1" lon="1" lat="1"/><node id="2" lon="3" lat="1"/><node id="3" lon="3" lat="3"/><node id="4" lon="1" lat="3"/><node id="5" lon="1.5" lat="1.5"/><node id="6" lon="2" lat="1.5"/><node id="7" lon="2" lat="2"/><node id="8" lon="1.5" lat="2"/><way id="10"><nd ref="1"/><nd ref="2"/><nd ref="3"/></way><way id="11"><nd ref="1"/><nd ref="4"/><nd ref="3"/></way><way id="12"><nd ref="5"/><nd ref="6"/><nd ref="7"/><nd ref="8"/><nd ref="5"/></way><relation id="20"><member type="way" ref="10" role="outer"/><member type="way" ref="11" role="outer"/><member type="way" ref="12" role="inner"/><tag k="type" v="multipolygon"/><tag k="amenity" v="college"/><tag k="name" v="A &amp; B"/></relation></osm>"#;
    #[test]
    fn stitches_reversed_members_and_preserves_holes() {
        let c = parse(XML.as_bytes()).unwrap();
        assert_eq!(c.len(), 1);
        assert_eq!(c[0].name, "A & B");
        assert_eq!(c[0].outer.len(), 1);
        assert_eq!(c[0].inner.len(), 1);
        assert!(!crate::boundary::contains(
            &c[0].triangles().unwrap(),
            Coord::new(1.75, 1.75)
        ));
    }
    #[test]
    fn missing_members_open_rings_and_buildings_are_not_campuses() {
        assert!(parse(
            XML.replace("ref=\"11\" role", "ref=\"999\" role")
                .as_bytes()
        )
        .unwrap()
        .is_empty());
        assert!(parse(
            XML.replace(
                "<tag k=\"amenity\"",
                "<tag k=\"building\" v=\"yes\"/><tag k=\"amenity\""
            )
            .as_bytes()
        )
        .unwrap()
        .is_empty());
        assert!(parse(b"<osm><node id=\"1\" lat=\"1\" lon=\"1\"><tag k=\"amenity\" v=\"university\"/></node></osm>").unwrap().is_empty());
        assert!(parse(b"<osm>").is_err());
    }
}

#[cfg(test)]
mod real_map_tests {
    #[test]
    fn chengdu_neusoft_public_boundaries_parse_as_separate_candidates() {
        let campuses =
            super::parse(include_bytes!("../tests/fixtures/chengdu-neusoft.osm")).unwrap();
        assert_eq!(campuses.len(), 2);
        for c in campuses {
            assert!(!c.triangles().unwrap().is_empty());
            assert_eq!(c.name, "成都东软学院");
        }
    }
}
