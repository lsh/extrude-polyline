//! Extrude Polyline
//! This project is a port of Matt Deslauriers's [extrude-polyline](https://github.com/mattdesl/extrude-polyline) library.

#[derive(Debug, Clone, Copy, PartialEq, Hash)]
pub enum StrokeJoin {
    Miter,
    Bevel,
}

#[derive(Debug, Clone, Copy, PartialEq, Hash)]
pub enum StrokeCap {
    Butt,
    Square,
}

#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Stroke {
    pub miter_limit: f64,
    pub thickness: f64,
    pub join: StrokeJoin,
    pub cap: StrokeCap,
}

impl Default for Stroke {
    fn default() -> Self {
        Self {
            miter_limit: 10.0,
            thickness: 1.0,
            join: StrokeJoin::Miter,
            cap: StrokeCap::Butt,
        }
    }
}

#[derive(Debug, Default, Clone, PartialEq)]
pub struct Mesh {
    pub positions: Vec<[f64; 2]>,
    pub indices: Vec<[u32; 3]>,
}

#[derive(Debug, Copy, Clone, PartialEq)]
struct StrokeState {
    last_flip: f64,
    started: bool,
    normal: Option<[f64; 2]>,
}

impl Default for StrokeState {
    fn default() -> Self {
        Self {
            last_flip: -1.0,
            started: false,
            normal: None,
        }
    }
}

#[derive(Debug, Copy, Clone, PartialEq)]
struct SegArgs {
    index: u32,
    last: [f64; 2],
    cur: [f64; 2],
    next: Option<[f64; 2]>,
    half_thick: f64,
}

impl Stroke {
    pub fn thickness(mut self, thickness: f64) -> Self {
        self.thickness = thickness;
        self
    }

    pub fn miter_limit(mut self, miter_limit: f64) -> Self {
        self.miter_limit = miter_limit;
        self
    }

    pub fn join(mut self, join: StrokeJoin) -> Self {
        self.join = join;
        self
    }

    pub fn cap(mut self, cap: StrokeCap) -> Self {
        self.cap = cap;
        self
    }

    pub fn build(self, points: &[[f64; 2]]) -> Mesh {
        self.build_with_thickness_fn(points, |_, _, _| self.thickness)
    }

    pub fn build_with_thickness_fn(
        self,
        points: &[[f64; 2]],
        mut thickness_fn: impl FnMut([f64; 2], usize, &[[f64; 2]]) -> f64,
    ) -> Mesh {
        let mut complex = Mesh::default();

        if points.len() <= 1 {
            return complex;
        }

        let mut state = StrokeState::default();
        // join each segment
        let mut count = 0;
        for (i, pt) in points.windows(2).enumerate() {
            if let [last, current] = pt {
                let next = points.get(i + 2).copied();
                let thickness = thickness_fn(*current, i, points);
                let amt = self.seg(
                    &mut state,
                    &mut complex,
                    SegArgs {
                        index: count,
                        last: *last,
                        cur: *current,
                        next,
                        half_thick: thickness * 0.5,
                    },
                );
                count += amt;
            }
        }

        complex
    }

    fn seg(
        &self,
        state: &mut StrokeState,
        complex: &mut Mesh,
        SegArgs {
            index,
            mut last,
            mut cur,
            next,
            half_thick,
        }: SegArgs,
    ) -> u32 {
        let mut count = 0;
        let cells = &mut complex.indices;
        let positions = &mut complex.positions;
        let cap_square = matches!(self.cap, StrokeCap::Square);
        let join_bevel = matches!(self.join, StrokeJoin::Bevel);

        // get unit direction of line
        let line_a = direction(cur, last);

        // if we don't yet have a normal from previous join,
        // compute based on line start - end
        // note that normal is now always defined so we can unwrap it
        if state.normal.is_none() {
            state.normal = Some(normal(line_a));
        }

        // if we haven't started yet, add the first two points
        if !state.started {
            state.started = true;

            // if the end cap is type square, we can just push the verts out a bit
            if cap_square {
                last = scale_and_add(last, line_a, -half_thick);
            }
            positions.extend_from_slice(&extrusions(last, state.normal.unwrap(), half_thick));
        }

        cells.push([index, index + 1, index + 2]);

        /*
        // now determine the type of join with next segment

        - round (TODO)
        - bevel
        - miter
        - none (i.e. no next segment, use normal)
         */
        if let Some(next) = next {
            // we have a next segment, start with miter
            // get unit dir of next line
            let line_b = direction(next, cur);

            // stores tangent & miter
            let (tangent, miter, miter_len) = compute_miter(line_a, line_b, half_thick);

            // get orientation
            let mut flip = if dot(tangent, state.normal.unwrap()) < 0.0 {
                -1.0
            } else {
                1.0
            };

            let mut bevel = join_bevel;
            if matches!(self.join, StrokeJoin::Miter) {
                let limit = miter_len / half_thick;
                if limit > self.miter_limit {
                    bevel = true;
                }
            }

            if bevel {
                // next two points in our first segment
                let tmp = scale_and_add(cur, state.normal.unwrap(), -half_thick * flip);
                positions.push(tmp);
                let tmp = scale_and_add(cur, miter, miter_len * flip);
                positions.push(tmp);

                cells.push(if state.last_flip != -flip {
                    [index, index + 2, index + 3]
                } else {
                    [index + 2, index + 1, index + 3]
                });
                cells.push([index + 2, index + 3, index + 4]);

                let tmp = normal(line_b);
                state.normal = Some(tmp); // store normal for next round

                let tmp = scale_and_add(cur, tmp, -half_thick * flip);
                positions.push(tmp);

                // the miter is now the normal for our next join
                count += 3;
            } else {
                // miter
                // next two points for our miter join
                positions.extend_from_slice(&extrusions(cur, miter, miter_len));

                cells.push(if state.last_flip == 1.0 {
                    [index, index + 2, index + 3]
                } else {
                    [index + 2, index + 1, index + 3]
                });

                flip = -1.0;

                // the miter is now the normal for our next join
                state.normal = Some(miter);
                count += 2
            }
            state.last_flip = flip;
        } else {
            // no next segment, simple extrusion
            // now reset normal to finish cap
            state.normal = Some(normal(line_a));

            // push square end cap out a bit
            if cap_square {
                cur = scale_and_add(cur, line_a, half_thick);
            }

            positions.extend_from_slice(&extrusions(cur, state.normal.unwrap(), half_thick));
            cells.push(if state.last_flip == 1.0 {
                [index, index + 2, index + 3]
            } else {
                [index + 2, index + 1, index + 3]
            });

            count += 2;
        }
        count
    }
}

fn extrusions(point: [f64; 2], normal: [f64; 2], scale: f64) -> [[f64; 2]; 2] {
    [
        scale_and_add(point, normal, -scale),
        scale_and_add(point, normal, scale),
    ]
}

fn scale_and_add(a: [f64; 2], b: [f64; 2], scale: f64) -> [f64; 2] {
    [a[0] + b[0] * scale, a[1] + b[1] * scale]
}

fn subtract(a: [f64; 2], b: [f64; 2]) -> [f64; 2] {
    [a[0] - b[0], a[1] - b[1]]
}

fn add(a: [f64; 2], b: [f64; 2]) -> [f64; 2] {
    [a[0] + b[0], a[1] + b[1]]
}

fn normal(dir: [f64; 2]) -> [f64; 2] {
    [-dir[1], dir[0]]
}

fn direction(a: [f64; 2], b: [f64; 2]) -> [f64; 2] {
    normalize(subtract(a, b))
}

fn compute_miter(line_a: [f64; 2], line_b: [f64; 2], half_thick: f64) -> ([f64; 2], [f64; 2], f64) {
    // get tangent line
    let tangent = normalize(add(line_a, line_b));

    // get miter as a unit vector
    let miter = [-tangent[1], tangent[0]];
    let tmp = [-line_a[1], line_a[0]];

    // get the necessary length of our miter
    (tangent, miter, half_thick / dot(miter, tmp))
}

fn normalize(a: [f64; 2]) -> [f64; 2] {
    let [x, y] = a;
    let len = x * x + y * y;
    let len = if len > 0.0 { 1.0 / len.sqrt() } else { len };
    [a[0] * len, a[1] * len]
}

fn dot(a: [f64; 2], b: [f64; 2]) -> f64 {
    a[0] * b[0] + a[1] * b[1]
}

#[cfg(test)]
mod tests {
    use super::{Mesh, Stroke};

    #[test]
    fn should_extrude_path_two_points() {
        let mesh = Stroke::default()
            .thickness(2.5)
            .build(&[[2.0, 0.0], [2.0, 10.0]]);

        assert_eq!(
            mesh,
            Mesh {
                positions: vec![[3.25, 0.0], [0.75, 0.0], [3.25, 10.0], [0.75, 10.0]],
                indices: vec![[0, 1, 2], [2, 1, 3]]
            }
        )
    }

    #[test]
    fn should_extrude_path_three_points() {
        let mesh = Stroke::default()
            .thickness(1.0)
            .build(&[[0.0, 0.0], [1.0, 1.0], [2.0, 0.0]]);

        assert_eq!(
            mesh,
            Mesh {
                positions: vec![
                    [0.35355339059327373, -0.35355339059327373],
                    [-0.35355339059327373, 0.35355339059327373],
                    [1.0, 0.2928932188134524],
                    [1.0, 1.7071067811865475],
                    [1.6464466094067263, -0.35355339059327373],
                    [2.353553390593274, 0.35355339059327373]
                ],
                indices: vec![[0, 1, 2], [2, 1, 3], [2, 3, 4], [4, 3, 5]]
            }
        )
    }
}
