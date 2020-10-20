/// # Resources
///
/// Inspired by Randy Gaul's qu3e engine
/// [https://github.com/RandyGaul/qu3e/blob/master/src/collision/q3Collide.cpp](qu3e/q3Collide.cpp)
use bevy::math::*;
use bevy::prelude::*;
use smallvec::{smallvec, SmallVec};

use super::*;

trait Mult {
    fn mult(&self, v: Vec3) -> Vec3;
}

impl Mult for Transform {
    fn mult(&self, v: Vec3) -> Vec3 {
        self.rotation.conjugate() * (v - self.translation)
    }
}

impl Mult for Quat {
    fn mult(&self, v: Vec3) -> Vec3 {
        self.conjugate() * v
    }
}

impl Mult for Mat3 {
    fn mult(&self, v: Vec3) -> Vec3 {
        self.transpose() * v
    }
}

trait Mat3Ext {
    fn column0(&self) -> Vec3;
    fn column1(&self) -> Vec3;
    fn column2(&self) -> Vec3;
    fn row0(&self) -> Vec3;
    fn row1(&self) -> Vec3;
    fn row2(&self) -> Vec3;
}

impl Mat3Ext for Mat3 {
    fn column0(&self) -> Vec3 {
        Vec3::from(self.to_cols_array_2d()[0])
    }

    fn column1(&self) -> Vec3 {
        Vec3::from(self.to_cols_array_2d()[1])
    }

    fn column2(&self) -> Vec3 {
        Vec3::from(self.to_cols_array_2d()[2])
    }

    fn row0(&self) -> Vec3 {
        self.transpose().column0()
    }

    fn row1(&self) -> Vec3 {
        self.transpose().column1()
    }

    fn row2(&self) -> Vec3 {
        self.transpose().column2()
    }
}

trait Mat4Ext {
    fn truncate(&self) -> Mat3;
}

impl Mat4Ext for Mat4 {
    fn truncate(&self) -> Mat3 {
        Mat3::from_cols(
            self.x_axis().truncate().into(),
            self.y_axis().truncate().into(),
            self.z_axis().truncate().into(),
        )
    }
}

enum TrackFaceAxis {
    None,
    Some { axis: u32, max: f32, normal: Vec3 },
    Yes,
}

fn track_face_axis(n: u32, s: f32, smax: f32, normal: Vec3) -> TrackFaceAxis {
    if s > 0.0 {
        return TrackFaceAxis::None;
    }

    if s > smax {
        let max = s;
        let axis = n;
        return TrackFaceAxis::Some { max, axis, normal };
    }

    TrackFaceAxis::Yes
}

enum TrackEdgeAxis {
    None,
    Some { axis: u32, max: f32, normal: Vec3 },
    Yes,
}

fn track_edge_axis(n: u32, mut s: f32, smax: f32, normal: Vec3) -> TrackEdgeAxis {
    if s > 0.0 {
        return TrackEdgeAxis::None;
    }

    let l = normal.length_recip();
    s *= l;

    if s > smax {
        let max = s;
        let axis = n;
        let normal = normal * l;
        return TrackEdgeAxis::Some { max, axis, normal };
    }

    TrackEdgeAxis::Yes
}

fn compute_incident_face(itx: Transform, e: Vec3, n: Vec3) -> [Vec3; 4] {
    let n = -itx.rotation.mult(n);
    let absn = n.abs();
    let itx_matrix = itx.compute_matrix();
    if absn.x() > absn.y() && absn.x() > absn.z() {
        if n.x() > 0.0 {
            [
                itx_matrix.transform_point3(Vec3::new(e.x(), e.y(), -e.z())),
                itx_matrix.transform_point3(Vec3::new(e.x(), e.y(), e.z())),
                itx_matrix.transform_point3(Vec3::new(e.x(), -e.y(), e.z())),
                itx_matrix.transform_point3(Vec3::new(e.x(), -e.y(), -e.z())),
            ]
        } else {
            [
                itx_matrix.transform_point3(Vec3::new(-e.x(), -e.y(), e.z())),
                itx_matrix.transform_point3(Vec3::new(-e.x(), e.y(), e.z())),
                itx_matrix.transform_point3(Vec3::new(-e.x(), e.y(), -e.z())),
                itx_matrix.transform_point3(Vec3::new(-e.x(), -e.y(), -e.z())),
            ]
        }
    } else if absn.y() > absn.x() && absn.y() > absn.z() {
        if n.y() > 0.0 {
            [
                itx_matrix.transform_point3(Vec3::new(-e.x(), e.y(), e.z())),
                itx_matrix.transform_point3(Vec3::new(e.x(), e.y(), e.z())),
                itx_matrix.transform_point3(Vec3::new(e.x(), e.y(), -e.z())),
                itx_matrix.transform_point3(Vec3::new(-e.x(), e.y(), -e.z())),
            ]
        } else {
            [
                itx_matrix.transform_point3(Vec3::new(e.x(), -e.y(), e.z())),
                itx_matrix.transform_point3(Vec3::new(-e.x(), -e.y(), e.z())),
                itx_matrix.transform_point3(Vec3::new(-e.x(), -e.y(), -e.z())),
                itx_matrix.transform_point3(Vec3::new(e.x(), -e.y(), -e.z())),
            ]
        }
    } else {
        if n.z() > 0.0 {
            [
                itx_matrix.transform_point3(Vec3::new(-e.x(), e.y(), e.z())),
                itx_matrix.transform_point3(Vec3::new(-e.x(), -e.y(), e.z())),
                itx_matrix.transform_point3(Vec3::new(e.x(), -e.y(), e.z())),
                itx_matrix.transform_point3(Vec3::new(e.x(), e.y(), e.z())),
            ]
        } else {
            [
                itx_matrix.transform_point3(Vec3::new(e.x(), -e.y(), -e.z())),
                itx_matrix.transform_point3(Vec3::new(-e.x(), -e.y(), -e.z())),
                itx_matrix.transform_point3(Vec3::new(-e.x(), e.y(), -e.z())),
                itx_matrix.transform_point3(Vec3::new(e.x(), e.y(), -e.z())),
            ]
        }
    }
}

struct RefEb {
    basis: Mat3,
    e: Vec3,
}

fn compute_reference_edges_and_basis(er: Vec3, rtx: Transform, n: Vec3, mut axis: u32) -> RefEb {
    let n = rtx.rotation.mult(n);

    if axis >= 3 {
        axis -= 3;
    }

    let rot = rtx.compute_matrix().truncate();
    match axis {
        0 => {
            if n.x() > 0.0 {
                let e = Vec3::new(er.z(), er.y(), er.x());
                let basis = rot * Mat3::from_quat(Quat::from_rotation_y(90.0_f32.to_radians()));
                RefEb { basis, e }
            } else {
                let e = Vec3::new(er.z(), er.y(), er.x());
                let basis = rot * Mat3::from_quat(Quat::from_rotation_y(90.0_f32.to_radians()));
                RefEb { basis, e }
            }
        }
        1 => {
            if n.y() > 0.0 {
                let e = Vec3::new(er.x(), er.z(), er.y());
                let basis = rot * Mat3::from_quat(Quat::from_rotation_x(90.0_f32.to_radians()));
                RefEb { basis, e }
            } else {
                let e = Vec3::new(er.x(), er.z(), er.y());
                let basis = rot * Mat3::from_quat(Quat::from_rotation_x(90.0_f32.to_radians()));
                RefEb { basis, e }
            }
        }
        2 => {
            if n.z() > 0.0 {
                let e = Vec3::new(er.x(), er.y(), er.z());
                let basis = rot;
                RefEb { basis, e }
            } else {
                let e = Vec3::new(er.x(), er.y(), er.z());
                let basis = rot;
                RefEb { basis, e }
            }
        }
        _ => unimplemented!(),
    }
}

fn orthographic(sign: f32, e: f32, axis: u32, vin: &[Vec3]) -> SmallVec<[Vec3; 8]> {
    fn in_front(a: f32) -> bool {
        a < 0.0
    }

    fn behind(a: f32) -> bool {
        a >= 0.0
    }

    fn on(a: f32) -> bool {
        a < 0.005 && a > -0.005
    }

    let mut out = SmallVec::new();
    let mut a = *vin.last().unwrap();

    for &b in vin {
        let da = sign * a[axis as usize] - e;
        let db = sign * b[axis as usize] - e;

        if in_front(da) && in_front(db) || on(da) || on(db) {
            debug_assert!(out.len() < 8);
            out.push(b);
        } else if in_front(da) && behind(db) {
            let cv = a + (b - a) * (da / (da - db));
            debug_assert!(out.len() < 8);
            out.push(cv);
        } else if behind(da) && in_front(db) {
            let cv = a + (b - a) * (da / (da - db));
            debug_assert!(out.len() < 8);
            out.push(cv);

            debug_assert!(out.len() < 8);
            out.push(b);
        }

        a = b;
    }

    out
}

#[derive(Default)]
struct Clip {
    out: SmallVec<[(Vec3, f32); 8]>,
}

fn clip(rpos: Vec3, e: Vec3, basis: Mat3, incident: [Vec3; 4]) -> Clip {
    let mut vin = SmallVec::<[_; 8]>::new();
    let mut vout;

    for &inc in &incident {
        vin.push(basis.mult(inc - rpos));
    }

    vout = orthographic(1.0, e.x(), 0, vin.as_slice());

    if vout.is_empty() {
        return Clip::default();
    }

    vin = orthographic(1.0, e.y(), 1, vout.as_slice());

    if vin.is_empty() {
        return Clip::default();
    }

    vout = orthographic(-1.0, e.x(), 0, vin.as_slice());

    if vout.is_empty() {
        return Clip::default();
    }

    vin = orthographic(-1.0, e.y(), 1, vout.as_slice());

    let mut clipped = SmallVec::new();

    for cv in vin {
        let d = cv.z() - e.z();

        if d <= 0.0 {
            let vertex = basis * cv + rpos;

            clipped.push((vertex, d));
        }
    }

    debug_assert!(clipped.len() <= 8);

    Clip { out: clipped }
}

fn edges_contact(pa: Vec3, qa: Vec3, pb: Vec3, qb: Vec3) -> [Vec3; 2] {
    let da = qa - pa;
    let db = qb - pb;
    let r = pa - pb;
    let a = da.dot(da);
    let e = db.dot(db);
    let f = db.dot(r);
    let c = da.dot(r);

    let b = da.dot(db);
    let denom = a * e - b * b;

    let ta = (b * f - c * e) / denom;
    let tb = (b * ta + f) / e;

    [pa + da * ta, pb + db * tb]
}

fn support_edge(tx: Transform, e: Vec3, n: Vec3) -> [Vec3; 2] {
    let n = tx.rotation.mult(n);
    let absn = n.abs();
    let a;
    let b;

    if absn.x() > absn.y() {
        if absn.y() > absn.z() {
            a = Vec3::new(e.x(), e.y(), e.z());
            b = Vec3::new(e.x(), e.y(), -e.z());
        } else {
            a = Vec3::new(e.x(), e.y(), e.z());
            b = Vec3::new(e.x(), -e.y(), e.z());
        }
    } else {
        if absn.x() > absn.z() {
            a = Vec3::new(e.x(), e.y(), e.z());
            b = Vec3::new(e.x(), e.y(), -e.z());
        } else {
            a = Vec3::new(e.x(), e.y(), e.z());
            b = Vec3::new(-e.x(), e.y(), e.z());
        }
    }

    let sign = n.signum();

    let a = a * sign;
    let b = b * sign;

    [
        tx.compute_matrix().transform_point3(a),
        tx.compute_matrix().transform_point3(b),
    ]
}

pub fn box_to_box(a: &Obb, b: &Obb) -> Option<Manifold> {
    let mut atx = a.transform;
    let mut btx = b.transform;
    let al = a.local;
    let bl = b.local;

    atx = Transform::from_matrix(atx.compute_matrix() * al.compute_matrix());
    btx = Transform::from_matrix(btx.compute_matrix() * bl.compute_matrix());

    let ea = a.extent;
    let eb = b.extent;

    // conjugate is the same as inverse for unit squaternions,
    // inverse is the same as transpose for rotation matrices
    let c = Mat3::from_quat(atx.rotation.conjugate() * btx.rotation);
    let ca = c.to_cols_array_2d();

    let mut absc = [[0.0; 3]; 3];
    let mut parallel = false;
    const EPS: f32 = 1.0_e-6;
    for i in 0..3 {
        for j in 0..3 {
            let val = ca[i][j].abs();
            absc[i][j] = val;

            if val + EPS >= 1.0 {
                parallel = true
            }
        }
    }

    let absca = absc;
    let absc = Mat3::from_cols_array_2d(&absca);

    let t = atx.rotation.mult(btx.translation - atx.translation);

    let mut s;
    let mut amax = f32::MIN;
    let mut bmax = f32::MIN;
    let mut emax = f32::MIN;
    let mut aaxis = u32::MAX;
    let mut baxis = u32::MAX;
    let mut eaxis = u32::MAX;
    let mut na = Vec3::zero();
    let mut nb = Vec3::zero();
    let mut ne = Vec3::zero();

    let atxr = atx.compute_matrix().truncate();

    s = t.x().abs() - (ea.x() + absc.column0().dot(eb));
    match track_face_axis(0, s, amax, atxr.row0()) {
        TrackFaceAxis::None => return None,
        TrackFaceAxis::Some { max, axis, normal } => {
            amax = max;
            aaxis = axis;
            na = normal;
        }
        _ => {}
    }

    s = t.y().abs() - (ea.y() + absc.column1().dot(eb));
    match track_face_axis(1, s, amax, atxr.row1()) {
        TrackFaceAxis::None => return None,
        TrackFaceAxis::Some { max, axis, normal } => {
            amax = max;
            aaxis = axis;
            na = normal;
        }
        _ => {}
    }

    s = t.z().abs() - (ea.z() + absc.column2().dot(eb));
    match track_face_axis(2, s, amax, atxr.row2()) {
        TrackFaceAxis::None => return None,
        TrackFaceAxis::Some { max, axis, normal } => {
            amax = max;
            aaxis = axis;
            na = normal;
        }
        _ => {}
    }

    let btxr = btx.compute_matrix().truncate();

    s = t.dot(c.row0()).abs() - (eb.x() + absc.row0().dot(ea));
    match track_face_axis(3, s, bmax, btxr.row0()) {
        TrackFaceAxis::None => return None,
        TrackFaceAxis::Some { max, axis, normal } => {
            bmax = max;
            baxis = axis;
            nb = normal;
        }
        _ => {}
    }

    s = t.dot(c.row1()).abs() - (eb.y() + absc.row1().dot(ea));
    match track_face_axis(4, s, bmax, btxr.row1()) {
        TrackFaceAxis::None => return None,
        TrackFaceAxis::Some { max, axis, normal } => {
            bmax = max;
            baxis = axis;
            nb = normal;
        }
        _ => {}
    }

    s = t.dot(c.row2()).abs() - (eb.z() + absc.row2().dot(ea));
    match track_face_axis(5, s, bmax, btxr.row2()) {
        TrackFaceAxis::None => return None,
        TrackFaceAxis::Some { max, axis, normal } => {
            bmax = max;
            baxis = axis;
            nb = normal;
        }
        _ => {}
    }

    if !parallel {
        let mut ra;
        let mut rb;

        ra = ea.y() * absca[2][0] + ea.z() * absca[1][0];
        rb = eb.y() * absca[0][2] + eb.z() * absca[0][1];
        s = (t.z() * ca[1][0] - t.y() * ca[2][0]).abs() - (ra + rb);
        let normal = Vec3::new(0.0, -ca[2][0], ca[1][0]);
        match track_edge_axis(6, s, emax, normal) {
            TrackEdgeAxis::None => return None,
            TrackEdgeAxis::Some { max, axis, normal } => {
                emax = max;
                eaxis = axis;
                ne = normal;
            }
            _ => {}
        }

        ra = ea.y() * absca[2][1] + ea.z() * absca[1][1];
        rb = eb.x() * absca[0][2] + eb.z() * absca[0][0];
        s = (t.z() * ca[1][1] - t.y() * ca[2][1]).abs() - (ra + rb);
        let normal = Vec3::new(0.0, -ca[2][1], ca[1][1]);
        match track_edge_axis(7, s, emax, normal) {
            TrackEdgeAxis::None => return None,
            TrackEdgeAxis::Some { max, axis, normal } => {
                emax = max;
                eaxis = axis;
                ne = normal;
            }
            _ => {}
        }

        ra = ea.y() * absca[2][2] + ea.z() * absca[1][2];
        rb = eb.x() * absca[0][1] + eb.y() * absca[0][0];
        s = (t.z() * ca[1][2] - t.y() * ca[2][2]).abs() - (ra + rb);
        let normal = Vec3::new(0.0, -ca[2][2], ca[1][2]);
        match track_edge_axis(8, s, emax, normal) {
            TrackEdgeAxis::None => return None,
            TrackEdgeAxis::Some { max, axis, normal } => {
                emax = max;
                eaxis = axis;
                ne = normal;
            }
            _ => {}
        }

        ra = ea.x() * absca[2][0] + ea.z() * absca[0][0];
        rb = eb.y() * absca[1][2] + eb.z() * absca[1][1];
        s = (t.x() * ca[2][0] - t.z() * ca[0][0]).abs() - (ra + rb);
        let normal = Vec3::new(ca[2][0], 0.0, -ca[0][0]);
        match track_edge_axis(9, s, emax, normal) {
            TrackEdgeAxis::None => return None,
            TrackEdgeAxis::Some { max, axis, normal } => {
                emax = max;
                eaxis = axis;
                ne = normal;
            }
            _ => {}
        }

        ra = ea.x() * absca[2][1] + ea.z() * absca[0][1];
        rb = eb.x() * absca[1][2] + eb.z() * absca[1][0];
        s = (t.x() * ca[2][1] - t.z() * ca[0][1]).abs() - (ra + rb);
        let normal = Vec3::new(ca[2][1], 0.0, -ca[0][1]);
        match track_edge_axis(10, s, emax, normal) {
            TrackEdgeAxis::None => return None,
            TrackEdgeAxis::Some { max, axis, normal } => {
                emax = max;
                eaxis = axis;
                ne = normal;
            }
            _ => {}
        }

        ra = ea.x() * absca[2][2] + ea.z() * absca[0][2];
        rb = eb.x() * absca[1][1] + eb.y() * absca[1][0];
        s = (t.x() * ca[2][2] - t.z() * ca[0][2]).abs() - (ra + rb);
        let normal = Vec3::new(ca[2][2], 0.0, -ca[0][2]);
        match track_edge_axis(11, s, emax, normal) {
            TrackEdgeAxis::None => return None,
            TrackEdgeAxis::Some { max, axis, normal } => {
                emax = max;
                eaxis = axis;
                ne = normal;
            }
            _ => {}
        }

        ra = ea.x() * absca[1][0] + ea.y() * absca[0][0];
        rb = eb.y() * absca[2][2] + eb.z() * absca[2][1];
        s = (t.y() * ca[0][0] - t.x() * ca[1][0]).abs() - (ra + rb);
        let normal = Vec3::new(-ca[1][0], ca[0][0], 0.0);
        match track_edge_axis(12, s, emax, normal) {
            TrackEdgeAxis::None => return None,
            TrackEdgeAxis::Some { max, axis, normal } => {
                emax = max;
                eaxis = axis;
                ne = normal;
            }
            _ => {}
        }

        ra = ea.x() * absca[1][1] + ea.y() * absca[0][1];
        rb = eb.x() * absca[2][2] + eb.z() * absca[2][0];
        s = (t.y() * ca[0][1] - t.x() * ca[1][1]).abs() - (ra + rb);
        let normal = Vec3::new(-ca[1][1], ca[0][1], 0.0);
        match track_edge_axis(13, s, emax, normal) {
            TrackEdgeAxis::None => return None,
            TrackEdgeAxis::Some { max, axis, normal } => {
                emax = max;
                eaxis = axis;
                ne = normal;
            }
            _ => {}
        }

        ra = ea.x() * absca[1][2] + ea.y() * absca[0][2];
        rb = eb.x() * absca[2][1] + eb.y() * absca[2][0];
        s = (t.y() * ca[0][2] - t.x() * ca[1][2]).abs() - (ra + rb);
        let normal = Vec3::new(-ca[1][2], ca[0][2], 0.0);
        match track_edge_axis(14, s, emax, normal) {
            TrackEdgeAxis::None => return None,
            TrackEdgeAxis::Some { max, axis, normal } => {
                emax = max;
                eaxis = axis;
                ne = normal;
            }
            _ => {}
        }
    }

    const REL_TOLERANCE: f32 = 0.95;
    const ABS_TOLERANCE: f32 = 0.01;

    let axis;
    let smax;
    let mut n;
    let facemax = amax.max(bmax);
    if REL_TOLERANCE * emax > facemax + ABS_TOLERANCE {
        axis = eaxis;
        smax = emax;
        n = ne;
    } else if REL_TOLERANCE * bmax > amax + ABS_TOLERANCE {
        axis = baxis;
        smax = bmax;
        n = nb;
    } else {
        axis = aaxis;
        smax = amax;
        n = na;
    }

    if n.dot(btx.translation - atx.translation) < 0.0 {
        n = -n;
    }

    if axis == u32::MAX {
        return None;
    }

    if axis < 6 {
        let rtx;
        let itx;
        let er;
        let ei;
        let flip;

        if axis < 3 {
            rtx = atx;
            itx = btx;
            er = ea;
            ei = eb;
            flip = false;
        } else {
            rtx = btx;
            itx = atx;
            er = eb;
            ei = ea;
            flip = true;
            n = -n;
        }

        let incident = compute_incident_face(itx, ei, n);
        let refeb = compute_reference_edges_and_basis(er, rtx, n, axis);
        let basis = refeb.basis;
        let e = refeb.e;

        let clip = clip(rtx.translation, e, basis, incident);
        let out = clip.out;

        if out.len() > 0 {
            let normal = if flip { -n } else { n };

            let mut contacts = SmallVec::new();
            for (v, d) in out {
                let contact = Contact {
                    position: v,
                    penetration: d,
                };
                contacts.push(contact);
            }
            Some(Manifold {
                body1: a.body,
                body2: b.body,
                normal,
                penetration: smax,
                contacts,
            })
        } else {
            None
        }
    } else {
        let mut n = atx.rotation * n;

        if n.dot(btx.translation - atx.translation) < 0.0 {
            n = -n;
        }

        let [pa, qa] = support_edge(atx, ea, n);
        let [pb, qb] = support_edge(btx, eb, -n);

        let [ca, cb] = edges_contact(pa, qa, pb, qb);

        let normal = n;
        Some(Manifold {
            body1: a.body,
            body2: b.body,
            normal,
            penetration: smax,
            contacts: smallvec![Contact {
                position: (ca + cb) * 0.5,
                penetration: smax,
            }],
        })
    }
}
