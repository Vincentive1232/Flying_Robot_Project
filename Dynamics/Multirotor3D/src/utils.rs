use nalgebra::{Quaternion, Vector3};

pub fn vector_augument(vec: Vector3<f32>) -> Quaternion<f32> {
    Quaternion::new(0.0, vec[0], vec[1], vec[2])
}

pub fn circle_dot(q: Quaternion<f32>, vec: Vector3<f32>) -> Vector3<f32> {
    let q_temp = Quaternion::new(0.0, vec[0], vec[1], vec[2]);

    let q_result = q*q_temp*q.conjugate();

    Vector3::new(q_result.i, q_result.j, q_result.k)
}


