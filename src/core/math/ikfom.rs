//! iKFoM (iterated Kalman Filter on Manifold)
//!
//! Equivalent to use-ikfom.hpp in S-FAST_LIO
use sophus::nalgebra::{Matrix3, SMatrix, Vector3};
use sophus::lie::Rotation3F64;

/// Gravity acceleration constant (m/s²)
pub const G_M_S2: f64 = 9.81;

/// Maximum number of iterations for IMU initialization
pub const MAX_INI_COUNT: u32 = 10;

/// 24-dimensional state vector for iKFoM
/// 
/// Corresponds to `state_ikfom` in C++:
/// - pos (3): position in world frame
/// - rot (3): rotation (SO3) in world frame  
/// - offset_R_L_I (3): rotation from LiDAR to IMU frame
/// - offset_T_L_I (3): translation from LiDAR to IMU frame
/// - vel (3): velocity in world frame
/// - bg (3): gyroscope bias
/// - ba (3): accelerometer bias
/// - grav (3): gravity vector
#[derive(Clone, Debug)]
pub struct StateIkfom {
    pub pos: Vector3<f64>,              // position in world frame
    pub rot: Rotation3F64,              // rotation (SO3) from body to world
    pub offset_r_l_i: Rotation3F64,     // rotation from LiDAR to IMU
    pub offset_t_l_i: Vector3<f64>,     // translation from LiDAR to IMU
    pub vel: Vector3<f64>,              // velocity in world frame
    pub bg: Vector3<f64>,               // gyroscope bias
    pub ba: Vector3<f64>,               // accelerometer bias
    pub grav: Vector3<f64>,             // gravity vector
}

impl Default for StateIkfom {
    fn default() -> Self {
        Self {
            pos: Vector3::zeros(),
            rot: Rotation3F64::identity(),
            offset_r_l_i: Rotation3F64::identity(),
            offset_t_l_i: Vector3::zeros(),
            vel: Vector3::zeros(),
            bg: Vector3::zeros(),
            ba: Vector3::zeros(),
            grav: Vector3::new(0.0, 0.0, -G_M_S2),
        }
    }
}

/// Input for IMU propagation
/// 
/// Corresponds to `input_ikfom` in C++
#[derive(Clone, Debug, Default)]
pub struct InputIkfom {
    pub acc: Vector3<f64>,   // accelerometer measurement
    pub gyro: Vector3<f64>,  // gyroscope measurement
}

/// Initialize process noise covariance Q (12x12)
/// 
/// Corresponds to `process_noise_cov()` in C++ (equation 8 in paper)
pub fn process_noise_cov() -> SMatrix<f64, 12, 12> {
    let mut q = SMatrix::<f64, 12, 12>::zeros();
    
    // Gyroscope noise (rows 0-2)
    q.fixed_view_mut::<3, 3>(0, 0)
        .copy_from(&(Matrix3::identity() * 0.0001));
    
    // Accelerometer noise (rows 3-5)
    q.fixed_view_mut::<3, 3>(3, 3)
        .copy_from(&(Matrix3::identity() * 0.0001));
    
    // Gyroscope bias random walk (rows 6-8)
    q.fixed_view_mut::<3, 3>(6, 6)
        .copy_from(&(Matrix3::identity() * 0.00001));
    
    // Accelerometer bias random walk (rows 9-11)
    q.fixed_view_mut::<3, 3>(9, 9)
        .copy_from(&(Matrix3::identity() * 0.00001));
    
    q
}

/// Compute f(x, u) - state transition function (equation 2 in paper)
/// 
/// Returns a 24x1 vector representing state derivatives
pub fn get_f(s: &StateIkfom, input: &InputIkfom) -> SMatrix<f64, 24, 1> {
    let mut res = SMatrix::<f64, 24, 1>::zeros();
    
    // omega = gyro - bg (angular velocity minus bias)
    let omega = input.gyro - s.bg;
    
    // a_inertial = R * (acc - ba) (acceleration in world frame)
    let rot_matrix = s.rot.matrix();
    let acc_body = input.acc - s.ba;
    let a_inertial = rot_matrix * acc_body;
    
    // Fill result vector:
    // [0-2]: velocity (derivative of position)
    // [3-5]: angular velocity (derivative of rotation)
    // [12-14]: acceleration + gravity (derivative of velocity)
    for i in 0..3 {
        res[i] = s.vel[i];                      // velocity
        res[i + 3] = omega[i];                  // angular velocity
        res[i + 12] = a_inertial[i] + s.grav[i]; // acceleration
    }
    
    res
}

/// Compute Jacobian df/dx (equation 7 in paper, Fx matrix)
/// 
/// Note: This matrix is not multiplied by dt and doesn't include identity
pub fn df_dx(s: &StateIkfom, input: &InputIkfom) -> SMatrix<f64, 24, 24> {
    let mut cov = SMatrix::<f64, 24, 24>::zeros();
    
    // Row 2, Col 3: I (velocity derivative w.r.t. position)
    cov.fixed_view_mut::<3, 3>(0, 12)
        .copy_from(&Matrix3::identity());
    
    // acc_ = a_m - ba (measured acceleration minus bias)
    let acc_ = input.acc - s.ba;
    
    // Row 3, Col 1: -R * hat(acc_) (acceleration derivative w.r.t. rotation)
    let rot_matrix = s.rot.matrix();
    let hat_acc = hat(&acc_);
    let neg_r_hat = -(rot_matrix * hat_acc);
    for i in 0..3 {
        for j in 0..3 {
            cov[(12 + i, 3 + j)] = neg_r_hat[(i, j)];
        }
    }
    
    // Row 3, Col 5: -R (acceleration derivative w.r.t. accelerometer bias)
    let neg_rot = -rot_matrix;
    for i in 0..3 {
        for j in 0..3 {
            cov[(12 + i, 18 + j)] = neg_rot[(i, j)];
        }
    }
    
    // Row 3, Col 6: I (acceleration derivative w.r.t. gravity)
    cov.fixed_view_mut::<3, 3>(12, 21)
        .copy_from(&Matrix3::identity());
    
    // Row 1, Col 4: -I (rotation derivative w.r.t. gyroscope bias)
    cov.fixed_view_mut::<3, 3>(3, 15)
        .copy_from(&(-Matrix3::identity()));
    
    cov
}

/// Compute Jacobian df/dw (equation 7 in paper, Fw matrix)
/// 
/// Note: This matrix is not multiplied by dt
pub fn df_dw(s: &StateIkfom, _input: &InputIkfom) -> SMatrix<f64, 24, 12> {
    let mut cov = SMatrix::<f64, 24, 12>::zeros();
    
    let rot_matrix = s.rot.matrix();
    
    // Row 3, Col 2: -R (acceleration noise)
    let neg_rot = -rot_matrix;
    for i in 0..3 {
        for j in 0..3 {
            cov[(12 + i, 3 + j)] = neg_rot[(i, j)];
        }
    }
    
    // Row 1, Col 1: -I (gyroscope noise)
    cov.fixed_view_mut::<3, 3>(3, 0)
        .copy_from(&(-Matrix3::identity()));
    
    // Row 4, Col 3: I (gyroscope bias random walk)
    cov.fixed_view_mut::<3, 3>(15, 6)
        .copy_from(&Matrix3::identity());
    
    // Row 5, Col 4: I (accelerometer bias random walk)
    cov.fixed_view_mut::<3, 3>(18, 9)
        .copy_from(&Matrix3::identity());
    
    cov
}

/// Skew-symmetric matrix (hat operator) for cross product
/// 
/// hat(v) * u = v × u
#[inline]
pub fn hat(v: &Vector3<f64>) -> Matrix3<f64> {
    Matrix3::new(
        0.0, -v[2], v[1],
        v[2], 0.0, -v[0],
        -v[1], v[0], 0.0,
    )
}

/// Simple Error-State Extended Kalman Filter on Manifold
/// 
/// Simplified version of esekfom for IMU-LiDAR fusion
#[derive(Clone, Debug)]
pub struct EsEkfom {
    /// Current state estimate
    pub x: StateIkfom,
    /// State covariance matrix (24x24)
    pub p: SMatrix<f64, 24, 24>,
}

impl Default for EsEkfom {
    fn default() -> Self {
        Self {
            x: StateIkfom::default(),
            p: SMatrix::<f64, 24, 24>::identity(),
        }
    }
}

impl EsEkfom {
    pub fn new() -> Self {
        Self::default()
    }
    
    /// Get current state
    pub fn get_x(&self) -> &StateIkfom {
        &self.x
    }
    
    /// Get mutable reference to state
    pub fn get_x_mut(&mut self) -> &mut StateIkfom {
        &mut self.x
    }
    
    /// Update state
    pub fn change_x(&mut self, new_state: StateIkfom) {
        self.x = new_state;
    }
    
    /// Update covariance
    pub fn change_p(&mut self, new_p: SMatrix<f64, 24, 24>) {
        self.p = new_p;
    }
    
    /// IMU forward propagation using discrete midpoint integration
    /// 
    /// # Arguments
    /// * `dt` - Time step
    /// * `q` - Process noise covariance (12x12)
    /// * `input` - IMU measurements (acceleration and angular velocity)
    pub fn predict(&mut self, dt: f64, q: &SMatrix<f64, 12, 12>, input: &InputIkfom) {
        // Get state transition function f(x, u)
        let _f = get_f(&self.x, input);
        
        // Get Jacobians
        let fx = df_dx(&self.x, input);
        let fw = df_dw(&self.x, input);
        
        // State transition matrix: F = I + Fx * dt
        let f_mat = SMatrix::<f64, 24, 24>::identity() + fx * dt;
        
        // Update state using Euler integration
        // Position update: pos += vel * dt
        self.x.pos += self.x.vel * dt;
        
        // Velocity update: vel += (R * (acc - ba) + grav) * dt
        let omega = input.gyro - self.x.bg;
        let rot_matrix = self.x.rot.matrix();
        let acc_body = input.acc - self.x.ba;
        let acc_world = rot_matrix * acc_body + self.x.grav;
        self.x.vel += acc_world * dt;
        
        // Rotation update using exponential map: rot = rot * exp(omega * dt)
        let omega_dt = omega * dt;
        let delta_rot = Rotation3F64::exp(omega_dt);
        // Compose rotations: rot_new = rot * delta_rot
        let new_rot_matrix = rot_matrix * delta_rot.matrix();
        self.x.rot = Rotation3F64::try_from_mat(new_rot_matrix)
            .unwrap_or(self.x.rot.clone());
        
        // Covariance propagation: P = F * P * F^T + Fw * Q * Fw^T * dt^2
        let fw_q_fw_t = fw * q * fw.transpose() * dt * dt;
        self.p = f_mat * self.p * f_mat.transpose() + fw_q_fw_t;
    }
}
