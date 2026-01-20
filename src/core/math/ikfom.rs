//! iKFoM (iterated Kalman Filter on Manifold)
//!
//! Equivalent to use-ikfom.hpp in S-FAST_LIO
use sophus::nalgebra::{Matrix3, SMatrix, DMatrix, DVector, Vector3};
use sophus::lie::Rotation3F64;
use crate::core::math::ikd_tree::{IkdTree, IkdTreePoint, PointVector};

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

    // ========================================================================
    // ESIKF Measurement Update
    // ========================================================================

    /// Iterated ESIKF update using point-to-plane residuals
    /// 
    /// This is equivalent to `update_iterated_dyn_share_modified` in C++ S-FAST_LIO.
    /// 
    /// # Arguments
    /// * `r` - Measurement noise covariance (scalar, typically 0.001)
    /// * `feats_down_body` - Downsampled feature points in body (LiDAR) frame
    /// * `ikdtree` - ikd-tree map for nearest neighbor search
    /// * `max_iterations` - Maximum number of iterations (typically 4)
    /// * `extrinsic_est_en` - Whether to estimate LiDAR-IMU extrinsic
    /// 
    /// # Returns
    /// * `true` if update succeeded, `false` otherwise
    pub fn update_iterated(
        &mut self,
        r: f64,
        feats_down_body: &PointVector,
        ikdtree: &IkdTree,
        max_iterations: usize,
        extrinsic_est_en: bool,
    ) -> bool {
        const EPSI: f64 = 0.001;  // Convergence threshold
        const NUM_MATCH_POINTS: usize = 5;  // Number of nearest neighbors for plane fitting
        const MAX_DIST_SQ: f64 = 5.0;  // Maximum squared distance for valid correspondence
        
        if feats_down_body.is_empty() || ikdtree.is_empty() {
            return false;
        }
        
        let feats_down_size = feats_down_body.len();
        
        // Store propagated state for iteration
        let x_propagated = self.x.clone();
        let p_propagated = self.p;
        
        // Convergence counter
        let mut converge_count = 0;
        
        // Nearest points storage (reused across iterations)
        let mut nearest_points: Vec<PointVector> = vec![Vec::new(); feats_down_size];
        let mut point_selected: Vec<bool> = vec![false; feats_down_size];
        
        for iter in 0..max_iterations {
            let need_search = converge_count == 0 || iter == 0;
            
            // Compute residuals and Jacobian
            let (h, h_x, valid_count) = self.compute_residuals_and_jacobian(
                feats_down_body,
                ikdtree,
                &mut nearest_points,
                &mut point_selected,
                need_search,
                NUM_MATCH_POINTS,
                MAX_DIST_SQ,
                extrinsic_est_en,
            );
            
            if valid_count < 10 {
                tracing::warn!("Not enough valid correspondences: {}", valid_count);
                return false;
            }
            
            // Compute dx_new = x - x_propagated (on manifold)
            let dx_new = self.boxminus(&x_propagated);
            
            // Compute Kalman gain and update
            // H is m x 12 (only first 12 columns non-zero), so we use block operations
            // K = (H^T * H / R + P^-1)^-1 * H^T / R
            
            // Build H^T * H (24x24, but only 12x12 block is non-zero)
            let mut hth = SMatrix::<f64, 24, 24>::zeros();
            for i in 0..valid_count {
                for j in 0..12 {
                    for k in 0..12 {
                        hth[(j, k)] += h_x[(i, j)] * h_x[(i, k)];
                    }
                }
            }
            
            // Compute (H^T * H / R + P^-1)^-1
            let hth_r = hth / r;
            
            // P^-1 using Cholesky decomposition
            let p_inv = match self.p.try_inverse() {
                Some(inv) => inv,
                None => {
                    tracing::warn!("Failed to invert covariance matrix");
                    return false;
                }
            };
            
            let k_front = match (hth_r + p_inv).try_inverse() {
                Some(inv) => inv,
                None => {
                    tracing::warn!("Failed to compute Kalman gain");
                    return false;
                }
            };
            
            // K = K_front[:, :12] * H^T / R
            // dx = K * h + (K * H - I) * dx_new
            let mut dx = SMatrix::<f64, 24, 1>::zeros();
            
            // Compute K * h
            for i in 0..24 {
                for j in 0..valid_count {
                    let mut k_ij = 0.0;
                    for k in 0..12 {
                        k_ij += k_front[(i, k)] * h_x[(j, k)];
                    }
                    dx[i] += k_ij * h[j] / r;
                }
            }
            
            // Compute (K * H - I) * dx_new
            let mut kh = SMatrix::<f64, 24, 24>::zeros();
            for i in 0..24 {
                for j in 0..12 {
                    for k in 0..valid_count {
                        let mut k_ik = 0.0;
                        for l in 0..12 {
                            k_ik += k_front[(i, l)] * h_x[(k, l)] / r;
                        }
                        kh[(i, j)] += k_ik * h_x[(k, j)];
                    }
                }
            }
            
            let kh_minus_i = kh - SMatrix::<f64, 24, 24>::identity();
            dx += kh_minus_i * dx_new;
            
            // Update state: x = x ⊕ dx
            self.x = self.boxplus(&dx);
            
            // Check convergence
            let mut converged = true;
            for i in 0..24 {
                if dx[i].abs() > EPSI {
                    converged = false;
                    break;
                }
            }
            
            if converged {
                converge_count += 1;
            }
            
            // Exit conditions
            if converge_count > 1 || iter == max_iterations - 1 {
                // Update covariance: P = (I - K * H) * P
                let i_kh = SMatrix::<f64, 24, 24>::identity() - kh;
                self.p = i_kh * p_propagated;
                return true;
            }
        }
        
        true
    }
    
    /// Compute point-to-plane residuals and Jacobian matrix
    fn compute_residuals_and_jacobian(
        &self,
        feats_down_body: &PointVector,
        ikdtree: &IkdTree,
        nearest_points: &mut [PointVector],
        point_selected: &mut [bool],
        need_search: bool,
        num_match_points: usize,
        max_dist_sq: f64,
        extrinsic_est_en: bool,
    ) -> (DVector<f64>, DMatrix<f64>, usize) {
        let feats_down_size = feats_down_body.len();
        
        // Transform points to world frame and find correspondences
        let rot_matrix = self.x.rot.matrix();
        let offset_r = self.x.offset_r_l_i.matrix();
        
        // First pass: search for nearest neighbors and fit planes
        let mut plane_coeffs: Vec<Option<[f64; 4]>> = vec![None; feats_down_size];
        
        for i in 0..feats_down_size {
            let p_body = &feats_down_body[i];
            
            // Transform to world frame: p_world = R * (R_L_I * p_body + T_L_I) + pos
            let p_body_v = Vector3::new(p_body.x, p_body.y, p_body.z);
            let p_imu = offset_r * p_body_v + self.x.offset_t_l_i;
            let p_world = rot_matrix * p_imu + self.x.pos;
            
            let p_world_pt = IkdTreePoint::new(p_world[0], p_world[1], p_world[2]);
            
            if need_search {
                // Search for nearest neighbors
                let (neighbors, dists) = ikdtree.nearest_search(
                    &p_world_pt,
                    num_match_points,
                    f64::INFINITY,
                );
                
                nearest_points[i] = neighbors;
                
                // Check if enough neighbors and within distance threshold
                point_selected[i] = nearest_points[i].len() >= num_match_points 
                    && dists.last().map_or(false, |&d| d <= max_dist_sq);
            }
            
            if !point_selected[i] {
                continue;
            }
            
            // Fit plane to nearest neighbors
            if let Some(plane) = fit_plane(&nearest_points[i], 0.1) {
                // Compute point-to-plane distance
                let pd = plane[0] * p_world[0] + plane[1] * p_world[1] 
                       + plane[2] * p_world[2] + plane[3];
                
                // Check if residual is acceptable (adaptive threshold based on distance)
                let p_body_norm = p_body_v.norm();
                let threshold = if p_body_norm > 1e-6 {
                    1.0 - 0.9 * pd.abs() / p_body_norm.sqrt()
                } else {
                    0.0
                };
                
                if threshold > 0.9 {
                    plane_coeffs[i] = Some(plane);
                } else {
                    point_selected[i] = false;
                }
            } else {
                point_selected[i] = false;
            }
        }
        
        // Count valid correspondences
        let valid_count: usize = point_selected.iter().filter(|&&x| x).count();
        
        if valid_count == 0 {
            return (DVector::zeros(0), DMatrix::zeros(0, 12), 0);
        }
        
        // Second pass: compute Jacobian and residuals
        let mut h = DVector::zeros(valid_count);
        let mut h_x = DMatrix::zeros(valid_count, 12);
        
        let mut idx = 0;
        for i in 0..feats_down_size {
            if !point_selected[i] {
                continue;
            }
            
            let plane = plane_coeffs[i].unwrap();
            let norm_vec = Vector3::new(plane[0], plane[1], plane[2]);
            
            let p_body = &feats_down_body[i];
            let p_body_v = Vector3::new(p_body.x, p_body.y, p_body.z);
            
            // p_I = R_L_I * p_body + T_L_I (point in IMU frame)
            let p_imu = offset_r * p_body_v + self.x.offset_t_l_i;
            
            // p_world = R * p_I + pos
            let p_world = rot_matrix * p_imu + self.x.pos;
            
            // Residual: point-to-plane distance
            let residual = plane[0] * p_world[0] + plane[1] * p_world[1] 
                         + plane[2] * p_world[2] + plane[3];
            h[idx] = -residual;  // Negative because we want to minimize
            
            // Jacobian computation
            // h(x) = n^T * (R * (R_L_I * p + T_L_I) + pos) + d
            // 
            // dh/d_pos = n^T  (columns 0-2)
            // dh/d_rot = n^T * R * hat(p_I) = n^T * hat(R^T * n) ... actually:
            //           = -n^T * R * [p_I]_x
            // For rotation: using the formula from paper
            // C = R^T * n
            // A = [p_I]_x * C = p_I × C
            
            let c = rot_matrix.transpose() * norm_vec;
            let a = p_imu.cross(&c);
            
            // dh/d_pos = n^T
            h_x[(idx, 0)] = norm_vec[0];
            h_x[(idx, 1)] = norm_vec[1];
            h_x[(idx, 2)] = norm_vec[2];
            
            // dh/d_rot = A^T
            h_x[(idx, 3)] = a[0];
            h_x[(idx, 4)] = a[1];
            h_x[(idx, 5)] = a[2];
            
            if extrinsic_est_en {
                // dh/d_offset_R = B = [p_body]_x * R_L_I^T * C
                let b = p_body_v.cross(&(offset_r.transpose() * c));
                h_x[(idx, 6)] = b[0];
                h_x[(idx, 7)] = b[1];
                h_x[(idx, 8)] = b[2];
                
                // dh/d_offset_T = C
                h_x[(idx, 9)] = c[0];
                h_x[(idx, 10)] = c[1];
                h_x[(idx, 11)] = c[2];
            }
            
            idx += 1;
        }
        
        (h, h_x, valid_count)
    }
    
    /// Generalized addition on manifold (boxplus)
    /// x_new = x ⊕ dx
    fn boxplus(&self, dx: &SMatrix<f64, 24, 1>) -> StateIkfom {
        StateIkfom {
            pos: self.x.pos + dx.fixed_rows::<3>(0).into_owned(),
            rot: {
                let delta = Vector3::new(dx[3], dx[4], dx[5]);
                let delta_rot = Rotation3F64::exp(delta);
                let new_matrix = self.x.rot.matrix() * delta_rot.matrix();
                Rotation3F64::try_from_mat(new_matrix).unwrap_or(self.x.rot.clone())
            },
            offset_r_l_i: {
                let delta = Vector3::new(dx[6], dx[7], dx[8]);
                let delta_rot = Rotation3F64::exp(delta);
                let new_matrix = self.x.offset_r_l_i.matrix() * delta_rot.matrix();
                Rotation3F64::try_from_mat(new_matrix).unwrap_or(self.x.offset_r_l_i.clone())
            },
            offset_t_l_i: self.x.offset_t_l_i + dx.fixed_rows::<3>(9).into_owned(),
            vel: self.x.vel + dx.fixed_rows::<3>(12).into_owned(),
            bg: self.x.bg + dx.fixed_rows::<3>(15).into_owned(),
            ba: self.x.ba + dx.fixed_rows::<3>(18).into_owned(),
            grav: self.x.grav + dx.fixed_rows::<3>(21).into_owned(),
        }
    }
    
    /// Generalized subtraction on manifold (boxminus)
    /// dx = x ⊖ x_ref
    fn boxminus(&self, x_ref: &StateIkfom) -> SMatrix<f64, 24, 1> {
        let mut dx = SMatrix::<f64, 24, 1>::zeros();
        
        // Position difference
        dx.fixed_rows_mut::<3>(0).copy_from(&(self.x.pos - x_ref.pos));
        
        // Rotation difference: log(R_ref^T * R)
        let rot_diff = x_ref.rot.matrix().transpose() * self.x.rot.matrix();
        let log_rot = Rotation3F64::try_from_mat(rot_diff)
            .map(|r| r.log())
            .unwrap_or(Vector3::zeros());
        dx[3] = log_rot[0];
        dx[4] = log_rot[1];
        dx[5] = log_rot[2];
        
        // Offset rotation difference
        let offset_rot_diff = x_ref.offset_r_l_i.matrix().transpose() * self.x.offset_r_l_i.matrix();
        let log_offset = Rotation3F64::try_from_mat(offset_rot_diff)
            .map(|r| r.log())
            .unwrap_or(Vector3::zeros());
        dx[6] = log_offset[0];
        dx[7] = log_offset[1];
        dx[8] = log_offset[2];
        
        // Other vector differences
        dx.fixed_rows_mut::<3>(9).copy_from(&(self.x.offset_t_l_i - x_ref.offset_t_l_i));
        dx.fixed_rows_mut::<3>(12).copy_from(&(self.x.vel - x_ref.vel));
        dx.fixed_rows_mut::<3>(15).copy_from(&(self.x.bg - x_ref.bg));
        dx.fixed_rows_mut::<3>(18).copy_from(&(self.x.ba - x_ref.ba));
        dx.fixed_rows_mut::<3>(21).copy_from(&(self.x.grav - x_ref.grav));
        
        dx
    }
}

// ============================================================================
// Plane Fitting
// ============================================================================

/// Fit a plane to a set of points using least squares
/// 
/// Returns plane coefficients [a, b, c, d] where ax + by + cz + d = 0
/// and (a, b, c) is the unit normal
fn fit_plane(points: &PointVector, threshold: f32) -> Option<[f64; 4]> {
    if points.len() < 3 {
        return None;
    }
    
    // Use first point as reference
    let _p0 = Vector3::new(points[0].x, points[0].y, points[0].z);
    
    // Build matrix A for least squares: A * n = -1
    // For each point: (p - p0) · n = 0, but we solve n · p + d = 0
    let n = points.len();
    let mut ata = Matrix3::<f64>::zeros();
    let mut atb = Vector3::<f64>::zeros();
    
    for point in points {
        let p = Vector3::new(point.x, point.y, point.z);
        ata += p * p.transpose();
        atb += p;
    }
    
    // Solve using SVD or normal equations
    // We want to find n such that sum((n · p_i + d)^2) is minimized
    // subject to |n| = 1
    
    // Compute centroid
    let centroid = atb / n as f64;
    
    // Compute covariance matrix
    let mut cov = Matrix3::<f64>::zeros();
    for point in points {
        let p = Vector3::new(point.x, point.y, point.z);
        let diff = p - centroid;
        cov += diff * diff.transpose();
    }
    
    // Find eigenvector with smallest eigenvalue (plane normal)
    let eigen = cov.symmetric_eigen();
    
    // Find index of smallest eigenvalue
    let mut min_idx = 0;
    let mut min_val = eigen.eigenvalues[0];
    for i in 1..3 {
        if eigen.eigenvalues[i] < min_val {
            min_val = eigen.eigenvalues[i];
            min_idx = i;
        }
    }
    
    // Normal vector
    let normal = eigen.eigenvectors.column(min_idx).into_owned();
    
    // Check planarity: ratio of smallest to second-smallest eigenvalue
    let mut sorted_eig: Vec<f64> = eigen.eigenvalues.iter().copied().collect();
    sorted_eig.sort_by(|a, b| a.partial_cmp(b).unwrap());
    
    if sorted_eig[1] > 1e-10 && sorted_eig[0] / sorted_eig[1] > threshold as f64 {
        return None;  // Not planar enough
    }
    
    // Compute d: d = -n · centroid
    let d = -normal.dot(&centroid);
    
    Some([normal[0], normal[1], normal[2], d])
}
