//! ikd-Tree: Incremental K-D Tree for Robotic Applications
//!
//! A Rust port of the ikd-Tree implementation from FAST-LIO2.
//! Reference: <https://github.com/hku-mars/ikd-Tree>
//!
//! This implementation provides:
//! - Dynamic point insertion and deletion
//! - Box-wise deletion for map management
//! - K-nearest neighbor search with range constraint
//! - Automatic re-balancing
//! - Downsampling support

#![allow(dead_code)]

use sophus::nalgebra::Vector3;
use std::cmp::Ordering;

// ============================================================================
// Constants
// ============================================================================

const EPSS: f64 = 1e-6;
const MINIMUM_UNBALANCED_TREE_SIZE: usize = 10;
/// Threshold for multi-threaded rebuild (reserved for future use)
const MULTI_THREAD_REBUILD_POINT_NUM: usize = 1500;
const DOWNSAMPLE_SWITCH: bool = true;
/// Force rebuild percentage threshold (reserved for future use)
const FORCE_REBUILD_PERCENTAGE: f64 = 0.2;

// ============================================================================
// Point Types
// ============================================================================

/// 3D point type used in ikd-tree
#[derive(Clone, Copy, Debug, Default)]
pub struct IkdTreePoint {
    pub x: f64,
    pub y: f64,
    pub z: f64,
}

impl IkdTreePoint {
    pub fn new(x: f64, y: f64, z: f64) -> Self {
        Self { x, y, z }
    }

    pub fn from_vector3(v: &Vector3<f64>) -> Self {
        Self { x: v.x, y: v.y, z: v.z }
    }

    pub fn from_vector3_f32(v: &Vector3<f32>) -> Self {
        Self { x: v.x as f64, y: v.y as f64, z: v.z as f64 }
    }

    pub fn to_vector3(&self) -> Vector3<f64> {
        Vector3::new(self.x, self.y, self.z)
    }

    /// Get coordinate by axis index (0=x, 1=y, 2=z)
    #[inline]
    pub fn coord(&self, axis: usize) -> f64 {
        match axis {
            0 => self.x,
            1 => self.y,
            _ => self.z,
        }
    }
}

/// Type alias for point vector
pub type PointVector = Vec<IkdTreePoint>;

// ============================================================================
// Box Point Type
// ============================================================================

/// Axis-aligned bounding box defined by min and max vertices
#[derive(Clone, Copy, Debug, Default)]
pub struct BoxPointType {
    pub vertex_min: [f64; 3],
    pub vertex_max: [f64; 3],
}

impl BoxPointType {
    pub fn new(min: [f64; 3], max: [f64; 3]) -> Self {
        Self { vertex_min: min, vertex_max: max }
    }

    /// Create a box from center point and half-size
    pub fn from_center_halfsize(center: &IkdTreePoint, half_size: f64) -> Self {
        Self {
            vertex_min: [center.x - half_size, center.y - half_size, center.z - half_size],
            vertex_max: [center.x + half_size, center.y + half_size, center.z + half_size],
        }
    }

    /// Check if a point is inside the box
    pub fn contains(&self, point: &IkdTreePoint) -> bool {
        self.vertex_min[0] <= point.x && point.x < self.vertex_max[0]
            && self.vertex_min[1] <= point.y && point.y < self.vertex_max[1]
            && self.vertex_min[2] <= point.z && point.z < self.vertex_max[2]
    }

    /// Check if this box completely contains another box's range
    pub fn contains_range(&self, range: &[[f64; 2]; 3]) -> bool {
        self.vertex_min[0] <= range[0][0] && self.vertex_max[0] > range[0][1]
            && self.vertex_min[1] <= range[1][0] && self.vertex_max[1] > range[1][1]
            && self.vertex_min[2] <= range[2][0] && self.vertex_max[2] > range[2][1]
    }

    /// Check if this box intersects with a range
    pub fn intersects_range(&self, range: &[[f64; 2]; 3]) -> bool {
        !(self.vertex_max[0] <= range[0][0] || self.vertex_min[0] > range[0][1]
            || self.vertex_max[1] <= range[1][0] || self.vertex_min[1] > range[1][1]
            || self.vertex_max[2] <= range[2][0] || self.vertex_min[2] > range[2][1])
    }
}

// ============================================================================
// Operation Types
// ============================================================================

#[derive(Clone, Copy, Debug)]
enum OperationType {
    AddPoint,
    DeletePoint,
    AddBox,
    DeleteBox,
    DownSampleDelete,
    PushDown,
}

#[derive(Clone, Copy, Debug)]
enum DeletePointStorageType {
    NotRecord,
    DeletePointsRec,
    MultiThreadRec,
}

/// Operation logger for rebuild operations
#[derive(Clone, Debug)]
struct OperationLogger {
    point: IkdTreePoint,
    boxpoint: BoxPointType,
    tree_deleted: bool,
    tree_downsample_deleted: bool,
    op: OperationType,
}

// ============================================================================
// Point with Distance (for K-NN search)
// ============================================================================

/// Point with distance for nearest neighbor search
#[derive(Clone, Copy, Debug)]
struct PointWithDist {
    point: IkdTreePoint,
    dist: f64,
}

impl PointWithDist {
    fn new(point: IkdTreePoint, dist: f64) -> Self {
        Self { point, dist }
    }
}

impl PartialEq for PointWithDist {
    fn eq(&self, other: &Self) -> bool {
        (self.dist - other.dist).abs() < 1e-10 && (self.point.x - other.point.x).abs() < 1e-10
    }
}

impl Eq for PointWithDist {}

impl PartialOrd for PointWithDist {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}

impl Ord for PointWithDist {
    fn cmp(&self, other: &Self) -> Ordering {
        if (self.dist - other.dist).abs() < 1e-10 {
            self.point.x.partial_cmp(&other.point.x).unwrap_or(Ordering::Equal)
        } else {
            self.dist.partial_cmp(&other.dist).unwrap_or(Ordering::Equal)
        }
    }
}

// ============================================================================
// Manual Max-Heap for K-NN Search
// ============================================================================

/// Max-heap for K-nearest neighbor search candidates
struct ManualHeap {
    heap: Vec<PointWithDist>,
    cap: usize,
}

impl ManualHeap {
    fn new(max_capacity: usize) -> Self {
        Self {
            heap: Vec::with_capacity(max_capacity),
            cap: max_capacity,
        }
    }

    fn push(&mut self, item: PointWithDist) {
        if self.heap.len() >= self.cap {
            return;
        }
        self.heap.push(item);
        self.float_up(self.heap.len() - 1);
    }

    fn pop(&mut self) {
        if self.heap.is_empty() {
            return;
        }
        let last = self.heap.len() - 1;
        self.heap.swap(0, last);
        self.heap.pop();
        if !self.heap.is_empty() {
            self.move_down(0);
        }
    }

    fn top(&self) -> Option<&PointWithDist> {
        self.heap.first()
    }

    fn size(&self) -> usize {
        self.heap.len()
    }

    fn clear(&mut self) {
        self.heap.clear();
    }

    fn move_down(&mut self, mut idx: usize) {
        let len = self.heap.len();
        loop {
            let left = 2 * idx + 1;
            if left >= len {
                break;
            }
            let right = left + 1;
            let mut largest = idx;
            
            if self.heap[left] > self.heap[largest] {
                largest = left;
            }
            if right < len && self.heap[right] > self.heap[largest] {
                largest = right;
            }
            
            if largest == idx {
                break;
            }
            self.heap.swap(idx, largest);
            idx = largest;
        }
    }

    fn float_up(&mut self, mut idx: usize) {
        while idx > 0 {
            let parent = (idx - 1) / 2;
            if self.heap[idx] > self.heap[parent] {
                self.heap.swap(idx, parent);
                idx = parent;
            } else {
                break;
            }
        }
    }
}

// ============================================================================
// KD-Tree Node
// ============================================================================

/// Node in the ikd-tree
pub struct KdTreeNode {
    // Data point
    pub point: IkdTreePoint,
    
    // Division axis (0=x, 1=y, 2=z)
    pub division_axis: u8,
    
    // Tree statistics
    pub tree_size: i32,
    pub invalid_point_num: i32,
    pub down_del_num: i32,
    
    // Deletion flags
    pub point_deleted: bool,
    pub tree_deleted: bool,
    pub point_downsample_deleted: bool,
    pub tree_downsample_deleted: bool,
    
    // Push-down flags
    pub need_push_down_to_left: bool,
    pub need_push_down_to_right: bool,
    
    // Working flag for concurrent access
    pub working_flag: bool,
    
    // Bounding box range [min, max] for each axis
    pub node_range: [[f64; 2]; 3],
    
    // Squared radius of bounding sphere
    pub radius_sq: f64,
    
    // Children and parent
    pub left_son: Option<Box<KdTreeNode>>,
    pub right_son: Option<Box<KdTreeNode>>,
    
    // For statistics
    pub alpha_del: f64,
    pub alpha_bal: f64,
}

impl Default for KdTreeNode {
    fn default() -> Self {
        Self {
            point: IkdTreePoint::default(),
            division_axis: 0,
            tree_size: 1,
            invalid_point_num: 0,
            down_del_num: 0,
            point_deleted: false,
            tree_deleted: false,
            point_downsample_deleted: false,
            tree_downsample_deleted: false,
            need_push_down_to_left: false,
            need_push_down_to_right: false,
            working_flag: false,
            node_range: [[0.0, 0.0], [0.0, 0.0], [0.0, 0.0]],
            radius_sq: 0.0,
            left_son: None,
            right_son: None,
            alpha_del: 0.0,
            alpha_bal: 0.5,
        }
    }
}

impl KdTreeNode {
    fn new() -> Self {
        Self::default()
    }
}

// ============================================================================
// IKD-Tree Main Structure
// ============================================================================

/// Incremental K-D Tree for robotic applications
pub struct IkdTree {
    // Root node
    root: Option<Box<KdTreeNode>>,
    
    // Parameters
    delete_criterion_param: f64,        // deletion criterion threshold
    balance_criterion_param: f64,       // balance criterion threshold
    downsample_size: f64,               // downsample voxel size, in meters
    
    // Storage for deleted points
    points_deleted: PointVector,
    downsample_storage: PointVector,
    
    // Temporary storage for rebuild
    pcl_storage: PointVector,
    
    // Statistics
    pub max_queue_size: usize,
}

impl Default for IkdTree {
    fn default() -> Self {
        Self::new(0.5, 0.6, 0.05)
    }
}

impl IkdTree {
    /// Create a new ikd-tree with specified parameters
    ///
    /// # Arguments
    /// * `delete_param` - Threshold for deletion criterion (default 0.5)
    /// * `balance_param` - Threshold for balance criterion (default 0.6)
    /// * `box_length` - Downsample voxel size (default 0.2)
    pub fn new(delete_param: f64, balance_param: f64, box_length: f64) -> Self {
        Self {
            root: None,
            delete_criterion_param: delete_param,
            balance_criterion_param: balance_param,
            downsample_size: box_length,
            points_deleted: Vec::new(),
            downsample_storage: Vec::new(),
            pcl_storage: Vec::new(),
            max_queue_size: 0,
        }
    }

    /// Set delete criterion parameter
    pub fn set_delete_criterion_param(&mut self, delete_param: f64) {
        self.delete_criterion_param = delete_param;
    }

    /// Set balance criterion parameter
    pub fn set_balance_criterion_param(&mut self, balance_param: f64) {
        self.balance_criterion_param = balance_param;
    }

    /// Set downsample parameter (voxel size)
    pub fn set_downsample_param(&mut self, downsample_param: f64) {
        self.downsample_size = downsample_param;
    }

    /// Initialize KD tree with parameters
    pub fn initialize(&mut self, delete_param: f64, balance_param: f64, box_length: f64) {
        self.set_delete_criterion_param(delete_param);
        self.set_balance_criterion_param(balance_param);
        self.set_downsample_param(box_length);
    }

    /// Get total number of nodes in the tree
    pub fn size(&self) -> i32 {
        match &self.root {
            Some(root) => root.tree_size,
            None => 0,
        }
    }

    /// Get number of valid (non-deleted) points
    pub fn validnum(&self) -> i32 {
        match &self.root {
            Some(root) => root.tree_size - root.invalid_point_num,
            None => 0,
        }
    }

    /// Check if tree is empty
    pub fn is_empty(&self) -> bool {
        self.root.is_none()
    }

    /// Get the bounding box of the tree
    pub fn tree_range(&self) -> BoxPointType {
        match &self.root {
            Some(root) => BoxPointType {
                vertex_min: [root.node_range[0][0], root.node_range[1][0], root.node_range[2][0]],
                vertex_max: [root.node_range[0][1], root.node_range[1][1], root.node_range[2][1]],
            },
            None => BoxPointType::default(),
        }
    }

    /// Get alpha values (balance and delete ratios) of root
    pub fn root_alpha(&self) -> (f64, f64) {
        match &self.root {
            Some(root) => (root.alpha_bal, root.alpha_del),
            None => (0.5, 0.0),
        }
    }

    fn point_to_voxel_index(&self, point: &IkdTreePoint) -> (i64, i64, i64) {
        let voxel_x = ((point.x + EPSS) / self.downsample_size).floor() as i64;
        let voxel_y = ((point.y + EPSS) / self.downsample_size).floor() as i64;
        let voxel_z = ((point.z + EPSS) / self.downsample_size).floor() as i64;
        (voxel_x, voxel_y, voxel_z)
    }

    // ========================================================================
    // Build Functions
    // ========================================================================

    /// Build the tree from a point cloud
    pub fn build(&mut self, mut point_cloud: PointVector) {
        // Clear existing tree
        self.root = None;
        
        if point_cloud.is_empty() {
            return;
        }
        
        // Build tree recursively
        let len = point_cloud.len();
        self.root = Self::build_tree(&mut point_cloud, 0, len - 1);
    }

    /// Recursively build tree
    fn build_tree(storage: &mut PointVector, l: usize, r: usize) -> Option<Box<KdTreeNode>> {
        if l > r {
            return None;
        }
        
        let mut node = Box::new(KdTreeNode::new());
        let mid = (l + r) / 2;
        
        // Find the best division axis (axis with maximum range)
        let mut min_value = [f64::INFINITY; 3];
        let mut max_value = [f64::NEG_INFINITY; 3];
        
        for i in l..=r {
            let p = &storage[i];
            min_value[0] = min_value[0].min(p.x);
            min_value[1] = min_value[1].min(p.y);
            min_value[2] = min_value[2].min(p.z);
            max_value[0] = max_value[0].max(p.x);
            max_value[1] = max_value[1].max(p.y);
            max_value[2] = max_value[2].max(p.z);
        }
        
        let dim_range = [
            max_value[0] - min_value[0],
            max_value[1] - min_value[1],
            max_value[2] - min_value[2],
        ];
        
        let div_axis = if dim_range[1] > dim_range[0] && dim_range[1] > dim_range[2] {
            1
        } else if dim_range[2] > dim_range[0] && dim_range[2] > dim_range[1] {
            2
        } else {
            0
        };
        
        node.division_axis = div_axis as u8;
        
        // Partition points using nth_element equivalent
        let slice = &mut storage[l..=r];
        let mid_idx = mid - l;
        slice.select_nth_unstable_by(mid_idx, |a, b| {
            a.coord(div_axis).partial_cmp(&b.coord(div_axis)).unwrap_or(Ordering::Equal)
        });
        
        node.point = storage[mid];
        
        // Recursively build children
        if mid > l {
            node.left_son = Self::build_tree(storage, l, mid - 1);
        }
        if mid < r {
            node.right_son = Self::build_tree(storage, mid + 1, r);
        }
        
        // Update node statistics
        Self::update_node(&mut node);
        
        Some(node)
    }

    // ========================================================================
    // Search Functions
    // ========================================================================

    /// K-nearest neighbor search
    ///
    /// # Arguments
    /// * `point` - Query point
    /// * `k_nearest` - Number of nearest neighbors to find
    /// * `max_dist` - Maximum search distance
    ///
    /// # Returns
    /// * Tuple of (nearest_points, distances)
    pub fn nearest_search(
        &self,
        point: &IkdTreePoint,
        k_nearest: usize,
        max_dist: f64,
    ) -> (PointVector, Vec<f64>) {
        let mut heap = ManualHeap::new(2 * k_nearest);
        
        if let Some(root) = &self.root {
            Self::search(root, k_nearest, point, &mut heap, max_dist);
        }
        
        let k_found = k_nearest.min(heap.size());
        let mut nearest_points = Vec::with_capacity(k_found);
        let mut distances = Vec::with_capacity(k_found);
        
        // Extract results from heap (they come out in reverse order)
        while heap.size() > 0 {
            if let Some(top) = heap.top() {
                nearest_points.insert(0, top.point);
                distances.insert(0, top.dist);
            }
            heap.pop();
        }
        
        // Keep only k_found results
        nearest_points.truncate(k_found);
        distances.truncate(k_found);
        
        (nearest_points, distances)
    }

    /// Recursive search function
    fn search(
        node: &KdTreeNode,
        k_nearest: usize,
        point: &IkdTreePoint,
        heap: &mut ManualHeap,
        max_dist: f64,
    ) {
        if node.tree_deleted {
            return;
        }
        
        // Check if this subtree can possibly contain nearer neighbors
        let cur_dist = Self::calc_box_dist(node, point);
        let max_dist_sqr = max_dist * max_dist;
        if cur_dist > max_dist_sqr {
            return;
        }
        
        // Check current node's point
        if !node.point_deleted {
            let dist = Self::calc_dist(&node.point, point);
            let should_add = dist <= max_dist_sqr 
                && (heap.size() < k_nearest || dist < heap.top().map_or(f64::INFINITY, |t| t.dist));
            
            if should_add {
                if heap.size() >= k_nearest {
                    heap.pop();
                }
                heap.push(PointWithDist::new(node.point, dist));
            }
        }
        
        // Calculate distances to children
        let dist_left = node.left_son.as_ref()
            .map(|n| Self::calc_box_dist(n, point))
            .unwrap_or(f64::INFINITY);
        let dist_right = node.right_son.as_ref()
            .map(|n| Self::calc_box_dist(n, point))
            .unwrap_or(f64::INFINITY);
        
        // Search children in order of proximity
        let heap_dist = heap.top().map_or(f64::INFINITY, |t| t.dist);
        
        if heap.size() < k_nearest || (dist_left < heap_dist && dist_right < heap_dist) {
            // Need to search both, prioritize closer one
            if dist_left <= dist_right {
                if let Some(left) = &node.left_son {
                    Self::search(left, k_nearest, point, heap, max_dist);
                }
                let heap_dist = heap.top().map_or(f64::INFINITY, |t| t.dist);
                if heap.size() < k_nearest || dist_right < heap_dist {
                    if let Some(right) = &node.right_son {
                        Self::search(right, k_nearest, point, heap, max_dist);
                    }
                }
            } else {
                if let Some(right) = &node.right_son {
                    Self::search(right, k_nearest, point, heap, max_dist);
                }
                let heap_dist = heap.top().map_or(f64::INFINITY, |t| t.dist);
                if heap.size() < k_nearest || dist_left < heap_dist {
                    if let Some(left) = &node.left_son {
                        Self::search(left, k_nearest, point, heap, max_dist);
                    }
                }
            }
        } else {
            // Only search children that might have better candidates
            if dist_left < heap_dist {
                if let Some(left) = &node.left_son {
                    Self::search(left, k_nearest, point, heap, max_dist);
                }
            }
            if dist_right < heap_dist {
                if let Some(right) = &node.right_son {
                    Self::search(right, k_nearest, point, heap, max_dist);
                }
            }
        }
    }

    /// Box search - find all points within a box
    pub fn box_search(&self, boxpoint: &BoxPointType) -> PointVector {
        let mut storage = Vec::new();
        if let Some(root) = &self.root {
            Self::search_by_range(root, boxpoint, &mut storage);
        }
        storage
    }

    /// Recursive box search
    fn search_by_range(node: &KdTreeNode, boxpoint: &BoxPointType, storage: &mut PointVector) {
        // Check intersection with node's bounding box
        if !boxpoint.intersects_range(&node.node_range) {
            return;
        }
        
        // Check if box completely contains node's range
        if boxpoint.contains_range(&node.node_range) {
            Self::flatten(node, storage, DeletePointStorageType::NotRecord, &mut Vec::new());
            return;
        }
        
        // Check current point
        if !node.point_deleted && boxpoint.contains(&node.point) {
            storage.push(node.point);
        }
        
        // Recurse into children
        if let Some(left) = &node.left_son {
            Self::search_by_range(left, boxpoint, storage);
        }
        if let Some(right) = &node.right_son {
            Self::search_by_range(right, boxpoint, storage);
        }
    }

    /// Radius search - find all points within radius of a point
    pub fn radius_search(&self, point: &IkdTreePoint, radius: f64) -> PointVector {
        let mut storage = Vec::new();
        if let Some(root) = &self.root {
            Self::search_by_radius(root, point, radius, &mut storage);
        }
        storage
    }

    /// Recursive radius search
    fn search_by_radius(
        node: &KdTreeNode,
        point: &IkdTreePoint,
        radius: f64,
        storage: &mut PointVector,
    ) {
        // Calculate distance from point to node's bounding box center
        let range_center = IkdTreePoint::new(
            (node.node_range[0][0] + node.node_range[0][1]) * 0.5,
            (node.node_range[1][0] + node.node_range[1][1]) * 0.5,
            (node.node_range[2][0] + node.node_range[2][1]) * 0.5,
        );
        
        let dist = Self::calc_dist(&range_center, point).sqrt();
        if dist > radius + node.radius_sq.sqrt() {
            return;
        }
        
        // If entire subtree is within radius
        if dist <= radius - node.radius_sq.sqrt() {
            Self::flatten(node, storage, DeletePointStorageType::NotRecord, &mut Vec::new());
            return;
        }
        
        // Check current point
        if !node.point_deleted && Self::calc_dist(&node.point, point) <= radius * radius {
            storage.push(node.point);
        }
        
        // Recurse into children
        if let Some(left) = &node.left_son {
            Self::search_by_radius(left, point, radius, storage);
        }
        if let Some(right) = &node.right_son {
            Self::search_by_radius(right, point, radius, storage);
        }
    }

    // ========================================================================
    // Add Points
    // ========================================================================

    /// Add points to the tree
    ///
    /// # Arguments
    /// * `points` - Points to add
    /// * `downsample_on` - Whether to enable downsampling
    ///
    /// # Returns
    /// * Number of points actually added
    pub fn add_points(&mut self, points: &PointVector, downsample_on: bool) -> i32 {
        let downsample_switch = downsample_on && DOWNSAMPLE_SWITCH;
        let mut tmp_counter = 0;
        
        for point in points {
            if downsample_switch {
                // Calculate voxel indices (snap to grid)
                let (voxel_x, voxel_y, voxel_z) = self.point_to_voxel_index(point);
                
                // Calculate voxel box for this point
                let box_of_point = BoxPointType {
                    vertex_min: [
                        voxel_x as f64 * self.downsample_size - EPSS,
                        voxel_y as f64 * self.downsample_size - EPSS,
                        voxel_z as f64 * self.downsample_size - EPSS,
                    ],
                    vertex_max: [
                        (voxel_x as f64 + 1.0) * self.downsample_size + EPSS,
                        (voxel_y as f64 + 1.0) * self.downsample_size + EPSS,
                        (voxel_z as f64 + 1.0) * self.downsample_size + EPSS,
                    ],
                };
                
                // Calculate true voxel center (without epsilon expansion)
                let mid_point = IkdTreePoint::new(
                    (voxel_x as f64 + 0.5) * self.downsample_size,
                    (voxel_y as f64 + 0.5) * self.downsample_size,
                    (voxel_z as f64 + 0.5) * self.downsample_size,
                );
                
                // Search for existing points in this voxel
                self.downsample_storage = self.box_search(&box_of_point);
                
                // Find the point closest to voxel center
                let mut min_dist = Self::calc_dist(point, &mid_point);
                let mut downsample_result = *point;
                
                for existing in &self.downsample_storage {
                    let tmp_dist = Self::calc_dist(existing, &mid_point);
                    if tmp_dist < min_dist {
                        min_dist = tmp_dist;
                        downsample_result = *existing;
                    }
                }
                
                // Determine if we should add/update:
                // - Empty voxel: always add the new point
                // - 1 existing point: only add if new point is closer to center (replaces old)
                // - Multiple existing: clean up and keep the best one
                let should_update = match self.downsample_storage.len() {
                    0 => true,  // Empty voxel - always add
                    1 => Self::same_point(point, &downsample_result),  // Only if new point is better
                    _ => true,  // Multiple points - need to clean up regardless
                };
                
                if should_update {
                    if !self.downsample_storage.is_empty() {
                        self.delete_by_range(&box_of_point, true, true);
                    }
                    self.add_by_point(&downsample_result, true);
                    tmp_counter += 1;
                }
            } else {
                self.add_by_point(point, true);
                tmp_counter += 1;
            }
        }
        
        tmp_counter
    }

    /// Add a single point
    fn add_by_point(&mut self, point: &IkdTreePoint, allow_rebuild: bool) {
        if self.root.is_none() {
            let mut node = Box::new(KdTreeNode::new());
            node.point = *point;
            node.division_axis = 0;
            Self::update_node(&mut node);
            self.root = Some(node);
            return;
        }
        
        let root = self.root.take().unwrap();
        self.root = Some(Self::add_by_point_recursive(root, point, allow_rebuild, 0, 
            self.delete_criterion_param, self.balance_criterion_param, &mut self.pcl_storage));
    }

    fn add_by_point_recursive(
        mut node: Box<KdTreeNode>,
        point: &IkdTreePoint,
        allow_rebuild: bool,
        _father_axis: u8,
        delete_param: f64,
        balance_param: f64,
        pcl_storage: &mut PointVector,
    ) -> Box<KdTreeNode> {
        node.working_flag = true;
        Self::push_down(&mut node);
        
        let go_left = match node.division_axis {
            0 => point.x < node.point.x,
            1 => point.y < node.point.y,
            _ => point.z < node.point.z,
        };
        
        if go_left {
            if let Some(left) = node.left_son.take() {
                node.left_son = Some(Self::add_by_point_recursive(
                    left, point, allow_rebuild, node.division_axis, delete_param, balance_param, pcl_storage
                ));
            } else {
                let mut new_node = Box::new(KdTreeNode::new());
                new_node.point = *point;
                new_node.division_axis = (node.division_axis + 1) % 3;
                Self::update_node(&mut new_node);
                node.left_son = Some(new_node);
            }
        } else {
            if let Some(right) = node.right_son.take() {
                node.right_son = Some(Self::add_by_point_recursive(
                    right, point, allow_rebuild, node.division_axis, delete_param, balance_param, pcl_storage
                ));
            } else {
                let mut new_node = Box::new(KdTreeNode::new());
                new_node.point = *point;
                new_node.division_axis = (node.division_axis + 1) % 3;
                Self::update_node(&mut new_node);
                node.right_son = Some(new_node);
            }
        }
        
        Self::update_node(&mut node);
        
        // Check if rebuild is needed
        let need_rebuild = allow_rebuild && Self::criterion_check(&node, delete_param, balance_param);
        if need_rebuild {
            node = Self::rebuild_node(node, pcl_storage);
        }
        
        node.working_flag = false;
        node
    }

    // ========================================================================
    // Delete Points
    // ========================================================================

    /// Delete points from the tree
    pub fn delete_points(&mut self, points: &PointVector) {
        for point in points {
            self.delete_by_point(point, true);
        }
    }

    /// Delete a single point
    fn delete_by_point(&mut self, point: &IkdTreePoint, allow_rebuild: bool) {
        if let Some(root) = self.root.take() {
            self.root = Some(Self::delete_by_point_recursive(
                root, point, allow_rebuild,
                self.delete_criterion_param, self.balance_criterion_param,
                &mut self.pcl_storage, &mut self.points_deleted
            ));
        }
    }

    fn delete_by_point_recursive(
        mut node: Box<KdTreeNode>,
        point: &IkdTreePoint,
        allow_rebuild: bool,
        delete_param: f64,
        balance_param: f64,
        pcl_storage: &mut PointVector,
        points_deleted: &mut PointVector,
    ) -> Box<KdTreeNode> {
        if node.tree_deleted {
            return node;
        }
        
        node.working_flag = true;
        Self::push_down(&mut node);
        
        // Check if this is the point to delete
        if Self::same_point(&node.point, point) && !node.point_deleted {
            node.point_deleted = true;
            node.invalid_point_num += 1;
            if node.invalid_point_num == node.tree_size {
                node.tree_deleted = true;
            }
            node.working_flag = false;
            return node;
        }
        
        // Determine which subtree to search
        let go_left = match node.division_axis {
            0 => point.x < node.point.x,
            1 => point.y < node.point.y,
            _ => point.z < node.point.z,
        };
        
        if go_left {
            if let Some(left) = node.left_son.take() {
                node.left_son = Some(Self::delete_by_point_recursive(
                    left, point, allow_rebuild, delete_param, balance_param, pcl_storage, points_deleted
                ));
            }
        } else {
            if let Some(right) = node.right_son.take() {
                node.right_son = Some(Self::delete_by_point_recursive(
                    right, point, allow_rebuild, delete_param, balance_param, pcl_storage, points_deleted
                ));
            }
        }
        
        Self::update_node(&mut node);
        
        let need_rebuild = allow_rebuild && Self::criterion_check(&node, delete_param, balance_param);
        if need_rebuild {
            node = Self::rebuild_node(node, pcl_storage);
        }
        
        node.working_flag = false;
        node
    }

    /// Delete points within boxes
    pub fn delete_point_boxes(&mut self, boxes: &[BoxPointType]) -> i32 {
        let mut tmp_counter = 0;
        for boxpoint in boxes {
            tmp_counter += self.delete_by_range(boxpoint, true, false);
        }
        tmp_counter
    }

    /// Delete points within a range
    fn delete_by_range(&mut self, boxpoint: &BoxPointType, allow_rebuild: bool, is_downsample: bool) -> i32 {
        if let Some(root) = self.root.take() {
            let (new_root, count) = Self::delete_by_range_recursive(
                root, boxpoint, allow_rebuild, is_downsample,
                self.delete_criterion_param, self.balance_criterion_param,
                &mut self.pcl_storage, &mut self.points_deleted
            );
            self.root = Some(new_root);
            return count;
        }
        0
    }

    fn delete_by_range_recursive(
        mut node: Box<KdTreeNode>,
        boxpoint: &BoxPointType,
        allow_rebuild: bool,
        is_downsample: bool,
        delete_param: f64,
        balance_param: f64,
        pcl_storage: &mut PointVector,
        points_deleted: &mut PointVector,
    ) -> (Box<KdTreeNode>, i32) {
        if node.tree_deleted {
            return (node, 0);
        }
        
        node.working_flag = true;
        Self::push_down(&mut node);
        
        let mut tmp_counter = 0;
        
        // Check intersection with node's bounding box
        if !boxpoint.intersects_range(&node.node_range) {
            node.working_flag = false;
            return (node, 0);
        }
        
        // Check if box completely contains node's range - mark entire subtree as deleted
        if boxpoint.contains_range(&node.node_range) {
            node.tree_deleted = true;
            node.point_deleted = true;
            node.need_push_down_to_left = true;
            node.need_push_down_to_right = true;
            tmp_counter = node.tree_size - node.invalid_point_num;
            node.invalid_point_num = node.tree_size;
            
            if is_downsample {
                node.tree_downsample_deleted = true;
                node.point_downsample_deleted = true;
                node.down_del_num = node.tree_size;
            }
            
            node.working_flag = false;
            return (node, tmp_counter);
        }
        
        // Check if current point is in box
        if !node.point_deleted && boxpoint.contains(&node.point) {
            node.point_deleted = true;
            tmp_counter += 1;
            if is_downsample {
                node.point_downsample_deleted = true;
            }
        }
        
        // Recurse into children
        if let Some(left) = node.left_son.take() {
            let (new_left, count) = Self::delete_by_range_recursive(
                left, boxpoint, allow_rebuild, is_downsample,
                delete_param, balance_param, pcl_storage, points_deleted
            );
            node.left_son = Some(new_left);
            tmp_counter += count;
        }
        
        if let Some(right) = node.right_son.take() {
            let (new_right, count) = Self::delete_by_range_recursive(
                right, boxpoint, allow_rebuild, is_downsample,
                delete_param, balance_param, pcl_storage, points_deleted
            );
            node.right_son = Some(new_right);
            tmp_counter += count;
        }
        
        Self::update_node(&mut node);
        
        let need_rebuild = allow_rebuild && Self::criterion_check(&node, delete_param, balance_param);
        if need_rebuild {
            node = Self::rebuild_node(node, pcl_storage);
        }
        
        node.working_flag = false;
        (node, tmp_counter)
    }

    // ========================================================================
    // Utility Functions
    // ========================================================================

    /// Flatten tree to a point vector
    fn flatten(
        node: &KdTreeNode,
        storage: &mut PointVector,
        storage_type: DeletePointStorageType,
        points_deleted: &mut PointVector,
    ) {
        if !node.point_deleted {
            storage.push(node.point);
        } else if matches!(storage_type, DeletePointStorageType::DeletePointsRec | DeletePointStorageType::MultiThreadRec)
            && node.point_deleted && !node.point_downsample_deleted
        {
            points_deleted.push(node.point);
        }
        
        if let Some(left) = &node.left_son {
            Self::flatten(left, storage, storage_type, points_deleted);
        }
        if let Some(right) = &node.right_son {
            Self::flatten(right, storage, storage_type, points_deleted);
        }
    }

    /// Collect all valid points in the tree
    pub fn flatten_all(&self) -> PointVector {
        let mut storage = Vec::new();
        if let Some(root) = &self.root {
            Self::flatten(root, &mut storage, DeletePointStorageType::NotRecord, &mut Vec::new());
        }
        storage
    }

    /// Get removed points and clear the storage
    pub fn acquire_removed_points(&mut self) -> PointVector {
        std::mem::take(&mut self.points_deleted)
    }

    /// Push down deletion flags to children
    fn push_down(node: &mut KdTreeNode) {
        if node.need_push_down_to_left {
            if let Some(left) = &mut node.left_son {
                left.tree_downsample_deleted |= node.tree_downsample_deleted;
                left.point_downsample_deleted |= node.tree_downsample_deleted;
                left.tree_deleted = node.tree_deleted || left.tree_downsample_deleted;
                left.point_deleted = left.tree_deleted || left.point_downsample_deleted;
                
                if node.tree_downsample_deleted {
                    left.down_del_num = left.tree_size;
                }
                if node.tree_deleted {
                    left.invalid_point_num = left.tree_size;
                } else {
                    left.invalid_point_num = left.down_del_num;
                }
                
                left.need_push_down_to_left = true;
                left.need_push_down_to_right = true;
            }
            node.need_push_down_to_left = false;
        }
        
        if node.need_push_down_to_right {
            if let Some(right) = &mut node.right_son {
                right.tree_downsample_deleted |= node.tree_downsample_deleted;
                right.point_downsample_deleted |= node.tree_downsample_deleted;
                right.tree_deleted = node.tree_deleted || right.tree_downsample_deleted;
                right.point_deleted = right.tree_deleted || right.point_downsample_deleted;
                
                if node.tree_downsample_deleted {
                    right.down_del_num = right.tree_size;
                }
                if node.tree_deleted {
                    right.invalid_point_num = right.tree_size;
                } else {
                    right.invalid_point_num = right.down_del_num;
                }
                
                right.need_push_down_to_left = true;
                right.need_push_down_to_right = true;
            }
            node.need_push_down_to_right = false;
        }
    }

    /// Update node statistics (tree size, ranges, etc.)
    fn update_node(node: &mut KdTreeNode) {
        let left = &node.left_son;
        let right = &node.right_son;
        
        let mut tmp_range = [[f64::INFINITY, f64::NEG_INFINITY]; 3];
        
        match (left, right) {
            (Some(l), Some(r)) => {
                node.tree_size = l.tree_size + r.tree_size + 1;
                node.invalid_point_num = l.invalid_point_num + r.invalid_point_num + if node.point_deleted { 1 } else { 0 };
                node.down_del_num = l.down_del_num + r.down_del_num + if node.point_downsample_deleted { 1 } else { 0 };
                node.tree_downsample_deleted = l.tree_downsample_deleted && r.tree_downsample_deleted && node.point_downsample_deleted;
                node.tree_deleted = l.tree_deleted && r.tree_deleted && node.point_deleted;
                
                if node.tree_deleted || (!l.tree_deleted && !r.tree_deleted && !node.point_deleted) {
                    for i in 0..3 {
                        tmp_range[i][0] = l.node_range[i][0].min(r.node_range[i][0]).min(node.point.coord(i));
                        tmp_range[i][1] = l.node_range[i][1].max(r.node_range[i][1]).max(node.point.coord(i));
                    }
                } else {
                    if !l.tree_deleted {
                        for i in 0..3 {
                            tmp_range[i][0] = tmp_range[i][0].min(l.node_range[i][0]);
                            tmp_range[i][1] = tmp_range[i][1].max(l.node_range[i][1]);
                        }
                    }
                    if !r.tree_deleted {
                        for i in 0..3 {
                            tmp_range[i][0] = tmp_range[i][0].min(r.node_range[i][0]);
                            tmp_range[i][1] = tmp_range[i][1].max(r.node_range[i][1]);
                        }
                    }
                    if !node.point_deleted {
                        for i in 0..3 {
                            tmp_range[i][0] = tmp_range[i][0].min(node.point.coord(i));
                            tmp_range[i][1] = tmp_range[i][1].max(node.point.coord(i));
                        }
                    }
                }
            }
            (Some(l), None) => {
                node.tree_size = l.tree_size + 1;
                node.invalid_point_num = l.invalid_point_num + if node.point_deleted { 1 } else { 0 };
                node.down_del_num = l.down_del_num + if node.point_downsample_deleted { 1 } else { 0 };
                node.tree_downsample_deleted = l.tree_downsample_deleted && node.point_downsample_deleted;
                node.tree_deleted = l.tree_deleted && node.point_deleted;
                
                if node.tree_deleted || (!l.tree_deleted && !node.point_deleted) {
                    for i in 0..3 {
                        tmp_range[i][0] = l.node_range[i][0].min(node.point.coord(i));
                        tmp_range[i][1] = l.node_range[i][1].max(node.point.coord(i));
                    }
                } else {
                    if !l.tree_deleted {
                        for i in 0..3 {
                            tmp_range[i][0] = tmp_range[i][0].min(l.node_range[i][0]);
                            tmp_range[i][1] = tmp_range[i][1].max(l.node_range[i][1]);
                        }
                    }
                    if !node.point_deleted {
                        for i in 0..3 {
                            tmp_range[i][0] = tmp_range[i][0].min(node.point.coord(i));
                            tmp_range[i][1] = tmp_range[i][1].max(node.point.coord(i));
                        }
                    }
                }
            }
            (None, Some(r)) => {
                node.tree_size = r.tree_size + 1;
                node.invalid_point_num = r.invalid_point_num + if node.point_deleted { 1 } else { 0 };
                node.down_del_num = r.down_del_num + if node.point_downsample_deleted { 1 } else { 0 };
                node.tree_downsample_deleted = r.tree_downsample_deleted && node.point_downsample_deleted;
                node.tree_deleted = r.tree_deleted && node.point_deleted;
                
                if node.tree_deleted || (!r.tree_deleted && !node.point_deleted) {
                    for i in 0..3 {
                        tmp_range[i][0] = r.node_range[i][0].min(node.point.coord(i));
                        tmp_range[i][1] = r.node_range[i][1].max(node.point.coord(i));
                    }
                } else {
                    if !r.tree_deleted {
                        for i in 0..3 {
                            tmp_range[i][0] = tmp_range[i][0].min(r.node_range[i][0]);
                            tmp_range[i][1] = tmp_range[i][1].max(r.node_range[i][1]);
                        }
                    }
                    if !node.point_deleted {
                        for i in 0..3 {
                            tmp_range[i][0] = tmp_range[i][0].min(node.point.coord(i));
                            tmp_range[i][1] = tmp_range[i][1].max(node.point.coord(i));
                        }
                    }
                }
            }
            (None, None) => {
                node.tree_size = 1;
                node.invalid_point_num = if node.point_deleted { 1 } else { 0 };
                node.down_del_num = if node.point_downsample_deleted { 1 } else { 0 };
                node.tree_downsample_deleted = node.point_downsample_deleted;
                node.tree_deleted = node.point_deleted;
                
                for i in 0..3 {
                    tmp_range[i][0] = node.point.coord(i);
                    tmp_range[i][1] = node.point.coord(i);
                }
            }
        }
        
        node.node_range = tmp_range;
        
        // Calculate bounding sphere radius
        let x_l = (node.node_range[0][1] - node.node_range[0][0]) * 0.5;
        let y_l = (node.node_range[1][1] - node.node_range[1][0]) * 0.5;
        let z_l = (node.node_range[2][1] - node.node_range[2][0]) * 0.5;
        node.radius_sq = x_l * x_l + y_l * y_l + z_l * z_l;
        
        // Update alpha values for root statistics
        if node.tree_size > 3 {
            let son_size = node.left_son.as_ref()
                .map(|l| l.tree_size)
                .or_else(|| node.right_son.as_ref().map(|r| r.tree_size))
                .unwrap_or(0);
            
            let tmp_bal = son_size as f64 / (node.tree_size - 1) as f64;
            node.alpha_del = node.invalid_point_num as f64 / node.tree_size as f64;
            node.alpha_bal = if tmp_bal >= 0.5 - EPSS { tmp_bal } else { 1.0 - tmp_bal };
        }
    }

    /// Check if rebuild is needed based on balance and deletion criteria
    fn criterion_check(node: &KdTreeNode, delete_param: f64, balance_param: f64) -> bool {
        if node.tree_size <= MINIMUM_UNBALANCED_TREE_SIZE as i32 {
            return false;
        }
        
        let son_ptr = node.left_son.as_ref().or(node.right_son.as_ref());
        if son_ptr.is_none() {
            return false;
        }
        let son = son_ptr.unwrap();
        
        let delete_evaluation = node.invalid_point_num as f64 / node.tree_size as f64;
        let balance_evaluation = son.tree_size as f64 / (node.tree_size - 1) as f64;
        
        if delete_evaluation > delete_param {
            return true;
        }
        
        if balance_evaluation > balance_param || balance_evaluation < 1.0 - balance_param {
            return true;
        }
        
        false
    }

    /// Rebuild a subtree
    fn rebuild_node(node: Box<KdTreeNode>, pcl_storage: &mut PointVector) -> Box<KdTreeNode> {
        pcl_storage.clear();
        Self::flatten(&node, pcl_storage, DeletePointStorageType::DeletePointsRec, &mut Vec::new());
        
        if pcl_storage.is_empty() {
            return node;
        }
        
        if let Some(new_node) = Self::build_tree(pcl_storage, 0, pcl_storage.len() - 1) {
            new_node
        } else {
            node
        }
    }

    /// Check if two points are the same
    #[inline]
    fn same_point(a: &IkdTreePoint, b: &IkdTreePoint) -> bool {
        (a.x - b.x).abs() < EPSS && (a.y - b.y).abs() < EPSS && (a.z - b.z).abs() < EPSS
    }

    /// Calculate squared distance between two points
    #[inline]
    fn calc_dist(a: &IkdTreePoint, b: &IkdTreePoint) -> f64 {
        let dx = a.x - b.x;
        let dy = a.y - b.y;
        let dz = a.z - b.z;
        dx * dx + dy * dy + dz * dz
    }

    /// Calculate squared distance from a point to a node's bounding box
    #[inline]
    fn calc_box_dist(node: &KdTreeNode, point: &IkdTreePoint) -> f64 {
        let mut min_dist = 0.0;
        
        if point.x < node.node_range[0][0] {
            let d = point.x - node.node_range[0][0];
            min_dist += d * d;
        } else if point.x > node.node_range[0][1] {
            let d = point.x - node.node_range[0][1];
            min_dist += d * d;
        }
        
        if point.y < node.node_range[1][0] {
            let d = point.y - node.node_range[1][0];
            min_dist += d * d;
        } else if point.y > node.node_range[1][1] {
            let d = point.y - node.node_range[1][1];
            min_dist += d * d;
        }
        
        if point.z < node.node_range[2][0] {
            let d = point.z - node.node_range[2][0];
            min_dist += d * d;
        } else if point.z > node.node_range[2][1] {
            let d = point.z - node.node_range[2][1];
            min_dist += d * d;
        }
        
        min_dist
    }
}

// ============================================================================
// Tests
// ============================================================================

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_basic_build_and_search() {
        let points: PointVector = vec![
            IkdTreePoint::new(0.0, 0.0, 0.0),
            IkdTreePoint::new(1.0, 0.0, 0.0),
            IkdTreePoint::new(0.0, 1.0, 0.0),
            IkdTreePoint::new(0.0, 0.0, 1.0),
            IkdTreePoint::new(1.0, 1.0, 1.0),
        ];
        
        let mut tree = IkdTree::default();
        tree.build(points.clone());
        
        assert_eq!(tree.size(), 5);
        assert_eq!(tree.validnum(), 5);
        
        // Test nearest search
        let query = IkdTreePoint::new(0.1, 0.1, 0.1);
        let (nearest, distances) = tree.nearest_search(&query, 1, f64::INFINITY);
        
        assert_eq!(nearest.len(), 1);
        // Should find (0, 0, 0) as nearest
        assert!((nearest[0].x - 0.0).abs() < 0.01);
    }

    #[test]
    fn test_add_points() {
        let mut tree = IkdTree::default();
        
        let points: PointVector = vec![
            IkdTreePoint::new(0.0, 0.0, 0.0),
            IkdTreePoint::new(1.0, 0.0, 0.0),
        ];
        
        tree.build(points);
        assert_eq!(tree.size(), 2);
        
        let new_points: PointVector = vec![
            IkdTreePoint::new(2.0, 0.0, 0.0),
        ];
        
        tree.add_points(&new_points, false);
        assert_eq!(tree.size(), 3);
    }

    #[test]
    fn test_box_search() {
        let points: PointVector = vec![
            IkdTreePoint::new(0.0, 0.0, 0.0),
            IkdTreePoint::new(1.0, 0.0, 0.0),
            IkdTreePoint::new(0.0, 1.0, 0.0),
            IkdTreePoint::new(5.0, 5.0, 5.0),
        ];
        
        let mut tree = IkdTree::default();
        tree.build(points);
        
        let search_box = BoxPointType::new([-0.5, -0.5, -0.5], [1.5, 1.5, 1.5]);
        let results = tree.box_search(&search_box);
        
        assert_eq!(results.len(), 3);
    }

    #[test]
    fn test_delete_by_range() {
        let points: PointVector = vec![
            IkdTreePoint::new(0.0, 0.0, 0.0),
            IkdTreePoint::new(1.0, 0.0, 0.0),
            IkdTreePoint::new(0.0, 1.0, 0.0),
            IkdTreePoint::new(5.0, 5.0, 5.0),
        ];
        
        let mut tree = IkdTree::default();
        tree.build(points);
        
        assert_eq!(tree.validnum(), 4);
        
        let delete_box = BoxPointType::new([-0.5, -0.5, -0.5], [1.5, 1.5, 1.5]);
        let deleted = tree.delete_point_boxes(&[delete_box]);
        
        assert_eq!(deleted, 3);
        assert_eq!(tree.validnum(), 1);
    }

    #[test]
    fn test_radius_search() {
        let points: PointVector = vec![
            IkdTreePoint::new(0.0, 0.0, 0.0),
            IkdTreePoint::new(0.5, 0.0, 0.0),
            IkdTreePoint::new(2.0, 0.0, 0.0),
        ];
        
        let mut tree = IkdTree::default();
        tree.build(points);
        
        let query = IkdTreePoint::new(0.0, 0.0, 0.0);
        let results = tree.radius_search(&query, 1.0);
        
        assert_eq!(results.len(), 2); // (0,0,0) and (0.5,0,0)
    }

    #[test]
    fn test_add_points_with_downsample_from_empty() {
        // Test adding points to an empty tree with downsampling enabled
        let mut tree = IkdTree::new(0.5, 0.6, 0.5); // 0.5m voxel size
        
        // Points in different voxels (spread out by more than voxel size)
        let points: PointVector = vec![
            IkdTreePoint::new(0.0, 0.0, 0.0),   // voxel [0, 0.5)
            IkdTreePoint::new(1.0, 0.0, 0.0),   // voxel [0.5, 1) - different voxel
            IkdTreePoint::new(2.0, 0.0, 0.0),   // voxel [1.5, 2) - different voxel
            IkdTreePoint::new(0.0, 1.0, 0.0),   // voxel [0, 0.5) x [0.5, 1) - different voxel
            IkdTreePoint::new(0.0, 0.0, 1.0),   // voxel [0, 0.5) x [0, 0.5) x [0.5, 1) - different
        ];
        
        let added = tree.add_points(&points, true); // downsample ON
        
        // All 5 points should be added (they're in different voxels)
        assert_eq!(tree.validnum(), 5, "Expected 5 points in different voxels");
        assert_eq!(added, 5, "Expected 5 points to be added");
    }

    #[test]
    fn test_add_points_downsample_same_voxel() {
        // Test that points in the same voxel get downsampled to one
        let mut tree = IkdTree::new(0.5, 0.6, 1.0); // 1.0m voxel size
        
        // All points within 1m voxel [0, 1)^3, center at (0.5, 0.5, 0.5)
        let points: PointVector = vec![
            IkdTreePoint::new(0.1, 0.1, 0.1),
            IkdTreePoint::new(0.2, 0.2, 0.2),
            IkdTreePoint::new(0.5, 0.5, 0.5),  // This is closest to center
            IkdTreePoint::new(0.8, 0.8, 0.8),
            IkdTreePoint::new(0.9, 0.9, 0.9),
        ];
        
        tree.add_points(&points, true); // downsample ON
        
        // Only 1 point should remain (the one closest to voxel center)
        assert_eq!(tree.validnum(), 1, "Expected only 1 point after downsampling same voxel");
        
        // The remaining point should be (0.5, 0.5, 0.5) - closest to center
        let all_points = tree.flatten_all();
        assert_eq!(all_points.len(), 1);
        assert!((all_points[0].x - 0.5).abs() < 0.01, "Expected point at voxel center");
    }

    #[test]
    fn test_add_points_downsample_boundary_noise() {
        // Test that small floating-point noise at voxel boundaries doesn't create duplicates
        let mut tree = IkdTree::new(0.5, 0.6, 0.1); // 0.1m voxel size
        
        // Simulate a static point with tiny sensor noise across multiple frames
        // All these points should end up in the same voxel around x=0.1
        let frame1: PointVector = vec![
            IkdTreePoint::new(0.10000001, 0.05, 0.05),
        ];
        let frame2: PointVector = vec![
            IkdTreePoint::new(0.09999999, 0.05, 0.05),  // Just below boundary
        ];
        let frame3: PointVector = vec![
            IkdTreePoint::new(0.10000002, 0.0500001, 0.05),
        ];
        let frame4: PointVector = vec![
            IkdTreePoint::new(0.1, 0.05, 0.05),
        ];
        
        tree.add_points(&frame1, true);
        assert_eq!(tree.validnum(), 1, "Frame 1: should have 1 point");
        
        tree.add_points(&frame2, true);
        assert_eq!(tree.validnum(), 1, "Frame 2: should still have 1 point (boundary noise)");
        
        tree.add_points(&frame3, true);
        assert_eq!(tree.validnum(), 1, "Frame 3: should still have 1 point");
        
        tree.add_points(&frame4, true);
        assert_eq!(tree.validnum(), 1, "Frame 4: should still have 1 point");
    }
}
