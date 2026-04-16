// 2D Physics — wraps Rapier2D for rigid body simulation
//
// Provides C ABI functions for:
//   - Desktop JVM via Panama FFM (java.lang.foreign)
//   - Scala Native via @extern
//
// All public functions are prefixed with sge_phys_ to avoid symbol collisions.
// The world state is stored in a heap-allocated PhysicsWorld struct, passed
// as an opaque *mut c_void handle. Body/collider/joint handles are Rapier's
// internal indices encoded as u64.

use std::ffi::c_void;
use std::slice;

use rapier2d::prelude::*;

// ---------------------------------------------------------------------------
// World state
// ---------------------------------------------------------------------------

struct PhysicsWorld {
    gravity: Vector<Real>,
    integration_parameters: IntegrationParameters,
    physics_pipeline: PhysicsPipeline,
    island_manager: IslandManager,
    broad_phase: DefaultBroadPhase,
    narrow_phase: NarrowPhase,
    rigid_body_set: RigidBodySet,
    collider_set: ColliderSet,
    impulse_joint_set: ImpulseJointSet,
    multibody_joint_set: MultibodyJointSet,
    ccd_solver: CCDSolver,
    query_pipeline: QueryPipeline,
    // Contact event buffers
    contact_start_buf: Vec<(u64, u64)>,
    contact_stop_buf: Vec<(u64, u64)>,
}

impl PhysicsWorld {
    fn new(gx: f32, gy: f32) -> Self {
        PhysicsWorld {
            gravity: vector![gx, gy],
            integration_parameters: IntegrationParameters::default(),
            physics_pipeline: PhysicsPipeline::new(),
            island_manager: IslandManager::new(),
            broad_phase: DefaultBroadPhase::new(),
            narrow_phase: NarrowPhase::new(),
            rigid_body_set: RigidBodySet::new(),
            collider_set: ColliderSet::new(),
            impulse_joint_set: ImpulseJointSet::new(),
            multibody_joint_set: MultibodyJointSet::new(),
            ccd_solver: CCDSolver::new(),
            query_pipeline: QueryPipeline::new(),
            contact_start_buf: Vec::new(),
            contact_stop_buf: Vec::new(),
        }
    }

    fn step(&mut self, dt: f32) {
        self.integration_parameters.dt = dt;

        // Collect contact events before stepping
        self.contact_start_buf.clear();
        self.contact_stop_buf.clear();

        let (contact_send, contact_recv) = crossbeam::channel::unbounded();
        let (intersection_send, _intersection_recv) = crossbeam::channel::unbounded();

        let event_handler = ChannelEventCollector::new(contact_send, intersection_send);

        self.physics_pipeline.step(
            &self.gravity,
            &self.integration_parameters,
            &mut self.island_manager,
            &mut self.broad_phase,
            &mut self.narrow_phase,
            &mut self.rigid_body_set,
            &mut self.collider_set,
            &mut self.impulse_joint_set,
            &mut self.multibody_joint_set,
            &mut self.ccd_solver,
            None,
            &(),
            &event_handler,
        );

        // Drain collision events
        while let Ok(event) = contact_recv.try_recv() {
            match event {
                CollisionEvent::Started(c1, c2, _flags) => {
                    self.contact_start_buf.push((
                        collider_handle_to_u64(c1),
                        collider_handle_to_u64(c2),
                    ));
                }
                CollisionEvent::Stopped(c1, c2, _flags) => {
                    self.contact_stop_buf.push((
                        collider_handle_to_u64(c1),
                        collider_handle_to_u64(c2),
                    ));
                }
            }
        }

        self.query_pipeline.update(&self.collider_set);
    }
}

// ---------------------------------------------------------------------------
// Handle encoding/decoding
// ---------------------------------------------------------------------------

fn body_handle_to_u64(h: RigidBodyHandle) -> u64 {
    let (index, generation) = h.into_raw_parts();
    ((generation as u64) << 32) | (index as u64)
}

fn u64_to_body_handle(v: u64) -> RigidBodyHandle {
    let index = v as u32;
    let generation = (v >> 32) as u32;
    RigidBodyHandle::from_raw_parts(index, generation)
}

fn collider_handle_to_u64(h: ColliderHandle) -> u64 {
    let (index, generation) = h.into_raw_parts();
    ((generation as u64) << 32) | (index as u64)
}

fn u64_to_collider_handle(v: u64) -> ColliderHandle {
    let index = v as u32;
    let generation = (v >> 32) as u32;
    ColliderHandle::from_raw_parts(index, generation)
}

fn joint_handle_to_u64(h: ImpulseJointHandle) -> u64 {
    let (index, generation) = h.into_raw_parts();
    ((generation as u64) << 32) | (index as u64)
}

// ---------------------------------------------------------------------------
// World lifecycle (C ABI)
// ---------------------------------------------------------------------------

#[no_mangle]
pub extern "C" fn sge_phys_create_world(gx: f32, gy: f32) -> *mut c_void {
    let world = Box::new(PhysicsWorld::new(gx, gy));
    Box::into_raw(world) as *mut c_void
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys_destroy_world(world: *mut c_void) {
    if !world.is_null() {
        drop(Box::from_raw(world as *mut PhysicsWorld));
    }
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys_world_step(world: *mut c_void, dt: f32) {
    let w = &mut *(world as *mut PhysicsWorld);
    w.step(dt);
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys_world_set_gravity(world: *mut c_void, gx: f32, gy: f32) {
    let w = &mut *(world as *mut PhysicsWorld);
    w.gravity = vector![gx, gy];
}

/// Fills `out` with [gx, gy].
#[no_mangle]
pub unsafe extern "C" fn sge_phys_world_get_gravity(world: *mut c_void, out: *mut f32) {
    let w = &*(world as *mut PhysicsWorld);
    let arr = slice::from_raw_parts_mut(out, 2);
    arr[0] = w.gravity.x;
    arr[1] = w.gravity.y;
}

// ---------------------------------------------------------------------------
// Rigid body lifecycle
// ---------------------------------------------------------------------------

unsafe fn create_body(world: *mut c_void, body_type: RigidBodyType, x: f32, y: f32, angle: f32) -> u64 {
    let w = &mut *(world as *mut PhysicsWorld);
    let body = RigidBodyBuilder::new(body_type)
        .translation(vector![x, y])
        .rotation(angle)
        .build();
    body_handle_to_u64(w.rigid_body_set.insert(body))
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys_create_dynamic_body(world: *mut c_void, x: f32, y: f32, angle: f32) -> u64 {
    create_body(world, RigidBodyType::Dynamic, x, y, angle)
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys_create_static_body(world: *mut c_void, x: f32, y: f32, angle: f32) -> u64 {
    create_body(world, RigidBodyType::Fixed, x, y, angle)
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys_create_kinematic_body(world: *mut c_void, x: f32, y: f32, angle: f32) -> u64 {
    create_body(world, RigidBodyType::KinematicPositionBased, x, y, angle)
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys_destroy_body(world: *mut c_void, body: u64) {
    let w = &mut *(world as *mut PhysicsWorld);
    let handle = u64_to_body_handle(body);
    w.rigid_body_set.remove(
        handle,
        &mut w.island_manager,
        &mut w.collider_set,
        &mut w.impulse_joint_set,
        &mut w.multibody_joint_set,
        true,
    );
}

// ---------------------------------------------------------------------------
// Body accessors
// ---------------------------------------------------------------------------

/// Fills `out` with [x, y].
#[no_mangle]
pub unsafe extern "C" fn sge_phys_body_get_position(world: *mut c_void, body: u64, out: *mut f32) {
    let w = &*(world as *mut PhysicsWorld);
    if let Some(b) = w.rigid_body_set.get(u64_to_body_handle(body)) {
        let pos = b.translation();
        let arr = slice::from_raw_parts_mut(out, 2);
        arr[0] = pos.x;
        arr[1] = pos.y;
    }
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys_body_get_angle(world: *mut c_void, body: u64) -> f32 {
    let w = &*(world as *mut PhysicsWorld);
    w.rigid_body_set
        .get(u64_to_body_handle(body))
        .map(|b| b.rotation().angle())
        .unwrap_or(0.0)
}

/// Fills `out` with [vx, vy].
#[no_mangle]
pub unsafe extern "C" fn sge_phys_body_get_linear_velocity(world: *mut c_void, body: u64, out: *mut f32) {
    let w = &*(world as *mut PhysicsWorld);
    if let Some(b) = w.rigid_body_set.get(u64_to_body_handle(body)) {
        let vel = b.linvel();
        let arr = slice::from_raw_parts_mut(out, 2);
        arr[0] = vel.x;
        arr[1] = vel.y;
    }
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys_body_get_angular_velocity(world: *mut c_void, body: u64) -> f32 {
    let w = &*(world as *mut PhysicsWorld);
    w.rigid_body_set
        .get(u64_to_body_handle(body))
        .map(|b| b.angvel())
        .unwrap_or(0.0)
}

// ---------------------------------------------------------------------------
// Body setters
// ---------------------------------------------------------------------------

#[no_mangle]
pub unsafe extern "C" fn sge_phys_body_set_position(world: *mut c_void, body: u64, x: f32, y: f32) {
    let w = &mut *(world as *mut PhysicsWorld);
    if let Some(b) = w.rigid_body_set.get_mut(u64_to_body_handle(body)) {
        b.set_translation(vector![x, y], true);
    }
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys_body_set_angle(world: *mut c_void, body: u64, angle: f32) {
    let w = &mut *(world as *mut PhysicsWorld);
    if let Some(b) = w.rigid_body_set.get_mut(u64_to_body_handle(body)) {
        b.set_rotation(Rotation::new(angle), true);
    }
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys_body_set_linear_velocity(world: *mut c_void, body: u64, vx: f32, vy: f32) {
    let w = &mut *(world as *mut PhysicsWorld);
    if let Some(b) = w.rigid_body_set.get_mut(u64_to_body_handle(body)) {
        b.set_linvel(vector![vx, vy], true);
    }
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys_body_set_angular_velocity(world: *mut c_void, body: u64, omega: f32) {
    let w = &mut *(world as *mut PhysicsWorld);
    if let Some(b) = w.rigid_body_set.get_mut(u64_to_body_handle(body)) {
        b.set_angvel(omega, true);
    }
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys_body_apply_force(world: *mut c_void, body: u64, fx: f32, fy: f32) {
    let w = &mut *(world as *mut PhysicsWorld);
    if let Some(b) = w.rigid_body_set.get_mut(u64_to_body_handle(body)) {
        b.add_force(vector![fx, fy], true);
    }
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys_body_apply_impulse(world: *mut c_void, body: u64, ix: f32, iy: f32) {
    let w = &mut *(world as *mut PhysicsWorld);
    if let Some(b) = w.rigid_body_set.get_mut(u64_to_body_handle(body)) {
        b.apply_impulse(vector![ix, iy], true);
    }
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys_body_apply_torque(world: *mut c_void, body: u64, torque: f32) {
    let w = &mut *(world as *mut PhysicsWorld);
    if let Some(b) = w.rigid_body_set.get_mut(u64_to_body_handle(body)) {
        b.add_torque(torque, true);
    }
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys_body_set_linear_damping(world: *mut c_void, body: u64, damping: f32) {
    let w = &mut *(world as *mut PhysicsWorld);
    if let Some(b) = w.rigid_body_set.get_mut(u64_to_body_handle(body)) {
        b.set_linear_damping(damping);
    }
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys_body_set_angular_damping(world: *mut c_void, body: u64, damping: f32) {
    let w = &mut *(world as *mut PhysicsWorld);
    if let Some(b) = w.rigid_body_set.get_mut(u64_to_body_handle(body)) {
        b.set_angular_damping(damping);
    }
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys_body_set_gravity_scale(world: *mut c_void, body: u64, scale: f32) {
    let w = &mut *(world as *mut PhysicsWorld);
    if let Some(b) = w.rigid_body_set.get_mut(u64_to_body_handle(body)) {
        b.set_gravity_scale(scale, true);
    }
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys_body_is_awake(world: *mut c_void, body: u64) -> i32 {
    let w = &*(world as *mut PhysicsWorld);
    w.rigid_body_set
        .get(u64_to_body_handle(body))
        .map(|b| !b.is_sleeping() as i32)
        .unwrap_or(0)
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys_body_wake_up(world: *mut c_void, body: u64) {
    let w = &mut *(world as *mut PhysicsWorld);
    if let Some(b) = w.rigid_body_set.get_mut(u64_to_body_handle(body)) {
        b.wake_up(true);
    }
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys_body_set_fixed_rotation(world: *mut c_void, body: u64, fixed: i32) {
    let w = &mut *(world as *mut PhysicsWorld);
    if let Some(b) = w.rigid_body_set.get_mut(u64_to_body_handle(body)) {
        b.lock_rotations(fixed != 0, true);
    }
}

// ---------------------------------------------------------------------------
// Collider creation
// ---------------------------------------------------------------------------

#[no_mangle]
pub unsafe extern "C" fn sge_phys_create_circle_collider(
    world: *mut c_void,
    body: u64,
    radius: f32,
) -> u64 {
    let w = &mut *(world as *mut PhysicsWorld);
    let collider = ColliderBuilder::ball(radius).build();
    let handle = w.collider_set.insert_with_parent(
        collider,
        u64_to_body_handle(body),
        &mut w.rigid_body_set,
    );
    collider_handle_to_u64(handle)
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys_create_box_collider(
    world: *mut c_void,
    body: u64,
    half_width: f32,
    half_height: f32,
) -> u64 {
    let w = &mut *(world as *mut PhysicsWorld);
    let collider = ColliderBuilder::cuboid(half_width, half_height).build();
    let handle = w.collider_set.insert_with_parent(
        collider,
        u64_to_body_handle(body),
        &mut w.rigid_body_set,
    );
    collider_handle_to_u64(handle)
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys_create_capsule_collider(
    world: *mut c_void,
    body: u64,
    half_height: f32,
    radius: f32,
) -> u64 {
    let w = &mut *(world as *mut PhysicsWorld);
    let collider = ColliderBuilder::capsule_y(half_height, radius).build();
    let handle = w.collider_set.insert_with_parent(
        collider,
        u64_to_body_handle(body),
        &mut w.rigid_body_set,
    );
    collider_handle_to_u64(handle)
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys_create_polygon_collider(
    world: *mut c_void,
    body: u64,
    vertices: *const f32,
    vertex_count: i32,
) -> u64 {
    let w = &mut *(world as *mut PhysicsWorld);
    let verts = slice::from_raw_parts(vertices, (vertex_count * 2) as usize);
    let points: Vec<Point<Real>> = (0..vertex_count as usize)
        .map(|i| point![verts[i * 2], verts[i * 2 + 1]])
        .collect();

    let collider = ColliderBuilder::convex_hull(&points)
        .unwrap_or_else(|| ColliderBuilder::ball(0.1))
        .build();
    let handle = w.collider_set.insert_with_parent(
        collider,
        u64_to_body_handle(body),
        &mut w.rigid_body_set,
    );
    collider_handle_to_u64(handle)
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys_destroy_collider(world: *mut c_void, collider: u64) {
    let w = &mut *(world as *mut PhysicsWorld);
    w.collider_set.remove(
        u64_to_collider_handle(collider),
        &mut w.island_manager,
        &mut w.rigid_body_set,
        true,
    );
}

// ---------------------------------------------------------------------------
// Collider properties
// ---------------------------------------------------------------------------

#[no_mangle]
pub unsafe extern "C" fn sge_phys_collider_set_density(world: *mut c_void, collider: u64, density: f32) {
    let w = &mut *(world as *mut PhysicsWorld);
    if let Some(c) = w.collider_set.get_mut(u64_to_collider_handle(collider)) {
        c.set_density(density);
    }
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys_collider_set_friction(world: *mut c_void, collider: u64, friction: f32) {
    let w = &mut *(world as *mut PhysicsWorld);
    if let Some(c) = w.collider_set.get_mut(u64_to_collider_handle(collider)) {
        c.set_friction(friction);
    }
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys_collider_set_restitution(world: *mut c_void, collider: u64, restitution: f32) {
    let w = &mut *(world as *mut PhysicsWorld);
    if let Some(c) = w.collider_set.get_mut(u64_to_collider_handle(collider)) {
        c.set_restitution(restitution);
    }
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys_collider_set_sensor(world: *mut c_void, collider: u64, sensor: i32) {
    let w = &mut *(world as *mut PhysicsWorld);
    if let Some(c) = w.collider_set.get_mut(u64_to_collider_handle(collider)) {
        c.set_sensor(sensor != 0);
    }
}

// ---------------------------------------------------------------------------
// Collision filtering
// ---------------------------------------------------------------------------

/// Sets the collision groups for a collider.
/// `memberships` defines which groups this collider belongs to.
/// `filter` defines which groups this collider can collide with.
#[no_mangle]
pub unsafe extern "C" fn sge_phys_collider_set_collision_groups(
    world: *mut c_void,
    collider: u64,
    memberships: u32,
    filter: u32,
) {
    let w = &mut *(world as *mut PhysicsWorld);
    if let Some(c) = w.collider_set.get_mut(u64_to_collider_handle(collider)) {
        c.set_collision_groups(InteractionGroups::new(
            Group::from_bits_truncate(memberships),
            Group::from_bits_truncate(filter),
        ));
    }
}

/// Gets the collision groups for a collider.
/// Fills `out` with [memberships, filter] as two u32 values (cast to i32 for C ABI).
#[no_mangle]
pub unsafe extern "C" fn sge_phys_collider_get_collision_groups(
    world: *mut c_void,
    collider: u64,
    out: *mut i32,
) {
    let w = &*(world as *mut PhysicsWorld);
    let arr = slice::from_raw_parts_mut(out, 2);
    if let Some(c) = w.collider_set.get(u64_to_collider_handle(collider)) {
        let groups = c.collision_groups();
        arr[0] = groups.memberships.bits() as i32;
        arr[1] = groups.filter.bits() as i32;
    } else {
        arr[0] = 0;
        arr[1] = 0;
    }
}

/// Sets the solver groups for a collider.
/// Solver groups control which colliders have their contacts solved (force response).
#[no_mangle]
pub unsafe extern "C" fn sge_phys_collider_set_solver_groups(
    world: *mut c_void,
    collider: u64,
    memberships: u32,
    filter: u32,
) {
    let w = &mut *(world as *mut PhysicsWorld);
    if let Some(c) = w.collider_set.get_mut(u64_to_collider_handle(collider)) {
        c.set_solver_groups(InteractionGroups::new(
            Group::from_bits_truncate(memberships),
            Group::from_bits_truncate(filter),
        ));
    }
}

/// Gets the solver groups for a collider.
/// Fills `out` with [memberships, filter] as two u32 values (cast to i32 for C ABI).
#[no_mangle]
pub unsafe extern "C" fn sge_phys_collider_get_solver_groups(
    world: *mut c_void,
    collider: u64,
    out: *mut i32,
) {
    let w = &*(world as *mut PhysicsWorld);
    let arr = slice::from_raw_parts_mut(out, 2);
    if let Some(c) = w.collider_set.get(u64_to_collider_handle(collider)) {
        let groups = c.solver_groups();
        arr[0] = groups.memberships.bits() as i32;
        arr[1] = groups.filter.bits() as i32;
    } else {
        arr[0] = 0;
        arr[1] = 0;
    }
}

// ---------------------------------------------------------------------------
// Joints
// ---------------------------------------------------------------------------

#[no_mangle]
pub unsafe extern "C" fn sge_phys_create_revolute_joint(
    world: *mut c_void,
    body1: u64,
    body2: u64,
    anchor_x: f32,
    anchor_y: f32,
) -> u64 {
    let w = &mut *(world as *mut PhysicsWorld);
    let joint = RevoluteJointBuilder::new()
        .local_anchor1(point![anchor_x, anchor_y])
        .local_anchor2(point![0.0, 0.0])
        .build();
    let handle = w.impulse_joint_set.insert(
        u64_to_body_handle(body1),
        u64_to_body_handle(body2),
        joint,
        true,
    );
    joint_handle_to_u64(handle)
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys_create_prismatic_joint(
    world: *mut c_void,
    body1: u64,
    body2: u64,
    axis_x: f32,
    axis_y: f32,
) -> u64 {
    let w = &mut *(world as *mut PhysicsWorld);
    let axis = UnitVector::new_normalize(vector![axis_x, axis_y]);
    let joint = PrismaticJointBuilder::new(axis).build();
    let handle = w.impulse_joint_set.insert(
        u64_to_body_handle(body1),
        u64_to_body_handle(body2),
        joint,
        true,
    );
    joint_handle_to_u64(handle)
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys_create_fixed_joint(
    world: *mut c_void,
    body1: u64,
    body2: u64,
) -> u64 {
    let w = &mut *(world as *mut PhysicsWorld);
    let joint = FixedJointBuilder::new().build();
    let handle = w.impulse_joint_set.insert(
        u64_to_body_handle(body1),
        u64_to_body_handle(body2),
        joint,
        true,
    );
    joint_handle_to_u64(handle)
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys_destroy_joint(world: *mut c_void, joint: u64) {
    let w = &mut *(world as *mut PhysicsWorld);
    let index = joint as u32;
    let generation = (joint >> 32) as u32;
    let handle = ImpulseJointHandle::from_raw_parts(index, generation);
    w.impulse_joint_set.remove(handle, true);
}

// ---------------------------------------------------------------------------
// Revolute joint limits and motors
// ---------------------------------------------------------------------------

fn u64_to_joint_handle(v: u64) -> ImpulseJointHandle {
    let index = v as u32;
    let generation = (v >> 32) as u32;
    ImpulseJointHandle::from_raw_parts(index, generation)
}

/// Enables or disables angular limits on a revolute joint.
#[no_mangle]
pub unsafe extern "C" fn sge_phys_revolute_joint_enable_limits(
    world: *mut c_void,
    joint: u64,
    enable: i32,
) {
    let w = &mut *(world as *mut PhysicsWorld);
    if let Some(j) = w.impulse_joint_set.get_mut(u64_to_joint_handle(joint)) {
        if let Some(rev) = j.data.as_revolute_mut() {
            if enable != 0 {
                // Enable with reasonable default limits if not already set
                if rev.limits().is_none() {
                    rev.set_limits([-std::f32::consts::PI, std::f32::consts::PI]);
                }
            } else {
                // Rapier doesn't have a direct "disable limits" - we set very wide limits
                rev.set_limits([-1000.0, 1000.0]);
            }
        }
    }
}

/// Sets the angular limits (in radians) for a revolute joint.
#[no_mangle]
pub unsafe extern "C" fn sge_phys_revolute_joint_set_limits(
    world: *mut c_void,
    joint: u64,
    lower: f32,
    upper: f32,
) {
    let w = &mut *(world as *mut PhysicsWorld);
    if let Some(j) = w.impulse_joint_set.get_mut(u64_to_joint_handle(joint)) {
        if let Some(rev) = j.data.as_revolute_mut() {
            rev.set_limits([lower, upper]);
        }
    }
}

/// Gets the angular limits for a revolute joint. Fills `out` with [lower, upper].
#[no_mangle]
pub unsafe extern "C" fn sge_phys_revolute_joint_get_limits(
    world: *mut c_void,
    joint: u64,
    out: *mut f32,
) {
    let w = &*(world as *mut PhysicsWorld);
    let arr = slice::from_raw_parts_mut(out, 2);
    if let Some(j) = w.impulse_joint_set.get(u64_to_joint_handle(joint)) {
        if let Some(rev) = j.data.as_revolute() {
            if let Some(limits) = rev.limits() {
                arr[0] = limits.min;
                arr[1] = limits.max;
                return;
            }
        }
    }
    arr[0] = 0.0;
    arr[1] = 0.0;
}

/// Returns 1 if the revolute joint has limits enabled, 0 otherwise.
#[no_mangle]
pub unsafe extern "C" fn sge_phys_revolute_joint_is_limit_enabled(
    world: *mut c_void,
    joint: u64,
) -> i32 {
    let w = &*(world as *mut PhysicsWorld);
    if let Some(j) = w.impulse_joint_set.get(u64_to_joint_handle(joint)) {
        if let Some(rev) = j.data.as_revolute() {
            if let Some(limits) = rev.limits() {
                // Consider limits "disabled" if they span a very wide range
                if limits.min <= -100.0 && limits.max >= 100.0 {
                    return 0;
                }
                return 1;
            }
        }
    }
    0
}

/// Enables the motor on a revolute joint.
#[no_mangle]
pub unsafe extern "C" fn sge_phys_revolute_joint_enable_motor(
    world: *mut c_void,
    joint: u64,
    enable: i32,
) {
    let w = &mut *(world as *mut PhysicsWorld);
    if let Some(j) = w.impulse_joint_set.get_mut(u64_to_joint_handle(joint)) {
        if let Some(rev) = j.data.as_revolute_mut() {
            if enable != 0 {
                // Enable motor with velocity target
                rev.set_motor_velocity(0.0, 1.0);
            } else {
                // Disable motor by setting zero max torque
                rev.set_motor_velocity(0.0, 0.0);
            }
        }
    }
}

/// Sets the target velocity for the revolute joint motor (radians/second).
#[no_mangle]
pub unsafe extern "C" fn sge_phys_revolute_joint_set_motor_speed(
    world: *mut c_void,
    joint: u64,
    speed: f32,
) {
    let w = &mut *(world as *mut PhysicsWorld);
    if let Some(j) = w.impulse_joint_set.get_mut(u64_to_joint_handle(joint)) {
        if let Some(rev) = j.data.as_revolute_mut() {
            // Get current damping factor, preserve it
            let damping = rev.motor().map(|m| m.damping).unwrap_or(1.0);
            rev.set_motor_velocity(speed, damping);
        }
    }
}

/// Sets the maximum torque the revolute joint motor can apply.
#[no_mangle]
pub unsafe extern "C" fn sge_phys_revolute_joint_set_max_motor_torque(
    world: *mut c_void,
    joint: u64,
    torque: f32,
) {
    let w = &mut *(world as *mut PhysicsWorld);
    if let Some(j) = w.impulse_joint_set.get_mut(u64_to_joint_handle(joint)) {
        if let Some(rev) = j.data.as_revolute_mut() {
            rev.set_motor_max_force(torque);
        }
    }
}

/// Gets the current motor speed setting for a revolute joint.
#[no_mangle]
pub unsafe extern "C" fn sge_phys_revolute_joint_get_motor_speed(
    world: *mut c_void,
    joint: u64,
) -> f32 {
    let w = &*(world as *mut PhysicsWorld);
    if let Some(j) = w.impulse_joint_set.get(u64_to_joint_handle(joint)) {
        if let Some(rev) = j.data.as_revolute() {
            if let Some(motor) = rev.motor() {
                return motor.target_vel;
            }
        }
    }
    0.0
}

/// Gets the current angle of the revolute joint (radians).
#[no_mangle]
pub unsafe extern "C" fn sge_phys_revolute_joint_get_angle(
    world: *mut c_void,
    joint: u64,
) -> f32 {
    let w = &*(world as *mut PhysicsWorld);
    if let Some(j) = w.impulse_joint_set.get(u64_to_joint_handle(joint)) {
        if let Some(rev) = j.data.as_revolute() {
            // Get body rotations to compute joint angle
            let body1_handle = j.body1;
            let body2_handle = j.body2;
            if let (Some(b1), Some(b2)) = (
                w.rigid_body_set.get(body1_handle),
                w.rigid_body_set.get(body2_handle),
            ) {
                return rev.angle(b1.rotation(), b2.rotation());
            }
        }
    }
    0.0
}

// ---------------------------------------------------------------------------
// Prismatic joint limits and motors
// ---------------------------------------------------------------------------

/// Enables or disables translation limits on a prismatic joint.
#[no_mangle]
pub unsafe extern "C" fn sge_phys_prismatic_joint_enable_limits(
    world: *mut c_void,
    joint: u64,
    enable: i32,
) {
    let w = &mut *(world as *mut PhysicsWorld);
    if let Some(j) = w.impulse_joint_set.get_mut(u64_to_joint_handle(joint)) {
        if let Some(pris) = j.data.as_prismatic_mut() {
            if enable != 0 {
                // Enable with reasonable default limits if not already set
                if pris.limits().is_none() {
                    pris.set_limits([-1.0, 1.0]);
                }
            } else {
                pris.set_limits([-1e6, 1e6]);
            }
        }
    }
}

/// Sets the translation limits for a prismatic joint.
#[no_mangle]
pub unsafe extern "C" fn sge_phys_prismatic_joint_set_limits(
    world: *mut c_void,
    joint: u64,
    lower: f32,
    upper: f32,
) {
    let w = &mut *(world as *mut PhysicsWorld);
    if let Some(j) = w.impulse_joint_set.get_mut(u64_to_joint_handle(joint)) {
        if let Some(pris) = j.data.as_prismatic_mut() {
            pris.set_limits([lower, upper]);
        }
    }
}

/// Gets the translation limits for a prismatic joint. Fills `out` with [lower, upper].
#[no_mangle]
pub unsafe extern "C" fn sge_phys_prismatic_joint_get_limits(
    world: *mut c_void,
    joint: u64,
    out: *mut f32,
) {
    let w = &*(world as *mut PhysicsWorld);
    let arr = slice::from_raw_parts_mut(out, 2);
    if let Some(j) = w.impulse_joint_set.get(u64_to_joint_handle(joint)) {
        if let Some(pris) = j.data.as_prismatic() {
            if let Some(limits) = pris.limits() {
                arr[0] = limits.min;
                arr[1] = limits.max;
                return;
            }
        }
    }
    arr[0] = 0.0;
    arr[1] = 0.0;
}

/// Enables the motor on a prismatic joint.
#[no_mangle]
pub unsafe extern "C" fn sge_phys_prismatic_joint_enable_motor(
    world: *mut c_void,
    joint: u64,
    enable: i32,
) {
    let w = &mut *(world as *mut PhysicsWorld);
    if let Some(j) = w.impulse_joint_set.get_mut(u64_to_joint_handle(joint)) {
        if let Some(pris) = j.data.as_prismatic_mut() {
            if enable != 0 {
                pris.set_motor_velocity(0.0, 1.0);
            } else {
                pris.set_motor_velocity(0.0, 0.0);
            }
        }
    }
}

/// Sets the target velocity for the prismatic joint motor.
#[no_mangle]
pub unsafe extern "C" fn sge_phys_prismatic_joint_set_motor_speed(
    world: *mut c_void,
    joint: u64,
    speed: f32,
) {
    let w = &mut *(world as *mut PhysicsWorld);
    if let Some(j) = w.impulse_joint_set.get_mut(u64_to_joint_handle(joint)) {
        if let Some(pris) = j.data.as_prismatic_mut() {
            let damping = pris.motor().map(|m| m.damping).unwrap_or(1.0);
            pris.set_motor_velocity(speed, damping);
        }
    }
}

/// Sets the maximum force the prismatic joint motor can apply.
#[no_mangle]
pub unsafe extern "C" fn sge_phys_prismatic_joint_set_max_motor_force(
    world: *mut c_void,
    joint: u64,
    force: f32,
) {
    let w = &mut *(world as *mut PhysicsWorld);
    if let Some(j) = w.impulse_joint_set.get_mut(u64_to_joint_handle(joint)) {
        if let Some(pris) = j.data.as_prismatic_mut() {
            pris.set_motor_max_force(force);
        }
    }
}

/// Gets the current translation of the prismatic joint.
#[no_mangle]
pub unsafe extern "C" fn sge_phys_prismatic_joint_get_translation(
    world: *mut c_void,
    joint: u64,
) -> f32 {
    let w = &*(world as *mut PhysicsWorld);
    if let Some(j) = w.impulse_joint_set.get(u64_to_joint_handle(joint)) {
        if let Some(_pris) = j.data.as_prismatic() {
            // Get the bodies to compute actual translation
            let body1_handle = j.body1;
            let body2_handle = j.body2;
            if let (Some(b1), Some(b2)) = (
                w.rigid_body_set.get(body1_handle),
                w.rigid_body_set.get(body2_handle),
            ) {
                // Compute the displacement between body centers
                // This gives an approximate translation along the joint axis
                let diff = b2.translation() - b1.translation();
                return diff.norm();
            }
        }
    }
    0.0
}

// ---------------------------------------------------------------------------
// Rope joint (distance constraint)
// ---------------------------------------------------------------------------

/// Creates a rope joint constraining two bodies within a maximum distance.
/// Rapier's rope joint enforces max distance; setting min_dist == max_dist
/// gives a rigid distance constraint (like Box2D's DistanceJoint).
#[no_mangle]
pub unsafe extern "C" fn sge_phys_create_rope_joint(
    world: *mut c_void,
    body1: u64,
    body2: u64,
    max_dist: f32,
) -> u64 {
    let w = &mut *(world as *mut PhysicsWorld);
    let joint = RopeJointBuilder::new(max_dist).build();
    let handle = w.impulse_joint_set.insert(
        u64_to_body_handle(body1),
        u64_to_body_handle(body2),
        joint,
        true,
    );
    joint_handle_to_u64(handle)
}

/// Sets the maximum allowed distance for a rope joint.
#[no_mangle]
pub unsafe extern "C" fn sge_phys_rope_joint_set_max_distance(
    world: *mut c_void,
    joint: u64,
    max_dist: f32,
) {
    let w = &mut *(world as *mut PhysicsWorld);
    if let Some(j) = w.impulse_joint_set.get_mut(u64_to_joint_handle(joint)) {
        if let Some(rope) = j.data.as_rope_mut() {
            rope.set_max_distance(max_dist);
        }
    }
}

/// Gets the maximum allowed distance for a rope joint.
#[no_mangle]
pub unsafe extern "C" fn sge_phys_rope_joint_get_max_distance(
    world: *mut c_void,
    joint: u64,
) -> f32 {
    let w = &*(world as *mut PhysicsWorld);
    if let Some(j) = w.impulse_joint_set.get(u64_to_joint_handle(joint)) {
        if let Some(rope) = j.data.as_rope() {
            return rope.max_distance();
        }
    }
    0.0
}

// ---------------------------------------------------------------------------
// Segment collider (Edge shape — line segment)
// ---------------------------------------------------------------------------

/// Attaches a segment (line segment) collider to a body. Returns a collider handle.
#[no_mangle]
pub unsafe extern "C" fn sge_phys_create_segment_collider(
    world: *mut c_void,
    body: u64,
    x1: f32,
    y1: f32,
    x2: f32,
    y2: f32,
) -> u64 {
    let w = &mut *(world as *mut PhysicsWorld);
    let collider = ColliderBuilder::segment(point![x1, y1], point![x2, y2]).build();
    let handle = w.collider_set.insert_with_parent(
        collider,
        u64_to_body_handle(body),
        &mut w.rigid_body_set,
    );
    collider_handle_to_u64(handle)
}

// ---------------------------------------------------------------------------
// Polyline collider (Chain shape — connected line segments)
// ---------------------------------------------------------------------------

/// Attaches a polyline collider to a body. Returns a collider handle.
///
/// `vertices` is a flat array [x0, y0, x1, y1, ...] of length `vertex_count * 2`.
/// The polyline consists of segments connecting consecutive vertices.
/// If `vertex_count` < 2, returns 0 (invalid handle).
#[no_mangle]
pub unsafe extern "C" fn sge_phys_create_polyline_collider(
    world: *mut c_void,
    body: u64,
    vertices: *const f32,
    vertex_count: i32,
) -> u64 {
    if vertex_count < 2 {
        return 0;
    }
    let w = &mut *(world as *mut PhysicsWorld);
    let verts = slice::from_raw_parts(vertices, (vertex_count * 2) as usize);
    let points: Vec<Point<Real>> = (0..vertex_count as usize)
        .map(|i| point![verts[i * 2], verts[i * 2 + 1]])
        .collect();

    // Build index pairs for consecutive segments
    let indices: Vec<[u32; 2]> = (0..points.len() as u32 - 1)
        .map(|i| [i, i + 1])
        .collect();

    let collider = ColliderBuilder::polyline(points, Some(indices)).build();
    let handle = w.collider_set.insert_with_parent(
        collider,
        u64_to_body_handle(body),
        &mut w.rigid_body_set,
    );
    collider_handle_to_u64(handle)
}

// ---------------------------------------------------------------------------
// Body mass/inertia queries
// ---------------------------------------------------------------------------

/// Gets the total mass of a rigid body.
#[no_mangle]
pub unsafe extern "C" fn sge_phys_body_get_mass(world: *mut c_void, body: u64) -> f32 {
    let w = &*(world as *mut PhysicsWorld);
    w.rigid_body_set
        .get(u64_to_body_handle(body))
        .map(|b| b.mass())
        .unwrap_or(0.0)
}

/// Gets the angular inertia of a rigid body.
#[no_mangle]
pub unsafe extern "C" fn sge_phys_body_get_inertia(world: *mut c_void, body: u64) -> f32 {
    let w = &*(world as *mut PhysicsWorld);
    w.rigid_body_set
        .get(u64_to_body_handle(body))
        .map(|b| {
            // In 2D, principal_inertia returns the single rotational inertia value
            b.mass_properties().local_mprops.principal_inertia()
        })
        .unwrap_or(0.0)
}

/// Gets the local center of mass. Fills `out` with [x, y].
#[no_mangle]
pub unsafe extern "C" fn sge_phys_body_get_local_center_of_mass(
    world: *mut c_void,
    body: u64,
    out: *mut f32,
) {
    let w = &*(world as *mut PhysicsWorld);
    let arr = slice::from_raw_parts_mut(out, 2);
    if let Some(b) = w.rigid_body_set.get(u64_to_body_handle(body)) {
        let com = b.mass_properties().local_mprops.local_com;
        arr[0] = com.x;
        arr[1] = com.y;
    } else {
        arr[0] = 0.0;
        arr[1] = 0.0;
    }
}

/// Forces recomputation of mass properties from attached colliders.
#[no_mangle]
pub unsafe extern "C" fn sge_phys_body_recompute_mass_properties(world: *mut c_void, body: u64) {
    let w = &mut *(world as *mut PhysicsWorld);
    if let Some(b) = w.rigid_body_set.get_mut(u64_to_body_handle(body)) {
        b.recompute_mass_properties_from_colliders(&w.collider_set);
    }
}

// ---------------------------------------------------------------------------
// Queries
// ---------------------------------------------------------------------------

/// Ray cast. Fills `out` with:
///   [hitX, hitY, normalX, normalY, toi, bodyHandleLo, bodyHandleHi, colliderHandleLo, colliderHandleHi]
/// (9 floats). Handles are split across two f32 slots each (low 32 bits, high 32 bits).
/// Returns 1 if hit, 0 otherwise.
#[no_mangle]
pub unsafe extern "C" fn sge_phys_ray_cast(
    world: *mut c_void,
    origin_x: f32,
    origin_y: f32,
    dir_x: f32,
    dir_y: f32,
    max_dist: f32,
    out: *mut f32,
) -> i32 {
    let w = &*(world as *mut PhysicsWorld);
    let ray = Ray::new(point![origin_x, origin_y], vector![dir_x, dir_y]);

    if let Some((handle, intersection)) = w.query_pipeline.cast_ray_and_get_normal(
        &w.rigid_body_set,
        &w.collider_set,
        &ray,
        max_dist,
        true,
        QueryFilter::default(),
    ) {
        let hit_point = ray.point_at(intersection.time_of_impact);
        let arr = slice::from_raw_parts_mut(out, 9);
        arr[0] = hit_point.x;
        arr[1] = hit_point.y;
        arr[2] = intersection.normal.x;
        arr[3] = intersection.normal.y;
        arr[4] = intersection.time_of_impact;
        // Encode body handle (via collider's parent)
        arr[5] = 0.0;
        arr[6] = 0.0;
        if let Some(collider) = w.collider_set.get(handle) {
            if let Some(parent) = collider.parent() {
                let h = body_handle_to_u64(parent);
                arr[5] = f32::from_bits(h as u32);
                arr[6] = f32::from_bits((h >> 32) as u32);
            }
        }
        // Encode collider handle directly
        let ch = collider_handle_to_u64(handle);
        arr[7] = f32::from_bits(ch as u32);
        arr[8] = f32::from_bits((ch >> 32) as u32);
        1
    } else {
        0
    }
}

/// AABB query. Finds all colliders intersecting the given axis-aligned bounding box.
/// Fills `out_colliders` with collider handles. Returns count of hits.
#[no_mangle]
pub unsafe extern "C" fn sge_phys_query_aabb(
    world: *mut c_void,
    min_x: f32,
    min_y: f32,
    max_x: f32,
    max_y: f32,
    out_colliders: *mut u64,
    max_results: i32,
) -> i32 {
    let w = &*(world as *mut PhysicsWorld);
    let aabb = Aabb::new(point![min_x, min_y], point![max_x, max_y]);
    let out = slice::from_raw_parts_mut(out_colliders, max_results as usize);
    let mut count = 0i32;

    w.query_pipeline.colliders_with_aabb_intersecting_aabb(
        &aabb,
        |handle| {
            if count < max_results {
                out[count as usize] = collider_handle_to_u64(*handle);
                count += 1;
            }
            count < max_results // continue if we have room
        },
    );
    count
}

/// Point query. Fills `out_bodies` with body handles. Returns count of hits.
#[no_mangle]
pub unsafe extern "C" fn sge_phys_query_point(
    world: *mut c_void,
    x: f32,
    y: f32,
    out_bodies: *mut u64,
    max_results: i32,
) -> i32 {
    let w = &*(world as *mut PhysicsWorld);
    let point = point![x, y];
    let out = slice::from_raw_parts_mut(out_bodies, max_results as usize);
    let mut count = 0i32;

    w.query_pipeline.intersections_with_point(
        &w.rigid_body_set,
        &w.collider_set,
        &point,
        QueryFilter::default(),
        |handle| {
            if count < max_results {
                if let Some(collider) = w.collider_set.get(handle) {
                    if let Some(parent) = collider.parent() {
                        out[count as usize] = body_handle_to_u64(parent);
                        count += 1;
                    }
                }
            }
            count < max_results // continue if we have room
        },
    );
    count
}

// ---------------------------------------------------------------------------
// Contact events (polling)
// ---------------------------------------------------------------------------

#[no_mangle]
pub unsafe extern "C" fn sge_phys_poll_contact_start_events(
    world: *mut c_void,
    out_collider1: *mut u64,
    out_collider2: *mut u64,
    max_events: i32,
) -> i32 {
    let w = &*(world as *mut PhysicsWorld);
    let c1 = slice::from_raw_parts_mut(out_collider1, max_events as usize);
    let c2 = slice::from_raw_parts_mut(out_collider2, max_events as usize);
    let count = w.contact_start_buf.len().min(max_events as usize);
    for i in 0..count {
        c1[i] = w.contact_start_buf[i].0;
        c2[i] = w.contact_start_buf[i].1;
    }
    count as i32
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys_poll_contact_stop_events(
    world: *mut c_void,
    out_collider1: *mut u64,
    out_collider2: *mut u64,
    max_events: i32,
) -> i32 {
    let w = &*(world as *mut PhysicsWorld);
    let c1 = slice::from_raw_parts_mut(out_collider1, max_events as usize);
    let c2 = slice::from_raw_parts_mut(out_collider2, max_events as usize);
    let count = w.contact_stop_buf.len().min(max_events as usize);
    for i in 0..count {
        c1[i] = w.contact_stop_buf[i].0;
        c2[i] = w.contact_stop_buf[i].1;
    }
    count as i32
}

// ---------------------------------------------------------------------------
// Tests
// ---------------------------------------------------------------------------

#[cfg(test)]
mod tests {
    use super::*;

    // -- World lifecycle ----------------------------------------------------

    #[test]
    fn world_create_get_gravity_destroy() {
        unsafe {
            let world = sge_phys_create_world(0.0, -9.81);
            assert!(!world.is_null());

            let mut gravity = [0.0f32; 2];
            sge_phys_world_get_gravity(world, gravity.as_mut_ptr());
            assert_eq!(gravity[0], 0.0);
            assert!((gravity[1] - (-9.81)).abs() < 1e-5);

            sge_phys_destroy_world(world);
        }
    }

    #[test]
    fn world_set_gravity() {
        unsafe {
            let world = sge_phys_create_world(0.0, 0.0);

            sge_phys_world_set_gravity(world, 1.0, -20.0);

            let mut gravity = [0.0f32; 2];
            sge_phys_world_get_gravity(world, gravity.as_mut_ptr());
            assert_eq!(gravity[0], 1.0);
            assert_eq!(gravity[1], -20.0);

            sge_phys_destroy_world(world);
        }
    }

    #[test]
    fn world_step_no_crash() {
        unsafe {
            let world = sge_phys_create_world(0.0, -9.81);
            // Step several times without any bodies — should not crash
            for _ in 0..10 {
                sge_phys_world_step(world, 1.0 / 60.0);
            }
            sge_phys_destroy_world(world);
        }
    }

    // -- Gravity simulation ------------------------------------------------

    #[test]
    fn dynamic_body_falls_under_gravity() {
        unsafe {
            let world = sge_phys_create_world(0.0, -9.81);

            // Create a dynamic body at (0, 10)
            let body = sge_phys_create_dynamic_body(world, 0.0, 10.0, 0.0);

            // Add a box collider (required for mass)
            let _collider = sge_phys_create_box_collider(world, body, 0.5, 0.5);

            // Record initial position
            let mut pos = [0.0f32; 2];
            sge_phys_body_get_position(world, body, pos.as_mut_ptr());
            let initial_y = pos[1];
            assert!((initial_y - 10.0).abs() < 1e-5);

            // Step 60 times at dt=1/60
            for _ in 0..60 {
                sge_phys_world_step(world, 1.0 / 60.0);
            }

            // Check position: Y should have decreased due to gravity
            sge_phys_body_get_position(world, body, pos.as_mut_ptr());
            assert!(
                pos[1] < initial_y,
                "body should have fallen: initial_y={}, final_y={}",
                initial_y, pos[1]
            );
            // X should remain ~0
            assert!(
                pos[0].abs() < 1e-3,
                "X should be near 0, got {}",
                pos[0]
            );

            sge_phys_destroy_world(world);
        }
    }

    #[test]
    fn static_body_does_not_move() {
        unsafe {
            let world = sge_phys_create_world(0.0, -9.81);

            let body = sge_phys_create_static_body(world, 5.0, 5.0, 0.0);
            let _collider = sge_phys_create_box_collider(world, body, 1.0, 1.0);

            for _ in 0..60 {
                sge_phys_world_step(world, 1.0 / 60.0);
            }

            let mut pos = [0.0f32; 2];
            sge_phys_body_get_position(world, body, pos.as_mut_ptr());
            assert!((pos[0] - 5.0).abs() < 1e-5);
            assert!((pos[1] - 5.0).abs() < 1e-5);

            sge_phys_destroy_world(world);
        }
    }

    // -- Raycast -----------------------------------------------------------

    #[test]
    fn raycast_hits_static_box() {
        unsafe {
            let world = sge_phys_create_world(0.0, 0.0); // no gravity

            // Create a static body with a box at origin
            let body = sge_phys_create_static_body(world, 0.0, 0.0, 0.0);
            let _collider = sge_phys_create_box_collider(world, body, 1.0, 1.0);

            // Update the query pipeline by stepping once
            sge_phys_world_step(world, 1.0 / 60.0);

            // Cast ray from (0, 10) downward (direction 0, -1)
            let mut out = [0.0f32; 9];
            let hit = sge_phys_ray_cast(
                world,
                0.0, 10.0,  // origin
                0.0, -1.0,  // direction (down)
                100.0,       // max distance
                out.as_mut_ptr(),
            );

            assert_eq!(hit, 1, "raycast should hit the box");

            // Hit point should be near (0, 1) — top of the box
            let hit_x = out[0];
            let hit_y = out[1];
            assert!(hit_x.abs() < 0.1, "hit X should be near 0, got {}", hit_x);
            assert!(
                (hit_y - 1.0).abs() < 0.1,
                "hit Y should be near 1.0, got {}",
                hit_y
            );

            // TOI should be ~9.0 (distance from origin (0,10) to (0,1))
            let toi = out[4];
            assert!(
                (toi - 9.0).abs() < 0.1,
                "TOI should be near 9.0, got {}",
                toi
            );

            sge_phys_destroy_world(world);
        }
    }

    #[test]
    fn raycast_misses() {
        unsafe {
            let world = sge_phys_create_world(0.0, 0.0);

            // Create a static body at (100, 100) — far away from ray
            let body = sge_phys_create_static_body(world, 100.0, 100.0, 0.0);
            let _collider = sge_phys_create_box_collider(world, body, 0.5, 0.5);

            sge_phys_world_step(world, 1.0 / 60.0);

            // Cast ray from origin going right — should miss the box at (100,100)
            let mut out = [0.0f32; 9];
            let hit = sge_phys_ray_cast(
                world,
                0.0, 0.0,  // origin
                1.0, 0.0,  // direction (right)
                10.0,       // max distance (only 10 units)
                out.as_mut_ptr(),
            );

            assert_eq!(hit, 0, "raycast should miss");

            sge_phys_destroy_world(world);
        }
    }

    // -- Body properties ---------------------------------------------------

    #[test]
    fn body_get_set_angle() {
        unsafe {
            let world = sge_phys_create_world(0.0, 0.0);

            let body = sge_phys_create_dynamic_body(world, 0.0, 0.0, 1.5);
            let angle = sge_phys_body_get_angle(world, body);
            assert!((angle - 1.5).abs() < 1e-5);

            sge_phys_body_set_angle(world, body, 3.0);
            let angle = sge_phys_body_get_angle(world, body);
            assert!((angle - 3.0).abs() < 1e-5);

            sge_phys_destroy_world(world);
        }
    }

    #[test]
    fn body_linear_velocity() {
        unsafe {
            let world = sge_phys_create_world(0.0, 0.0);
            let body = sge_phys_create_dynamic_body(world, 0.0, 0.0, 0.0);
            let _collider = sge_phys_create_box_collider(world, body, 0.5, 0.5);

            sge_phys_body_set_linear_velocity(world, body, 5.0, -3.0);

            let mut vel = [0.0f32; 2];
            sge_phys_body_get_linear_velocity(world, body, vel.as_mut_ptr());
            assert!((vel[0] - 5.0).abs() < 1e-5);
            assert!((vel[1] - (-3.0)).abs() < 1e-5);

            sge_phys_destroy_world(world);
        }
    }

    // -- Rope joint --------------------------------------------------------

    // -- Rope joint --------------------------------------------------------

    #[test]
    fn rope_joint_constrains_distance() {
        unsafe {
            let world = sge_phys_create_world(0.0, 0.0);

            let b1 = sge_phys_create_dynamic_body(world, 0.0, 0.0, 0.0);
            let _c1 = sge_phys_create_box_collider(world, b1, 0.5, 0.5);
            let b2 = sge_phys_create_dynamic_body(world, 3.0, 0.0, 0.0);
            let _c2 = sge_phys_create_box_collider(world, b2, 0.5, 0.5);

            let joint = sge_phys_create_rope_joint(world, b1, b2, 5.0);

            let max_dist = sge_phys_rope_joint_get_max_distance(world, joint);
            assert!((max_dist - 5.0).abs() < 1e-5, "initial max dist should be 5.0, got {}", max_dist);

            sge_phys_rope_joint_set_max_distance(world, joint, 2.0);
            let max_dist = sge_phys_rope_joint_get_max_distance(world, joint);
            assert!((max_dist - 2.0).abs() < 1e-5, "updated max dist should be 2.0, got {}", max_dist);

            sge_phys_destroy_world(world);
        }
    }

    // -- Segment collider -------------------------------------------------

    #[test]
    fn segment_collider_raycast() {
        unsafe {
            let world = sge_phys_create_world(0.0, 0.0); // no gravity
            let body = sge_phys_create_static_body(world, 0.0, 0.0, 0.0);

            // Horizontal segment from (-5,0) to (5,0)
            let _collider = sge_phys_create_segment_collider(world, body, -5.0, 0.0, 5.0, 0.0);

            // Step to update query pipeline
            sge_phys_world_step(world, 1.0 / 60.0);

            // Cast ray downward — should hit the segment
            let mut out = [0.0f32; 9];
            let hit = sge_phys_ray_cast(world, 0.0, 5.0, 0.0, -1.0, 10.0, out.as_mut_ptr());
            assert_eq!(hit, 1, "ray should hit segment collider");
            assert!((out[1]).abs() < 0.1, "hit Y should be near 0, got {}", out[1]);

            sge_phys_destroy_world(world);
        }
    }

    // -- Polyline collider ------------------------------------------------

    #[test]
    fn polyline_collider_creation() {
        unsafe {
            let world = sge_phys_create_world(0.0, 0.0);
            let body = sge_phys_create_static_body(world, 0.0, 0.0, 0.0);

            // L-shaped polyline: (0,0) → (5,0) → (5,5)
            let vertices: [f32; 6] = [0.0, 0.0, 5.0, 0.0, 5.0, 5.0];
            let _collider = sge_phys_create_polyline_collider(
                world, body, vertices.as_ptr(), 3
            );

            // Step and raycast the horizontal segment
            sge_phys_world_step(world, 1.0 / 60.0);

            let mut out = [0.0f32; 9];
            let hit = sge_phys_ray_cast(world, 2.5, 5.0, 0.0, -1.0, 10.0, out.as_mut_ptr());
            assert_eq!(hit, 1, "ray should hit horizontal segment of polyline");

            // Verify minimum vertex count returns 0
            let invalid = sge_phys_create_polyline_collider(world, body, vertices.as_ptr(), 1);
            assert_eq!(invalid, 0, "polyline with 1 vertex should return 0");

            sge_phys_destroy_world(world);
        }
    }

    #[test]
    fn body_destroy_no_crash() {
        unsafe {
            let world = sge_phys_create_world(0.0, -9.81);
            let body = sge_phys_create_dynamic_body(world, 0.0, 0.0, 0.0);
            let _collider = sge_phys_create_box_collider(world, body, 0.5, 0.5);

            sge_phys_destroy_body(world, body);

            // Stepping after body destruction should not crash
            sge_phys_world_step(world, 1.0 / 60.0);

            sge_phys_destroy_world(world);
        }
    }
}
