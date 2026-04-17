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

use std::collections::HashMap;
use std::ffi::c_void;
use std::slice;
use std::sync::mpsc;

use rapier2d::prelude::*;
use rapier2d::parry::query::{DefaultQueryDispatcher, ShapeCastOptions};

// ---------------------------------------------------------------------------
// World state
// ---------------------------------------------------------------------------

struct PhysicsWorld {
    gravity: Vector,
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
    // Contact event buffers
    contact_start_buf: Vec<(u64, u64)>,
    contact_stop_buf: Vec<(u64, u64)>,
    // Contact force event buffer: (collider1, collider2, total_force_magnitude)
    contact_force_buf: Vec<(u64, u64, f32)>,
    // Per-collider one-way platform config: collider_handle_u64 -> (allowed_local_n1, allowed_angle)
    one_way_platforms: HashMap<u64, ([f32; 2], f32)>,
}

impl PhysicsWorld {
    fn new(gx: f32, gy: f32) -> Self {
        PhysicsWorld {
            gravity: Vector::new(gx, gy),
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
            contact_start_buf: Vec::new(),
            contact_stop_buf: Vec::new(),
            contact_force_buf: Vec::new(),
            one_way_platforms: HashMap::new(),
        }
    }

    fn step(&mut self, dt: f32) {
        self.integration_parameters.dt = dt;

        // Clear event buffers before stepping
        self.contact_start_buf.clear();
        self.contact_stop_buf.clear();
        self.contact_force_buf.clear();

        let (contact_send, contact_recv) = mpsc::channel();
        let (force_send, force_recv) = mpsc::channel();

        let event_handler = ChannelEventCollector::new(contact_send, force_send);

        // Temporarily take the one-way platforms map to avoid borrow conflict with self
        let owp = std::mem::take(&mut self.one_way_platforms);
        let hooks = SgePhysicsHooks2D { one_way_platforms: &owp };

        self.physics_pipeline.step(
            self.gravity,
            &self.integration_parameters,
            &mut self.island_manager,
            &mut self.broad_phase,
            &mut self.narrow_phase,
            &mut self.rigid_body_set,
            &mut self.collider_set,
            &mut self.impulse_joint_set,
            &mut self.multibody_joint_set,
            &mut self.ccd_solver,
            &hooks,
            &event_handler,
        );

        // Restore the one-way platforms map
        self.one_way_platforms = owp;

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

        // Drain force events
        while let Ok(event) = force_recv.try_recv() {
            self.contact_force_buf.push((
                collider_handle_to_u64(event.collider1),
                collider_handle_to_u64(event.collider2),
                event.total_force_magnitude,
            ));
        }
    }
}

// ---------------------------------------------------------------------------
// Physics hooks — one-way platform support (2D)
// ---------------------------------------------------------------------------

struct SgePhysicsHooks2D<'a> {
    one_way_platforms: &'a HashMap<u64, ([f32; 2], f32)>,
}

unsafe impl Send for SgePhysicsHooks2D<'_> {}
unsafe impl Sync for SgePhysicsHooks2D<'_> {}

impl PhysicsHooks for SgePhysicsHooks2D<'_> {
    fn modify_solver_contacts(&self, context: &mut ContactModificationContext) {
        let c1 = collider_handle_to_u64(context.collider1);
        let c2 = collider_handle_to_u64(context.collider2);

        if let Some(&(dir, angle)) = self.one_way_platforms.get(&c1) {
            let allowed_local_n1 = Vector::new(dir[0], dir[1]);
            context.update_as_oneway_platform(allowed_local_n1, angle);
        }
        if let Some(&(dir, angle)) = self.one_way_platforms.get(&c2) {
            // For collider2, the normal points outward from collider1, so we
            // need to flip it. Rapier's update_as_oneway_platform uses
            // manifold.local_n1 which is relative to collider1. For collider2
            // as the platform, we negate the allowed direction so the dot
            // product check works correctly.
            let allowed_local_n1 = Vector::new(-dir[0], -dir[1]);
            context.update_as_oneway_platform(allowed_local_n1, angle);
        }
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
    w.gravity = Vector::new(gx, gy);
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
        .translation(Vector::new(x, y))
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
        b.set_translation(Vector::new(x, y), true);
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
        b.set_linvel(Vector::new(vx, vy), true);
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
        b.add_force(Vector::new(fx, fy), true);
    }
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys_body_apply_impulse(world: *mut c_void, body: u64, ix: f32, iy: f32) {
    let w = &mut *(world as *mut PhysicsWorld);
    if let Some(b) = w.rigid_body_set.get_mut(u64_to_body_handle(body)) {
        b.apply_impulse(Vector::new(ix, iy), true);
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
// Body — forces at point
// ---------------------------------------------------------------------------

#[no_mangle]
pub unsafe extern "C" fn sge_phys_body_apply_force_at_point(
    world: *mut c_void, body: u64, fx: f32, fy: f32, px: f32, py: f32,
) {
    let w = &mut *(world as *mut PhysicsWorld);
    if let Some(b) = w.rigid_body_set.get_mut(u64_to_body_handle(body)) {
        b.add_force_at_point(Vector::new(fx, fy), Vector::new(px, py), true);
    }
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys_body_apply_impulse_at_point(
    world: *mut c_void, body: u64, ix: f32, iy: f32, px: f32, py: f32,
) {
    let w = &mut *(world as *mut PhysicsWorld);
    if let Some(b) = w.rigid_body_set.get_mut(u64_to_body_handle(body)) {
        b.apply_impulse_at_point(Vector::new(ix, iy), Vector::new(px, py), true);
    }
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys_body_apply_torque_impulse(world: *mut c_void, body: u64, impulse: f32) {
    let w = &mut *(world as *mut PhysicsWorld);
    if let Some(b) = w.rigid_body_set.get_mut(u64_to_body_handle(body)) {
        b.apply_torque_impulse(impulse, true);
    }
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys_body_reset_forces(world: *mut c_void, body: u64) {
    let w = &mut *(world as *mut PhysicsWorld);
    if let Some(b) = w.rigid_body_set.get_mut(u64_to_body_handle(body)) {
        b.reset_forces(true);
    }
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys_body_reset_torques(world: *mut c_void, body: u64) {
    let w = &mut *(world as *mut PhysicsWorld);
    if let Some(b) = w.rigid_body_set.get_mut(u64_to_body_handle(body)) {
        b.reset_torques(true);
    }
}

// ---------------------------------------------------------------------------
// Body — getters for existing setters
// ---------------------------------------------------------------------------

#[no_mangle]
pub unsafe extern "C" fn sge_phys_body_get_linear_damping(world: *mut c_void, body: u64) -> f32 {
    let w = &*(world as *mut PhysicsWorld);
    w.rigid_body_set.get(u64_to_body_handle(body)).map(|b| b.linear_damping()).unwrap_or(0.0)
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys_body_get_angular_damping(world: *mut c_void, body: u64) -> f32 {
    let w = &*(world as *mut PhysicsWorld);
    w.rigid_body_set.get(u64_to_body_handle(body)).map(|b| b.angular_damping()).unwrap_or(0.0)
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys_body_get_gravity_scale(world: *mut c_void, body: u64) -> f32 {
    let w = &*(world as *mut PhysicsWorld);
    w.rigid_body_set.get(u64_to_body_handle(body)).map(|b| b.gravity_scale()).unwrap_or(1.0)
}

// ---------------------------------------------------------------------------
// Body — type query, enable/disable, dominance, locking
// ---------------------------------------------------------------------------

/// Returns body type: 0 = dynamic, 1 = fixed (static), 2 = kinematic position-based, 3 = kinematic velocity-based
#[no_mangle]
pub unsafe extern "C" fn sge_phys_body_get_type(world: *mut c_void, body: u64) -> i32 {
    let w = &*(world as *mut PhysicsWorld);
    w.rigid_body_set.get(u64_to_body_handle(body)).map(|b| match b.body_type() {
        RigidBodyType::Dynamic                  => 0,
        RigidBodyType::Fixed                    => 1,
        RigidBodyType::KinematicPositionBased   => 2,
        RigidBodyType::KinematicVelocityBased   => 3,
    }).unwrap_or(-1)
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys_body_set_enabled(world: *mut c_void, body: u64, enabled: i32) {
    let w = &mut *(world as *mut PhysicsWorld);
    if let Some(b) = w.rigid_body_set.get_mut(u64_to_body_handle(body)) {
        b.set_enabled(enabled != 0);
    }
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys_body_is_enabled(world: *mut c_void, body: u64) -> i32 {
    let w = &*(world as *mut PhysicsWorld);
    w.rigid_body_set.get(u64_to_body_handle(body)).map(|b| b.is_enabled() as i32).unwrap_or(0)
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys_body_set_enabled_translations(
    world: *mut c_void, body: u64, x: i32, y: i32,
) {
    let w = &mut *(world as *mut PhysicsWorld);
    if let Some(b) = w.rigid_body_set.get_mut(u64_to_body_handle(body)) {
        b.set_enabled_translations(x != 0, y != 0, true);
    }
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys_body_is_translation_locked_x(world: *mut c_void, body: u64) -> i32 {
    let w = &*(world as *mut PhysicsWorld);
    w.rigid_body_set.get(u64_to_body_handle(body))
        .map(|b| b.locked_axes().contains(LockedAxes::TRANSLATION_LOCKED_X) as i32)
        .unwrap_or(0)
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys_body_is_translation_locked_y(world: *mut c_void, body: u64) -> i32 {
    let w = &*(world as *mut PhysicsWorld);
    w.rigid_body_set.get(u64_to_body_handle(body))
        .map(|b| b.locked_axes().contains(LockedAxes::TRANSLATION_LOCKED_Y) as i32)
        .unwrap_or(0)
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys_body_is_rotation_locked(world: *mut c_void, body: u64) -> i32 {
    let w = &*(world as *mut PhysicsWorld);
    w.rigid_body_set.get(u64_to_body_handle(body))
        .map(|b| b.locked_axes().contains(LockedAxes::ROTATION_LOCKED_Z) as i32)
        .unwrap_or(0)
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys_body_set_dominance_group(world: *mut c_void, body: u64, group: i32) {
    let w = &mut *(world as *mut PhysicsWorld);
    if let Some(b) = w.rigid_body_set.get_mut(u64_to_body_handle(body)) {
        b.set_dominance_group(group as i8);
    }
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys_body_get_dominance_group(world: *mut c_void, body: u64) -> i32 {
    let w = &*(world as *mut PhysicsWorld);
    w.rigid_body_set.get(u64_to_body_handle(body))
        .map(|b| b.dominance_group() as i32)
        .unwrap_or(0)
}

/// Gets world-space center of mass. Fills `out` with [x, y].
#[no_mangle]
pub unsafe extern "C" fn sge_phys_body_get_world_center_of_mass(
    world: *mut c_void, body: u64, out: *mut f32,
) {
    let w = &*(world as *mut PhysicsWorld);
    let arr = slice::from_raw_parts_mut(out, 2);
    if let Some(b) = w.rigid_body_set.get(u64_to_body_handle(body)) {
        let com = b.center_of_mass();
        arr[0] = com.x;
        arr[1] = com.y;
    } else {
        arr[0] = 0.0;
        arr[1] = 0.0;
    }
}

// ---------------------------------------------------------------------------
// Body — CCD
// ---------------------------------------------------------------------------

#[no_mangle]
pub unsafe extern "C" fn sge_phys_body_enable_ccd(world: *mut c_void, body: u64, enable: i32) {
    let w = &mut *(world as *mut PhysicsWorld);
    if let Some(b) = w.rigid_body_set.get_mut(u64_to_body_handle(body)) {
        b.enable_ccd(enable != 0);
    }
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys_body_is_ccd_enabled(world: *mut c_void, body: u64) -> i32 {
    let w = &*(world as *mut PhysicsWorld);
    w.rigid_body_set.get(u64_to_body_handle(body)).map(|b| b.is_ccd_enabled() as i32).unwrap_or(0)
}

// ---------------------------------------------------------------------------
// Body — sleep
// ---------------------------------------------------------------------------

#[no_mangle]
pub unsafe extern "C" fn sge_phys_body_sleep(world: *mut c_void, body: u64) {
    let w = &mut *(world as *mut PhysicsWorld);
    if let Some(b) = w.rigid_body_set.get_mut(u64_to_body_handle(body)) {
        b.sleep();
    }
}

// ---------------------------------------------------------------------------
// Body — velocity at point
// ---------------------------------------------------------------------------

/// Gets the velocity of a point on the body in world space. Fills `out` with [vx, vy].
#[no_mangle]
pub unsafe extern "C" fn sge_phys_body_get_velocity_at_point(
    world: *mut c_void, body: u64, px: f32, py: f32, out: *mut f32,
) {
    let w = &*(world as *mut PhysicsWorld);
    let arr = slice::from_raw_parts_mut(out, 2);
    if let Some(b) = w.rigid_body_set.get(u64_to_body_handle(body)) {
        let vel = b.velocity_at_point(Vector::new(px, py));
        arr[0] = vel.x;
        arr[1] = vel.y;
    } else {
        arr[0] = 0.0;
        arr[1] = 0.0;
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
    let points: Vec<Vector> = (0..vertex_count as usize)
        .map(|i| Vector::new(verts[i * 2], verts[i * 2 + 1]))
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
// Collider — getters, enable/disable, position, AABB, mass
// ---------------------------------------------------------------------------

#[no_mangle]
pub unsafe extern "C" fn sge_phys_collider_get_density(world: *mut c_void, collider: u64) -> f32 {
    let w = &*(world as *mut PhysicsWorld);
    w.collider_set.get(u64_to_collider_handle(collider)).map(|c| c.density()).unwrap_or(0.0)
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys_collider_get_friction(world: *mut c_void, collider: u64) -> f32 {
    let w = &*(world as *mut PhysicsWorld);
    w.collider_set.get(u64_to_collider_handle(collider)).map(|c| c.friction()).unwrap_or(0.0)
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys_collider_get_restitution(world: *mut c_void, collider: u64) -> f32 {
    let w = &*(world as *mut PhysicsWorld);
    w.collider_set.get(u64_to_collider_handle(collider)).map(|c| c.restitution()).unwrap_or(0.0)
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys_collider_is_sensor(world: *mut c_void, collider: u64) -> i32 {
    let w = &*(world as *mut PhysicsWorld);
    w.collider_set.get(u64_to_collider_handle(collider)).map(|c| c.is_sensor() as i32).unwrap_or(0)
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys_collider_set_enabled(world: *mut c_void, collider: u64, enabled: i32) {
    let w = &mut *(world as *mut PhysicsWorld);
    if let Some(c) = w.collider_set.get_mut(u64_to_collider_handle(collider)) {
        c.set_enabled(enabled != 0);
    }
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys_collider_is_enabled(world: *mut c_void, collider: u64) -> i32 {
    let w = &*(world as *mut PhysicsWorld);
    w.collider_set.get(u64_to_collider_handle(collider)).map(|c| c.is_enabled() as i32).unwrap_or(0)
}

/// Gets collider position relative to parent body. Fills `out` with [x, y, angle].
#[no_mangle]
pub unsafe extern "C" fn sge_phys_collider_get_position_wrt_parent(
    world: *mut c_void, collider: u64, out: *mut f32,
) {
    let w = &*(world as *mut PhysicsWorld);
    let arr = slice::from_raw_parts_mut(out, 3);
    if let Some(c) = w.collider_set.get(u64_to_collider_handle(collider)) {
        if let Some(rel) = c.position_wrt_parent() {
            arr[0] = rel.translation.x;
            arr[1] = rel.translation.y;
            arr[2] = rel.rotation.angle();
        } else {
            arr[0] = 0.0; arr[1] = 0.0; arr[2] = 0.0;
        }
    } else {
        arr[0] = 0.0; arr[1] = 0.0; arr[2] = 0.0;
    }
}

/// Sets collider position relative to parent body.
#[no_mangle]
pub unsafe extern "C" fn sge_phys_collider_set_position_wrt_parent(
    world: *mut c_void, collider: u64, x: f32, y: f32, angle: f32,
) {
    let w = &mut *(world as *mut PhysicsWorld);
    if let Some(c) = w.collider_set.get_mut(u64_to_collider_handle(collider)) {
        c.set_position_wrt_parent(Pose::new(Vector::new(x, y), angle));
    }
}

/// Gets collider world position. Fills `out` with [x, y, angle].
#[no_mangle]
pub unsafe extern "C" fn sge_phys_collider_get_position(
    world: *mut c_void, collider: u64, out: *mut f32,
) {
    let w = &*(world as *mut PhysicsWorld);
    let arr = slice::from_raw_parts_mut(out, 3);
    if let Some(c) = w.collider_set.get(u64_to_collider_handle(collider)) {
        let pos = c.position();
        arr[0] = pos.translation.x;
        arr[1] = pos.translation.y;
        arr[2] = pos.rotation.angle();
    } else {
        arr[0] = 0.0; arr[1] = 0.0; arr[2] = 0.0;
    }
}

/// Returns collider shape type: 0=ball, 1=cuboid, 2=capsule, 3=segment, 4=triangle,
/// 5=trimesh, 6=polyline, 7=heightfield, 8=compound, 9=convex_polygon, 99=unknown
#[no_mangle]
pub unsafe extern "C" fn sge_phys_collider_get_shape_type(world: *mut c_void, collider: u64) -> i32 {
    let w = &*(world as *mut PhysicsWorld);
    w.collider_set.get(u64_to_collider_handle(collider)).map(|c| {
        let shape = c.shape();
        if shape.as_ball().is_some()            { 0 }
        else if shape.as_cuboid().is_some()     { 1 }
        else if shape.as_capsule().is_some()    { 2 }
        else if shape.as_segment().is_some()    { 3 }
        else if shape.as_triangle().is_some()   { 4 }
        else if shape.as_trimesh().is_some()    { 5 }
        else if shape.as_polyline().is_some()   { 6 }
        else if shape.as_heightfield().is_some(){ 7 }
        else if shape.as_compound().is_some()   { 8 }
        else if shape.as_convex_polygon().is_some() { 9 }
        else { 99 }
    }).unwrap_or(-1)
}

/// Gets collider AABB. Fills `out` with [minX, minY, maxX, maxY].
#[no_mangle]
pub unsafe extern "C" fn sge_phys_collider_get_aabb(
    world: *mut c_void, collider: u64, out: *mut f32,
) {
    let w = &*(world as *mut PhysicsWorld);
    let arr = slice::from_raw_parts_mut(out, 4);
    if let Some(c) = w.collider_set.get(u64_to_collider_handle(collider)) {
        let aabb = c.compute_aabb();
        arr[0] = aabb.mins.x;
        arr[1] = aabb.mins.y;
        arr[2] = aabb.maxs.x;
        arr[3] = aabb.maxs.y;
    } else {
        arr[0] = 0.0; arr[1] = 0.0; arr[2] = 0.0; arr[3] = 0.0;
    }
}

/// Gets the parent body handle of a collider. Returns 0 if no parent (unattached).
#[no_mangle]
pub unsafe extern "C" fn sge_phys_collider_get_parent_body(world: *mut c_void, collider: u64) -> u64 {
    let w = &*(world as *mut PhysicsWorld);
    w.collider_set.get(u64_to_collider_handle(collider))
        .and_then(|c| c.parent())
        .map(body_handle_to_u64)
        .unwrap_or(0)
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys_collider_get_mass(world: *mut c_void, collider: u64) -> f32 {
    let w = &*(world as *mut PhysicsWorld);
    w.collider_set.get(u64_to_collider_handle(collider)).map(|c| c.mass()).unwrap_or(0.0)
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys_collider_set_mass(world: *mut c_void, collider: u64, mass: f32) {
    let w = &mut *(world as *mut PhysicsWorld);
    if let Some(c) = w.collider_set.get_mut(u64_to_collider_handle(collider)) {
        c.set_mass(mass);
    }
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys_collider_set_contact_skin(world: *mut c_void, collider: u64, skin: f32) {
    let w = &mut *(world as *mut PhysicsWorld);
    if let Some(c) = w.collider_set.get_mut(u64_to_collider_handle(collider)) {
        c.set_contact_skin(skin);
    }
}

// ---------------------------------------------------------------------------
// Collider — active events / collision types
// ---------------------------------------------------------------------------

#[no_mangle]
pub unsafe extern "C" fn sge_phys_collider_set_active_events(world: *mut c_void, collider: u64, flags: i32) {
    let w = &mut *(world as *mut PhysicsWorld);
    if let Some(c) = w.collider_set.get_mut(u64_to_collider_handle(collider)) {
        c.set_active_events(ActiveEvents::from_bits_truncate(flags as u32));
    }
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys_collider_get_active_events(world: *mut c_void, collider: u64) -> i32 {
    let w = &*(world as *mut PhysicsWorld);
    w.collider_set.get(u64_to_collider_handle(collider))
        .map(|c| c.active_events().bits() as i32)
        .unwrap_or(0)
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys_collider_set_active_collision_types(world: *mut c_void, collider: u64, flags: i32) {
    let w = &mut *(world as *mut PhysicsWorld);
    if let Some(c) = w.collider_set.get_mut(u64_to_collider_handle(collider)) {
        c.set_active_collision_types(ActiveCollisionTypes::from_bits_truncate(flags as u16));
    }
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys_collider_get_active_collision_types(world: *mut c_void, collider: u64) -> i32 {
    let w = &*(world as *mut PhysicsWorld);
    w.collider_set.get(u64_to_collider_handle(collider))
        .map(|c| c.active_collision_types().bits() as i32)
        .unwrap_or(0)
}

// ---------------------------------------------------------------------------
// Collider — trimesh and heightfield shapes
// ---------------------------------------------------------------------------

/// Creates a triangle mesh collider. Returns a collider handle.
/// `vertices`: flat [x0,y0,x1,y1,...], `indices`: flat [i0,i1,i2, i3,i4,i5,...] (triangles).
#[no_mangle]
pub unsafe extern "C" fn sge_phys_create_trimesh_collider(
    world: *mut c_void, body: u64,
    vertices: *const f32, vertex_count: i32,
    indices: *const u32, index_count: i32,
) -> u64 {
    let w = &mut *(world as *mut PhysicsWorld);
    let verts = slice::from_raw_parts(vertices, (vertex_count * 2) as usize);
    let idxs  = slice::from_raw_parts(indices, index_count as usize);

    let points: Vec<Vector> = (0..vertex_count as usize)
        .map(|i| Vector::new(verts[i * 2], verts[i * 2 + 1]))
        .collect();
    let tris: Vec<[u32; 3]> = idxs.chunks_exact(3)
        .map(|c| [c[0], c[1], c[2]])
        .collect();

    let collider = ColliderBuilder::trimesh(points, tris).unwrap().build();
    let handle = w.collider_set.insert_with_parent(
        collider, u64_to_body_handle(body), &mut w.rigid_body_set,
    );
    collider_handle_to_u64(handle)
}

/// Creates a heightfield collider. Returns a collider handle.
/// `heights`: row-major array of `num_cols` height values.
#[no_mangle]
pub unsafe extern "C" fn sge_phys_create_heightfield_collider(
    world: *mut c_void, body: u64,
    heights: *const f32, num_cols: i32,
    scale_x: f32, scale_y: f32,
) -> u64 {
    let w = &mut *(world as *mut PhysicsWorld);
    let h = slice::from_raw_parts(heights, num_cols as usize);
    let heights_vec: Vec<Real> = h.to_vec();
    let collider = ColliderBuilder::heightfield(heights_vec, Vector::new(scale_x, scale_y)).build();
    let handle = w.collider_set.insert_with_parent(
        collider, u64_to_body_handle(body), &mut w.rigid_body_set,
    );
    collider_handle_to_u64(handle)
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
            InteractionTestMode::And,
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
            InteractionTestMode::And,
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
        .local_anchor1(Vector::new(anchor_x, anchor_y))
        .local_anchor2(Vector::new(0.0, 0.0))
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
    let axis = Vector::new(axis_x, axis_y).normalize();
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
    if let Some(j) = w.impulse_joint_set.get_mut(u64_to_joint_handle(joint), true) {
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
    if let Some(j) = w.impulse_joint_set.get_mut(u64_to_joint_handle(joint), true) {
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
    if let Some(j) = w.impulse_joint_set.get_mut(u64_to_joint_handle(joint), true) {
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
    if let Some(j) = w.impulse_joint_set.get_mut(u64_to_joint_handle(joint), true) {
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
    if let Some(j) = w.impulse_joint_set.get_mut(u64_to_joint_handle(joint), true) {
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
    if let Some(j) = w.impulse_joint_set.get_mut(u64_to_joint_handle(joint), true) {
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
    if let Some(j) = w.impulse_joint_set.get_mut(u64_to_joint_handle(joint), true) {
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
    if let Some(j) = w.impulse_joint_set.get_mut(u64_to_joint_handle(joint), true) {
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
    if let Some(j) = w.impulse_joint_set.get_mut(u64_to_joint_handle(joint), true) {
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
    if let Some(j) = w.impulse_joint_set.get_mut(u64_to_joint_handle(joint), true) {
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
                return diff.length();
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
    if let Some(j) = w.impulse_joint_set.get_mut(u64_to_joint_handle(joint), true) {
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
// Motor joint (controls relative position/angle between two bodies)
// ---------------------------------------------------------------------------

/// Creates a motor joint between two bodies.
/// Uses Rapier's GenericJoint with per-axis position motors to control
/// the relative translation and rotation (maps to Box2D's MotorJoint).
#[no_mangle]
pub unsafe extern "C" fn sge_phys_create_motor_joint(
    world: *mut c_void,
    body1: u64,
    body2: u64,
) -> u64 {
    let w = &mut *(world as *mut PhysicsWorld);
    // Create a generic joint with motors on all 2D axes (X, Y, AngX).
    // Default: zero offset, moderate stiffness/damping.
    let mut joint = GenericJointBuilder::new(JointAxesMask::empty()).build();
    let stiffness = 100.0;
    let damping   = 20.0;
    joint.set_motor(JointAxis::LinX,    0.0, 0.0, stiffness, damping);
    joint.set_motor(JointAxis::LinY,    0.0, 0.0, stiffness, damping);
    joint.set_motor(JointAxis::AngX, 0.0, 0.0, stiffness, damping);
    let handle = w.impulse_joint_set.insert(
        u64_to_body_handle(body1),
        u64_to_body_handle(body2),
        joint,
        true,
    );
    joint_handle_to_u64(handle)
}

/// Sets the target linear offset for a motor joint.
#[no_mangle]
pub unsafe extern "C" fn sge_phys_motor_joint_set_linear_offset(
    world: *mut c_void,
    joint: u64,
    x: f32,
    y: f32,
) {
    let w = &mut *(world as *mut PhysicsWorld);
    if let Some(j) = w.impulse_joint_set.get_mut(u64_to_joint_handle(joint), true) {
        let sx = j.data.motor(JointAxis::LinX).map(|m| m.stiffness).unwrap_or(100.0);
        let dx = j.data.motor(JointAxis::LinX).map(|m| m.damping).unwrap_or(20.0);
        let sy = j.data.motor(JointAxis::LinY).map(|m| m.stiffness).unwrap_or(100.0);
        let dy = j.data.motor(JointAxis::LinY).map(|m| m.damping).unwrap_or(20.0);
        j.data.set_motor(JointAxis::LinX, x, 0.0, sx, dx);
        j.data.set_motor(JointAxis::LinY, y, 0.0, sy, dy);
    }
}

/// Gets the target linear offset for a motor joint. Fills `out` with [x, y].
#[no_mangle]
pub unsafe extern "C" fn sge_phys_motor_joint_get_linear_offset(
    world: *mut c_void,
    joint: u64,
    out: *mut f32,
) {
    let w = &*(world as *mut PhysicsWorld);
    let arr = slice::from_raw_parts_mut(out, 2);
    if let Some(j) = w.impulse_joint_set.get(u64_to_joint_handle(joint)) {
        arr[0] = j.data.motor(JointAxis::LinX).map(|m| m.target_pos).unwrap_or(0.0);
        arr[1] = j.data.motor(JointAxis::LinY).map(|m| m.target_pos).unwrap_or(0.0);
    } else {
        arr[0] = 0.0;
        arr[1] = 0.0;
    }
}

/// Sets the target angular offset for a motor joint.
#[no_mangle]
pub unsafe extern "C" fn sge_phys_motor_joint_set_angular_offset(
    world: *mut c_void,
    joint: u64,
    angle: f32,
) {
    let w = &mut *(world as *mut PhysicsWorld);
    if let Some(j) = w.impulse_joint_set.get_mut(u64_to_joint_handle(joint), true) {
        let s = j.data.motor(JointAxis::AngX).map(|m| m.stiffness).unwrap_or(100.0);
        let d = j.data.motor(JointAxis::AngX).map(|m| m.damping).unwrap_or(20.0);
        j.data.set_motor(JointAxis::AngX, angle, 0.0, s, d);
    }
}

/// Gets the target angular offset for a motor joint.
#[no_mangle]
pub unsafe extern "C" fn sge_phys_motor_joint_get_angular_offset(
    world: *mut c_void,
    joint: u64,
) -> f32 {
    let w = &*(world as *mut PhysicsWorld);
    if let Some(j) = w.impulse_joint_set.get(u64_to_joint_handle(joint)) {
        return j.data.motor(JointAxis::AngX).map(|m| m.target_pos).unwrap_or(0.0);
    }
    0.0
}

/// Sets the maximum linear force for a motor joint.
#[no_mangle]
pub unsafe extern "C" fn sge_phys_motor_joint_set_max_force(
    world: *mut c_void,
    joint: u64,
    force: f32,
) {
    let w = &mut *(world as *mut PhysicsWorld);
    if let Some(j) = w.impulse_joint_set.get_mut(u64_to_joint_handle(joint), true) {
        j.data.set_motor_max_force(JointAxis::LinX, force);
        j.data.set_motor_max_force(JointAxis::LinY, force);
    }
}

/// Sets the maximum angular torque for a motor joint.
#[no_mangle]
pub unsafe extern "C" fn sge_phys_motor_joint_set_max_torque(
    world: *mut c_void,
    joint: u64,
    torque: f32,
) {
    let w = &mut *(world as *mut PhysicsWorld);
    if let Some(j) = w.impulse_joint_set.get_mut(u64_to_joint_handle(joint), true) {
        j.data.set_motor_max_force(JointAxis::AngX, torque);
    }
}

/// Sets the correction factor (stiffness) for all motor axes.
/// Higher values make the motor snap to the target faster.
#[no_mangle]
pub unsafe extern "C" fn sge_phys_motor_joint_set_correction_factor(
    world: *mut c_void,
    joint: u64,
    factor: f32,
) {
    let w = &mut *(world as *mut PhysicsWorld);
    if let Some(j) = w.impulse_joint_set.get_mut(u64_to_joint_handle(joint), true) {
        // Map correction factor to stiffness; damping = 2 * sqrt(stiffness) for critical damping
        let stiffness = factor * 100.0;
        let damping   = 2.0 * stiffness.sqrt();
        for axis in [JointAxis::LinX, JointAxis::LinY, JointAxis::AngX] {
            let target = j.data.motor(axis).map(|m| m.target_pos).unwrap_or(0.0);
            j.data.set_motor(axis, target, 0.0, stiffness, damping);
        }
    }
}

// ---------------------------------------------------------------------------
// Contact detail queries
// ---------------------------------------------------------------------------

/// Gets the number of contact points between two colliders.
/// Returns 0 if the colliders are not in contact.
#[no_mangle]
pub unsafe extern "C" fn sge_phys_contact_pair_count(
    world: *mut c_void,
    collider1: u64,
    collider2: u64,
) -> i32 {
    let w = &*(world as *mut PhysicsWorld);
    let c1 = u64_to_collider_handle(collider1);
    let c2 = u64_to_collider_handle(collider2);
    if let Some(pair) = w.narrow_phase.contact_pair(c1, c2) {
        pair.manifolds.iter().map(|m| m.points.len() as i32).sum()
    } else {
        0
    }
}

/// Gets contact details between two colliders.
///
/// Fills `out` with `[normalX, normalY, pointX, pointY, penetration]` per contact point
/// (5 floats each). Points are in world space. Returns the number of points written
/// (capped at `max_points`).
#[no_mangle]
pub unsafe extern "C" fn sge_phys_contact_pair_points(
    world: *mut c_void,
    collider1: u64,
    collider2: u64,
    out: *mut f32,
    max_points: i32,
) -> i32 {
    let w = &*(world as *mut PhysicsWorld);
    let c1 = u64_to_collider_handle(collider1);
    let c2 = u64_to_collider_handle(collider2);

    let arr = slice::from_raw_parts_mut(out, (max_points * 5) as usize);
    let mut count = 0i32;

    if let Some(pair) = w.narrow_phase.contact_pair(c1, c2) {
        // Transform local contact points to world space using collider positions
        let pos1 = w.collider_set.get(c1).map(|c| *c.position()).unwrap_or(Pose::identity());
        for manifold in &pair.manifolds {
            let normal = manifold.data.normal;
            for pt in &manifold.points {
                if count >= max_points { return count; }
                let idx = (count * 5) as usize;
                let world_pt = pos1 * pt.local_p1;
                arr[idx]     = normal.x;
                arr[idx + 1] = normal.y;
                arr[idx + 2] = world_pt.x;
                arr[idx + 3] = world_pt.y;
                arr[idx + 4] = pt.dist; // negative = penetration
                count += 1;
            }
        }
    }
    count
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
    let collider = ColliderBuilder::segment(Vector::new(x1, y1), Vector::new(x2, y2)).build();
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
    let points: Vec<Vector> = (0..vertex_count as usize)
        .map(|i| Vector::new(verts[i * 2], verts[i * 2 + 1]))
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
    let ray = Ray::new(Vector::new(origin_x, origin_y), Vector::new(dir_x, dir_y));

    let query_pipeline = w.broad_phase.as_query_pipeline(
        &DefaultQueryDispatcher,
        &w.rigid_body_set,
        &w.collider_set,
        QueryFilter::default(),
    );

    if let Some((handle, intersection)) = query_pipeline.cast_ray_and_get_normal(
        &ray,
        max_dist,
        true,
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
    let aabb = Aabb::new(Vector::new(min_x, min_y), Vector::new(max_x, max_y));
    let out = slice::from_raw_parts_mut(out_colliders, max_results as usize);
    let mut count = 0i32;

    let query_pipeline = w.broad_phase.as_query_pipeline(
        &DefaultQueryDispatcher,
        &w.rigid_body_set,
        &w.collider_set,
        QueryFilter::default(),
    );

    for (handle, _collider) in query_pipeline.intersect_aabb_conservative(aabb) {
        if count >= max_results {
            break;
        }
        out[count as usize] = collider_handle_to_u64(handle);
        count += 1;
    }
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
    let pt = Vector::new(x, y);
    let out = slice::from_raw_parts_mut(out_bodies, max_results as usize);
    let mut count = 0i32;

    let query_pipeline = w.broad_phase.as_query_pipeline(
        &DefaultQueryDispatcher,
        &w.rigid_body_set,
        &w.collider_set,
        QueryFilter::default(),
    );

    for (_handle, collider) in query_pipeline.intersect_point(pt) {
        if count >= max_results {
            break;
        }
        if let Some(parent) = collider.parent() {
            out[count as usize] = body_handle_to_u64(parent);
            count += 1;
        }
    }
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
// Joint — missing getters
// ---------------------------------------------------------------------------

#[no_mangle]
pub unsafe extern "C" fn sge_phys_revolute_joint_get_max_motor_torque(world: *mut c_void, joint: u64) -> f32 {
    let w = &*(world as *mut PhysicsWorld);
    w.impulse_joint_set.get(u64_to_joint_handle(joint))
        .and_then(|j| j.data.as_revolute())
        .and_then(|r| r.motor())
        .map(|m| m.max_force)
        .unwrap_or(0.0)
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys_prismatic_joint_get_motor_speed(world: *mut c_void, joint: u64) -> f32 {
    let w = &*(world as *mut PhysicsWorld);
    w.impulse_joint_set.get(u64_to_joint_handle(joint))
        .and_then(|j| j.data.as_prismatic())
        .and_then(|p| p.motor())
        .map(|m| m.target_vel)
        .unwrap_or(0.0)
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys_prismatic_joint_get_max_motor_force(world: *mut c_void, joint: u64) -> f32 {
    let w = &*(world as *mut PhysicsWorld);
    w.impulse_joint_set.get(u64_to_joint_handle(joint))
        .and_then(|j| j.data.as_prismatic())
        .and_then(|p| p.motor())
        .map(|m| m.max_force)
        .unwrap_or(0.0)
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys_motor_joint_get_max_force(world: *mut c_void, joint: u64) -> f32 {
    let w = &*(world as *mut PhysicsWorld);
    w.impulse_joint_set.get(u64_to_joint_handle(joint))
        .and_then(|j| j.data.motor(JointAxis::LinX))
        .map(|m| m.max_force)
        .unwrap_or(0.0)
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys_motor_joint_get_max_torque(world: *mut c_void, joint: u64) -> f32 {
    let w = &*(world as *mut PhysicsWorld);
    w.impulse_joint_set.get(u64_to_joint_handle(joint))
        .and_then(|j| j.data.motor(JointAxis::AngX))
        .map(|m| m.max_force)
        .unwrap_or(0.0)
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys_motor_joint_get_correction_factor(world: *mut c_void, joint: u64) -> f32 {
    let w = &*(world as *mut PhysicsWorld);
    // Correction factor was stored as stiffness / 100.0. Reverse it.
    w.impulse_joint_set.get(u64_to_joint_handle(joint))
        .and_then(|j| j.data.motor(JointAxis::LinX))
        .map(|m| m.stiffness / 100.0)
        .unwrap_or(0.0)
}

// ---------------------------------------------------------------------------
// Spring joint
// ---------------------------------------------------------------------------

/// Creates a spring joint emulated via GenericJoint with position motors.
/// The motor target on the X axis is set to `rest_length`, with the given
/// `stiffness` and `damping` controlling the spring-damper behavior.
#[no_mangle]
pub unsafe extern "C" fn sge_phys_create_spring_joint(
    world: *mut c_void, body1: u64, body2: u64,
    rest_length: f32, stiffness: f32, damping: f32,
) -> u64 {
    let w = &mut *(world as *mut PhysicsWorld);
    let mut joint = GenericJointBuilder::new(JointAxesMask::empty()).build();
    // Use LinX motor to maintain rest_length distance with spring behavior
    joint.set_motor(JointAxis::LinX, rest_length, 0.0, stiffness, damping);
    joint.set_motor(JointAxis::LinY, 0.0, 0.0, stiffness, damping);
    let handle = w.impulse_joint_set.insert(
        u64_to_body_handle(body1), u64_to_body_handle(body2), joint, true,
    );
    joint_handle_to_u64(handle)
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys_spring_joint_set_rest_length(world: *mut c_void, joint: u64, rest_length: f32) {
    let w = &mut *(world as *mut PhysicsWorld);
    if let Some(j) = w.impulse_joint_set.get_mut(u64_to_joint_handle(joint), true) {
        let s = j.data.motor(JointAxis::LinX).map(|m| m.stiffness).unwrap_or(100.0);
        let d = j.data.motor(JointAxis::LinX).map(|m| m.damping).unwrap_or(10.0);
        j.data.set_motor(JointAxis::LinX, rest_length, 0.0, s, d);
    }
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys_spring_joint_get_rest_length(world: *mut c_void, joint: u64) -> f32 {
    let w = &*(world as *mut PhysicsWorld);
    w.impulse_joint_set.get(u64_to_joint_handle(joint))
        .and_then(|j| j.data.motor(JointAxis::LinX))
        .map(|m| m.target_pos)
        .unwrap_or(0.0)
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys_spring_joint_set_params(
    world: *mut c_void, joint: u64, stiffness: f32, damping: f32,
) {
    let w = &mut *(world as *mut PhysicsWorld);
    if let Some(j) = w.impulse_joint_set.get_mut(u64_to_joint_handle(joint), true) {
        let target = j.data.motor(JointAxis::LinX).map(|m| m.target_pos).unwrap_or(0.0);
        j.data.set_motor(JointAxis::LinX, target, 0.0, stiffness, damping);
        j.data.set_motor(JointAxis::LinY, 0.0, 0.0, stiffness, damping);
    }
}

// ---------------------------------------------------------------------------
// Queries — shape cast, ray cast all, point projection
// ---------------------------------------------------------------------------

/// Shape cast (sweep test). Returns 1 on hit, 0 on miss.
/// `shape_type`: 0=circle, 1=box, 2=capsule. `shape_params` depends on type:
///   circle: [radius], box: [halfWidth, halfHeight], capsule: [halfHeight, radius]
/// `out`: [hitX, hitY, normalX, normalY, toi, colliderLo, colliderHi] = 7 floats.
#[no_mangle]
pub unsafe extern "C" fn sge_phys_cast_shape(
    world: *mut c_void,
    shape_type: i32, shape_params: *const f32,
    origin_x: f32, origin_y: f32,
    dir_x: f32, dir_y: f32,
    max_dist: f32,
    out: *mut f32,
) -> i32 {
    let w = &*(world as *mut PhysicsWorld);
    let params = slice::from_raw_parts(shape_params, 2);

    let shape: Box<dyn Shape> = match shape_type {
        0 => Box::new(Ball::new(params[0])),
        1 => Box::new(Cuboid::new(Vector::new(params[0], params[1]))),
        2 => Box::new(Capsule::new_y(params[0], params[1])),
        _ => return 0,
    };

    let origin = Pose::new(Vector::new(origin_x, origin_y), 0.0);
    let dir    = Vector::new(dir_x, dir_y);

    let query_pipeline = w.broad_phase.as_query_pipeline(
        &DefaultQueryDispatcher,
        &w.rigid_body_set,
        &w.collider_set,
        QueryFilter::default(),
    );

    if let Some((handle, toi_result)) = query_pipeline.cast_shape(
        &origin, dir, shape.as_ref(),
        ShapeCastOptions { max_time_of_impact: max_dist, ..Default::default() },
    ) {
        let arr = slice::from_raw_parts_mut(out, 7);
        let hit_point = Vector::new(origin_x, origin_y) + dir * toi_result.time_of_impact;
        arr[0] = hit_point.x;
        arr[1] = hit_point.y;
        arr[2] = toi_result.normal1.x;
        arr[3] = toi_result.normal1.y;
        arr[4] = toi_result.time_of_impact;
        let ch = collider_handle_to_u64(handle);
        arr[5] = f32::from_bits(ch as u32);
        arr[6] = f32::from_bits((ch >> 32) as u32);
        1
    } else {
        0
    }
}

/// Ray cast returning ALL intersections. Each hit = 7 floats:
/// [hitX, hitY, normalX, normalY, toi, colliderLo, colliderHi].
/// Returns the number of hits (capped at `max_hits`).
#[no_mangle]
pub unsafe extern "C" fn sge_phys_ray_cast_all(
    world: *mut c_void,
    ox: f32, oy: f32, dx: f32, dy: f32, max_dist: f32,
    out_hits: *mut f32, max_hits: i32,
) -> i32 {
    let w = &*(world as *mut PhysicsWorld);
    let ray = Ray::new(Vector::new(ox, oy), Vector::new(dx, dy));
    let arr = slice::from_raw_parts_mut(out_hits, (max_hits * 7) as usize);
    let mut count = 0i32;

    let query_pipeline = w.broad_phase.as_query_pipeline(
        &DefaultQueryDispatcher,
        &w.rigid_body_set,
        &w.collider_set,
        QueryFilter::default(),
    );

    for (handle, _collider, intersection) in query_pipeline.intersect_ray(ray, max_dist, true) {
        if count >= max_hits {
            break;
        }
        let idx = (count * 7) as usize;
        let hit = ray.point_at(intersection.time_of_impact);
        arr[idx]     = hit.x;
        arr[idx + 1] = hit.y;
        arr[idx + 2] = intersection.normal.x;
        arr[idx + 3] = intersection.normal.y;
        arr[idx + 4] = intersection.time_of_impact;
        let ch = collider_handle_to_u64(handle);
        arr[idx + 5] = f32::from_bits(ch as u32);
        arr[idx + 6] = f32::from_bits((ch >> 32) as u32);
        count += 1;
    }
    count
}

/// Projects a point onto the closest collider. Returns 1 if found, 0 otherwise.
/// `out`: [projX, projY, isInside (1.0 or 0.0), colliderLo, colliderHi] = 5 floats.
#[no_mangle]
pub unsafe extern "C" fn sge_phys_project_point(
    world: *mut c_void, x: f32, y: f32, out: *mut f32,
) -> i32 {
    let w = &*(world as *mut PhysicsWorld);
    let pt = Vector::new(x, y);

    let query_pipeline = w.broad_phase.as_query_pipeline(
        &DefaultQueryDispatcher,
        &w.rigid_body_set,
        &w.collider_set,
        QueryFilter::default(),
    );

    if let Some((handle, projection)) = query_pipeline.project_point(
        pt, Real::MAX, true,
    ) {
        let arr = slice::from_raw_parts_mut(out, 5);
        arr[0] = projection.point.x;
        arr[1] = projection.point.y;
        arr[2] = if projection.is_inside { 1.0 } else { 0.0 };
        let ch = collider_handle_to_u64(handle);
        arr[3] = f32::from_bits(ch as u32);
        arr[4] = f32::from_bits((ch >> 32) as u32);
        1
    } else {
        0
    }
}

// ---------------------------------------------------------------------------
// Intersection events (sensor overlaps)
// ---------------------------------------------------------------------------

#[no_mangle]
pub unsafe extern "C" fn sge_phys_poll_intersection_start_events(
    world: *mut c_void, out_collider1: *mut u64, out_collider2: *mut u64, max_events: i32,
) -> i32 {
    // Intersection events are collected via the ChannelEventCollector's intersection channel.
    // Currently, the world step only buffers contact events. We need to extend.
    // For now, return 0 — intersection event buffering will be added in a follow-up.
    let _ = (world, out_collider1, out_collider2, max_events);
    0
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys_poll_intersection_stop_events(
    world: *mut c_void, out_collider1: *mut u64, out_collider2: *mut u64, max_events: i32,
) -> i32 {
    let _ = (world, out_collider1, out_collider2, max_events);
    0
}

// ---------------------------------------------------------------------------
// World — simulation parameters
// ---------------------------------------------------------------------------

#[no_mangle]
pub unsafe extern "C" fn sge_phys_world_set_num_solver_iterations(world: *mut c_void, iters: i32) {
    let w = &mut *(world as *mut PhysicsWorld);
    w.integration_parameters.num_solver_iterations = (iters as usize).max(1);
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys_world_get_num_solver_iterations(world: *mut c_void) -> i32 {
    let w = &*(world as *mut PhysicsWorld);
    w.integration_parameters.num_solver_iterations as i32
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys_world_set_num_additional_friction_iterations(world: *mut c_void, iters: i32) {
    let w = &mut *(world as *mut PhysicsWorld);
    // num_additional_friction_iterations was removed in rapier 0.32;
    // use num_internal_stabilization_iterations as the closest equivalent.
    w.integration_parameters.num_internal_stabilization_iterations = iters as usize;
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys_world_set_num_internal_pgs_iterations(world: *mut c_void, iters: i32) {
    let w = &mut *(world as *mut PhysicsWorld);
    w.integration_parameters.num_internal_pgs_iterations = iters as usize;
}

// ---------------------------------------------------------------------------
// Queries — shape intersection
// ---------------------------------------------------------------------------

/// Tests if a shape at a given position overlaps any collider.
/// `shape_type`: 0=circle, 1=box, 2=capsule. `shape_params` depends on type:
///   circle: [radius], box: [halfWidth, halfHeight], capsule: [halfHeight, radius]
/// Fills `out_colliders` with collider handles. Returns the count (capped at `max_results`).
#[no_mangle]
pub unsafe extern "C" fn sge_phys_intersect_shape(
    world: *mut c_void,
    shape_type: i32, shape_params: *const f32,
    pos_x: f32, pos_y: f32, angle: f32,
    out_colliders: *mut u64, max_results: i32,
) -> i32 {
    let w = &*(world as *mut PhysicsWorld);
    let params = slice::from_raw_parts(shape_params, 2);

    let shape: Box<dyn Shape> = match shape_type {
        0 => Box::new(Ball::new(params[0])),
        1 => Box::new(Cuboid::new(Vector::new(params[0], params[1]))),
        2 => Box::new(Capsule::new_y(params[0], params[1])),
        _ => return 0,
    };

    let pos = Pose::new(Vector::new(pos_x, pos_y), angle);
    let arr = slice::from_raw_parts_mut(out_colliders, max_results as usize);
    let mut count = 0i32;

    let query_pipeline = w.broad_phase.as_query_pipeline(
        &DefaultQueryDispatcher,
        &w.rigid_body_set,
        &w.collider_set,
        QueryFilter::default(),
    );

    for (handle, _collider) in query_pipeline.intersect_shape(
        pos, shape.as_ref(),
    ) {
        if count >= max_results { break; }
        arr[count as usize] = collider_handle_to_u64(handle);
        count += 1;
    }
    count
}

// ---------------------------------------------------------------------------
// Contact force events (polling)
// ---------------------------------------------------------------------------

/// Polls contact force events since the last step.
/// Fills out_collider1, out_collider2, and out_force arrays.
/// Returns the event count (capped at max_events).
#[no_mangle]
pub unsafe extern "C" fn sge_phys_poll_contact_force_events(
    world: *mut c_void,
    out_collider1: *mut u64,
    out_collider2: *mut u64,
    out_force: *mut f32,
    max_events: i32,
) -> i32 {
    let w = &*(world as *mut PhysicsWorld);
    let c1 = slice::from_raw_parts_mut(out_collider1, max_events as usize);
    let c2 = slice::from_raw_parts_mut(out_collider2, max_events as usize);
    let forces = slice::from_raw_parts_mut(out_force, max_events as usize);
    let count = w.contact_force_buf.len().min(max_events as usize);
    for i in 0..count {
        c1[i] = w.contact_force_buf[i].0;
        c2[i] = w.contact_force_buf[i].1;
        forces[i] = w.contact_force_buf[i].2;
    }
    count as i32
}

// ---------------------------------------------------------------------------
// Collider — contact force event threshold
// ---------------------------------------------------------------------------

/// Sets the contact force event threshold for a collider.
/// Force events are only generated when total force exceeds this threshold.
/// Requires ActiveEvents::CONTACT_FORCE_EVENTS to be set on the collider.
#[no_mangle]
pub unsafe extern "C" fn sge_phys_collider_set_contact_force_event_threshold(
    world: *mut c_void, collider: u64, threshold: f32,
) {
    let w = &mut *(world as *mut PhysicsWorld);
    if let Some(c) = w.collider_set.get_mut(u64_to_collider_handle(collider)) {
        c.set_contact_force_event_threshold(threshold);
    }
}

/// Gets the contact force event threshold for a collider.
#[no_mangle]
pub unsafe extern "C" fn sge_phys_collider_get_contact_force_event_threshold(
    world: *mut c_void, collider: u64,
) -> f32 {
    let w = &*(world as *mut PhysicsWorld);
    w.collider_set.get(u64_to_collider_handle(collider))
        .map(|c| c.contact_force_event_threshold())
        .unwrap_or(0.0)
}

// ---------------------------------------------------------------------------
// Collider — active hooks
// ---------------------------------------------------------------------------

/// Sets the active hooks flags for a collider (bitmask of ActiveHooks bits).
/// Bit 0x01 = FILTER_CONTACT_PAIRS
/// Bit 0x02 = FILTER_INTERSECTION_PAIR
/// Bit 0x04 = MODIFY_SOLVER_CONTACTS (required for one-way platforms)
#[no_mangle]
pub unsafe extern "C" fn sge_phys_collider_set_active_hooks(
    world: *mut c_void, collider: u64, flags: i32,
) {
    let w = &mut *(world as *mut PhysicsWorld);
    if let Some(c) = w.collider_set.get_mut(u64_to_collider_handle(collider)) {
        c.set_active_hooks(ActiveHooks::from_bits_truncate(flags as u32));
    }
}

/// Gets the active hooks flags for a collider.
#[no_mangle]
pub unsafe extern "C" fn sge_phys_collider_get_active_hooks(
    world: *mut c_void, collider: u64,
) -> i32 {
    let w = &*(world as *mut PhysicsWorld);
    w.collider_set.get(u64_to_collider_handle(collider))
        .map(|c| c.active_hooks().bits() as i32)
        .unwrap_or(0)
}

// ---------------------------------------------------------------------------
// Collider — one-way platform
// ---------------------------------------------------------------------------

/// Marks a collider as a one-way platform. Contacts with this collider are
/// only kept if the contact normal aligns with the given direction.
/// The allowed_angle (radians) controls the tolerance cone around the direction.
/// Set nx=0, ny=0 to disable one-way behavior for this collider.
///
/// Requires ActiveHooks::MODIFY_SOLVER_CONTACTS (0x04) to be set on the collider
/// via sge_phys_collider_set_active_hooks.
#[no_mangle]
pub unsafe extern "C" fn sge_phys_collider_set_one_way_direction(
    world: *mut c_void, collider: u64, nx: f32, ny: f32, allowed_angle: f32,
) {
    let w = &mut *(world as *mut PhysicsWorld);
    if nx == 0.0 && ny == 0.0 {
        w.one_way_platforms.remove(&collider);
    } else {
        w.one_way_platforms.insert(collider, ([nx, ny], allowed_angle));
    }
}

/// Returns 1 if the collider has one-way platform behavior, 0 otherwise.
/// If it does, fills out_nx, out_ny, out_angle with the configured direction and angle.
#[no_mangle]
pub unsafe extern "C" fn sge_phys_collider_get_one_way_direction(
    world: *mut c_void, collider: u64,
    out_nx: *mut f32, out_ny: *mut f32, out_angle: *mut f32,
) -> i32 {
    let w = &*(world as *mut PhysicsWorld);
    if let Some(&(dir, angle)) = w.one_way_platforms.get(&collider) {
        *out_nx = dir[0];
        *out_ny = dir[1];
        *out_angle = angle;
        1
    } else {
        0
    }
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

    // -- Motor joint --------------------------------------------------------

    #[test]
    fn motor_joint_offset_control() {
        unsafe {
            let world = sge_phys_create_world(0.0, 0.0);

            let b1 = sge_phys_create_static_body(world, 0.0, 0.0, 0.0);
            let _c1 = sge_phys_create_box_collider(world, b1, 0.5, 0.5);
            let b2 = sge_phys_create_dynamic_body(world, 1.0, 0.0, 0.0);
            let _c2 = sge_phys_create_box_collider(world, b2, 0.5, 0.5);

            let joint = sge_phys_create_motor_joint(world, b1, b2);

            // Set linear offset
            sge_phys_motor_joint_set_linear_offset(world, joint, 3.0, 2.0);
            let mut offset = [0.0f32; 2];
            sge_phys_motor_joint_get_linear_offset(world, joint, offset.as_mut_ptr());
            assert!((offset[0] - 3.0).abs() < 1e-5, "x offset should be 3.0, got {}", offset[0]);
            assert!((offset[1] - 2.0).abs() < 1e-5, "y offset should be 2.0, got {}", offset[1]);

            // Set angular offset
            sge_phys_motor_joint_set_angular_offset(world, joint, 1.5);
            let angle = sge_phys_motor_joint_get_angular_offset(world, joint);
            assert!((angle - 1.5).abs() < 1e-5, "angular offset should be 1.5, got {}", angle);

            sge_phys_destroy_world(world);
        }
    }

    // -- Contact details ---------------------------------------------------

    #[test]
    fn contact_pair_reports_collision() {
        unsafe {
            let world = sge_phys_create_world(0.0, -9.81);

            // Create two overlapping boxes
            let b1 = sge_phys_create_static_body(world, 0.0, 0.0, 0.0);
            let c1 = sge_phys_create_box_collider(world, b1, 1.0, 1.0);
            let b2 = sge_phys_create_dynamic_body(world, 0.0, 1.5, 0.0);
            let c2 = sge_phys_create_box_collider(world, b2, 1.0, 1.0);

            // Step to generate contacts
            for _ in 0..10 {
                sge_phys_world_step(world, 1.0 / 60.0);
            }

            let count = sge_phys_contact_pair_count(world, c1, c2);
            assert!(count > 0, "should have contact points, got {}", count);

            let mut out = [0.0f32; 10]; // room for 2 points × 5 floats
            let pts = sge_phys_contact_pair_points(world, c1, c2, out.as_mut_ptr(), 2);
            assert!(pts > 0, "should return at least 1 contact point, got {}", pts);

            sge_phys_destroy_world(world);
        }
    }

    #[test]
    fn contact_pair_no_collision() {
        unsafe {
            let world = sge_phys_create_world(0.0, 0.0);

            // Two boxes far apart — no contact
            let b1 = sge_phys_create_static_body(world, 0.0, 0.0, 0.0);
            let c1 = sge_phys_create_box_collider(world, b1, 0.5, 0.5);
            let b2 = sge_phys_create_static_body(world, 100.0, 0.0, 0.0);
            let c2 = sge_phys_create_box_collider(world, b2, 0.5, 0.5);

            sge_phys_world_step(world, 1.0 / 60.0);

            let count = sge_phys_contact_pair_count(world, c1, c2);
            assert_eq!(count, 0, "should have no contacts");

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
