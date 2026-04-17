// 3D Physics — wraps Rapier3D for rigid body simulation
//
// Provides C ABI functions for:
//   - Desktop JVM via Panama FFM (java.lang.foreign)
//   - Scala Native via @extern
//
// All public functions are prefixed with sge_phys3d_ to avoid symbol collisions.
// The world state is stored in a heap-allocated PhysicsWorld struct, passed
// as an opaque *mut c_void handle. Body/collider/joint handles are Rapier's
// internal indices encoded as u64.
//
// 3D differences from 2D:
//   - Positions are [x, y, z] (3 floats)
//   - Rotations are quaternions [qx, qy, qz, qw] (4 floats)
//   - Angular velocity is a vector [wx, wy, wz] (3 floats)
//   - Additional shapes: Cylinder, Cone
//   - Motor joints have 6 DOF: LinX, LinY, LinZ, AngX, AngY, AngZ

use std::collections::HashMap;
use std::ffi::c_void;
use std::slice;
use std::sync::mpsc;

use rapier3d::prelude::*;
use rapier3d::parry::query::{DefaultQueryDispatcher, ShapeCastOptions};

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
    contact_start_buf: Vec<(u64, u64)>,
    contact_stop_buf: Vec<(u64, u64)>,
    // Contact force event buffer: (collider1, collider2, total_force_magnitude)
    contact_force_buf: Vec<(u64, u64, f32)>,
    // Per-collider one-way platform config: collider_handle_u64 -> (allowed_local_n1, allowed_angle)
    one_way_platforms: HashMap<u64, ([f32; 3], f32)>,
}

impl PhysicsWorld {
    fn new(gx: f32, gy: f32, gz: f32) -> Self {
        PhysicsWorld {
            gravity: Vector::new(gx, gy, gz),
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
        let hooks = SgePhysicsHooks3D { one_way_platforms: &owp };

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
                    self.contact_start_buf.push((collider_handle_to_u64(c1), collider_handle_to_u64(c2)));
                }
                CollisionEvent::Stopped(c1, c2, _flags) => {
                    self.contact_stop_buf.push((collider_handle_to_u64(c1), collider_handle_to_u64(c2)));
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
// Physics hooks — one-way platform support (3D)
// ---------------------------------------------------------------------------

struct SgePhysicsHooks3D<'a> {
    one_way_platforms: &'a HashMap<u64, ([f32; 3], f32)>,
}

unsafe impl Send for SgePhysicsHooks3D<'_> {}
unsafe impl Sync for SgePhysicsHooks3D<'_> {}

impl PhysicsHooks for SgePhysicsHooks3D<'_> {
    fn modify_solver_contacts(&self, context: &mut ContactModificationContext) {
        let c1 = collider_handle_to_u64(context.collider1);
        let c2 = collider_handle_to_u64(context.collider2);

        if let Some(&(dir, angle)) = self.one_way_platforms.get(&c1) {
            let allowed_local_n1 = Vector::new(dir[0], dir[1], dir[2]);
            context.update_as_oneway_platform(allowed_local_n1, angle);
        }
        if let Some(&(dir, angle)) = self.one_way_platforms.get(&c2) {
            // For collider2, negate the direction since Rapier's
            // update_as_oneway_platform checks manifold.local_n1 which
            // is relative to collider1.
            let allowed_local_n1 = Vector::new(-dir[0], -dir[1], -dir[2]);
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
    RigidBodyHandle::from_raw_parts(v as u32, (v >> 32) as u32)
}
fn collider_handle_to_u64(h: ColliderHandle) -> u64 {
    let (index, generation) = h.into_raw_parts();
    ((generation as u64) << 32) | (index as u64)
}
fn u64_to_collider_handle(v: u64) -> ColliderHandle {
    ColliderHandle::from_raw_parts(v as u32, (v >> 32) as u32)
}
fn joint_handle_to_u64(h: ImpulseJointHandle) -> u64 {
    let (index, generation) = h.into_raw_parts();
    ((generation as u64) << 32) | (index as u64)
}
fn u64_to_joint_handle(v: u64) -> ImpulseJointHandle {
    ImpulseJointHandle::from_raw_parts(v as u32, (v >> 32) as u32)
}

// ---------------------------------------------------------------------------
// World lifecycle
// ---------------------------------------------------------------------------

#[no_mangle]
pub extern "C" fn sge_phys3d_create_world(gx: f32, gy: f32, gz: f32) -> *mut c_void {
    Box::into_raw(Box::new(PhysicsWorld::new(gx, gy, gz))) as *mut c_void
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_destroy_world(world: *mut c_void) {
    if !world.is_null() { drop(Box::from_raw(world as *mut PhysicsWorld)); }
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_world_step(world: *mut c_void, dt: f32) {
    (&mut *(world as *mut PhysicsWorld)).step(dt);
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_world_set_gravity(world: *mut c_void, gx: f32, gy: f32, gz: f32) {
    (&mut *(world as *mut PhysicsWorld)).gravity = Vector::new(gx, gy, gz);
}

/// Fills `out` with [gx, gy, gz].
#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_world_get_gravity(world: *mut c_void, out: *mut f32) {
    let w = &*(world as *mut PhysicsWorld);
    let arr = slice::from_raw_parts_mut(out, 3);
    arr[0] = w.gravity.x; arr[1] = w.gravity.y; arr[2] = w.gravity.z;
}

// ---------------------------------------------------------------------------
// Rigid body lifecycle
// ---------------------------------------------------------------------------

unsafe fn create_body(world: *mut c_void, body_type: RigidBodyType,
    x: f32, y: f32, z: f32, qx: f32, qy: f32, qz: f32, qw: f32) -> u64
{
    let w = &mut *(world as *mut PhysicsWorld);
    let quat = Rotation::from_xyzw(qx, qy, qz, qw);
    let (axis, angle) = quat.to_axis_angle();
    let body = RigidBodyBuilder::new(body_type)
        .translation(Vector::new(x, y, z))
        .rotation(axis * angle)
        .build();
    body_handle_to_u64(w.rigid_body_set.insert(body))
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_create_dynamic_body(
    world: *mut c_void, x: f32, y: f32, z: f32, qx: f32, qy: f32, qz: f32, qw: f32
) -> u64 { create_body(world, RigidBodyType::Dynamic, x, y, z, qx, qy, qz, qw) }

#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_create_static_body(
    world: *mut c_void, x: f32, y: f32, z: f32, qx: f32, qy: f32, qz: f32, qw: f32
) -> u64 { create_body(world, RigidBodyType::Fixed, x, y, z, qx, qy, qz, qw) }

#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_create_kinematic_body(
    world: *mut c_void, x: f32, y: f32, z: f32, qx: f32, qy: f32, qz: f32, qw: f32
) -> u64 { create_body(world, RigidBodyType::KinematicPositionBased, x, y, z, qx, qy, qz, qw) }

#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_destroy_body(world: *mut c_void, body: u64) {
    let w = &mut *(world as *mut PhysicsWorld);
    w.rigid_body_set.remove(u64_to_body_handle(body),
        &mut w.island_manager, &mut w.collider_set,
        &mut w.impulse_joint_set, &mut w.multibody_joint_set, true);
}

// ---------------------------------------------------------------------------
// Body accessors
// ---------------------------------------------------------------------------

/// Fills `out` with [x, y, z].
#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_body_get_position(world: *mut c_void, body: u64, out: *mut f32) {
    let w = &*(world as *mut PhysicsWorld);
    if let Some(b) = w.rigid_body_set.get(u64_to_body_handle(body)) {
        let pos = b.translation();
        let arr = slice::from_raw_parts_mut(out, 3);
        arr[0] = pos.x; arr[1] = pos.y; arr[2] = pos.z;
    }
}

/// Fills `out` with [qx, qy, qz, qw].
#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_body_get_rotation(world: *mut c_void, body: u64, out: *mut f32) {
    let w = &*(world as *mut PhysicsWorld);
    if let Some(b) = w.rigid_body_set.get(u64_to_body_handle(body)) {
        let q = b.rotation();
        let arr = slice::from_raw_parts_mut(out, 4);
        arr[0] = q.x; arr[1] = q.y; arr[2] = q.z; arr[3] = q.w;
    }
}

/// Fills `out` with [vx, vy, vz].
#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_body_get_linear_velocity(world: *mut c_void, body: u64, out: *mut f32) {
    let w = &*(world as *mut PhysicsWorld);
    if let Some(b) = w.rigid_body_set.get(u64_to_body_handle(body)) {
        let vel = b.linvel();
        let arr = slice::from_raw_parts_mut(out, 3);
        arr[0] = vel.x; arr[1] = vel.y; arr[2] = vel.z;
    }
}

/// Fills `out` with [wx, wy, wz].
#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_body_get_angular_velocity(world: *mut c_void, body: u64, out: *mut f32) {
    let w = &*(world as *mut PhysicsWorld);
    if let Some(b) = w.rigid_body_set.get(u64_to_body_handle(body)) {
        let ang = b.angvel();
        let arr = slice::from_raw_parts_mut(out, 3);
        arr[0] = ang.x; arr[1] = ang.y; arr[2] = ang.z;
    }
}

// ---------------------------------------------------------------------------
// Body setters
// ---------------------------------------------------------------------------

#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_body_set_position(world: *mut c_void, body: u64, x: f32, y: f32, z: f32) {
    let w = &mut *(world as *mut PhysicsWorld);
    if let Some(b) = w.rigid_body_set.get_mut(u64_to_body_handle(body)) {
        b.set_translation(Vector::new(x, y, z), true);
    }
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_body_set_rotation(world: *mut c_void, body: u64, qx: f32, qy: f32, qz: f32, qw: f32) {
    let w = &mut *(world as *mut PhysicsWorld);
    if let Some(b) = w.rigid_body_set.get_mut(u64_to_body_handle(body)) {
        b.set_rotation(Rotation::from_xyzw(qx, qy, qz, qw), true);
    }
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_body_set_linear_velocity(world: *mut c_void, body: u64, vx: f32, vy: f32, vz: f32) {
    let w = &mut *(world as *mut PhysicsWorld);
    if let Some(b) = w.rigid_body_set.get_mut(u64_to_body_handle(body)) {
        b.set_linvel(Vector::new(vx, vy, vz), true);
    }
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_body_set_angular_velocity(world: *mut c_void, body: u64, wx: f32, wy: f32, wz: f32) {
    let w = &mut *(world as *mut PhysicsWorld);
    if let Some(b) = w.rigid_body_set.get_mut(u64_to_body_handle(body)) {
        b.set_angvel(Vector::new(wx, wy, wz), true);
    }
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_body_apply_force(world: *mut c_void, body: u64, fx: f32, fy: f32, fz: f32) {
    let w = &mut *(world as *mut PhysicsWorld);
    if let Some(b) = w.rigid_body_set.get_mut(u64_to_body_handle(body)) { b.add_force(Vector::new(fx, fy, fz), true); }
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_body_apply_impulse(world: *mut c_void, body: u64, ix: f32, iy: f32, iz: f32) {
    let w = &mut *(world as *mut PhysicsWorld);
    if let Some(b) = w.rigid_body_set.get_mut(u64_to_body_handle(body)) { b.apply_impulse(Vector::new(ix, iy, iz), true); }
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_body_apply_torque(world: *mut c_void, body: u64, tx: f32, ty: f32, tz: f32) {
    let w = &mut *(world as *mut PhysicsWorld);
    if let Some(b) = w.rigid_body_set.get_mut(u64_to_body_handle(body)) { b.add_torque(Vector::new(tx, ty, tz), true); }
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_body_apply_force_at_point(world: *mut c_void, body: u64, fx: f32, fy: f32, fz: f32, px: f32, py: f32, pz: f32) {
    let w = &mut *(world as *mut PhysicsWorld);
    if let Some(b) = w.rigid_body_set.get_mut(u64_to_body_handle(body)) { b.add_force_at_point(Vector::new(fx, fy, fz), Vector::new(px, py, pz).into(), true); }
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_body_apply_impulse_at_point(world: *mut c_void, body: u64, ix: f32, iy: f32, iz: f32, px: f32, py: f32, pz: f32) {
    let w = &mut *(world as *mut PhysicsWorld);
    if let Some(b) = w.rigid_body_set.get_mut(u64_to_body_handle(body)) { b.apply_impulse_at_point(Vector::new(ix, iy, iz), Vector::new(px, py, pz).into(), true); }
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_body_set_linear_damping(world: *mut c_void, body: u64, d: f32) {
    let w = &mut *(world as *mut PhysicsWorld);
    if let Some(b) = w.rigid_body_set.get_mut(u64_to_body_handle(body)) { b.set_linear_damping(d); }
}
#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_body_get_linear_damping(world: *mut c_void, body: u64) -> f32 {
    let w = &*(world as *mut PhysicsWorld);
    w.rigid_body_set.get(u64_to_body_handle(body)).map(|b| b.linear_damping()).unwrap_or(0.0)
}
#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_body_set_angular_damping(world: *mut c_void, body: u64, d: f32) {
    let w = &mut *(world as *mut PhysicsWorld);
    if let Some(b) = w.rigid_body_set.get_mut(u64_to_body_handle(body)) { b.set_angular_damping(d); }
}
#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_body_get_angular_damping(world: *mut c_void, body: u64) -> f32 {
    let w = &*(world as *mut PhysicsWorld);
    w.rigid_body_set.get(u64_to_body_handle(body)).map(|b| b.angular_damping()).unwrap_or(0.0)
}
#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_body_set_gravity_scale(world: *mut c_void, body: u64, s: f32) {
    let w = &mut *(world as *mut PhysicsWorld);
    if let Some(b) = w.rigid_body_set.get_mut(u64_to_body_handle(body)) { b.set_gravity_scale(s, true); }
}
#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_body_get_gravity_scale(world: *mut c_void, body: u64) -> f32 {
    let w = &*(world as *mut PhysicsWorld);
    w.rigid_body_set.get(u64_to_body_handle(body)).map(|b| b.gravity_scale()).unwrap_or(1.0)
}
#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_body_is_awake(world: *mut c_void, body: u64) -> i32 {
    let w = &*(world as *mut PhysicsWorld);
    w.rigid_body_set.get(u64_to_body_handle(body)).map(|b| !b.is_sleeping() as i32).unwrap_or(0)
}
#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_body_wake_up(world: *mut c_void, body: u64) {
    let w = &mut *(world as *mut PhysicsWorld);
    if let Some(b) = w.rigid_body_set.get_mut(u64_to_body_handle(body)) { b.wake_up(true); }
}
#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_body_sleep(world: *mut c_void, body: u64) {
    let w = &mut *(world as *mut PhysicsWorld);
    if let Some(b) = w.rigid_body_set.get_mut(u64_to_body_handle(body)) { b.sleep(); }
}
#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_body_set_fixed_rotation(world: *mut c_void, body: u64, fixed: i32) {
    let w = &mut *(world as *mut PhysicsWorld);
    if let Some(b) = w.rigid_body_set.get_mut(u64_to_body_handle(body)) { b.lock_rotations(fixed != 0, true); }
}
#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_body_enable_ccd(world: *mut c_void, body: u64, enable: i32) {
    let w = &mut *(world as *mut PhysicsWorld);
    if let Some(b) = w.rigid_body_set.get_mut(u64_to_body_handle(body)) { b.enable_ccd(enable != 0); }
}
#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_body_is_ccd_enabled(world: *mut c_void, body: u64) -> i32 {
    let w = &*(world as *mut PhysicsWorld);
    w.rigid_body_set.get(u64_to_body_handle(body)).map(|b| b.is_ccd_enabled() as i32).unwrap_or(0)
}
#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_body_set_enabled(world: *mut c_void, body: u64, e: i32) {
    let w = &mut *(world as *mut PhysicsWorld);
    if let Some(b) = w.rigid_body_set.get_mut(u64_to_body_handle(body)) { b.set_enabled(e != 0); }
}
#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_body_is_enabled(world: *mut c_void, body: u64) -> i32 {
    let w = &*(world as *mut PhysicsWorld);
    w.rigid_body_set.get(u64_to_body_handle(body)).map(|b| b.is_enabled() as i32).unwrap_or(0)
}
#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_body_set_dominance_group(world: *mut c_void, body: u64, g: i32) {
    let w = &mut *(world as *mut PhysicsWorld);
    if let Some(b) = w.rigid_body_set.get_mut(u64_to_body_handle(body)) { b.set_dominance_group(g as i8); }
}
#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_body_get_dominance_group(world: *mut c_void, body: u64) -> i32 {
    let w = &*(world as *mut PhysicsWorld);
    w.rigid_body_set.get(u64_to_body_handle(body)).map(|b| b.dominance_group() as i32).unwrap_or(0)
}
#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_body_get_mass(world: *mut c_void, body: u64) -> f32 {
    let w = &*(world as *mut PhysicsWorld);
    w.rigid_body_set.get(u64_to_body_handle(body)).map(|b| b.mass()).unwrap_or(0.0)
}
#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_body_recompute_mass_properties(world: *mut c_void, body: u64) {
    let w = &mut *(world as *mut PhysicsWorld);
    if let Some(b) = w.rigid_body_set.get_mut(u64_to_body_handle(body)) {
        b.recompute_mass_properties_from_colliders(&w.collider_set);
    }
}

// ---------------------------------------------------------------------------
// Collider creation (3D shapes)
// ---------------------------------------------------------------------------

#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_create_sphere_collider(world: *mut c_void, body: u64, radius: f32) -> u64 {
    let w = &mut *(world as *mut PhysicsWorld);
    let c = ColliderBuilder::ball(radius).build();
    collider_handle_to_u64(w.collider_set.insert_with_parent(c, u64_to_body_handle(body), &mut w.rigid_body_set))
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_create_box_collider(world: *mut c_void, body: u64, hx: f32, hy: f32, hz: f32) -> u64 {
    let w = &mut *(world as *mut PhysicsWorld);
    let c = ColliderBuilder::cuboid(hx, hy, hz).build();
    collider_handle_to_u64(w.collider_set.insert_with_parent(c, u64_to_body_handle(body), &mut w.rigid_body_set))
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_create_capsule_collider(world: *mut c_void, body: u64, half_height: f32, radius: f32) -> u64 {
    let w = &mut *(world as *mut PhysicsWorld);
    let c = ColliderBuilder::capsule_y(half_height, radius).build();
    collider_handle_to_u64(w.collider_set.insert_with_parent(c, u64_to_body_handle(body), &mut w.rigid_body_set))
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_create_cylinder_collider(world: *mut c_void, body: u64, half_height: f32, radius: f32) -> u64 {
    let w = &mut *(world as *mut PhysicsWorld);
    let c = ColliderBuilder::cylinder(half_height, radius).build();
    collider_handle_to_u64(w.collider_set.insert_with_parent(c, u64_to_body_handle(body), &mut w.rigid_body_set))
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_create_cone_collider(world: *mut c_void, body: u64, half_height: f32, radius: f32) -> u64 {
    let w = &mut *(world as *mut PhysicsWorld);
    let c = ColliderBuilder::cone(half_height, radius).build();
    collider_handle_to_u64(w.collider_set.insert_with_parent(c, u64_to_body_handle(body), &mut w.rigid_body_set))
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_create_convex_hull_collider(world: *mut c_void, body: u64, vertices: *const f32, vertex_count: i32) -> u64 {
    let w = &mut *(world as *mut PhysicsWorld);
    let verts = slice::from_raw_parts(vertices, (vertex_count * 3) as usize);
    let points: Vec<Vector> = (0..vertex_count as usize)
        .map(|i| Vector::new(verts[i*3], verts[i*3+1], verts[i*3+2]))
        .collect();
    let c = ColliderBuilder::convex_hull(&points.iter().map(|v| (*v).into()).collect::<Vec<_>>())
        .unwrap_or_else(|| ColliderBuilder::ball(0.1))
        .build();
    collider_handle_to_u64(w.collider_set.insert_with_parent(c, u64_to_body_handle(body), &mut w.rigid_body_set))
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_create_trimesh_collider(
    world: *mut c_void, body: u64,
    vertices: *const f32, vertex_count: i32,
    indices: *const u32, index_count: i32,
) -> u64 {
    let w = &mut *(world as *mut PhysicsWorld);
    let verts = slice::from_raw_parts(vertices, (vertex_count * 3) as usize);
    let idxs  = slice::from_raw_parts(indices, index_count as usize);
    let points: Vec<_> = (0..vertex_count as usize)
        .map(|i| Vector::new(verts[i*3], verts[i*3+1], verts[i*3+2]).into())
        .collect();
    let tris: Vec<[u32; 3]> = idxs.chunks_exact(3).map(|c| [c[0], c[1], c[2]]).collect();
    let c = ColliderBuilder::trimesh(points, tris).unwrap().build();
    collider_handle_to_u64(w.collider_set.insert_with_parent(c, u64_to_body_handle(body), &mut w.rigid_body_set))
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_destroy_collider(world: *mut c_void, collider: u64) {
    let w = &mut *(world as *mut PhysicsWorld);
    w.collider_set.remove(u64_to_collider_handle(collider), &mut w.island_manager, &mut w.rigid_body_set, true);
}

// ---------------------------------------------------------------------------
// Collider properties
// ---------------------------------------------------------------------------

#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_collider_set_density(world: *mut c_void, c: u64, v: f32) {
    let w = &mut *(world as *mut PhysicsWorld);
    if let Some(co) = w.collider_set.get_mut(u64_to_collider_handle(c)) { co.set_density(v); }
}
#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_collider_set_friction(world: *mut c_void, c: u64, v: f32) {
    let w = &mut *(world as *mut PhysicsWorld);
    if let Some(co) = w.collider_set.get_mut(u64_to_collider_handle(c)) { co.set_friction(v); }
}
#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_collider_set_restitution(world: *mut c_void, c: u64, v: f32) {
    let w = &mut *(world as *mut PhysicsWorld);
    if let Some(co) = w.collider_set.get_mut(u64_to_collider_handle(c)) { co.set_restitution(v); }
}
#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_collider_set_sensor(world: *mut c_void, c: u64, v: i32) {
    let w = &mut *(world as *mut PhysicsWorld);
    if let Some(co) = w.collider_set.get_mut(u64_to_collider_handle(c)) { co.set_sensor(v != 0); }
}

// ---------------------------------------------------------------------------
// Joints
// ---------------------------------------------------------------------------

#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_create_fixed_joint(world: *mut c_void, body1: u64, body2: u64) -> u64 {
    let w = &mut *(world as *mut PhysicsWorld);
    let joint = FixedJointBuilder::new().build();
    joint_handle_to_u64(w.impulse_joint_set.insert(u64_to_body_handle(body1), u64_to_body_handle(body2), joint, true))
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_create_rope_joint(world: *mut c_void, body1: u64, body2: u64, max_dist: f32) -> u64 {
    let w = &mut *(world as *mut PhysicsWorld);
    let joint = RopeJointBuilder::new(max_dist).build();
    joint_handle_to_u64(w.impulse_joint_set.insert(u64_to_body_handle(body1), u64_to_body_handle(body2), joint, true))
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_destroy_joint(world: *mut c_void, joint: u64) {
    let w = &mut *(world as *mut PhysicsWorld);
    w.impulse_joint_set.remove(u64_to_joint_handle(joint), true);
}

// ---------------------------------------------------------------------------
// Queries
// ---------------------------------------------------------------------------

/// Ray cast (closest hit). Fills `out` with [hitX, hitY, hitZ, normalX, normalY, normalZ, toi, colliderLo, colliderHi] = 9 floats.
#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_ray_cast(
    world: *mut c_void, ox: f32, oy: f32, oz: f32, dx: f32, dy: f32, dz: f32, max_dist: f32, out: *mut f32,
) -> i32 {
    let w = &*(world as *mut PhysicsWorld);
    let ray = Ray::new(Vector::new(ox, oy, oz).into(), Vector::new(dx, dy, dz));
    let qp = w.broad_phase.as_query_pipeline(&DefaultQueryDispatcher, &w.rigid_body_set, &w.collider_set, QueryFilter::default());
    if let Some((handle, intersection)) = qp.cast_ray_and_get_normal(&ray, max_dist, true) {
        let hit = ray.point_at(intersection.time_of_impact);
        let arr = slice::from_raw_parts_mut(out, 9);
        arr[0] = hit.x; arr[1] = hit.y; arr[2] = hit.z;
        arr[3] = intersection.normal.x; arr[4] = intersection.normal.y; arr[5] = intersection.normal.z;
        arr[6] = intersection.time_of_impact;
        let ch = collider_handle_to_u64(handle);
        arr[7] = f32::from_bits(ch as u32);
        arr[8] = f32::from_bits((ch >> 32) as u32);
        1
    } else { 0 }
}

// ---------------------------------------------------------------------------
// Contact events
// ---------------------------------------------------------------------------

#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_poll_contact_start_events(world: *mut c_void, out1: *mut u64, out2: *mut u64, max: i32) -> i32 {
    let w = &*(world as *mut PhysicsWorld);
    let c1 = slice::from_raw_parts_mut(out1, max as usize);
    let c2 = slice::from_raw_parts_mut(out2, max as usize);
    let count = w.contact_start_buf.len().min(max as usize);
    for i in 0..count { c1[i] = w.contact_start_buf[i].0; c2[i] = w.contact_start_buf[i].1; }
    count as i32
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_poll_contact_stop_events(world: *mut c_void, out1: *mut u64, out2: *mut u64, max: i32) -> i32 {
    let w = &*(world as *mut PhysicsWorld);
    let c1 = slice::from_raw_parts_mut(out1, max as usize);
    let c2 = slice::from_raw_parts_mut(out2, max as usize);
    let count = w.contact_stop_buf.len().min(max as usize);
    for i in 0..count { c1[i] = w.contact_stop_buf[i].0; c2[i] = w.contact_stop_buf[i].1; }
    count as i32
}

// ---------------------------------------------------------------------------
// Body extras
// ---------------------------------------------------------------------------

#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_body_apply_torque_impulse(world: *mut c_void, body: u64, tx: f32, ty: f32, tz: f32) {
    let w = &mut *(world as *mut PhysicsWorld);
    if let Some(b) = w.rigid_body_set.get_mut(u64_to_body_handle(body)) {
        b.apply_torque_impulse(Vector::new(tx, ty, tz), true);
    }
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_body_reset_forces(world: *mut c_void, body: u64) {
    let w = &mut *(world as *mut PhysicsWorld);
    if let Some(b) = w.rigid_body_set.get_mut(u64_to_body_handle(body)) {
        b.reset_forces(true);
    }
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_body_reset_torques(world: *mut c_void, body: u64) {
    let w = &mut *(world as *mut PhysicsWorld);
    if let Some(b) = w.rigid_body_set.get_mut(u64_to_body_handle(body)) {
        b.reset_torques(true);
    }
}

/// Returns body type: 0 = dynamic, 1 = fixed (static), 2 = kinematic position-based, 3 = kinematic velocity-based
#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_body_get_type(world: *mut c_void, body: u64) -> i32 {
    let w = &*(world as *mut PhysicsWorld);
    w.rigid_body_set.get(u64_to_body_handle(body)).map(|b| match b.body_type() {
        RigidBodyType::Dynamic                  => 0,
        RigidBodyType::Fixed                    => 1,
        RigidBodyType::KinematicPositionBased   => 2,
        RigidBodyType::KinematicVelocityBased   => 3,
    }).unwrap_or(-1)
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_body_set_enabled_translations(
    world: *mut c_void, body: u64, x: i32, y: i32, z: i32,
) {
    let w = &mut *(world as *mut PhysicsWorld);
    if let Some(b) = w.rigid_body_set.get_mut(u64_to_body_handle(body)) {
        b.set_enabled_translations(x != 0, y != 0, z != 0, true);
    }
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_body_is_translation_locked_x(world: *mut c_void, body: u64) -> i32 {
    let w = &*(world as *mut PhysicsWorld);
    w.rigid_body_set.get(u64_to_body_handle(body))
        .map(|b| b.locked_axes().contains(LockedAxes::TRANSLATION_LOCKED_X) as i32)
        .unwrap_or(0)
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_body_is_translation_locked_y(world: *mut c_void, body: u64) -> i32 {
    let w = &*(world as *mut PhysicsWorld);
    w.rigid_body_set.get(u64_to_body_handle(body))
        .map(|b| b.locked_axes().contains(LockedAxes::TRANSLATION_LOCKED_Y) as i32)
        .unwrap_or(0)
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_body_is_translation_locked_z(world: *mut c_void, body: u64) -> i32 {
    let w = &*(world as *mut PhysicsWorld);
    w.rigid_body_set.get(u64_to_body_handle(body))
        .map(|b| b.locked_axes().contains(LockedAxes::TRANSLATION_LOCKED_Z) as i32)
        .unwrap_or(0)
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_body_set_enabled_rotations(
    world: *mut c_void, body: u64, x: i32, y: i32, z: i32,
) {
    let w = &mut *(world as *mut PhysicsWorld);
    if let Some(b) = w.rigid_body_set.get_mut(u64_to_body_handle(body)) {
        b.set_enabled_rotations(x != 0, y != 0, z != 0, true);
    }
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_body_is_rotation_locked_x(world: *mut c_void, body: u64) -> i32 {
    let w = &*(world as *mut PhysicsWorld);
    w.rigid_body_set.get(u64_to_body_handle(body))
        .map(|b| b.locked_axes().contains(LockedAxes::ROTATION_LOCKED_X) as i32)
        .unwrap_or(0)
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_body_is_rotation_locked_y(world: *mut c_void, body: u64) -> i32 {
    let w = &*(world as *mut PhysicsWorld);
    w.rigid_body_set.get(u64_to_body_handle(body))
        .map(|b| b.locked_axes().contains(LockedAxes::ROTATION_LOCKED_Y) as i32)
        .unwrap_or(0)
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_body_is_rotation_locked_z(world: *mut c_void, body: u64) -> i32 {
    let w = &*(world as *mut PhysicsWorld);
    w.rigid_body_set.get(u64_to_body_handle(body))
        .map(|b| b.locked_axes().contains(LockedAxes::ROTATION_LOCKED_Z) as i32)
        .unwrap_or(0)
}

/// Gets world-space center of mass. Fills `out` with [x, y, z].
#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_body_get_world_center_of_mass(
    world: *mut c_void, body: u64, out: *mut f32,
) {
    let w = &*(world as *mut PhysicsWorld);
    let arr = slice::from_raw_parts_mut(out, 3);
    if let Some(b) = w.rigid_body_set.get(u64_to_body_handle(body)) {
        let com = b.center_of_mass();
        arr[0] = com.x; arr[1] = com.y; arr[2] = com.z;
    } else {
        arr[0] = 0.0; arr[1] = 0.0; arr[2] = 0.0;
    }
}

/// Gets the velocity of a point on the body in world space. Fills `out` with [vx, vy, vz].
#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_body_get_velocity_at_point(
    world: *mut c_void, body: u64, px: f32, py: f32, pz: f32, out: *mut f32,
) {
    let w = &*(world as *mut PhysicsWorld);
    let arr = slice::from_raw_parts_mut(out, 3);
    if let Some(b) = w.rigid_body_set.get(u64_to_body_handle(body)) {
        let vel = b.velocity_at_point(Vector::new(px, py, pz));
        arr[0] = vel.x; arr[1] = vel.y; arr[2] = vel.z;
    } else {
        arr[0] = 0.0; arr[1] = 0.0; arr[2] = 0.0;
    }
}

/// Gets the angular inertia of a rigid body (trace of the inertia tensor).
#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_body_get_inertia(world: *mut c_void, body: u64) -> f32 {
    let w = &*(world as *mut PhysicsWorld);
    w.rigid_body_set.get(u64_to_body_handle(body))
        .map(|b| {
            let pi = b.mass_properties().local_mprops.principal_inertia();
            pi.x + pi.y + pi.z
        })
        .unwrap_or(0.0)
}

/// Gets the local center of mass. Fills `out` with [x, y, z].
#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_body_get_local_center_of_mass(
    world: *mut c_void, body: u64, out: *mut f32,
) {
    let w = &*(world as *mut PhysicsWorld);
    let arr = slice::from_raw_parts_mut(out, 3);
    if let Some(b) = w.rigid_body_set.get(u64_to_body_handle(body)) {
        let com = b.mass_properties().local_mprops.local_com;
        arr[0] = com.x; arr[1] = com.y; arr[2] = com.z;
    } else {
        arr[0] = 0.0; arr[1] = 0.0; arr[2] = 0.0;
    }
}

// ---------------------------------------------------------------------------
// Collider getters/properties
// ---------------------------------------------------------------------------

#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_collider_get_density(world: *mut c_void, c: u64) -> f32 {
    let w = &*(world as *mut PhysicsWorld);
    w.collider_set.get(u64_to_collider_handle(c)).map(|co| co.density()).unwrap_or(0.0)
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_collider_get_friction(world: *mut c_void, c: u64) -> f32 {
    let w = &*(world as *mut PhysicsWorld);
    w.collider_set.get(u64_to_collider_handle(c)).map(|co| co.friction()).unwrap_or(0.0)
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_collider_get_restitution(world: *mut c_void, c: u64) -> f32 {
    let w = &*(world as *mut PhysicsWorld);
    w.collider_set.get(u64_to_collider_handle(c)).map(|co| co.restitution()).unwrap_or(0.0)
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_collider_is_sensor(world: *mut c_void, c: u64) -> i32 {
    let w = &*(world as *mut PhysicsWorld);
    w.collider_set.get(u64_to_collider_handle(c)).map(|co| co.is_sensor() as i32).unwrap_or(0)
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_collider_set_enabled(world: *mut c_void, c: u64, enabled: i32) {
    let w = &mut *(world as *mut PhysicsWorld);
    if let Some(co) = w.collider_set.get_mut(u64_to_collider_handle(c)) {
        co.set_enabled(enabled != 0);
    }
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_collider_is_enabled(world: *mut c_void, c: u64) -> i32 {
    let w = &*(world as *mut PhysicsWorld);
    w.collider_set.get(u64_to_collider_handle(c)).map(|co| co.is_enabled() as i32).unwrap_or(0)
}

/// Gets collider position relative to parent body. Fills `out` with [x,y,z,qx,qy,qz,qw].
#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_collider_get_position_wrt_parent(
    world: *mut c_void, c: u64, out: *mut f32,
) {
    let w = &*(world as *mut PhysicsWorld);
    let arr = slice::from_raw_parts_mut(out, 7);
    if let Some(co) = w.collider_set.get(u64_to_collider_handle(c)) {
        if let Some(rel) = co.position_wrt_parent() {
            arr[0] = rel.translation.x;
            arr[1] = rel.translation.y;
            arr[2] = rel.translation.z;
            let q = rel.rotation;
            arr[3] = q.x; arr[4] = q.y; arr[5] = q.z; arr[6] = q.w;
        } else {
            for i in 0..7 { arr[i] = 0.0; }
            arr[6] = 1.0; // identity quaternion w=1
        }
    } else {
        for i in 0..7 { arr[i] = 0.0; }
        arr[6] = 1.0;
    }
}

/// Sets collider position relative to parent body.
#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_collider_set_position_wrt_parent(
    world: *mut c_void, c: u64, x: f32, y: f32, z: f32, qx: f32, qy: f32, qz: f32, qw: f32,
) {
    let w = &mut *(world as *mut PhysicsWorld);
    if let Some(co) = w.collider_set.get_mut(u64_to_collider_handle(c)) {
        let pose = Pose::from_parts(
            Vector::new(x, y, z),
            Rotation::from_xyzw(qx, qy, qz, qw),
        );
        co.set_position_wrt_parent(pose);
    }
}

/// Gets collider world position. Fills `out` with [x,y,z,qx,qy,qz,qw].
#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_collider_get_position(
    world: *mut c_void, c: u64, out: *mut f32,
) {
    let w = &*(world as *mut PhysicsWorld);
    let arr = slice::from_raw_parts_mut(out, 7);
    if let Some(co) = w.collider_set.get(u64_to_collider_handle(c)) {
        let pos = co.position();
        arr[0] = pos.translation.x;
        arr[1] = pos.translation.y;
        arr[2] = pos.translation.z;
        let q = pos.rotation;
        arr[3] = q.x; arr[4] = q.y; arr[5] = q.z; arr[6] = q.w;
    } else {
        for i in 0..7 { arr[i] = 0.0; }
        arr[6] = 1.0;
    }
}

/// Returns collider shape type: 0=ball, 1=cuboid, 2=capsule, 3=cylinder, 4=cone,
/// 5=convex_polyhedron, 6=trimesh, 7=heightfield, 99=unknown
#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_collider_get_shape_type(world: *mut c_void, c: u64) -> i32 {
    let w = &*(world as *mut PhysicsWorld);
    w.collider_set.get(u64_to_collider_handle(c)).map(|co| {
        let shape = co.shape();
        if shape.as_ball().is_some()              { 0 }
        else if shape.as_cuboid().is_some()       { 1 }
        else if shape.as_capsule().is_some()      { 2 }
        else if shape.as_cylinder().is_some()     { 3 }
        else if shape.as_cone().is_some()         { 4 }
        else if shape.as_convex_polyhedron().is_some() { 5 }
        else if shape.as_trimesh().is_some()      { 6 }
        else if shape.as_heightfield().is_some()  { 7 }
        else { 99 }
    }).unwrap_or(-1)
}

/// Gets collider AABB. Fills `out` with [minX, minY, minZ, maxX, maxY, maxZ].
#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_collider_get_aabb(
    world: *mut c_void, c: u64, out: *mut f32,
) {
    let w = &*(world as *mut PhysicsWorld);
    let arr = slice::from_raw_parts_mut(out, 6);
    if let Some(co) = w.collider_set.get(u64_to_collider_handle(c)) {
        let aabb = co.compute_aabb();
        arr[0] = aabb.mins.x; arr[1] = aabb.mins.y; arr[2] = aabb.mins.z;
        arr[3] = aabb.maxs.x; arr[4] = aabb.maxs.y; arr[5] = aabb.maxs.z;
    } else {
        for i in 0..6 { arr[i] = 0.0; }
    }
}

/// Gets the parent body handle of a collider. Returns 0 if no parent.
#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_collider_get_parent_body(world: *mut c_void, c: u64) -> u64 {
    let w = &*(world as *mut PhysicsWorld);
    w.collider_set.get(u64_to_collider_handle(c))
        .and_then(|co| co.parent())
        .map(body_handle_to_u64)
        .unwrap_or(0)
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_collider_get_mass(world: *mut c_void, c: u64) -> f32 {
    let w = &*(world as *mut PhysicsWorld);
    w.collider_set.get(u64_to_collider_handle(c)).map(|co| co.mass()).unwrap_or(0.0)
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_collider_set_mass(world: *mut c_void, c: u64, mass: f32) {
    let w = &mut *(world as *mut PhysicsWorld);
    if let Some(co) = w.collider_set.get_mut(u64_to_collider_handle(c)) {
        co.set_mass(mass);
    }
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_collider_set_contact_skin(world: *mut c_void, c: u64, skin: f32) {
    let w = &mut *(world as *mut PhysicsWorld);
    if let Some(co) = w.collider_set.get_mut(u64_to_collider_handle(c)) {
        co.set_contact_skin(skin);
    }
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_collider_set_active_events(world: *mut c_void, c: u64, flags: i32) {
    let w = &mut *(world as *mut PhysicsWorld);
    if let Some(co) = w.collider_set.get_mut(u64_to_collider_handle(c)) {
        co.set_active_events(ActiveEvents::from_bits_truncate(flags as u32));
    }
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_collider_get_active_events(world: *mut c_void, c: u64) -> i32 {
    let w = &*(world as *mut PhysicsWorld);
    w.collider_set.get(u64_to_collider_handle(c))
        .map(|co| co.active_events().bits() as i32)
        .unwrap_or(0)
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_collider_set_active_collision_types(world: *mut c_void, c: u64, flags: i32) {
    let w = &mut *(world as *mut PhysicsWorld);
    if let Some(co) = w.collider_set.get_mut(u64_to_collider_handle(c)) {
        co.set_active_collision_types(ActiveCollisionTypes::from_bits_truncate(flags as u16));
    }
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_collider_get_active_collision_types(world: *mut c_void, c: u64) -> i32 {
    let w = &*(world as *mut PhysicsWorld);
    w.collider_set.get(u64_to_collider_handle(c))
        .map(|co| co.active_collision_types().bits() as i32)
        .unwrap_or(0)
}

/// Sets the collision groups for a collider.
#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_collider_set_collision_groups(
    world: *mut c_void, c: u64, memberships: u32, filter: u32,
) {
    let w = &mut *(world as *mut PhysicsWorld);
    if let Some(co) = w.collider_set.get_mut(u64_to_collider_handle(c)) {
        co.set_collision_groups(InteractionGroups::new(
            Group::from_bits_truncate(memberships),
            Group::from_bits_truncate(filter),
            InteractionTestMode::And,
        ));
    }
}

/// Gets the collision groups for a collider. Fills `out` with [memberships, filter].
#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_collider_get_collision_groups(
    world: *mut c_void, c: u64, out: *mut i32,
) {
    let w = &*(world as *mut PhysicsWorld);
    let arr = slice::from_raw_parts_mut(out, 2);
    if let Some(co) = w.collider_set.get(u64_to_collider_handle(c)) {
        let groups = co.collision_groups();
        arr[0] = groups.memberships.bits() as i32;
        arr[1] = groups.filter.bits() as i32;
    } else {
        arr[0] = 0; arr[1] = 0;
    }
}

/// Sets the solver groups for a collider.
#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_collider_set_solver_groups(
    world: *mut c_void, c: u64, memberships: u32, filter: u32,
) {
    let w = &mut *(world as *mut PhysicsWorld);
    if let Some(co) = w.collider_set.get_mut(u64_to_collider_handle(c)) {
        co.set_solver_groups(InteractionGroups::new(
            Group::from_bits_truncate(memberships),
            Group::from_bits_truncate(filter),
            InteractionTestMode::And,
        ));
    }
}

/// Gets the solver groups for a collider. Fills `out` with [memberships, filter].
#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_collider_get_solver_groups(
    world: *mut c_void, c: u64, out: *mut i32,
) {
    let w = &*(world as *mut PhysicsWorld);
    let arr = slice::from_raw_parts_mut(out, 2);
    if let Some(co) = w.collider_set.get(u64_to_collider_handle(c)) {
        let groups = co.solver_groups();
        arr[0] = groups.memberships.bits() as i32;
        arr[1] = groups.filter.bits() as i32;
    } else {
        arr[0] = 0; arr[1] = 0;
    }
}

// ---------------------------------------------------------------------------
// Heightfield shape (3D)
// ---------------------------------------------------------------------------

/// Creates a 3D heightfield collider. `heights` is row-major (nrows x ncols).
/// Scale determines the world-space dimensions [sx, sy, sz].
#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_create_heightfield_collider(
    world: *mut c_void, body: u64,
    heights: *const f32, nrows: i32, ncols: i32,
    scale_x: f32, scale_y: f32, scale_z: f32,
) -> u64 {
    let w = &mut *(world as *mut PhysicsWorld);
    let h = slice::from_raw_parts(heights, (nrows * ncols) as usize);
    // Array2 is column-major, but we accept row-major input — transpose
    let nr = nrows as usize;
    let nc = ncols as usize;
    let mut col_major = vec![0.0f32; nr * nc];
    for r in 0..nr {
        for c in 0..nc {
            col_major[r + c * nr] = h[r * nc + c];
        }
    }
    let heights_array = Array2::new(nr, nc, col_major);
    let collider = ColliderBuilder::heightfield(heights_array, Vector::new(scale_x, scale_y, scale_z)).build();
    collider_handle_to_u64(w.collider_set.insert_with_parent(collider, u64_to_body_handle(body), &mut w.rigid_body_set))
}

// ---------------------------------------------------------------------------
// Revolute joint (3D: anchor point + rotation axis)
// ---------------------------------------------------------------------------

#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_create_revolute_joint(
    world: *mut c_void, body1: u64, body2: u64,
    anchor_x: f32, anchor_y: f32, anchor_z: f32,
    axis_x: f32, axis_y: f32, axis_z: f32,
) -> u64 {
    let w = &mut *(world as *mut PhysicsWorld);
    let anchor = Vector::new(anchor_x, anchor_y, anchor_z);
    let axis = Vector::new(axis_x, axis_y, axis_z).normalize();
    let joint = RevoluteJointBuilder::new(axis)
        .local_anchor1(anchor)
        .local_anchor2(anchor)
        .build();
    joint_handle_to_u64(w.impulse_joint_set.insert(
        u64_to_body_handle(body1), u64_to_body_handle(body2), joint, true
    ))
}

/// Enables or disables angular limits on a revolute joint.
#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_revolute_joint_enable_limits(
    world: *mut c_void, joint: u64, enable: i32,
) {
    let w = &mut *(world as *mut PhysicsWorld);
    if let Some(j) = w.impulse_joint_set.get_mut(u64_to_joint_handle(joint), true) {
        if let Some(rev) = j.data.as_revolute_mut() {
            if enable != 0 {
                if rev.limits().is_none() {
                    rev.set_limits([-std::f32::consts::PI, std::f32::consts::PI]);
                }
            } else {
                rev.set_limits([-1000.0, 1000.0]);
            }
        }
    }
}

/// Sets the angular limits (in radians) for a revolute joint.
#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_revolute_joint_set_limits(
    world: *mut c_void, joint: u64, lower: f32, upper: f32,
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
pub unsafe extern "C" fn sge_phys3d_revolute_joint_get_limits(
    world: *mut c_void, joint: u64, out: *mut f32,
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
    arr[0] = 0.0; arr[1] = 0.0;
}

/// Returns 1 if the revolute joint has limits enabled, 0 otherwise.
#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_revolute_joint_is_limit_enabled(
    world: *mut c_void, joint: u64,
) -> i32 {
    let w = &*(world as *mut PhysicsWorld);
    if let Some(j) = w.impulse_joint_set.get(u64_to_joint_handle(joint)) {
        if let Some(rev) = j.data.as_revolute() {
            if let Some(limits) = rev.limits() {
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
pub unsafe extern "C" fn sge_phys3d_revolute_joint_enable_motor(
    world: *mut c_void, joint: u64, enable: i32,
) {
    let w = &mut *(world as *mut PhysicsWorld);
    if let Some(j) = w.impulse_joint_set.get_mut(u64_to_joint_handle(joint), true) {
        if let Some(rev) = j.data.as_revolute_mut() {
            if enable != 0 {
                rev.set_motor_velocity(0.0, 1.0);
            } else {
                rev.set_motor_velocity(0.0, 0.0);
            }
        }
    }
}

/// Sets the target velocity for the revolute joint motor (radians/second).
#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_revolute_joint_set_motor_speed(
    world: *mut c_void, joint: u64, speed: f32,
) {
    let w = &mut *(world as *mut PhysicsWorld);
    if let Some(j) = w.impulse_joint_set.get_mut(u64_to_joint_handle(joint), true) {
        if let Some(rev) = j.data.as_revolute_mut() {
            let damping = rev.motor().map(|m| m.damping).unwrap_or(1.0);
            rev.set_motor_velocity(speed, damping);
        }
    }
}

/// Sets the maximum torque the revolute joint motor can apply.
#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_revolute_joint_set_max_motor_torque(
    world: *mut c_void, joint: u64, torque: f32,
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
pub unsafe extern "C" fn sge_phys3d_revolute_joint_get_motor_speed(
    world: *mut c_void, joint: u64,
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
pub unsafe extern "C" fn sge_phys3d_revolute_joint_get_angle(
    world: *mut c_void, joint: u64,
) -> f32 {
    let w = &*(world as *mut PhysicsWorld);
    if let Some(j) = w.impulse_joint_set.get(u64_to_joint_handle(joint)) {
        if let Some(rev) = j.data.as_revolute() {
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

/// Gets the maximum motor torque for a revolute joint.
#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_revolute_joint_get_max_motor_torque(world: *mut c_void, joint: u64) -> f32 {
    let w = &*(world as *mut PhysicsWorld);
    w.impulse_joint_set.get(u64_to_joint_handle(joint))
        .and_then(|j| j.data.as_revolute())
        .and_then(|r| r.motor())
        .map(|m| m.max_force)
        .unwrap_or(0.0)
}

// ---------------------------------------------------------------------------
// Prismatic joint (3D: axis vector)
// ---------------------------------------------------------------------------

#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_create_prismatic_joint(
    world: *mut c_void, body1: u64, body2: u64,
    axis_x: f32, axis_y: f32, axis_z: f32,
) -> u64 {
    let w = &mut *(world as *mut PhysicsWorld);
    let axis = Vector::new(axis_x, axis_y, axis_z).normalize();
    let joint = PrismaticJointBuilder::new(axis).build();
    joint_handle_to_u64(w.impulse_joint_set.insert(
        u64_to_body_handle(body1), u64_to_body_handle(body2), joint, true
    ))
}

/// Enables or disables translation limits on a prismatic joint.
#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_prismatic_joint_enable_limits(
    world: *mut c_void, joint: u64, enable: i32,
) {
    let w = &mut *(world as *mut PhysicsWorld);
    if let Some(j) = w.impulse_joint_set.get_mut(u64_to_joint_handle(joint), true) {
        if let Some(pris) = j.data.as_prismatic_mut() {
            if enable != 0 {
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
pub unsafe extern "C" fn sge_phys3d_prismatic_joint_set_limits(
    world: *mut c_void, joint: u64, lower: f32, upper: f32,
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
pub unsafe extern "C" fn sge_phys3d_prismatic_joint_get_limits(
    world: *mut c_void, joint: u64, out: *mut f32,
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
    arr[0] = 0.0; arr[1] = 0.0;
}

/// Enables the motor on a prismatic joint.
#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_prismatic_joint_enable_motor(
    world: *mut c_void, joint: u64, enable: i32,
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
pub unsafe extern "C" fn sge_phys3d_prismatic_joint_set_motor_speed(
    world: *mut c_void, joint: u64, speed: f32,
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
pub unsafe extern "C" fn sge_phys3d_prismatic_joint_set_max_motor_force(
    world: *mut c_void, joint: u64, force: f32,
) {
    let w = &mut *(world as *mut PhysicsWorld);
    if let Some(j) = w.impulse_joint_set.get_mut(u64_to_joint_handle(joint), true) {
        if let Some(pris) = j.data.as_prismatic_mut() {
            pris.set_motor_max_force(force);
        }
    }
}

/// Gets the current motor speed for a prismatic joint.
#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_prismatic_joint_get_motor_speed(world: *mut c_void, joint: u64) -> f32 {
    let w = &*(world as *mut PhysicsWorld);
    w.impulse_joint_set.get(u64_to_joint_handle(joint))
        .and_then(|j| j.data.as_prismatic())
        .and_then(|p| p.motor())
        .map(|m| m.target_vel)
        .unwrap_or(0.0)
}

/// Gets the maximum motor force for a prismatic joint.
#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_prismatic_joint_get_max_motor_force(world: *mut c_void, joint: u64) -> f32 {
    let w = &*(world as *mut PhysicsWorld);
    w.impulse_joint_set.get(u64_to_joint_handle(joint))
        .and_then(|j| j.data.as_prismatic())
        .and_then(|p| p.motor())
        .map(|m| m.max_force)
        .unwrap_or(0.0)
}

/// Gets the current translation of the prismatic joint.
#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_prismatic_joint_get_translation(
    world: *mut c_void, joint: u64,
) -> f32 {
    let w = &*(world as *mut PhysicsWorld);
    if let Some(j) = w.impulse_joint_set.get(u64_to_joint_handle(joint)) {
        if let Some(_pris) = j.data.as_prismatic() {
            let body1_handle = j.body1;
            let body2_handle = j.body2;
            if let (Some(b1), Some(b2)) = (
                w.rigid_body_set.get(body1_handle),
                w.rigid_body_set.get(body2_handle),
            ) {
                let diff = b2.translation() - b1.translation();
                return diff.length();
            }
        }
    }
    0.0
}

// ---------------------------------------------------------------------------
// Motor joint (3D: 6 DOF — LinX, LinY, LinZ, AngX, AngY, AngZ)
// ---------------------------------------------------------------------------

/// Creates a motor joint between two bodies.
/// Uses GenericJoint with per-axis position motors to control
/// relative translation and rotation (6 DOF in 3D).
#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_create_motor_joint(world: *mut c_void, body1: u64, body2: u64) -> u64 {
    let w = &mut *(world as *mut PhysicsWorld);
    let mut joint = GenericJointBuilder::new(JointAxesMask::empty()).build();
    let stiffness = 100.0;
    let damping   = 20.0;
    for axis in [JointAxis::LinX, JointAxis::LinY, JointAxis::LinZ,
                 JointAxis::AngX, JointAxis::AngY, JointAxis::AngZ] {
        joint.set_motor(axis, 0.0, 0.0, stiffness, damping);
    }
    joint_handle_to_u64(w.impulse_joint_set.insert(
        u64_to_body_handle(body1), u64_to_body_handle(body2), joint, true
    ))
}

/// Sets the target linear offset for a motor joint. [x, y, z]
#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_motor_joint_set_linear_offset(
    world: *mut c_void, joint: u64, x: f32, y: f32, z: f32,
) {
    let w = &mut *(world as *mut PhysicsWorld);
    if let Some(j) = w.impulse_joint_set.get_mut(u64_to_joint_handle(joint), true) {
        let vals = [x, y, z];
        for (i, axis) in [JointAxis::LinX, JointAxis::LinY, JointAxis::LinZ].iter().enumerate() {
            let s = j.data.motor(*axis).map(|m| m.stiffness).unwrap_or(100.0);
            let d = j.data.motor(*axis).map(|m| m.damping).unwrap_or(20.0);
            j.data.set_motor(*axis, vals[i], 0.0, s, d);
        }
    }
}

/// Gets the target linear offset for a motor joint. Fills `out` with [x, y, z].
#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_motor_joint_get_linear_offset(
    world: *mut c_void, joint: u64, out: *mut f32,
) {
    let w = &*(world as *mut PhysicsWorld);
    let arr = slice::from_raw_parts_mut(out, 3);
    if let Some(j) = w.impulse_joint_set.get(u64_to_joint_handle(joint)) {
        arr[0] = j.data.motor(JointAxis::LinX).map(|m| m.target_pos).unwrap_or(0.0);
        arr[1] = j.data.motor(JointAxis::LinY).map(|m| m.target_pos).unwrap_or(0.0);
        arr[2] = j.data.motor(JointAxis::LinZ).map(|m| m.target_pos).unwrap_or(0.0);
    } else {
        arr[0] = 0.0; arr[1] = 0.0; arr[2] = 0.0;
    }
}

/// Sets the target angular offset for a motor joint (Euler angles in radians).
#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_motor_joint_set_angular_offset(
    world: *mut c_void, joint: u64, rx: f32, ry: f32, rz: f32,
) {
    let w = &mut *(world as *mut PhysicsWorld);
    if let Some(j) = w.impulse_joint_set.get_mut(u64_to_joint_handle(joint), true) {
        let vals = [rx, ry, rz];
        for (i, axis) in [JointAxis::AngX, JointAxis::AngY, JointAxis::AngZ].iter().enumerate() {
            let s = j.data.motor(*axis).map(|m| m.stiffness).unwrap_or(100.0);
            let d = j.data.motor(*axis).map(|m| m.damping).unwrap_or(20.0);
            j.data.set_motor(*axis, vals[i], 0.0, s, d);
        }
    }
}

/// Gets the target angular offset for a motor joint. Fills `out` with [rx, ry, rz].
#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_motor_joint_get_angular_offset(
    world: *mut c_void, joint: u64, out: *mut f32,
) {
    let w = &*(world as *mut PhysicsWorld);
    let arr = slice::from_raw_parts_mut(out, 3);
    if let Some(j) = w.impulse_joint_set.get(u64_to_joint_handle(joint)) {
        arr[0] = j.data.motor(JointAxis::AngX).map(|m| m.target_pos).unwrap_or(0.0);
        arr[1] = j.data.motor(JointAxis::AngY).map(|m| m.target_pos).unwrap_or(0.0);
        arr[2] = j.data.motor(JointAxis::AngZ).map(|m| m.target_pos).unwrap_or(0.0);
    } else {
        arr[0] = 0.0; arr[1] = 0.0; arr[2] = 0.0;
    }
}

/// Sets the maximum linear force for a motor joint.
#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_motor_joint_set_max_force(
    world: *mut c_void, joint: u64, force: f32,
) {
    let w = &mut *(world as *mut PhysicsWorld);
    if let Some(j) = w.impulse_joint_set.get_mut(u64_to_joint_handle(joint), true) {
        j.data.set_motor_max_force(JointAxis::LinX, force);
        j.data.set_motor_max_force(JointAxis::LinY, force);
        j.data.set_motor_max_force(JointAxis::LinZ, force);
    }
}

/// Sets the maximum angular torque for a motor joint.
#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_motor_joint_set_max_torque(
    world: *mut c_void, joint: u64, torque: f32,
) {
    let w = &mut *(world as *mut PhysicsWorld);
    if let Some(j) = w.impulse_joint_set.get_mut(u64_to_joint_handle(joint), true) {
        j.data.set_motor_max_force(JointAxis::AngX, torque);
        j.data.set_motor_max_force(JointAxis::AngY, torque);
        j.data.set_motor_max_force(JointAxis::AngZ, torque);
    }
}

/// Sets the correction factor (stiffness) for all motor axes.
#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_motor_joint_set_correction_factor(
    world: *mut c_void, joint: u64, factor: f32,
) {
    let w = &mut *(world as *mut PhysicsWorld);
    if let Some(j) = w.impulse_joint_set.get_mut(u64_to_joint_handle(joint), true) {
        let stiffness = factor * 100.0;
        let damping   = 2.0 * stiffness.sqrt();
        for axis in [JointAxis::LinX, JointAxis::LinY, JointAxis::LinZ,
                     JointAxis::AngX, JointAxis::AngY, JointAxis::AngZ] {
            let target = j.data.motor(axis).map(|m| m.target_pos).unwrap_or(0.0);
            j.data.set_motor(axis, target, 0.0, stiffness, damping);
        }
    }
}

/// Gets the maximum linear force for a motor joint.
#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_motor_joint_get_max_force(world: *mut c_void, joint: u64) -> f32 {
    let w = &*(world as *mut PhysicsWorld);
    w.impulse_joint_set.get(u64_to_joint_handle(joint))
        .and_then(|j| j.data.motor(JointAxis::LinX))
        .map(|m| m.max_force)
        .unwrap_or(0.0)
}

/// Gets the maximum angular torque for a motor joint.
#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_motor_joint_get_max_torque(world: *mut c_void, joint: u64) -> f32 {
    let w = &*(world as *mut PhysicsWorld);
    w.impulse_joint_set.get(u64_to_joint_handle(joint))
        .and_then(|j| j.data.motor(JointAxis::AngX))
        .map(|m| m.max_force)
        .unwrap_or(0.0)
}

/// Gets the correction factor for a motor joint.
#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_motor_joint_get_correction_factor(world: *mut c_void, joint: u64) -> f32 {
    let w = &*(world as *mut PhysicsWorld);
    w.impulse_joint_set.get(u64_to_joint_handle(joint))
        .and_then(|j| j.data.motor(JointAxis::LinX))
        .map(|m| m.stiffness / 100.0)
        .unwrap_or(0.0)
}

// ---------------------------------------------------------------------------
// Spring joint (3D)
// ---------------------------------------------------------------------------

/// Creates a spring joint emulated via GenericJoint with position motors.
#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_create_spring_joint(
    world: *mut c_void, body1: u64, body2: u64,
    rest_length: f32, stiffness: f32, damping: f32,
) -> u64 {
    let w = &mut *(world as *mut PhysicsWorld);
    let mut joint = GenericJointBuilder::new(JointAxesMask::empty()).build();
    joint.set_motor(JointAxis::LinX, rest_length, 0.0, stiffness, damping);
    joint.set_motor(JointAxis::LinY, 0.0, 0.0, stiffness, damping);
    joint.set_motor(JointAxis::LinZ, 0.0, 0.0, stiffness, damping);
    joint_handle_to_u64(w.impulse_joint_set.insert(
        u64_to_body_handle(body1), u64_to_body_handle(body2), joint, true
    ))
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_spring_joint_set_rest_length(world: *mut c_void, joint: u64, rest_length: f32) {
    let w = &mut *(world as *mut PhysicsWorld);
    if let Some(j) = w.impulse_joint_set.get_mut(u64_to_joint_handle(joint), true) {
        let s = j.data.motor(JointAxis::LinX).map(|m| m.stiffness).unwrap_or(100.0);
        let d = j.data.motor(JointAxis::LinX).map(|m| m.damping).unwrap_or(10.0);
        j.data.set_motor(JointAxis::LinX, rest_length, 0.0, s, d);
    }
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_spring_joint_get_rest_length(world: *mut c_void, joint: u64) -> f32 {
    let w = &*(world as *mut PhysicsWorld);
    w.impulse_joint_set.get(u64_to_joint_handle(joint))
        .and_then(|j| j.data.motor(JointAxis::LinX))
        .map(|m| m.target_pos)
        .unwrap_or(0.0)
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_spring_joint_set_params(
    world: *mut c_void, joint: u64, stiffness: f32, damping: f32,
) {
    let w = &mut *(world as *mut PhysicsWorld);
    if let Some(j) = w.impulse_joint_set.get_mut(u64_to_joint_handle(joint), true) {
        let target = j.data.motor(JointAxis::LinX).map(|m| m.target_pos).unwrap_or(0.0);
        j.data.set_motor(JointAxis::LinX, target, 0.0, stiffness, damping);
        j.data.set_motor(JointAxis::LinY, 0.0, 0.0, stiffness, damping);
        j.data.set_motor(JointAxis::LinZ, 0.0, 0.0, stiffness, damping);
    }
}

// ---------------------------------------------------------------------------
// Rope joint getters
// ---------------------------------------------------------------------------

/// Sets the maximum allowed distance for a rope joint.
#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_rope_joint_set_max_distance(
    world: *mut c_void, joint: u64, max_dist: f32,
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
pub unsafe extern "C" fn sge_phys3d_rope_joint_get_max_distance(
    world: *mut c_void, joint: u64,
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
// Queries (3D)
// ---------------------------------------------------------------------------

/// AABB query (3D: 6 bounds). Finds all colliders intersecting the AABB.
/// Fills `out_colliders` with collider handles. Returns count of hits.
#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_query_aabb(
    world: *mut c_void, min_x: f32, min_y: f32, min_z: f32,
    max_x: f32, max_y: f32, max_z: f32,
    out_colliders: *mut u64, max_results: i32,
) -> i32 {
    let w = &*(world as *mut PhysicsWorld);
    let aabb = Aabb::new(
        Vector::new(min_x, min_y, min_z),
        Vector::new(max_x, max_y, max_z),
    );
    let out = slice::from_raw_parts_mut(out_colliders, max_results as usize);
    let mut count = 0i32;

    let query_pipeline = w.broad_phase.as_query_pipeline(
        &DefaultQueryDispatcher,
        &w.rigid_body_set,
        &w.collider_set,
        QueryFilter::default(),
    );

    for (handle, _collider) in query_pipeline.intersect_aabb_conservative(aabb) {
        if count >= max_results { break; }
        out[count as usize] = collider_handle_to_u64(handle);
        count += 1;
    }
    count
}

/// Point query (3D). Fills `out_bodies` with body handles of colliders containing the point.
#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_query_point(
    world: *mut c_void, x: f32, y: f32, z: f32,
    out_bodies: *mut u64, max_results: i32,
) -> i32 {
    let w = &*(world as *mut PhysicsWorld);
    let pt = Vector::new(x, y, z);
    let out = slice::from_raw_parts_mut(out_bodies, max_results as usize);
    let mut count = 0i32;

    let query_pipeline = w.broad_phase.as_query_pipeline(
        &DefaultQueryDispatcher,
        &w.rigid_body_set,
        &w.collider_set,
        QueryFilter::default(),
    );

    for (_handle, collider) in query_pipeline.intersect_point(pt) {
        if count >= max_results { break; }
        if let Some(parent) = collider.parent() {
            out[count as usize] = body_handle_to_u64(parent);
            count += 1;
        }
    }
    count
}

/// Ray cast returning ALL intersections. Each hit = 9 floats:
/// [hitX, hitY, hitZ, normalX, normalY, normalZ, toi, colliderLo, colliderHi].
/// Returns the number of hits (capped at `max_hits`).
#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_ray_cast_all(
    world: *mut c_void,
    ox: f32, oy: f32, oz: f32, dx: f32, dy: f32, dz: f32, max_dist: f32,
    out_hits: *mut f32, max_hits: i32,
) -> i32 {
    let w = &*(world as *mut PhysicsWorld);
    let ray = Ray::new(Vector::new(ox, oy, oz).into(), Vector::new(dx, dy, dz));
    let arr = slice::from_raw_parts_mut(out_hits, (max_hits * 9) as usize);
    let mut count = 0i32;

    let query_pipeline = w.broad_phase.as_query_pipeline(
        &DefaultQueryDispatcher,
        &w.rigid_body_set,
        &w.collider_set,
        QueryFilter::default(),
    );

    for (handle, _collider, intersection) in query_pipeline.intersect_ray(ray, max_dist, true) {
        if count >= max_hits { break; }
        let idx = (count * 9) as usize;
        let hit = ray.point_at(intersection.time_of_impact);
        arr[idx]     = hit.x;
        arr[idx + 1] = hit.y;
        arr[idx + 2] = hit.z;
        arr[idx + 3] = intersection.normal.x;
        arr[idx + 4] = intersection.normal.y;
        arr[idx + 5] = intersection.normal.z;
        arr[idx + 6] = intersection.time_of_impact;
        let ch = collider_handle_to_u64(handle);
        arr[idx + 7] = f32::from_bits(ch as u32);
        arr[idx + 8] = f32::from_bits((ch >> 32) as u32);
        count += 1;
    }
    count
}

/// Projects a point onto the closest collider. Returns 1 if found, 0 otherwise.
/// `out`: [projX, projY, projZ, isInside (1.0 or 0.0), colliderLo, colliderHi] = 6 floats.
#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_project_point(
    world: *mut c_void, x: f32, y: f32, z: f32, out: *mut f32,
) -> i32 {
    let w = &*(world as *mut PhysicsWorld);
    let pt = Vector::new(x, y, z);

    let query_pipeline = w.broad_phase.as_query_pipeline(
        &DefaultQueryDispatcher,
        &w.rigid_body_set,
        &w.collider_set,
        QueryFilter::default(),
    );

    if let Some((handle, projection)) = query_pipeline.project_point(pt, Real::MAX, true) {
        let arr = slice::from_raw_parts_mut(out, 6);
        arr[0] = projection.point.x;
        arr[1] = projection.point.y;
        arr[2] = projection.point.z;
        arr[3] = if projection.is_inside { 1.0 } else { 0.0 };
        let ch = collider_handle_to_u64(handle);
        arr[4] = f32::from_bits(ch as u32);
        arr[5] = f32::from_bits((ch >> 32) as u32);
        1
    } else {
        0
    }
}

/// Shape cast (sweep test) in 3D. Returns 1 on hit, 0 on miss.
/// `shape_type`: 0=ball, 1=cuboid, 2=capsule. `shape_params` depends on type:
///   ball: [radius], cuboid: [hx, hy, hz], capsule: [halfHeight, radius]
/// `out`: [hitX, hitY, hitZ, normalX, normalY, normalZ, toi, colliderLo, colliderHi] = 9 floats.
#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_cast_shape(
    world: *mut c_void,
    shape_type: i32, shape_params: *const f32,
    origin_x: f32, origin_y: f32, origin_z: f32,
    dir_x: f32, dir_y: f32, dir_z: f32,
    max_dist: f32,
    out: *mut f32,
) -> i32 {
    let w = &*(world as *mut PhysicsWorld);
    let params = slice::from_raw_parts(shape_params, 3);

    let shape: Box<dyn Shape> = match shape_type {
        0 => Box::new(Ball::new(params[0])),
        1 => Box::new(Cuboid::new(Vector::new(params[0], params[1], params[2]))),
        2 => Box::new(Capsule::new_y(params[0], params[1])),
        _ => return 0,
    };

    let origin = Pose::from_parts(
        Vector::new(origin_x, origin_y, origin_z),
        Rotation::IDENTITY,
    );
    let dir = Vector::new(dir_x, dir_y, dir_z);

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
        let arr = slice::from_raw_parts_mut(out, 9);
        let hit_point = Vector::new(origin_x, origin_y, origin_z) + dir * toi_result.time_of_impact;
        arr[0] = hit_point.x;
        arr[1] = hit_point.y;
        arr[2] = hit_point.z;
        arr[3] = toi_result.normal1.x;
        arr[4] = toi_result.normal1.y;
        arr[5] = toi_result.normal1.z;
        arr[6] = toi_result.time_of_impact;
        let ch = collider_handle_to_u64(handle);
        arr[7] = f32::from_bits(ch as u32);
        arr[8] = f32::from_bits((ch >> 32) as u32);
        1
    } else {
        0
    }
}

/// Tests if a shape at a given position overlaps any collider.
/// `shape_type`: 0=ball, 1=cuboid, 2=capsule. `shape_params` depends on type.
/// Fills `out_colliders` with collider handles. Returns the count.
#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_intersect_shape(
    world: *mut c_void,
    shape_type: i32, shape_params: *const f32,
    pos_x: f32, pos_y: f32, pos_z: f32,
    qx: f32, qy: f32, qz: f32, qw: f32,
    out_colliders: *mut u64, max_results: i32,
) -> i32 {
    let w = &*(world as *mut PhysicsWorld);
    let params = slice::from_raw_parts(shape_params, 3);

    let shape: Box<dyn Shape> = match shape_type {
        0 => Box::new(Ball::new(params[0])),
        1 => Box::new(Cuboid::new(Vector::new(params[0], params[1], params[2]))),
        2 => Box::new(Capsule::new_y(params[0], params[1])),
        _ => return 0,
    };

    let pos = Pose::from_parts(
        Vector::new(pos_x, pos_y, pos_z),
        Rotation::from_xyzw(qx, qy, qz, qw),
    );
    let arr = slice::from_raw_parts_mut(out_colliders, max_results as usize);
    let mut count = 0i32;

    let query_pipeline = w.broad_phase.as_query_pipeline(
        &DefaultQueryDispatcher,
        &w.rigid_body_set,
        &w.collider_set,
        QueryFilter::default(),
    );

    for (handle, _collider) in query_pipeline.intersect_shape(pos, shape.as_ref()) {
        if count >= max_results { break; }
        arr[count as usize] = collider_handle_to_u64(handle);
        count += 1;
    }
    count
}

// ---------------------------------------------------------------------------
// Contact details (3D)
// ---------------------------------------------------------------------------

/// Gets the number of contact points between two colliders.
#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_contact_pair_count(
    world: *mut c_void, collider1: u64, collider2: u64,
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
/// Fills `out` with `[normalX, normalY, normalZ, pointX, pointY, pointZ, penetration]`
/// per contact point (7 floats each). Returns the number of points written.
#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_contact_pair_points(
    world: *mut c_void, collider1: u64, collider2: u64,
    out: *mut f32, max_points: i32,
) -> i32 {
    let w = &*(world as *mut PhysicsWorld);
    let c1 = u64_to_collider_handle(collider1);
    let c2 = u64_to_collider_handle(collider2);

    let arr = slice::from_raw_parts_mut(out, (max_points * 7) as usize);
    let mut count = 0i32;

    if let Some(pair) = w.narrow_phase.contact_pair(c1, c2) {
        let pos1 = w.collider_set.get(c1).map(|c| *c.position()).unwrap_or(Pose::identity());
        for manifold in &pair.manifolds {
            let normal = manifold.data.normal;
            for pt in &manifold.points {
                if count >= max_points { return count; }
                let idx = (count * 7) as usize;
                let world_pt = pos1 * pt.local_p1;
                arr[idx]     = normal.x;
                arr[idx + 1] = normal.y;
                arr[idx + 2] = normal.z;
                arr[idx + 3] = world_pt.x;
                arr[idx + 4] = world_pt.y;
                arr[idx + 5] = world_pt.z;
                arr[idx + 6] = pt.dist;
                count += 1;
            }
        }
    }
    count
}

// ---------------------------------------------------------------------------
// Intersection events (sensor overlaps)
// ---------------------------------------------------------------------------

#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_poll_intersection_start_events(
    world: *mut c_void, out_collider1: *mut u64, out_collider2: *mut u64, max_events: i32,
) -> i32 {
    // Intersection event buffering will be added in a follow-up.
    let _ = (world, out_collider1, out_collider2, max_events);
    0
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_poll_intersection_stop_events(
    world: *mut c_void, out_collider1: *mut u64, out_collider2: *mut u64, max_events: i32,
) -> i32 {
    let _ = (world, out_collider1, out_collider2, max_events);
    0
}

// ---------------------------------------------------------------------------
// World — simulation parameters
// ---------------------------------------------------------------------------

#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_world_set_num_solver_iterations(world: *mut c_void, iters: i32) {
    let w = &mut *(world as *mut PhysicsWorld);
    w.integration_parameters.num_solver_iterations = (iters as usize).max(1);
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_world_get_num_solver_iterations(world: *mut c_void) -> i32 {
    let w = &*(world as *mut PhysicsWorld);
    w.integration_parameters.num_solver_iterations as i32
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_world_set_num_additional_friction_iterations(world: *mut c_void, iters: i32) {
    let w = &mut *(world as *mut PhysicsWorld);
    w.integration_parameters.num_internal_stabilization_iterations = iters as usize;
}

#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_world_set_num_internal_pgs_iterations(world: *mut c_void, iters: i32) {
    let w = &mut *(world as *mut PhysicsWorld);
    w.integration_parameters.num_internal_pgs_iterations = iters as usize;
}

// ---------------------------------------------------------------------------
// Contact force events (polling)
// ---------------------------------------------------------------------------

/// Polls contact force events since the last step.
/// Fills out_collider1, out_collider2, and out_force arrays.
/// Returns the event count (capped at max_events).
#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_poll_contact_force_events(
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
pub unsafe extern "C" fn sge_phys3d_collider_set_contact_force_event_threshold(
    world: *mut c_void, collider: u64, threshold: f32,
) {
    let w = &mut *(world as *mut PhysicsWorld);
    if let Some(c) = w.collider_set.get_mut(u64_to_collider_handle(collider)) {
        c.set_contact_force_event_threshold(threshold);
    }
}

/// Gets the contact force event threshold for a collider.
#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_collider_get_contact_force_event_threshold(
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
pub unsafe extern "C" fn sge_phys3d_collider_set_active_hooks(
    world: *mut c_void, collider: u64, flags: i32,
) {
    let w = &mut *(world as *mut PhysicsWorld);
    if let Some(c) = w.collider_set.get_mut(u64_to_collider_handle(collider)) {
        c.set_active_hooks(ActiveHooks::from_bits_truncate(flags as u32));
    }
}

/// Gets the active hooks flags for a collider.
#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_collider_get_active_hooks(
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
/// Set nx=0, ny=0, nz=0 to disable one-way behavior for this collider.
///
/// Requires ActiveHooks::MODIFY_SOLVER_CONTACTS (0x04) to be set on the collider
/// via sge_phys3d_collider_set_active_hooks.
#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_collider_set_one_way_direction(
    world: *mut c_void, collider: u64, nx: f32, ny: f32, nz: f32, allowed_angle: f32,
) {
    let w = &mut *(world as *mut PhysicsWorld);
    if nx == 0.0 && ny == 0.0 && nz == 0.0 {
        w.one_way_platforms.remove(&collider);
    } else {
        w.one_way_platforms.insert(collider, ([nx, ny, nz], allowed_angle));
    }
}

/// Returns 1 if the collider has one-way platform behavior, 0 otherwise.
/// If it does, fills out_nx, out_ny, out_nz, out_angle with the configured direction and angle.
#[no_mangle]
pub unsafe extern "C" fn sge_phys3d_collider_get_one_way_direction(
    world: *mut c_void, collider: u64,
    out_nx: *mut f32, out_ny: *mut f32, out_nz: *mut f32, out_angle: *mut f32,
) -> i32 {
    let w = &*(world as *mut PhysicsWorld);
    if let Some(&(dir, angle)) = w.one_way_platforms.get(&collider) {
        *out_nx = dir[0];
        *out_ny = dir[1];
        *out_nz = dir[2];
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

    #[test]
    fn world_create_destroy() {
        unsafe {
            let world = sge_phys3d_create_world(0.0, -9.81, 0.0);
            assert!(!world.is_null());
            let mut g = [0.0f32; 3];
            sge_phys3d_world_get_gravity(world, g.as_mut_ptr());
            assert!((g[1] - (-9.81)).abs() < 1e-5);
            sge_phys3d_destroy_world(world);
        }
    }

    #[test]
    fn body_falls_under_gravity() {
        unsafe {
            let world = sge_phys3d_create_world(0.0, -9.81, 0.0);
            let body = sge_phys3d_create_dynamic_body(world, 0.0, 10.0, 0.0, 0.0, 0.0, 0.0, 1.0);
            let _col = sge_phys3d_create_box_collider(world, body, 0.5, 0.5, 0.5);
            let mut pos = [0.0f32; 3];
            sge_phys3d_body_get_position(world, body, pos.as_mut_ptr());
            let y0 = pos[1];
            for _ in 0..60 { sge_phys3d_world_step(world, 1.0 / 60.0); }
            sge_phys3d_body_get_position(world, body, pos.as_mut_ptr());
            assert!(pos[1] < y0, "body should fall: y0={}, y={}", y0, pos[1]);
            sge_phys3d_destroy_world(world);
        }
    }

    #[test]
    fn static_body_stays() {
        unsafe {
            let world = sge_phys3d_create_world(0.0, -9.81, 0.0);
            let body = sge_phys3d_create_static_body(world, 5.0, 5.0, 5.0, 0.0, 0.0, 0.0, 1.0);
            let _col = sge_phys3d_create_box_collider(world, body, 1.0, 1.0, 1.0);
            for _ in 0..60 { sge_phys3d_world_step(world, 1.0 / 60.0); }
            let mut pos = [0.0f32; 3];
            sge_phys3d_body_get_position(world, body, pos.as_mut_ptr());
            assert!((pos[0] - 5.0).abs() < 1e-5);
            assert!((pos[1] - 5.0).abs() < 1e-5);
            assert!((pos[2] - 5.0).abs() < 1e-5);
            sge_phys3d_destroy_world(world);
        }
    }

    #[test]
    fn raycast_3d() {
        unsafe {
            let world = sge_phys3d_create_world(0.0, 0.0, 0.0);
            let body = sge_phys3d_create_static_body(world, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
            let _col = sge_phys3d_create_box_collider(world, body, 1.0, 1.0, 1.0);
            sge_phys3d_world_step(world, 1.0 / 60.0);
            let mut out = [0.0f32; 9];
            let hit = sge_phys3d_ray_cast(world, 0.0, 10.0, 0.0, 0.0, -1.0, 0.0, 100.0, out.as_mut_ptr());
            assert_eq!(hit, 1, "ray should hit the box");
            assert!((out[1] - 1.0).abs() < 0.1, "hit Y should be ~1.0, got {}", out[1]);
            sge_phys3d_destroy_world(world);
        }
    }

    #[test]
    fn rope_joint_3d() {
        unsafe {
            let world = sge_phys3d_create_world(0.0, 0.0, 0.0);
            let b1 = sge_phys3d_create_dynamic_body(world, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
            let _c1 = sge_phys3d_create_box_collider(world, b1, 0.5, 0.5, 0.5);
            let b2 = sge_phys3d_create_dynamic_body(world, 3.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0);
            let _c2 = sge_phys3d_create_box_collider(world, b2, 0.5, 0.5, 0.5);
            let _joint = sge_phys3d_create_rope_joint(world, b1, b2, 5.0);
            sge_phys3d_world_step(world, 1.0 / 60.0);
            sge_phys3d_destroy_world(world);
        }
    }
}
