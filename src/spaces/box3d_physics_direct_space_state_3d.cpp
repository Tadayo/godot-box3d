#include "box3d_physics_direct_space_state_3d.hpp"

#include "../misc/box3d_globals.hpp"
#include "../misc/box3d_shape_proxy.hpp"
#include "../misc/type_conversions.hpp"
#include "../objects/box3d_area_impl_3d.hpp"
#include "../objects/box3d_body_impl_3d.hpp"
#include "../objects/box3d_shaped_object_impl_3d.hpp"
#include "../servers/box3d_physics_server_3d.hpp"
#include "../shapes/box3d_capsule_shape_impl_3d.hpp"
#include "../shapes/box3d_shape_impl_3d.hpp"
#include "box3d_query_filter_3d.hpp"
#include "box3d_space_3d.hpp"

#include <godot_cpp/classes/os.hpp>
#include <godot_cpp/variant/utility_functions.hpp>

#include <box3d/box3d.h>
#include <box3d/constants.h> // B3_LINEAR_SLOP

namespace {

struct OverlapContext {
	const Box3DQueryFilter3D* filter = nullptr;
	PhysicsServer3DExtensionShapeResult* results = nullptr;
	int32_t max_results = 0;
	int32_t count = 0;
};

bool should_report(void* p_user_data, const Box3DQueryFilter3D& p_filter, Box3DShapedObjectImpl3D*& r_object) {
	auto* object = static_cast<Box3DShapedObjectImpl3D*>(p_user_data);
	if (object == nullptr) {
		return false;
	}
	const bool is_area = dynamic_cast<Box3DAreaImpl3D*>(object) != nullptr;
	if (is_area && !p_filter.collide_with_areas) {
		return false;
	}
	if (!is_area && !p_filter.collide_with_bodies) {
		return false;
	}
	if (p_filter.should_exclude(object->get_rid())) {
		return false;
	}
	r_object = object;
	return true;
}

bool overlap_result_fcn(b3ShapeId p_shape_id, void* p_context) {
	auto* ctx = static_cast<OverlapContext*>(p_context);
	if (ctx->count >= ctx->max_results) {
		return false;
	}

	const b3BodyId body_id = b3Shape_GetBody(p_shape_id);
	Box3DShapedObjectImpl3D* object = nullptr;
	if (!should_report(b3Body_GetUserData(body_id), *ctx->filter, object)) {
		return true;
	}

	PhysicsServer3DExtensionShapeResult& result = ctx->results[ctx->count];
	result.rid = object->get_rid();
	result.collider_id = object->get_instance_id();
	result.shape = 0;
	ctx->count++;
	return true;
}

struct CollideShapeContext {
	const Box3DQueryFilter3D* filter = nullptr;
	const b3ShapeProxy* query_proxy = nullptr;
	Vector3* results = nullptr;
	int32_t max_results = 0;
	int32_t count = 0;
};

// Reports the closest points between the query shape and one overlapping shape. Godot wants
// world-space pairs, and b3ShapeDistance runs in frame A, which is world space here because
// Box3DShapeProxy3D already bakes the transform into its points.
bool collide_shape_result_fcn(b3ShapeId p_shape_id, void* p_context) {
	auto* ctx = static_cast<CollideShapeContext*>(p_context);
	if (ctx->count >= ctx->max_results) {
		return false;
	}

	const b3BodyId body_id = b3Shape_GetBody(p_shape_id);
	Box3DShapedObjectImpl3D* object = nullptr;
	if (!should_report(b3Body_GetUserData(body_id), *ctx->filter, object)) {
		return true;
	}

	const Transform3D object_transform = object->get_transform();
	for (int32_t i = 0; i < object->get_shape_count(); i++) {
		if (!object->has_shape_id(i) || !B3_ID_EQUALS(object->get_shape_id(i), p_shape_id)) {
			continue;
		}

		const Box3DShapeProxy3D other_proxy(object->get_shape(i), object_transform * object->get_shape_transform(i));
		if (!other_proxy.is_supported()) {
			return true;
		}

		b3DistanceInput input{};
		input.proxyA = *ctx->query_proxy;
		input.proxyB = other_proxy.get_proxy();
		input.transform = b3Transform_identity;
		input.useRadii = true;

		b3SimplexCache cache{};
		const b3DistanceOutput output = b3ShapeDistance(&input, &cache, nullptr, 0);

		// GJK cannot recover penetration depth, so an overlapping pair reports its witness
		// point for both sides rather than a fabricated depth.
		ctx->results[ctx->count * 2 + 0] = b3_to_godot(output.pointA);
		ctx->results[ctx->count * 2 + 1] = b3_to_godot(output.pointB);
		ctx->count++;
		return true;
	}
	return true;
}

struct RayContext {
	const Box3DQueryFilter3D* filter = nullptr;
	bool hit_from_inside = false;
	bool has_hit = false;
	b3ShapeId shape_id = b3_nullShapeId;
	b3Pos point{};
	b3Vec3 normal{};
	float fraction = 1.0f;
};

float cast_result_fcn(b3ShapeId p_shape_id, b3Pos p_point, b3Vec3 p_normal, float p_fraction, uint64_t, int, int, void* p_context) {
	auto* ctx = static_cast<RayContext*>(p_context);

	if (ctx->filter->should_exclude_shape(p_shape_id)) {
		return -1.0f;
	}
	const b3BodyId body_id = b3Shape_GetBody(p_shape_id);
	Box3DShapedObjectImpl3D* object = nullptr;
	if (!should_report(b3Body_GetUserData(body_id), *ctx->filter, object)) {
		return -1.0f;
	}

	ctx->has_hit = true;
	ctx->shape_id = p_shape_id;
	ctx->point = p_point;
	ctx->normal = p_normal;
	ctx->fraction = p_fraction;
	return p_fraction;
}

// Ray half of witness_normal, for colliders with no convex core (mesh, heightfield): the
// surface under a contact still answers a ray. One is fired from the query shape's centre
// through the reported contact point and the normal of the crossed triangle is read back.
// The callback clips to the target collider alone -- anything else along the ray is another
// contact's business, not this one's.
struct WitnessRayContext {
	const void* target_user_data = nullptr;
	bool has_hit = false;
	b3Vec3 normal{};
};

float witness_ray_fcn(b3ShapeId p_shape_id, b3Pos, b3Vec3 p_normal, float p_fraction, uint64_t, int, int, void* p_context) {
	auto* ctx = static_cast<WitnessRayContext*>(p_context);
	const b3BodyId body_id = b3Shape_GetBody(p_shape_id);
	if (b3Body_GetUserData(body_id) != ctx->target_user_data) {
		return -1.0f; // Not the collider being described; skip it and keep casting.
	}
	ctx->has_hit = true;
	ctx->normal = p_normal;
	return p_fraction; // Clip: keep the nearest crossing of the target.
}

bool ray_witness_normal(
		b3WorldId p_world,
		const Box3DShapedObjectImpl3D* p_target,
		const Vector3& p_from,
		const Vector3& p_contact,
		Vector3& r_normal) {
	Vector3 dir = p_contact - p_from;
	const float len = (float)dir.length();
	if (len < 1e-6f) {
		return false;
	}
	dir /= len;
	// Overshoot past the contact: a resting body's contact point sits ON the surface (or a
	// slop inside it), and a ray that stops exactly there may not register the crossing.
	const Vector3 translation = dir * (len + 0.25f);

	WitnessRayContext context;
	context.target_user_data = p_target;
	b3World_CastRay(p_world, godot_to_b3(p_from), godot_to_b3(translation), b3DefaultQueryFilter(), witness_ray_fcn, &context);
	if (!context.has_hit) {
		return false;
	}
	Vector3 normal = b3_to_godot(context.normal);
	if (normal.length_squared() < 1e-8f) {
		return false;
	}
	if (normal.dot(dir) > 0.0f) {
		normal = -normal; // Face the query shape, whichever winding the triangle had.
	}
	r_normal = normal.normalized();
	return true;
}

// Recovers the surface normal for a contact the shape cast could not describe. A cast that
// starts already touching reports the reverse of its own direction, so the floor a body is
// standing on looks like a head-on wall no matter which way the body tries to move. GJK
// between the two CORE shapes -- radii excluded, so a resting capsule's inner segment still
// stands clear of the floor it is sunk into -- recovers the direction that separates them.
// Concave colliders (mesh, heightfield) have no convex core for GJK, so their normal comes
// from ray_witness_normal above instead -- without that a frame standing on a mesh floor is
// pinned in place exactly the way the un-patched cast pinned one on a box floor.
// Returns false when the cores overlap too (deep penetration), where GJK has nothing to say.
bool witness_normal(
		b3WorldId p_world,
		const Vector3& p_contact,
		const Box3DShapeImpl3D* p_shape,
		const Transform3D& p_transform,
		b3ShapeId p_other_shape_id,
		Vector3& r_normal) {
	const b3BodyId body_id = b3Shape_GetBody(p_other_shape_id);
	auto* other = static_cast<Box3DShapedObjectImpl3D*>(b3Body_GetUserData(body_id));
	if (other == nullptr) {
		return false;
	}

	const Box3DShapeProxy3D self_proxy(p_shape, p_transform);
	if (!self_proxy.is_supported()) {
		return false;
	}

	const Transform3D other_transform = other->get_transform();
	for (int32_t i = 0; i < other->get_shape_count(); i++) {
		if (!other->has_shape_id(i) || !B3_ID_EQUALS(other->get_shape_id(i), p_other_shape_id)) {
			continue;
		}
		const Box3DShapeProxy3D other_proxy(other->get_shape(i), other_transform * other->get_shape_transform(i));
		if (!other_proxy.is_supported()) {
			return ray_witness_normal(p_world, other, p_transform.origin, p_contact, r_normal);
		}

		b3DistanceInput input{};
		input.proxyA = self_proxy.get_proxy();
		input.proxyB = other_proxy.get_proxy();
		input.transform = b3Transform_identity;
		input.useRadii = false;

		b3SimplexCache cache{};
		const b3DistanceOutput output = b3ShapeDistance(&input, &cache, nullptr, 0);
		const Vector3 delta = b3_to_godot(output.pointA) - b3_to_godot(output.pointB);
		if (delta.length_squared() < 1e-10f) {
			return false;
		}
		r_normal = delta.normalized();
		return true;
	}
	return false;
}

// --- character mover -------------------------------------------------------
//
// Box3D has a first-class kinematic character API and it is the documented way
// to move a capsule: b3World_CastMover sweeps one while "handling sliding along
// other shapes while reducing clipping", and b3World_CollideMover gathers the
// contact planes -- the docs are explicit that the cast is not a good source of
// touch information and the planes are. Between them they answer exactly what
// Godot's CharacterBody3D asks test_body_motion, and they replace the generic
// shape-cast path's shrink / GJK-witness / re-cast dance for capsules.

// Room for the planes one capsule can touch at once. A character in a corner
// sees three or four; the cap is generous and simply stops collecting past it.
constexpr int MOVER_MAX_PLANES = 16;

struct MoverContext {
	const Box3DQueryFilter3D* filter = nullptr;
	// Most-opposed plane found so far.
	Vector3 normal;
	Vector3 point;
	float opposition = 0.0f; // -dot(normal, motion_dir); larger is more head-on
	Vector3 motion_dir;
	b3ShapeId shape_id = b3_nullShapeId;
	bool has_plane = false;
	b3Plane planes[MOVER_MAX_PLANES];
	int count = 0;
};

bool mover_filter_fcn(b3ShapeId p_shape_id, void* p_context) {
	auto* ctx = static_cast<MoverContext*>(p_context);
	const b3BodyId body_id = b3Shape_GetBody(p_shape_id);
	Box3DShapedObjectImpl3D* object = nullptr;
	return should_report(b3Body_GetUserData(body_id), *ctx->filter, object);
}

bool mover_plane_fcn(b3ShapeId p_shape_id, const b3PlaneResult* p_plane, int, void* p_context) {
	auto* ctx = static_cast<MoverContext*>(p_context);
	const b3BodyId body_id = b3Shape_GetBody(p_shape_id);
	Box3DShapedObjectImpl3D* object = nullptr;
	if (!should_report(b3Body_GetUserData(body_id), *ctx->filter, object)) {
		return true;
	}

	const Vector3 normal = b3_to_godot(p_plane->plane.normal);
	if (normal.length_squared() < 1e-8f) {
		return true;
	}
	// Keep the plane the motion drives into hardest. A body standing on the floor
	// and stepping sideways gets a floor plane whose opposition is ~0, so the
	// floor never masks the wall in front -- the case the generic path needed a
	// whole re-cast loop to handle.
	const float opposition = -(float)normal.dot(ctx->motion_dir);
	if (!ctx->has_plane || opposition > ctx->opposition) {
		ctx->has_plane = true;
		ctx->opposition = opposition;
		ctx->normal = normal;
		ctx->point = b3_to_godot(p_plane->point);
		ctx->shape_id = p_shape_id;
	}

	if (ctx->count < MOVER_MAX_PLANES) {
		ctx->planes[ctx->count++] = p_plane->plane;
	}
	return true;
}

// How much of p_motion a capsule can travel before it reaches p_plane, as a
// fraction of the whole motion. Box3D's plane convention is
// separation = dot(normal, point) - offset (see hull.c), and everything here is
// in the mover's origin-relative frame, so no conversion is involved.
//
// This replaces the obvious use of b3SolvePlanes. That solves a POSITION -- it
// hands back a delta already slid along the floor -- and projecting that onto the
// motion line under-reports how far along the line the body got, which measured
// as walk speed collapsing from 7.5 to 0.42 m/s. Godot's contract wants distance
// along the motion before something blocks it, and does its own sliding.
float plane_travel_fraction(const b3Plane& p_plane, const b3Capsule& p_mover, const b3Vec3& p_motion) {
	const float closing = -(p_plane.normal.x * p_motion.x + p_plane.normal.y * p_motion.y + p_plane.normal.z * p_motion.z);
	if (closing <= 1e-6f) {
		// Parallel to the plane, or moving away from it. The floor underfoot while
		// walking along it lands here, which is exactly why walking is not slowed.
		return 1.0f;
	}
	const float d1 = p_plane.normal.x * p_mover.center1.x + p_plane.normal.y * p_mover.center1.y + p_plane.normal.z * p_mover.center1.z;
	const float d2 = p_plane.normal.x * p_mover.center2.x + p_plane.normal.y * p_mover.center2.y + p_plane.normal.z * p_mover.center2.z;
	const float separation = MIN(d1, d2) - p_plane.offset - p_mover.radius;
	if (separation <= 0.0f) {
		return 0.0f;
	}
	return CLAMP(separation / closing, 0.0f, 1.0f);
}

// Builds the b3Capsule a Godot capsule shape describes, in world space but
// expressed relative to p_origin the way the mover API wants it.
bool build_mover(const Box3DShapeImpl3D* p_shape, const Transform3D& p_transform,
		const Vector3& p_origin, b3Capsule& r_mover) {
	if (p_shape == nullptr || p_shape->get_type() != PhysicsServer3D::SHAPE_CAPSULE) {
		return false;
	}
	const auto* capsule = static_cast<const Box3DCapsuleShapeImpl3D*>(p_shape);
	const float radius = (float)capsule->get_radius();
	const float half_seg = MAX(0.0f, (float)capsule->get_height() * 0.5f - radius);
	r_mover.center1 = godot_to_b3(p_transform.xform(Vector3(0, half_seg, 0)) - p_origin);
	r_mover.center2 = godot_to_b3(p_transform.xform(Vector3(0, -half_seg, 0)) - p_origin);
	r_mover.radius = radius;
	return true;
}

} // namespace

bool Box3DPhysicsDirectSpaceState3D::_intersect_ray(
		const Vector3& p_from,
		const Vector3& p_to,
		uint32_t p_collision_mask,
		bool p_collide_with_bodies,
		bool p_collide_with_areas,
		bool p_hit_from_inside,
		bool p_hit_back_faces,
		bool p_pick_ray,
		PhysicsServer3DExtensionRayResult* p_result) {
	ERR_FAIL_NULL_V(space, false);

	Box3DQueryFilter3D filter(p_collision_mask, p_collide_with_bodies, p_collide_with_areas);
	filter.direct_state = this;

	RayContext context;
	context.filter = &filter;
	context.hit_from_inside = p_hit_from_inside;

	const b3Vec3 origin = godot_to_b3(p_from);
	const b3Vec3 translation = godot_to_b3(p_to - p_from);

	b3World_CastRay(space->get_world_id(), origin, translation, filter.filter, cast_result_fcn, &context);

	if (!context.has_hit) {
		return false;
	}

	const b3BodyId body_id = b3Shape_GetBody(context.shape_id);
	auto* object = static_cast<Box3DShapedObjectImpl3D*>(b3Body_GetUserData(body_id));
	if (object == nullptr) {
		return false;
	}

	p_result->position = b3_to_godot(context.point);
	p_result->normal = b3_to_godot(context.normal);
	p_result->rid = object->get_rid();
	p_result->collider_id = object->get_instance_id();
	p_result->shape = 0;
	return true;
}

int32_t Box3DPhysicsDirectSpaceState3D::_intersect_point(
		const Vector3& p_position,
		uint32_t p_collision_mask,
		bool p_collide_with_bodies,
		bool p_collide_with_areas,
		PhysicsServer3DExtensionShapeResult* p_results,
		int32_t p_max_results) {
	ERR_FAIL_NULL_V(space, 0);

	Box3DQueryFilter3D filter(p_collision_mask, p_collide_with_bodies, p_collide_with_areas);
	filter.direct_state = this;

	const b3Vec3 point = godot_to_b3(p_position);
	b3ShapeProxy proxy;
	proxy.points = &point;
	proxy.count = 1;
	proxy.radius = 0.0f;

	OverlapContext context;
	context.filter = &filter;
	context.results = p_results;
	context.max_results = p_max_results;

	b3World_OverlapShape(space->get_world_id(), b3Vec3_zero, &proxy, filter.filter, overlap_result_fcn, &context);

	return context.count;
}

int32_t Box3DPhysicsDirectSpaceState3D::_intersect_shape(
		const RID& p_shape_rid,
		const Transform3D& p_transform,
		const Vector3& p_motion,
		double p_margin,
		uint32_t p_collision_mask,
		bool p_collide_with_bodies,
		bool p_collide_with_areas,
		PhysicsServer3DExtensionShapeResult* p_results,
		int32_t p_max_results) {
	ERR_FAIL_NULL_V(space, 0);

	Box3DShapeImpl3D* shape = Box3DPhysicsServer3D::get_singleton()->get_shape(p_shape_rid);
	ERR_FAIL_NULL_V(shape, 0);

	const Box3DShapeProxy3D shape_proxy(shape, p_transform);
	if (!shape_proxy.is_supported()) {
		return 0;
	}

	Box3DQueryFilter3D filter(p_collision_mask, p_collide_with_bodies, p_collide_with_areas);
	filter.direct_state = this;

	OverlapContext context;
	context.filter = &filter;
	context.results = p_results;
	context.max_results = p_max_results;

	b3World_OverlapShape(space->get_world_id(), b3Vec3_zero, &shape_proxy.get_proxy(), filter.filter, overlap_result_fcn, &context);

	return context.count;
}

bool Box3DPhysicsDirectSpaceState3D::_cast_motion(
		const RID& p_shape_rid,
		const Transform3D& p_transform,
		const Vector3& p_motion,
		double p_margin,
		uint32_t p_collision_mask,
		bool p_collide_with_bodies,
		bool p_collide_with_areas,
		float* p_closest_safe,
		float* p_closest_unsafe,
		PhysicsServer3DExtensionShapeRestInfo* p_info) {
	ERR_FAIL_NULL_V(space, false);

	Box3DShapeImpl3D* shape = Box3DPhysicsServer3D::get_singleton()->get_shape(p_shape_rid);
	ERR_FAIL_NULL_V(shape, false);

	const Box3DShapeProxy3D shape_proxy(shape, p_transform);
	if (!shape_proxy.is_supported()) {
		*p_closest_safe = 1.0;
		*p_closest_unsafe = 1.0;
		return false;
	}

	Box3DQueryFilter3D filter(p_collision_mask, p_collide_with_bodies, p_collide_with_areas);
	filter.direct_state = this;

	RayContext context;
	context.filter = &filter;

	b3World_CastShape(space->get_world_id(), b3Vec3_zero, &shape_proxy.get_proxy(), godot_to_b3(p_motion), filter.filter, cast_result_fcn, &context);

	if (!context.has_hit) {
		*p_closest_safe = 1.0;
		*p_closest_unsafe = 1.0;
		return false;
	}

	*p_closest_safe = context.fraction;
	*p_closest_unsafe = context.fraction;
	return true;
}

bool Box3DPhysicsDirectSpaceState3D::_collide_shape(
		const RID& p_shape_rid,
		const Transform3D& p_transform,
		const Vector3& p_motion,
		double p_margin,
		uint32_t p_collision_mask,
		bool p_collide_with_bodies,
		bool p_collide_with_areas,
		void* p_results,
		int32_t p_max_results,
		int32_t* p_result_count) {
	*p_result_count = 0;
	ERR_FAIL_NULL_V(space, false);
	if (p_max_results <= 0) {
		return false;
	}

	Box3DShapeImpl3D* shape = Box3DPhysicsServer3D::get_singleton()->get_shape(p_shape_rid);
	ERR_FAIL_NULL_V(shape, false);

	const Box3DShapeProxy3D shape_proxy(shape, p_transform);
	if (!shape_proxy.is_supported()) {
		return false;
	}

	Box3DQueryFilter3D filter(p_collision_mask, p_collide_with_bodies, p_collide_with_areas);
	filter.direct_state = this;

	CollideShapeContext context;
	context.filter = &filter;
	context.query_proxy = &shape_proxy.get_proxy();
	context.results = static_cast<Vector3*>(p_results);
	context.max_results = p_max_results;

	b3World_OverlapShape(
			space->get_world_id(), b3Vec3_zero, &shape_proxy.get_proxy(), filter.filter, collide_shape_result_fcn, &context);

	*p_result_count = context.count;
	return context.count > 0;
}

bool Box3DPhysicsDirectSpaceState3D::_rest_info(
		const RID& p_shape_rid,
		const Transform3D& p_transform,
		const Vector3& p_motion,
		double p_margin,
		uint32_t p_collision_mask,
		bool p_collide_with_bodies,
		bool p_collide_with_areas,
		PhysicsServer3DExtensionShapeRestInfo* p_info) {
	ERR_FAIL_NULL_V(space, false);

	Box3DShapeImpl3D* shape = Box3DPhysicsServer3D::get_singleton()->get_shape(p_shape_rid);
	ERR_FAIL_NULL_V(shape, false);

	const Box3DShapeProxy3D shape_proxy(shape, p_transform);
	if (!shape_proxy.is_supported()) {
		return false;
	}

	Box3DQueryFilter3D filter(p_collision_mask, p_collide_with_bodies, p_collide_with_areas);
	filter.direct_state = this;

	RayContext context;
	context.filter = &filter;

	b3World_CastShape(space->get_world_id(), b3Vec3_zero, &shape_proxy.get_proxy(), godot_to_b3(p_motion), filter.filter, cast_result_fcn, &context);

	if (!context.has_hit) {
		return false;
	}

	const b3BodyId body_id = b3Shape_GetBody(context.shape_id);
	auto* object = static_cast<Box3DShapedObjectImpl3D*>(b3Body_GetUserData(body_id));
	if (object == nullptr) {
		return false;
	}

	p_info->point = b3_to_godot(context.point);
	p_info->normal = b3_to_godot(context.normal);
	p_info->rid = object->get_rid();
	p_info->collider_id = object->get_instance_id();
	p_info->shape = 0;

	auto* body = dynamic_cast<Box3DBodyImpl3D*>(object);
	if (body != nullptr) {
		p_info->linear_velocity = body->get_linear_velocity();
	}

	return true;
}

Vector3 Box3DPhysicsDirectSpaceState3D::_get_closest_point_to_object_volume(const RID& p_object, const Vector3& p_point) const {
	Box3DShapedObjectImpl3D* object = Box3DPhysicsServer3D::get_singleton()->get_body(p_object);
	if (object == nullptr) {
		object = Box3DPhysicsServer3D::get_singleton()->get_area(p_object);
	}
	if (object == nullptr || !object->has_body_id()) {
		return p_point;
	}

	b3Vec3 result_point{};
	b3Body_GetClosestPoint(object->get_body_id(), &result_point, godot_to_b3(p_point));
	return b3_to_godot(result_point);
}

bool Box3DPhysicsDirectSpaceState3D::test_body_motion(
		Box3DShapedObjectImpl3D& p_body,
		const Transform3D& p_transform,
		const Vector3& p_motion,
		double p_margin,
		int32_t p_max_collisions,
		bool p_recovery_as_collision,
		PhysicsServer3DExtensionMotionResult* p_result) const {
	ERR_FAIL_NULL_V(space, false);

	p_result->travel = Vector3();
	p_result->remainder = p_motion;
	p_result->collision_depth = 0.0f;
	p_result->collision_safe_fraction = 1.0f;
	p_result->collision_unsafe_fraction = 1.0f;
	p_result->collision_count = 0;

	if (p_body.get_shape_count() == 0) {
		p_result->travel = p_motion;
		p_result->remainder = Vector3();
		return false;
	}

	Box3DShapeImpl3D* first_shape = p_body.get_shape(0);
	if (first_shape == nullptr) {
		p_result->travel = p_motion;
		p_result->remainder = Vector3();
		return false;
	}

	Box3DQueryFilter3D filter;
	filter.set_collision_mask(p_body.get_collision_mask());
	filter.exclude.insert(p_body.get_rid());

	if (box3d_use_character_mover()) {
		const Transform3D mover_transform = p_transform * p_body.get_shape_transform(0);
		b3Capsule mover;
		if (build_mover(first_shape, mover_transform, mover_transform.origin, mover)) {
			return _test_mover_motion(
					p_body, mover, mover_transform.origin, p_motion, filter, p_max_collisions, p_result);
		}
	}

	// A body resting on the ground settles a linear slop *inside* the surface, so a full-size
	// sweep from that pose starts in contact. Shrinking the query shape past the slop lets
	// most such sweeps report an ordinary touch instead. Godot's own safe_margin wins when it
	// asks for more.
	const float shrink = MAX((float)p_margin, 1.5f * B3_LINEAR_SLOP);
	const Transform3D shape_transform = p_transform * p_body.get_shape_transform(0);
	const Vector3 motion_dir = p_motion.normalized();

	// Whatever the shrink leaves overlapping still reports the reverse of the cast direction
	// as its normal, which reads as a head-on wall in every direction -- the floor underfoot
	// would block a sideways step. So each contact's normal is recovered (witness_normal) and
	// contacts the motion is not driving into are skipped by excluding that collider and
	// casting again, which also uncovers the real blocker hiding behind the floor.
	RayContext context;
	Vector3 normal;
	Box3DShapedObjectImpl3D* other = nullptr;
	const int max_attempts = 4;
	for (int attempt = 0; attempt < max_attempts; attempt++) {
		const Box3DShapeProxy3D shape_proxy(first_shape, shape_transform, shrink);
		if (!shape_proxy.is_supported()) {
			p_result->travel = p_motion;
			p_result->remainder = Vector3();
			return false;
		}

		context = RayContext();
		context.filter = &filter;
		b3World_CastShape(space->get_world_id(), b3Vec3_zero, &shape_proxy.get_proxy(), godot_to_b3(p_motion), filter.filter, cast_result_fcn, &context);

		if (!context.has_hit) {
			p_result->travel = p_motion;
			p_result->remainder = Vector3();
			return false;
		}

		const b3BodyId body_id = b3Shape_GetBody(context.shape_id);
		other = static_cast<Box3DShapedObjectImpl3D*>(b3Body_GetUserData(body_id));

		normal = b3_to_godot(context.normal);
		Vector3 recovered;
		if (context.fraction <= 1e-4f && witness_normal(space->get_world_id(), b3_to_godot(context.point), first_shape, shape_transform, context.shape_id, recovered)) {
			normal = recovered;
		}
		if (normal.length_squared() < 1e-8f) {
			// Nothing could describe this contact: the cores overlap as well. Reporting the
			// reverse of the motion at least hands the caller a normalized vector to slide
			// along instead of tripping its asserts.
			normal = motion_dir == Vector3() ? Vector3(0, 1, 0) : -motion_dir;
		}

		const bool separating = motion_dir != Vector3() && normal.dot(motion_dir) > -1e-4f;
		if (!separating || other == nullptr) {
			break;
		}
		// Drop just the settled contact's SHAPE, not the collider: excluding the RID took
		// every shape of a multi-shape body with it, so a capsule standing on a wreck's
		// floor shape walked straight through that wreck's wall shapes.
		filter.exclude_shapes.insert(b3StoreShapeId(context.shape_id));
		if (attempt == max_attempts - 1) {
			// Out of attempts with nothing blocking found; treat the motion as unobstructed
			// rather than reporting a contact the body is moving away from.
			p_result->travel = p_motion;
			p_result->remainder = Vector3();
			return false;
		}
	}

	p_result->travel = p_motion * context.fraction;
	p_result->remainder = p_motion * (1.0f - context.fraction);
	p_result->collision_safe_fraction = context.fraction;
	p_result->collision_unsafe_fraction = context.fraction;

	if (other != nullptr && p_max_collisions > 0) {
		PhysicsServer3DExtensionMotionCollision& collision = p_result->collisions[0];
		collision.position = b3_to_godot(context.point);
		collision.normal = normal;
		collision.collider = other->get_rid();
		collision.collider_id = other->get_instance_id();
		collision.collider_shape = 0;
		collision.depth = 0.0f;
		p_result->collision_count = 1;
	}

	return true;
}

// Capsule bodies -- every CharacterBody3D in practice -- go through Box3D's own
// character API instead of the generic shape cast above. Two calls, in the order
// the docs prescribe: sweep with b3World_CastMover for how far the motion gets,
// then b3World_CollideMover AT THE STOP POSE for what it ran into. Collecting
// planes at the start pose instead would miss a wall the body has not reached
// yet, which is most of them.
bool Box3DPhysicsDirectSpaceState3D::_test_mover_motion(
		Box3DShapedObjectImpl3D& p_body,
		const b3Capsule& p_mover,
		const Vector3& p_origin,
		const Vector3& p_motion,
		const Box3DQueryFilter3D& p_filter,
		int32_t p_max_collisions,
		PhysicsServer3DExtensionMotionResult* p_result) const {
	const Vector3 motion_dir = p_motion.normalized();
	const real_t motion_len = p_motion.length();
	if (motion_dir == Vector3() || motion_len <= 0.0f) {
		p_result->travel = p_motion;
		p_result->remainder = Vector3();
		return false;
	}

	const b3WorldId world_id = space->get_world_id();
	const b3Pos origin = godot_to_b3(p_origin);

	// STEP 1 -- what is the mover already touching? These planes are what stops a
	// body resting on the ground from sinking through it. b3World_CastMover alone
	// will not: it lets the capsule encroach by up to a slop to reduce clipping, so
	// a sub-slop downward step reads as unobstructed and the body creeps down a
	// little every tick.
	//
	// Both this pose and the stop pose below are needed, and cheaper orderings do
	// not work. Gathering planes only where the sweep stops loses the floor while a
	// body is pressed against a wall -- it sinks and then walks through, measured.
	MoverContext at_start;
	at_start.filter = &p_filter;
	at_start.motion_dir = motion_dir;
	b3Capsule here = p_mover;
	b3World_CollideMover(world_id, origin, &here, p_filter.filter, mover_plane_fcn, &at_start);

	// STEP 2 -- how much of the motion survives those planes.
	real_t plane_fraction = 1.0f;
	const b3Vec3 motion_b3 = godot_to_b3(p_motion);
	for (int i = 0; i < at_start.count; i++) {
		plane_fraction = MIN(plane_fraction, (real_t)plane_travel_fraction(at_start.planes[i], p_mover, motion_b3));
	}

	// STEP 3 -- and how far the sweep gets before reaching something not yet touched.
	MoverContext sweep;
	sweep.filter = &p_filter;
	sweep.motion_dir = motion_dir;
	const real_t cast_fraction = CLAMP((real_t)b3World_CastMover(world_id, origin, &p_mover,
											   godot_to_b3(p_motion), p_filter.filter, mover_filter_fcn, &sweep),
			(real_t)0.0, (real_t)1.0);

	const real_t fraction = MIN(plane_fraction, cast_fraction);

	// Where to read the contact normal. A plane the body is ALREADY touching answers
	// for free -- it was gathered above -- so the second gather is reserved for the
	// case that genuinely needs it: the sweep ran into something not touched yet and
	// nothing underfoot opposes the motion. Gathering unconditionally instead costs
	// roughly 3x on the benchmark, because collecting planes against a height field
	// is the expensive half of this whole path.
	MoverContext* contact = &at_start;
	Vector3 contact_origin = p_origin;
	MoverContext at_stop;
	const bool start_opposes = at_start.has_plane && at_start.opposition > 1e-4f;
	if (!start_opposes && cast_fraction < 0.999f) {
		contact_origin = p_origin + p_motion * fraction;
		at_stop.filter = &p_filter;
		at_stop.motion_dir = motion_dir;
		b3Capsule stopped = p_mover;
		b3World_CollideMover(
				world_id, godot_to_b3(contact_origin), &stopped, p_filter.filter, mover_plane_fcn, &at_stop);
		contact = &at_stop;
	}

	// Nothing opposing the motion: a body walking along the floor it stands on
	// reaches here, and that is the point of the mover -- the floor does not stop
	// it. Reporting a contact anyway would make Godot slide against its own ground.
	const bool blocked = contact->has_plane && contact->opposition > 1e-4f;
	if (fraction >= 1.0f && !blocked) {
		p_result->travel = p_motion;
		p_result->remainder = Vector3();
		return false;
	}
	if (!blocked) {
		p_result->travel = p_motion * fraction;
		p_result->remainder = p_motion * (1.0f - fraction);
		p_result->collision_safe_fraction = fraction;
		p_result->collision_unsafe_fraction = fraction;
		return false;
	}

	p_result->travel = p_motion * fraction;
	p_result->remainder = p_motion * (1.0f - fraction);
	p_result->collision_safe_fraction = fraction;
	p_result->collision_unsafe_fraction = fraction;

	if (p_max_collisions > 0) {
		Box3DShapedObjectImpl3D* other = nullptr;
		if (B3_IS_NON_NULL(contact->shape_id)) {
			other = static_cast<Box3DShapedObjectImpl3D*>(b3Body_GetUserData(b3Shape_GetBody(contact->shape_id)));
		}
		PhysicsServer3DExtensionMotionCollision& collision = p_result->collisions[0];
		// Plane points come back relative to the origin they were gathered at.
		collision.position = contact_origin + contact->point;
		collision.normal = contact->normal;
		collision.collider = other != nullptr ? other->get_rid() : RID();
		collision.collider_id = other != nullptr ? other->get_instance_id() : ObjectID();
		collision.collider_shape = 0;
		collision.depth = 0.0f;
		p_result->collision_count = 1;
	}

	return true;
}
