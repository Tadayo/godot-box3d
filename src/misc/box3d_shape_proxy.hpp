#pragma once

#include <godot_cpp/templates/local_vector.hpp>
#include <godot_cpp/variant/transform3d.hpp>

#include <box3d/types.h>

using namespace godot;

class Box3DShapeImpl3D;

// Builds a b3ShapeProxy (point cloud + radius, as used by b3World_OverlapShape/CastShape)
// matching a given Godot Shape3D: sphere -> 1 point + radius; capsule -> 2 points +
// radius; box/convex hull -> N hull points + 0 radius, all transformed into p_transform's
// space. Mesh/heightfield/world-boundary shapes have no finite point-cloud representation
// usable with Box3D's GJK-based queries, so they report is_supported() == false; callers
// should fall back to a different strategy (e.g. AABB overlap) for those shape types.
class Box3DShapeProxy3D {
public:
	// p_shrink pulls every surface inward by that many metres (sphere/capsule radius,
	// box/cylinder extents). A shape cast that starts exactly in contact -- which is every
	// tick for a body resting on the ground, and Box3D lets resting bodies settle a linear
	// slop *inside* the surface -- reports fraction 0 with a zero normal, which is useless
	// to a caller that needs to know what it is standing on. Shrinking the query shape past
	// the slop turns that degenerate start into an ordinary touch with a real normal.
	// Convex hulls are not shrunk: there is no cheap correct inset for an arbitrary hull.
	Box3DShapeProxy3D(const Box3DShapeImpl3D* p_shape, const Transform3D& p_transform, float p_shrink = 0.0f);

	bool is_supported() const { return supported; }

	const b3ShapeProxy& get_proxy() const { return proxy; }

private:
	LocalVector<b3Vec3> points;
	b3ShapeProxy proxy{};
	bool supported = false;
};
