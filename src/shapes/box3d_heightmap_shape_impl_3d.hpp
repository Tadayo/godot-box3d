#pragma once

#include "box3d_shape_impl_3d.hpp"

#include <godot_cpp/variant/packed_float32_array.hpp>
#include <godot_cpp/variant/vector3.hpp>

#include <box3d/types.h>

// HeightMapShape3D -> a b3HeightFieldData* built via b3CreateHeightField() and shared by
// every attaching Box3DShapeInstance3D. Static bodies only (matches Box3D's own
// restriction).
//
// Two things Box3D does differently to Godot, both handled here:
//
// 1. GRID ORIGIN. Box3D's height field spans [0, countX-1] x [0, countZ-1] from its body
//    origin; Godot (and godot-jolt) centre the grid on the shape origin. b3CreateHeightField
//    has no origin field and b3CreateHeightFieldShape takes no transform, so the field
//    cannot be moved off the body origin -- the body frame is relocated by
//    centre_offset() instead, and every other shape on that body is compensated. See
//    Box3DShapedObjectImpl3D::height_field_offset.
//
// 2. SCALE. Box3D bakes grid spacing into the field at build time (b3HeightFieldDef::scale),
//    so a body scaled in the editor needs its own field. The field is therefore built
//    lazily per scale and rebuilt when a different one asks for it -- one field per shape
//    resource, which is the common case (one terrain, one scale).
class Box3DHeightMapShapeImpl3D final : public Box3DShapeImpl3D {
public:
	~Box3DHeightMapShapeImpl3D() override;

	ShapeType get_type() const override { return PhysicsServer3D::SHAPE_HEIGHTMAP; }

	Variant get_data() const override;

	void set_data(const Variant& p_data) override;

	AABB get_aabb() const override { return aabb; }

	// Field built for the given grid spacing. Rebuilds if the last caller wanted another.
	const b3HeightFieldData* get_height_field(const Vector3& p_scale);

	// Translation from the Godot shape origin to Box3D's grid corner, at that same scale.
	Vector3 centre_offset(const Vector3& p_scale) const {
		return Vector3(
				-(real_t)(width - 1) * 0.5f * p_scale.x,
				0.0f,
				-(real_t)(depth - 1) * 0.5f * p_scale.z);
	}

	int get_width() const { return width; }

	int get_depth() const { return depth; }

private:
	void _rebuild();

	PackedFloat32Array heights;
	int width = 0;
	int depth = 0;
	b3HeightFieldData* height_field = nullptr;
	Vector3 built_scale = Vector3(1, 1, 1);
	AABB aabb;
};
