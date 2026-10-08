#include "box3d_heightmap_shape_impl_3d.hpp"

#include <box3d/collision.h>

#include <limits>

Box3DHeightMapShapeImpl3D::~Box3DHeightMapShapeImpl3D() {
	if (height_field != nullptr) {
		b3DestroyHeightField(height_field);
		height_field = nullptr;
	}
}

Variant Box3DHeightMapShapeImpl3D::get_data() const {
	Dictionary data;
	data["width"] = width;
	data["depth"] = depth;
	data["heights"] = heights;
	return data;
}

void Box3DHeightMapShapeImpl3D::set_data(const Variant& p_data) {
	ERR_FAIL_COND(p_data.get_type() != Variant::DICTIONARY);

	const Dictionary data = p_data;

	const Variant maybe_heights = data.get("heights", Variant());
	ERR_FAIL_COND(maybe_heights.get_type() != Variant::PACKED_FLOAT32_ARRAY && maybe_heights.get_type() != Variant::PACKED_FLOAT64_ARRAY);

	const Variant maybe_width = data.get("width", Variant());
	ERR_FAIL_COND(maybe_width.get_type() != Variant::INT);

	const Variant maybe_depth = data.get("depth", Variant());
	ERR_FAIL_COND(maybe_depth.get_type() != Variant::INT);

	if (maybe_heights.get_type() == Variant::PACKED_FLOAT64_ARRAY) {
		const PackedFloat64Array heights64 = maybe_heights;
		heights.resize(heights64.size());
		for (int i = 0; i < heights64.size(); i++) {
			heights.set(i, (float)heights64[i]);
		}
	} else {
		heights = maybe_heights;
	}

	width = maybe_width;
	depth = maybe_depth;

	_rebuild();
}

void Box3DHeightMapShapeImpl3D::_rebuild() {
	if (height_field != nullptr) {
		b3DestroyHeightField(height_field);
		height_field = nullptr;
	}

	if (width < 2 || depth < 2 || heights.size() != width * depth) {
		return;
	}

	float min_height = heights[0];
	float max_height = heights[0];
	for (int i = 0; i < heights.size(); i++) {
		const float h = heights[i];
		if (h < min_height) {
			min_height = h;
		}
		if (h > max_height) {
			max_height = h;
		}
	}

	if (min_height == max_height) {
		// Box3D requires a non-degenerate quantization range.
		max_height = min_height + 0.001f;
	}

	b3HeightFieldDef def = {};
	def.heights = heights.ptrw();
	def.materialIndices = nullptr;
	// Grid spacing is baked in here -- Box3D has no runtime scale for a height field. See
	// get_height_field(), which rebuilds when a differently scaled body asks for one.
	def.scale = b3Vec3{(float)built_scale.x, (float)built_scale.y, (float)built_scale.z};
	def.countX = width;
	def.countZ = depth;
	def.globalMinimumHeight = min_height;
	def.globalMaximumHeight = max_height;
	def.clockwiseWinding = false;

	height_field = b3CreateHeightField(&def);

	const float span_x = (float)(width - 1) * (float)built_scale.x;
	const float span_z = (float)(depth - 1) * (float)built_scale.z;
	aabb = AABB(
		Vector3(-span_x * 0.5f, min_height * (float)built_scale.y, -span_z * 0.5f),
		Vector3(span_x, (max_height - min_height) * (float)built_scale.y, span_z)
	);
}

const b3HeightFieldData* Box3DHeightMapShapeImpl3D::get_height_field(const Vector3& p_scale) {
	// All components must be positive (b3HeightFieldDef contract) and a mirrored scale has
	// no meaning for a grid, so magnitudes are used.
	const Vector3 wanted(
			MAX((real_t)Math::abs(p_scale.x), (real_t)0.0001),
			MAX((real_t)Math::abs(p_scale.y), (real_t)0.0001),
			MAX((real_t)Math::abs(p_scale.z), (real_t)0.0001));
	if (height_field != nullptr && built_scale.is_equal_approx(wanted)) {
		return height_field;
	}
	built_scale = wanted;
	_rebuild();
	return height_field;
}
