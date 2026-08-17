#include "box3d_globals.hpp"

#include "box3d_cpu_topology.hpp"

#include <box3d/constants.h>

#include <godot_cpp/classes/project_settings.hpp>
#include <godot_cpp/variant/dictionary.hpp>

#include <algorithm>

using namespace godot;

namespace {
const char *WORKER_COUNT_SETTING = "physics/box3d/worker_count";
const char *CHARACTER_MOVER_SETTING = "physics/box3d/use_character_mover";
} // namespace

void box3d_initialize() {
	ProjectSettings *settings = ProjectSettings::get_singleton();
	if (!settings->has_setting(WORKER_COUNT_SETTING)) {
		settings->set_setting(WORKER_COUNT_SETTING, 0);
	}
	settings->set_initial_value(WORKER_COUNT_SETTING, 0);
	// Spaces capture the count at creation, so a change only applies after a restart.
	settings->set_restart_if_changed(WORKER_COUNT_SETTING, true);
	Dictionary info;
	info["name"] = WORKER_COUNT_SETTING;
	info["type"] = Variant::INT;
	info["hint"] = PROPERTY_HINT_RANGE;
	info["hint_string"] = "0,32,1";
	settings->add_property_info(info);

	// Route capsule CharacterBody3D motion through Box3D's character-mover API
	// (b3World_CollideMover + b3World_CastMover) instead of the generic shape cast.
	// Off by default: measured against 30 characters on a 4 km height field it
	// tracks Godot Physics/Jolt exactly -- walk, jump and ski all to four decimals,
	// where the generic path drifts slightly on ski friction -- but costs about
	// 1.9x, because gathering contact planes every tick is more work than one shape
	// cast. Turn it on where fidelity matters more than throughput.
	if (!settings->has_setting(CHARACTER_MOVER_SETTING)) {
		settings->set_setting(CHARACTER_MOVER_SETTING, false);
	}
	settings->set_initial_value(CHARACTER_MOVER_SETTING, false);
	Dictionary mover_info;
	mover_info["name"] = CHARACTER_MOVER_SETTING;
	mover_info["type"] = Variant::BOOL;
	settings->add_property_info(mover_info);
}

bool box3d_use_character_mover() {
	static const bool enabled = (bool)ProjectSettings::get_singleton()->get_setting_with_override(
			CHARACTER_MOVER_SETTING);
	return enabled;
}

void box3d_deinitialize() {
	// No global Box3D shutdown is required.
}

int box3d_worker_count() {
	static const int count = []() {
		const int setting = (int)ProjectSettings::get_singleton()->get_setting_with_override(
				WORKER_COUNT_SETTING);
		return setting > 0 ? std::clamp(setting, 1, B3_MAX_WORKERS) : box3d_default_worker_count();
	}();
	return count;
}
