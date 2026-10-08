#pragma once

// Extension-wide lifecycle hooks.

void box3d_initialize();

void box3d_deinitialize();

// The physics/box3d/worker_count setting, or the detected core count when it is 0 (auto).
int box3d_worker_count();

// physics/box3d/use_character_mover: route capsule CharacterBody3D motion through Box3D's
// character-mover API rather than the generic shape cast. Off by default -- more faithful,
// measurably slower. Read once.
bool box3d_use_character_mover();
