
#include "common.hpp"

void ReleaseDB(Context&) { SQ3(sqlite3_shutdown()); }

void ReleaseRender(Context& ctx)
{
	for (Model& m : ctx.models)
		UnloadModel(m);
	ctx.models.clear();
	UnloadShader(ctx.basicShader);
	UnloadShader(ctx.postFxShader);
	CloseWindow();
}

void ReleasePhysics(Context& ctx, GameMap& map)
{
	map.cubes.clear(CleanCube, ctx, map);
	map.spheres.clear(CleanSphere, ctx, map);
	for (auto*& rb : map.staticBodies)
		CleanRigidBody(ctx, map, rb);
	map.staticBodies.clear();
	delete map.dynamics_world;
	delete map.solver;
	delete map.overlapping_pair_cache;
	delete map.dispatcher;
	delete map.collision_configuration;
}

void ReleasePhysics(Context& ctx)
{
	auto& player = ctx.player;
	CleanRigidBody(ctx, *player.map, player.player);
	for (const auto& kv : ctx.gameMaps)
		ReleasePhysics(ctx, *kv.second);
	ctx.gameMaps.clear();
}

void Release(Context& ctx)
{
	ReleasePhysics(ctx);
	ReleaseRender(ctx);
	ReleaseDB(ctx);
	Tcl_DeleteInterp(ctx.tcl);
}
