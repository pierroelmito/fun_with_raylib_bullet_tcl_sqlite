
#include "common.hpp"

#include <algorithm>

void AddParticles(const Context& ctx, GameMap& map, Vector3 pos, Vector3 normal, float radius, int ptype, int count)
{
	for (int i = 0; i < count; ++i) {
		const float dx = float(GetRandomValue(-1000, 1000)) / 1000.0f;
		const float dy = float(GetRandomValue(-1000, 1000)) / 1000.0f;
		const float dz = float(GetRandomValue(-1000, 1000)) / 1000.0f;
		map.particles.push_back({ pos, Vector3Add(normal, Vector3Scale(Vector3Normalize({ dx, dy, dz }), radius)), ctx.gtime, ptype });
	}
}

bool MoveBullet(const Context& ctx, GameMap& map, const Particle& b)
{
	const auto cpos = CheckRayCollision(ctx, map.dynamics_world, b, BulletSpeed);
	if (!cpos)
		return false;

	const Vector3 pdir = Reflect(Vector3Normalize(b.dir), cpos->normal);
	AddParticles(ctx, map, cpos->pos, pdir, 0.5f, 0, 10);

	if (!cpos->st) {
		const ID id { .v = uint32_t(cpos->index1) };
		const float force = 100.0f;
		if (id.t == 0) {
			auto* rb = map.cubes.at(id.i).rb;
			rb->setActivationState(ACTIVE_TAG);
			rb->applyImpulse(force * toBtV3(b.dir), { 0, -0.1, 0 });
		} else if (id.t == 1) {
			auto* rb = map.spheres.at(id.i).rb;
			rb->setActivationState(ACTIVE_TAG);
			rb->applyImpulse(force * toBtV3(b.dir).normalize(), { 0, -0.1, 0 });
		}
	}

	return true;
}

void UpdateBullets(Context& ctx, GameMap& map)
{
	map.bullets.erase(std::remove_if(map.bullets.begin(), map.bullets.end(), [&](const Particle& b) { return MoveBullet(ctx, map, b); }), map.bullets.end());
}

void UpdateMap(Context& ctx, GameMap& map)
{
	map.dynamics_world->stepSimulation(1.0 / float(ctx.FPS), 1);
	UpdateBullets(ctx, map);
}

void UpdatePlayer(Context& ctx, const Camera& cam, Player& player)
{
	GameMap& map = *player.map;

	if (player.grapple) {
		const Particle& grapple = std::get<Particle>(*player.grapple);
		Vector3 pos {};
		auto cpos = CheckRayCollision(ctx, map.dynamics_world, grapple, GrappleSpeed, &pos);
		AddParticles(ctx, map, pos, grapple.dir, 0.1f, 1, 3);
		if (cpos) {
			const float force = -200.0f;
			const ID id { .v = uint32_t(cpos->index1) };
			const auto delta = (toBtV3(cpos->pos) - toBtV3(cam.position)).normalize();
			AddParticles(ctx, map, cpos->pos, cpos->normal, 0.5f, 1, 5);
			if (!cpos->st) {
				if (id.t == 0) {
					auto* rb = map.cubes.at(id.i).rb;
					rb->setActivationState(ACTIVE_TAG);
					rb->applyImpulse(force * delta, { 0, -0.1, 0 });
				} else if (id.t == 1) {
					auto* rb = map.spheres.at(id.i).rb;
					rb->setActivationState(ACTIVE_TAG);
					rb->applyImpulse(force * delta, { 0, -0.1, 0 });
				}
			} else {
				player.player->applyCentralImpulse(-force * delta);
			}
			player.grapple = {};
		}
	}
}

void UpdatePlayerInputs(Context& ctx, const Camera& cam, Player& player)
{
	GameMap& map = *player.map;

	{
		const float maxA = 0.99f * 0.5f * PI;
		const float camSpeed = 0.003f;
		player.camAngles = Vector2Add(player.camAngles, Vector2Multiply(GetMouseDelta(), { -camSpeed, camSpeed }));
		player.camAngles.y = std::max(-maxA, std::min(maxA, player.camAngles.y));
	}

	{
		const bool sprint = IsKeyDown(KEY_LEFT_SHIFT);
		const float speed = sprint ? 5.0f : 2.5f;
		const Vector3 delta { sinf(player.camAngles.x), 0, cosf(player.camAngles.x) };
		const Vector3 side { delta.z, delta.y, -delta.x };
		const Vector3 move = Vector3Scale(delta, -1);

		float fwd = 0.0f;
		if (IsKeyDown(KEY_W))
			fwd += 1.0f;
		if (IsKeyDown(KEY_S))
			fwd -= 1.0f;

		float strafe = 0.0f;
		if (IsKeyDown(KEY_A))
			strafe -= 1.0f;
		if (IsKeyDown(KEY_D))
			strafe += 1.0f;

		auto* rb = player.player;
		const btVector3 speedVec = fwd * toBtV3(move) + strafe * toBtV3(side);
		const btVector3 lv = rb->getLinearVelocity();
		const btVector3 nlv = { speed * speedVec.x(), lv.y(), speed * speedVec.z() };
		const float r0 = 16.0f;
		const float r1 = lv.length2();
		const float rt = 1.0f / (r0 + r1);
		rb->setLinearVelocity(rt * r1 * lv + rt * r0 * nlv);
		rb->setAngularFactor(0.0);
	}

	if (IsKeyPressed(KEY_F5)) {
		auto map = GetMap(ctx, "hub");
		TeleportPlayer(ctx, ctx.player, map);
	}

	if (IsKeyPressed(KEY_E)) {
		if (ctx.targetInteraction > 0 && ctx.targetInteraction <= int(map.interactions.size())) {
			const auto idx = ctx.targetInteraction - 1;
			ctx.targetInteraction = {};
			map.interactions[idx](ctx, map);
		}
	}

	if (IsKeyPressed(KEY_SPACE)) {
		const auto from = cam.position;
		const btVector3 bf { from.x, from.y, from.z };
		const btVector3 bt = bf + btVector3 { 0, -0.9, 0 };
		btCollisionWorld::ClosestRayResultCallback cb { bf, bt };
		cb.m_collisionFilterMask = cb.m_collisionFilterMask & ~RbGroups::PLAYER;
		player.map->dynamics_world->rayTest(bf, bt, cb);
		if (cb.m_collisionObject != nullptr) {
			// TraceLog(LOG_INFO, "jump");
			auto* rb = player.player;
			rb->applyCentralImpulse({ 0, 30, 0 });
		}
	}

	if (IsKeyPressed(KEY_G)) {
		const float sz = 0.5f;
		const Vector3 dir = Vector3Scale(Vector3Normalize(Vector3Subtract(cam.target, cam.position)), 20.0f);
		const Color col = RndCol(146, 100);
		map.cubes.push_back({ { sz, sz, sz }, col, 0 }, [&](size_t i, DynCube& c) {
			CreatePhysicCube(ctx, map, i, c, cam.target, { 1, 0, 0 }, Box, DynParams { 1.0f, dir });
		});
	}

	if (IsKeyPressed(KEY_F)) {
		const float sz = 0.5f * 0.5f;
		const Vector3 dir = Vector3Scale(Vector3Normalize(Vector3Subtract(cam.target, cam.position)), 20.0f);
		const Color col = RndCol(146, 100);
		map.spheres.push_back({ sz, col, 1 }, [&](size_t i, DynSphere& s) {
			CreatePhysicSphere(ctx, map, i, s, cam.target, Box, DynParams { 1.0f, dir });
		});
	}

	{
		const Vector3 dir = Vector3Normalize(Vector3Subtract(cam.target, cam.position));
		if (IsMouseButtonPressed(0)) {
			const Vector3 gunPos = GetCameraOffset(ctx, player, { 0, -0.09, -0.15 });
			map.bullets.push_back({ gunPos, dir, ctx.gtime });
			// shoot recoil
			player.deltaCamAngles.y -= 0.03f;
			player.deltaCamAngles.x += float(GetRandomValue(-10, 10)) / 2000.0f;
		}
		if (IsMouseButtonPressed(1)) {
			const Vector3 grapplePos = cam.position;
			player.grapple = Particle { grapplePos, dir, ctx.gtime };
		}
	}
}

bool Update(Context& ctx)
{
	auto& player = ctx.player;

	const bool esc = IsKeyPressed(KEY_ESCAPE);
	const bool quit = ctx.lockCursor && esc;

	player.deltaCamAngles = Vector2Scale(player.deltaCamAngles, 0.95f);
	ctx.pgtime = ctx.gtime;
	ctx.gtime += 1.0 / float(ctx.FPS);

	if (IsKeyPressed(KEY_F1) || esc) {
		ctx.lockCursor = !ctx.lockCursor;
		if (ctx.lockCursor)
			DisableCursor();
		else
			EnableCursor();
	}

	UpdateMap(ctx, *ctx.player.map);

	const Camera3D cam = GetCamera(ctx, player);

	UpdatePlayer(ctx, cam, player);

	if (ctx.lockCursor) {
		UpdatePlayerInputs(ctx, cam, player);
	} else {
		if (!ctx.command.empty()) {
			TraceLog(LOG_INFO, "console: %s", ctx.command.c_str());
			ExecCommand(ctx, ctx.command);
			ctx.history.push_back(ctx.command);
			ctx.command = {};
		}
	}

	{
		const Camera3D ncam = GetCamera(ctx, player);
		const auto farPos = Vector3Add(ncam.position, Vector3Scale(Vector3Normalize(Vector3Subtract(ncam.target, ncam.position)), 200.0f));
		const auto target = CheckRayCollision(ctx, player.map->dynamics_world, ncam.position, farPos, RbGroups::PLAYER);
		if (target) {
			ctx.targetPos = target->pos;
			if (ctx.targetInteraction != target->index2) {
				ctx.targetInteraction = target->index2;
				TraceLog(LOG_INFO, "target interaction: %d", ctx.targetInteraction);
			}
		} else {
			ctx.targetPos = {};
			ctx.targetInteraction = {};
		}
	}

	return !WindowShouldClose() && !quit;
}
