
#include "common.hpp"

#include <algorithm>

#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wmissing-field-initializers"
#pragma GCC diagnostic ignored "-Wenum-compare"
#pragma GCC diagnostic ignored "-Wunused-variable"
#pragma GCC diagnostic ignored "-Wunused-parameter"
#pragma GCC diagnostic ignored "-Wunused-but-set-variable"
#pragma GCC diagnostic ignored "-Wsign-compare"
#pragma GCC diagnostic ignored "-Wnarrowing"
#define RAYGUI_IMPLEMENTATION
#include "raygui.h"
#pragma GCC diagnostic pop

Rectangle Inset(Rectangle r, float i = 2.0f)
{
	return { r.x + i, r.y + i, r.width - 2.0f * i, r.height - 2.0f * i };
}

void DrawRope(Context&, Vector3 from0, Vector3 to0, Vector3 from1, Vector3 to1, int segments)
{
	const int sc = 4;
	const float rw = 0.005f;
	const float sr = 0.005f;
	for (int i = 1; i <= segments; ++i) {
		const float r0 = float(i - 1) / segments;
		const float r1 = float(i) / segments;
		const Vector3 nf0 = Vector3Lerp(from0, from1, r0);
		const Vector3 nf1 = Vector3Lerp(from0, from1, r1);
		const Vector3 nt0 = Vector3Lerp(to0, to1, r0);
		const Vector3 nt1 = Vector3Lerp(to0, to1, r1);
		const Vector3 sp0 = Vector3Lerp(nf0, nt0, 1 - r0);
		const Vector3 sp1 = Vector3Lerp(nf1, nt1, 1 - r1);
		DrawCapsule(sp0, sp1, rw, sc, sc, BLUE);
	}
	DrawSphere(to1, sr, WHITE);
}

void RenderParticles(Context& ctx, GameMap& map)
{
	const Color ctable[2] = {
		Color { 253, 249, 0, 255 },
		BLUE,
	};
	const float sztable[2] = {
		0.05f,
		0.01f,
	};
	map.particles.erase(
		std::remove_if(
			map.particles.begin(), map.particles.end(),
			[&](const Particle& p) -> bool {
				const float dt = ctx.gtime - p.startTime;
				const float maxt = 0.3f;
				if (dt > maxt)
					return true;
				const float speed = 3.0f;
				const float r = dt / maxt;
				const Vector3 pos = Vector3 { 0.0f, -dt * dt * 2.0f, 0.0f } + p.startPos + p.dir * (speed * dt);
				const uint8_t alpha = static_cast<uint8_t>(std::min(255.0f, 255.0f * (1.0f - r)));
				const float sz = sztable[p.ptype] * (1 + r);
				const Color bc = ctable[p.ptype];
				DrawCube(pos, sz, sz, sz, Color { bc.r, bc.g, bc.b, alpha });
				return false;
			}),
		map.particles.end());
}

void RenderBullets(Context& ctx, GameMap& map)
{
	Color col = YELLOW;
	col.a = 90;
	for (const auto& b : map.bullets) {
		const float dt0 = ctx.pgtime - b.startTime;
		const float dt1 = ctx.gtime - b.startTime;
		const float speed = BulletSpeed;
		const Vector3 from = b.startPos + b.dir * (speed * dt0);
		const Vector3 to = b.startPos + b.dir * (speed * dt1);
		const float sz = 0.01f;
		DrawCapsule(from, to, sz, 8, 4, col);
	}
}

void RenderViewMap(Context& ctx, const Camera&, Player& player, GameMap& map)
{
	if (player.grapple) {
		Vector3 pos {};
		const Vector3 startPos = GetCameraOffset(ctx, player, { 0, -0.09, 0.15 });
		const Particle& grapple = std::get<Particle>(*player.grapple);
		CheckParticleCollision(ctx, map.dynamics_world, grapple, GrappleSpeed, &pos);
		DrawRope(ctx, grapple.startPos, pos, startPos, pos, 16);
	}

	for (const auto& c : map.staticModels) {
		DrawModelEx(ctx.models[c.index], c.position, { 0, 1, 0 }, 0, c.size, c.col);
	}

	if (ctx.targetPos)
		DrawSphere(*ctx.targetPos, 0.01f + 0.005f * sinf(20.0f * ctx.gtime), { 255, 255, 255, 64 });

	const auto getTranform = [](const btRigidBody* rb) -> std::tuple<Vector3, Vector3, float> {
		btTransform bt;
		rb->getMotionState()->getWorldTransform(bt);
		const auto p = bt.getOrigin();
		const auto quat = bt.getRotation();
		const auto btaxis = quat.getAxis();
		const Vector3 pos = toRlV3(p);
		const Vector3 axis = toRlV3(btaxis);
		const float radian_scale = 180.0 / PI;
		const float angle = float(quat.getAngle()) * radian_scale;
		return { pos, axis, angle };
	};

	map.cubes.forEach([&](size_t, DynCube& c) {
		const auto& [pos, axis, angle] = getTranform(c.rb);
		DrawModelEx(ctx.models[c.index], pos, axis, angle, { c.size, c.size, c.size }, c.col);
	});

	map.spheres.forEach([&](size_t, DynSphere& s) {
		const auto& [pos, axis, angle] = getTranform(s.rb);
		DrawModelEx(ctx.models[s.index], pos, axis, angle, { s.size, s.size, s.size }, s.col);
	});

	RenderBullets(ctx, map);
	RenderParticles(ctx, map);
}

void RenderView(Context& ctx, const Camera& cam)
{
	Player& player = ctx.player;
	RenderViewMap(ctx, cam, player, *player.map);
}

void RenderGui(Context& ctx)
{
	if (ctx.lockCursor)
		return;
	const float o = 22.0f;
	const float y = GetScreenHeight() - o;
	const float w = GetScreenWidth();
	DrawRectangle(0, y - 0.0f, w, o, WHITE);
	GuiLabel(Inset({ 0, y, o, o }), "F1");
	if (GuiTextBox(Inset({ o, y, w - o, o }), ctx.commandBuffer, sizeof(ctx.commandBuffer), !ctx.lockCursor)) {
		ctx.command = ctx.commandBuffer;
		ctx.commandBuffer[0] = 0;
	}
}

void RenderHUD(Context& ctx)
{
	if (ctx.lockCursor) {
		const float x = 0.5 * GetScreenWidth();
		const float y = 0.5 * GetScreenHeight();
		DrawRectangleRec({ x - 2, y - 2, 5, 5 }, BLACK);
		DrawRectangleRec({ x - 1, y - 1, 3, 3 }, WHITE);
	}

#ifdef DEBUG
	char buffer[2048] {};
	snprintf(buffer, sizeof(buffer), "%d fps", GetFPS());
	DrawText(buffer, 0, 0, 20, WHITE);
#endif
}

void Render(Context& ctx)
{
	ctx.frame += 1;
	BeginDrawing();
	{
		BeginTextureMode(ctx.rt);
		ClearBackground(PINK);
		const Camera3D cam = GetCamera(ctx, ctx.player);
		BeginMode3D(cam);
		{
			RenderView(ctx, cam);
		}
		EndMode3D();
		RenderGui(ctx);
		RenderHUD(ctx);
		EndTextureMode();
		const float w = ctx.W;
		const float h = ctx.H;
		BeginShaderMode(ctx.postFxShader);
		{
			DrawTexturePro(ctx.rt.texture, { 0, 0, w, -h }, { 0, 0, w, h }, {}, 0, WHITE);
		}
		EndShaderMode();
	}
	EndDrawing();
}
