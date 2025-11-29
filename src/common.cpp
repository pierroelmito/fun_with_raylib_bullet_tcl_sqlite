
#include "common.hpp"

#include <cmath>
#include <optional>
#include <string>

void ExecCommand(Context& ctx, const std::string& cmd)
{
	const int r = Tcl_Eval(ctx.tcl, cmd.c_str());
	if (r != TCL_OK)
		TraceLog(LOG_ERROR, "TCL error");
}

void CleanRigidBody(Context&, GameMap& map, btRigidBody*& rb)
{
	map.dynamics_world->removeRigidBody(rb);
	delete rb->getCollisionShape();
	delete rb->getMotionState();
	delete rb;
	rb = nullptr;
}

void CleanCube(Context& ctx, GameMap& map, DynCube& c)
{
	CleanRigidBody(ctx, map, c.rb);
}
void CleanSphere(Context& ctx, GameMap& map, DynSphere& s)
{
	CleanRigidBody(ctx, map, s.rb);
}

Vector3 GetCameraOffset(Context&, Player& player, Vector3 o)
{
	btTransform bt;
	player.player->getMotionState()->getWorldTransform(bt);
	const auto p = bt.getOrigin();
	const Vector3 pos = toRlV3(p + btVector3 { 0, 0.3, 0 });
	const Vector3 d { sinf(player.camAngles.x), 0, cosf(player.camAngles.x) };
	const Vector3 n { -d.z, 0, d.x };
	const Vector3 v0 = Vector3Scale(d, o.x);
	const Vector3 v1 = Vector3Scale({ 0, 1, 0 }, o.y);
	const Vector3 v2 = Vector3Scale(n, o.z);
	return Vector3Add(pos, Vector3Add(Vector3Add(v0, v1), v2));
}

Camera3D GetCamera(Context&, Player& player)
{
	Camera3D r {};

	btTransform bt;
	player.player->getMotionState()->getWorldTransform(bt);
	const auto p = bt.getOrigin();
	const Vector3 pos = toRlV3(p + btVector3 { 0, 0.3, 0 });
	const Vector2 angles = Vector2Add(player.camAngles, player.deltaCamAngles);
	const Vector3 delta { cosf(angles.y) * sinf(angles.x), sinf(angles.y), cosf(angles.y) * cosf(angles.x) };

	r.fovy = 30.0f;
	r.up = { 0, 1, 0 };
	r.projection = CAMERA_PERSPECTIVE;
	r.target = Vector3Add(pos, Vector3Scale(delta, -2.0f));
	r.position = pos;

	return r;
}

btTransform MakeTransform(const Vector3& pos, const Vector3& rotation)
{
	btTransform transform;
	transform.setIdentity();
	transform.setOrigin(toBtV3(pos));
	transform.setRotation(btQuaternion(btScalar(rotation.z), btScalar(rotation.y), btScalar(rotation.x)));
	return transform;
}

void CreatePlayerCapsule(Context&, Player& player)
{
	btCapsuleShape* cs = new btCapsuleShape(0.4f, 0.2f);

	const btTransform transform = MakeTransform({ 0, 0, 0 }, { 0, 0, 0 });

	const float mass = 5.0f;
	const btScalar object_mass(mass);
	btVector3 li { 0, 0, 0 };
	cs->calculateLocalInertia(mass, li);
	btDefaultMotionState* ms = new btDefaultMotionState(transform);

	btRigidBody::btRigidBodyConstructionInfo rb_info { object_mass, ms, cs, li };
	rb_info.m_friction = 0;
	rb_info.m_rollingFriction = 0;
	rb_info.m_spinningFriction = 0;
	rb_info.m_linearDamping = 0.0f;
	rb_info.m_angularDamping = 0.0f;

	btRigidBody* body = new btRigidBody(rb_info);
	body->setActivationState(DISABLE_DEACTIVATION);
	player.player = body;
}

btRigidBody* CreatePhysicShape(Context&, GameMap& map, int group, btCollisionShape* collider_shape, uint32_t t, uint32_t index, const Vector3& pos, const Vector3& rotation, const PhysicMat& pm, const std::optional<DynParams>& params)
{
	const btTransform transform = MakeTransform(pos, rotation);

	const auto np = params.value_or(DynParams { 0.0f, { 0.0f, 0.0f, 0.0f } });
	const float mass = 5 * std::get<0>(np);
	const btScalar object_mass(mass);
	const Vector3 li = std::get<1>(np);
	const btVector3 speed = toBtV3(li);
	const ID id { .i = index, .t = t };

	btVector3 local_inertia { 0, 0, 0 };
	if (params || mass != 0.0)
		collider_shape->calculateLocalInertia(mass, local_inertia);

	btDefaultMotionState* motion_state = new btDefaultMotionState(transform);

	btRigidBody::btRigidBodyConstructionInfo rb_info { object_mass, motion_state, collider_shape, local_inertia };
	rb_info.m_friction = pm.friction;
	rb_info.m_rollingFriction = pm.frictionRoll;
	rb_info.m_spinningFriction = pm.frictionSpin;
	rb_info.m_linearDamping = 0.6f;
	rb_info.m_angularDamping = 0.4f;

	btRigidBody* body = new btRigidBody(rb_info);
	map.dynamics_world->addRigidBody(body, group, btBroadphaseProxy::AllFilter);
	body->setUserIndex(id.v);
	body->setLinearVelocity(speed);

	return body;
}

btRigidBody* CreatePhysicCube(Context& ctx, GameMap& map, const Vector3& pos, const Vector3& size, const Vector3& rotation, Color col, uint32_t mshIndex, const PhysicMat& pm, const std::optional<DynParams>& params)
{
	const Vector3 halfSize = Vector3Scale(size, 0.5f);
	btCollisionShape* collider_shape = new btBoxShape(toBtV3(halfSize) + btVector3 { pm.extent, 0.0f, pm.extent });
	map.staticModels.push_back({ size, pos, col, mshIndex });
	btRigidBody* rb = CreatePhysicShape(ctx, map, RbGroups::WALL, collider_shape, 0, ~0u, pos, rotation, pm, params);
	map.staticBodies.push_back(rb);
	return rb;
}

void CreatePhysicCube(Context& ctx, GameMap& map, uint32_t index, DynCube& c, const Vector3& pos, const Vector3& rotation, const PhysicMat& pm, const std::optional<DynParams>& params)
{
	const Vector3 halfSize = Vector3Scale(c.size, 0.5f);
	btCollisionShape* collider_shape = new btBoxShape(toBtV3(halfSize) + btVector3 { pm.extent, 0.0f, pm.extent });
	c.rb = CreatePhysicShape(ctx, map, RbGroups::OBJECT, collider_shape, 0, index, pos, rotation, pm, params);
}

void CreatePhysicSphere(Context& ctx, GameMap& map, uint32_t index, DynSphere& s, const Vector3& pos, const PhysicMat& pm, const std::optional<DynParams>& params)
{
	btCollisionShape* collider_shape = new btSphereShape(s.size);
	s.rb = CreatePhysicShape(ctx, map, RbGroups::OBJECT, collider_shape, 1, index, pos, { 0, 0, 0 }, pm, params);
}

void TeleportPlayer(Context& ctx, Player& player, std::shared_ptr<GameMap> map)
{
	if (player.map != map) {
		if (player.map)
			player.map->dynamics_world->removeRigidBody(ctx.player.player);
		player.map = map;
		if (player.map)
			map->dynamics_world->addRigidBody(ctx.player.player, RbGroups::PLAYER, btBroadphaseProxy::AllFilter);
	}
	btTransform t {};
	t.setOrigin(toBtV3(map->startPos));
	auto* rb = player.player;
	rb->setWorldTransform(t);
	rb->setLinearVelocity({ 0, 0, 0 });
	rb->setActivationState(DISABLE_DEACTIVATION);
	rb->clearForces();
}

std::optional<CResult> CheckRayCollision(const Context&, btDiscreteDynamicsWorld* world, Vector3 from, Vector3 to, int skipFlags)
{
	const btVector3 bf = toBtV3(from);
	const btVector3 bt = toBtV3(to);
	btCollisionWorld::ClosestRayResultCallback cb { bf, bt };
	cb.m_collisionFilterMask = cb.m_collisionFilterMask & ~skipFlags;
	world->rayTest(bf, bt, cb);
	if (!cb.hasHit())
		return {};
	const bool st = cb.m_collisionObject->isStaticObject();
	const int idx1 = cb.m_collisionObject->getUserIndex();
	const int idx2 = cb.m_collisionObject->getUserIndex2();
	return CResult { toRlV3(cb.m_hitPointWorld), toRlV3(cb.m_hitNormalWorld), idx1, idx2, st };
}

std::optional<CResult> CheckRayCollision(const Context& ctx, btDiscreteDynamicsWorld* world, const Particle& b, float speed, Vector3* pos)
{
	const float dt0 = ctx.pgtime - b.startTime;
	const float dt1 = ctx.gtime - b.startTime;
	const Vector3 from = Vector3Add(b.startPos, Vector3Scale(b.dir, speed * dt0));
	const Vector3 to = Vector3Add(b.startPos, Vector3Scale(b.dir, speed * dt1));
	if (pos)
		*pos = to;
	return CheckRayCollision(ctx, world, from, to, RbGroups::PLAYER);
}
