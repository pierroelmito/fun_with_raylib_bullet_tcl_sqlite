
#pragma once

#include <cstdint>
#include <functional>
#include <memory>
#include <variant>
#include <vector>

// script
#include "mc/tclpp.hpp"

#include <sqlite3.h>

#include <raylib.h>
#include <raymath.h>

#include <BulletCollision/CollisionShapes/btHeightfieldTerrainShape.h>
#include <btBulletDynamicsCommon.h>

inline btVector3 toBtV3(Vector3 v)
{
	return btVector3 { btScalar(v.x), btScalar(v.y), btScalar(v.z) };
}

inline Vector3 toRlV3(btVector3 v)
{
	return Vector3 { (float)v.getX(), (float)v.getY(), (float)v.getZ() };
}

inline Vector3 Reflect(Vector3 in, Vector3 n)
{
	const float v = Vector3DotProduct(n, in);
	return Vector3Add(in, Vector3Scale(n, -2.0f * v));
}

inline Color RndCol(int minr, int dr)
{
	const auto rc = [&]() { return uint8_t(GetRandomValue(minr, minr + dr)); };
	return Color { rc(), rc(), rc(), 255 };
}

struct ID {
	union {
		uint32_t v;
		struct {
			uint32_t i : 30;
			uint32_t t : 2;
		};
	};
};

struct RbGroups {
	enum Type {
		PLAYER = (1 << 11),
		WALL = (1 << 12),
		OBJECT = (1 << 13),
	};
};

static_assert(sizeof(ID) == sizeof(uint32_t));

struct PhysicMat {
	float friction {};
	float frictionRoll {};
	float frictionSpin {};
	float extent {};
};

constexpr static const PhysicMat Wall { 0.8f, 0.0f, 0.0f, 0.05f };
constexpr static const PhysicMat Box { 0.6f, 0.0f, 0.0f, 0.0f };

constexpr float BulletSpeed = 120.0f;
constexpr float GrappleSpeed = 80.0f;
constexpr float HmSize = 256.0f;
constexpr float HmHeight = 50.0f;

struct StaMdl {
	Vector3 size {};
	Vector3 position {};
	Color col { WHITE };
	uint32_t index {};
};

struct DynCube {
	Vector3 size {};
	Color col { WHITE };
	uint32_t index {};
	btRigidBody* rb {};
};

struct DynSphere {
	float size {};
	Color col { WHITE };
	uint32_t index {};
	btRigidBody* rb {};
};

struct Particle {
	Vector3 startPos {};
	Vector3 dir;
	double startTime {};
	int ptype {};
};

template <class T>
class ObjectList {
public:
	std::vector<T> data {};
	std::vector<size_t> freeIndex {};
	template <class F, typename... A>
	void clear(const F& f, A&&... args)
	{
		forEach([&](size_t, T& d) { f(args..., d); });
		data.clear();
		freeIndex.clear();
	}
	T& at(size_t i) { return data[i]; }
	const T& at(size_t i) const { return data[i]; }
	template <class F>
	void forEach(const F& f) const
	{
		uint32_t i = 0;
		for (const auto& c : data) {
			f(i++, c);
		}
	}
	template <class F>
	void forEach(const F& f)
	{
		uint32_t i = 0;
		for (auto& c : data) {
			f(i++, c);
		}
	}
	T& operator[](size_t i) { return data[i]; }
	const T& operator[](size_t i) const { return data[i]; }
	size_t push_back(T&& t)
	{
		size_t r = data.size();
		data.push_back(t);
		return r;
	}
	template <class F>
	void push_back(T&& t, const F& f)
	{
		size_t r = data.size();
		data.push_back(t);
		f(r, data[r]);
	}
};

struct Context;
struct GameMap;

using InteractionFn = std::function<void(Context&, GameMap&)>;

struct GameMap {
	GameMap() { }
	GameMap(const GameMap&) = delete;
	btDefaultCollisionConfiguration* collision_configuration {};
	btCollisionDispatcher* dispatcher {};
	btBroadphaseInterface* overlapping_pair_cache {};
	btSequentialImpulseConstraintSolver* solver {};
	btDiscreteDynamicsWorld* dynamics_world {};
	std::vector<Particle> particles {};
	std::vector<Particle> bullets {};
	ObjectList<DynCube> cubes;
	ObjectList<DynSphere> spheres;
	std::vector<StaMdl> staticModels {};
	std::vector<btRigidBody*> staticBodies {};
	Vector3 startPos {};
	std::vector<InteractionFn> interactions {};
};

struct Player {
	Player() { }
	Player(const Player&) = delete;
	Vector2 camAngles {};
	Vector2 deltaCamAngles {};
	btRigidBody* player {};
	std::shared_ptr<GameMap> map {};
	std::optional<std::variant<Particle>> grapple;
};

struct Context {
	Context() { }
	Context(const Context&) = delete;
	// script
	Tcl_Interp* tcl {};
	// map
	std::map<std::string, std::shared_ptr<GameMap>> gameMaps {};
	// player
	Player player {};
	std::optional<Vector3> targetPos {};
	int targetInteraction {};
	// GUI
	char commandBuffer[1024] {};
	std::string command {};
	std::vector<std::string> history {};
	bool lockCursor { true };
	// render
	int W { 1280 }, H { 720 };
	// int W{640}, H{400};
	int FPS { 60 };
	int frame {};
	double pgtime {};
	double gtime {};
	Shader basicShader {};
	Shader postFxShader {};
	RenderTexture rt {};
	Image hm {};
	std::vector<Model> models {};
};

using DynParams = std::tuple<float, Vector3>;

inline bool SQ3(int r, sqlite3* db = nullptr)
{
	const bool ok = r == SQLITE_OK;
	if (r != SQLITE_OK) {
		if (db)
			TraceLog(LOG_ERROR, "SQL error: %s", sqlite3_errmsg(db));
		else
			TraceLog(LOG_ERROR, "SQL error");
	}
	return ok;
}

inline bool TCL(Tcl_Interp* tcl, int r)
{
	if (r == TCL_ERROR) {
		const std::string err = Tcl_GetVar(tcl, "errorInfo", TCL_GLOBAL_ONLY);
		TraceLog(LOG_ERROR, "%s", err.c_str());
	}
	return r == TCL_OK;
}

struct CResult {
	Vector3 pos {};
	Vector3 normal {};
	int index1 {};
	int index2 {};
	bool st {};
};

std::optional<CResult> CheckRayCollision(const Context& ctx, btDiscreteDynamicsWorld* world, Vector3 from, Vector3 to, int skipFlags = 0);
std::optional<CResult> CheckRayCollision(const Context& ctx, btDiscreteDynamicsWorld* world, const Particle& b, float speed, Vector3* pos = nullptr);

Vector3 GetCameraOffset(Context& ctx, Player& player, Vector3 o);
Camera3D GetCamera(Context& ctx, Player& player);

void ExecCommand(Context& ctx, const std::string& cmd);

btRigidBody* CreatePhysicCube(Context& ctx, GameMap& map, const Vector3& pos, const Vector3& size, const Vector3& rotation, Color col, uint32_t mshIndex, const PhysicMat& pm, const std::optional<DynParams>& params = {});
btRigidBody* CreatePhysicShape(Context&, GameMap& map, int group, btCollisionShape* collider_shape, uint32_t t, uint32_t index, const Vector3& pos, const Vector3& rotation, const PhysicMat& pm, const std::optional<DynParams>& params = {});
void CreatePhysicCube(Context& ctx, GameMap& map, uint32_t index, DynCube& c, const Vector3& pos, const Vector3& rotation, const PhysicMat& pm, const std::optional<DynParams>& params = {});
void CreatePhysicSphere(Context& ctx, GameMap& map, uint32_t index, DynSphere& s, const Vector3& pos, const PhysicMat& pm, const std::optional<DynParams>& params = {});
void CreatePlayerCapsule(Context&, Player& player);

void CleanRigidBody(Context& ctx, GameMap& map, btRigidBody*& rb);
void CleanCube(Context& ctx, GameMap& map, DynCube& c);
void CleanSphere(Context& ctx, GameMap& map, DynSphere& s);

std::shared_ptr<GameMap> GetMap(Context& ctx, const std::string& mapName);

void TeleportPlayer(Context& ctx, Player& player, std::shared_ptr<GameMap> map);

void Init(Context& ctx);
void Release(Context& ctx);
void Render(Context& ctx);
bool Update(Context& ctx);
