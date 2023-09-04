
// STL
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <tuple>
#include <variant>
#include <vector>

// script
#include "tclpp.hpp"

// DB
#include <sqlite3.h>

// physics
#include <BulletCollision/CollisionShapes/btHeightfieldTerrainShape.h>
#include <btBulletDynamicsCommon.h>

// raylib
#include <raylib.h>
#include <raymath.h>
#define RAYGUI_IMPLEMENTATION
#include "raygui.h"

inline btVector3 toBtV3(Vector3 v) {
  return btVector3{btScalar(v.x), btScalar(v.y), btScalar(v.z)};
}

inline Vector3 toRlV3(btVector3 v) {
  return Vector3{v.getX(), v.getY(), v.getZ()};
}

inline Vector3 Reflect(Vector3 in, Vector3 n) {
  const float v = Vector3DotProduct(n, in);
  return Vector3Add(in, Vector3Scale(n, -2.0f * v));
}

inline Color RndCol(int minr, int dr) {
  return Color{uint8_t(GetRandomValue(minr, minr + dr)),
               uint8_t(GetRandomValue(minr, minr + dr)),
               uint8_t(GetRandomValue(minr, minr + dr)), 255};
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
  float friction{};
  float frictionRoll{};
  float frictionSpin{};
  float extent{};
};

constexpr static const PhysicMat Wall{0.8f, 0.0f, 0.0f, 0.05f};
constexpr static const PhysicMat Box{0.6f, 0.0f, 0.0f, 0.0f};

constexpr float BulletSpeed = 120.0f;
constexpr float GrappleSpeed = 80.0f;
constexpr float HmSize = 256.0f;
constexpr float HmHeight = 50.0f;

struct StaMdl {
  Vector3 size{};
  Vector3 position{};
  Color col{WHITE};
  uint32_t index{};
};

struct DynCube {
  Vector3 size{};
  Color col{WHITE};
  uint32_t index{};
  btRigidBody *rb{};
};

struct DynSphere {
  float size{};
  Color col{WHITE};
  uint32_t index{};
  btRigidBody *rb{};
};

struct Particle {
  Vector3 startPos{};
  Vector3 dir;
  double startTime{};
  int ptype{};
};

template <class T> class ObjectList {
public:
  std::vector<T> data{};
  std::vector<size_t> freeIndex{};
  template <class F, typename... A> void clear(const F &f, A &&...args) {
    forEach([&](size_t i, T &d) { f(args..., d); });
    data.clear();
    freeIndex.clear();
  }
  T &at(size_t i) { return data[i]; }
  const T &at(size_t i) const { return data[i]; }
  template <class F> void forEach(const F &f) const {
    uint32_t i = 0;
    for (const auto &c : data) {
      f(i++, c);
    }
  }
  template <class F> void forEach(const F &f) {
    uint32_t i = 0;
    for (auto &c : data) {
      f(i++, c);
    }
  }
  T &operator[](size_t i) { return data[i]; }
  const T &operator[](size_t i) const { return data[i]; }
  size_t push_back(T &&t) {
    size_t r = data.size();
    data.push_back(t);
    return r;
  }
  template <class F> void push_back(T &&t, const F &f) {
    size_t r = data.size();
    data.push_back(t);
    f(r, data[r]);
  }
};

struct Context;
struct GameMap;

using InteractionFn = std::function<void(Context &, GameMap &)>;

struct GameMap {
  GameMap() {}
  GameMap(const GameMap &) = delete;
  btDefaultCollisionConfiguration *collision_configuration{};
  btCollisionDispatcher *dispatcher{};
  btBroadphaseInterface *overlapping_pair_cache{};
  btSequentialImpulseConstraintSolver *solver{};
  btDiscreteDynamicsWorld *dynamics_world{};
  std::vector<Particle> particles{};
  std::vector<Particle> bullets{};
  ObjectList<DynCube> cubes;
  ObjectList<DynSphere> spheres;
  std::vector<StaMdl> staticModels{};
  std::vector<btRigidBody *> staticBodies{};
  Vector3 startPos{};
  std::vector<InteractionFn> interactions{};
};

struct Player {
  Player() {}
  Player(const Player &) = delete;
  Vector2 camAngles{};
  Vector2 deltaCamAngles{};
  btRigidBody *player{};
  std::shared_ptr<GameMap> map{};
  std::optional<std::variant<Particle>> grapple;
};

struct Context {
  Context() {}
  Context(const Context &) = delete;
  // script
  Tcl_Interp *tcl{};
  // map
  std::map<std::string, std::shared_ptr<GameMap>> gameMaps{};
  // player
  Player player{};
  std::optional<Vector3> targetPos{};
  int targetInteraction{};
  // GUI
  char commandBuffer[1024]{};
  std::string command{};
  std::vector<std::string> history{};
  bool lockCursor{true};
  // render
  int W{1280}, H{720};
  // int W{640}, H{400};
  int FPS{60};
  int frame{};
  double pgtime{};
  double gtime{};
  Shader basicShader{};
  Shader postFxShader{};
  RenderTexture rt{};
  Image hm{};
  std::vector<Model> models{};
};

using DynParams = std::tuple<float, Vector3>;

inline bool TCL(Tcl_Interp *tcl, int r) {
  if (r == TCL_ERROR) {
    const std::string err = Tcl_GetVar(tcl, "errorInfo", TCL_GLOBAL_ONLY);
    TraceLog(LOG_ERROR, "%s", err.c_str());
  }
  return r == TCL_OK;
}

inline bool SQ3(int r, sqlite3 *db = nullptr) {
  const bool ok = r == SQLITE_OK;
  if (r != SQLITE_OK) {
    if (db)
      TraceLog(LOG_ERROR, "SQL error: %s", sqlite3_errmsg(db));
    else
      TraceLog(LOG_ERROR, "SQL error");
  }
  return ok;
}

void ExecCommand(Context &ctx, const std::string &cmd) {
  const int r = Tcl_Eval(ctx.tcl, cmd.c_str());
  if (r != TCL_OK) {
    TraceLog(LOG_ERROR, "TCL error");
  }
}

void CleanRigidBody(Context &ctx, GameMap &map, btRigidBody *&rb) {
  map.dynamics_world->removeRigidBody(rb);
  delete rb->getCollisionShape();
  delete rb->getMotionState();
  delete rb;
  rb = nullptr;
}

void CleanCube(Context &ctx, GameMap &map, DynCube &c) {
  CleanRigidBody(ctx, map, c.rb);
}
void CleanSphere(Context &ctx, GameMap &map, DynSphere &s) {
  CleanRigidBody(ctx, map, s.rb);
}

Vector3 GetCameraOffset(Context &ctx, Player &player, Vector3 o) {
  btTransform bt;
  player.player->getMotionState()->getWorldTransform(bt);
  const auto p = bt.getOrigin();
  const Vector3 pos = toRlV3(p + btVector3{0, 0.3, 0});
  const Vector3 d{sinf(player.camAngles.x), 0, cosf(player.camAngles.x)};
  const Vector3 n{-d.z, 0, d.x};
  const Vector3 v0 = Vector3Scale(d, o.x);
  const Vector3 v1 = Vector3Scale({0, 1, 0}, o.y);
  const Vector3 v2 = Vector3Scale(n, o.z);
  return Vector3Add(pos, Vector3Add(Vector3Add(v0, v1), v2));
}

Camera3D GetCamera(Context &ctx, Player &player) {
  Camera3D r{};

  btTransform bt;
  player.player->getMotionState()->getWorldTransform(bt);
  const auto p = bt.getOrigin();
  const Vector3 pos = toRlV3(p + btVector3{0, 0.3, 0});
  const Vector2 angles = Vector2Add(player.camAngles, player.deltaCamAngles);
  const Vector3 delta{cosf(angles.y) * sinf(angles.x), sinf(angles.y),
                      cosf(angles.y) * cosf(angles.x)};

  r.fovy = 30.0f;
  r.up = {0, 1, 0};
  r.projection = CAMERA_PERSPECTIVE;
  r.target = Vector3Add(pos, Vector3Scale(delta, -2.0f));
  r.position = pos;

  return r;
}

btTransform MakeTransform(const Vector3 &pos, const Vector3 &rotation) {

  btTransform transform;
  transform.setIdentity();
  transform.setOrigin(toBtV3(pos));
  transform.setRotation(btQuaternion(btScalar(rotation.z), btScalar(rotation.y),
                                     btScalar(rotation.x)));
  return transform;
}

void CreatePlayerCapsule(Context &ctx, Player &player) {
  btCapsuleShape *cs = new btCapsuleShape(0.4f, 0.2f);

  const btTransform transform = MakeTransform({0, 0, 0}, {0, 0, 0});

  const float mass = 5.0f;
  const btScalar object_mass(mass);
  btVector3 li{0, 0, 0};
  cs->calculateLocalInertia(mass, li);
  btDefaultMotionState *ms = new btDefaultMotionState(transform);

  btRigidBody::btRigidBodyConstructionInfo rb_info{object_mass, ms, cs, li};
  rb_info.m_friction = 0;
  rb_info.m_rollingFriction = 0;
  rb_info.m_spinningFriction = 0;
  rb_info.m_linearDamping = 0.0f;
  rb_info.m_angularDamping = 0.0f;

  btRigidBody *body = new btRigidBody(rb_info);
  body->setActivationState(DISABLE_DEACTIVATION);
  player.player = body;
}

void TeleportPlayer(Context &ctx, Player &player,
                    std::shared_ptr<GameMap> map) {
  if (player.map != map) {
    if (player.map) {
      player.map->dynamics_world->removeRigidBody(ctx.player.player);
    }
    player.map = map;
    if (player.map) {
      map->dynamics_world->addRigidBody(ctx.player.player, RbGroups::PLAYER,
                                        btBroadphaseProxy::AllFilter);
    }
  }
  btTransform t{};
  t.setOrigin(toBtV3(map->startPos));
  auto *rb = player.player;
  rb->setWorldTransform(t);
  rb->setLinearVelocity({0, 0, 0});
  rb->setActivationState(DISABLE_DEACTIVATION);
  rb->clearForces();
}

btRigidBody *CreatePhysicShape(Context &ctx, GameMap &map, int group,
                               btCollisionShape *collider_shape, uint32_t t,
                               uint32_t index, const Vector3 &pos,
                               const Vector3 &rotation, const PhysicMat &pm,
                               const std::optional<DynParams> &params = {}) {
  const btTransform transform = MakeTransform(pos, rotation);

  const auto np = params.value_or(DynParams{0.0f, {0.0f, 0.0f, 0.0f}});
  const float mass = 5 * std::get<0>(np);
  const btScalar object_mass(mass);
  const Vector3 li = std::get<1>(np);
  const btVector3 speed = toBtV3(li);
  const ID id{.i = index, .t = t};

  btVector3 local_inertia{0, 0, 0};
  if (params || mass != 0.0)
    collider_shape->calculateLocalInertia(mass, local_inertia);

  btDefaultMotionState *motion_state = new btDefaultMotionState(transform);

  btRigidBody::btRigidBodyConstructionInfo rb_info{
      object_mass, motion_state, collider_shape, local_inertia};
  rb_info.m_friction = pm.friction;
  rb_info.m_rollingFriction = pm.frictionRoll;
  rb_info.m_spinningFriction = pm.frictionSpin;
  rb_info.m_linearDamping = 0.6f;
  rb_info.m_angularDamping = 0.4f;

  btRigidBody *body = new btRigidBody(rb_info);
  map.dynamics_world->addRigidBody(body, group, btBroadphaseProxy::AllFilter);
  body->setUserIndex(id.v);
  body->setLinearVelocity(speed);

  return body;
}

btRigidBody *CreatePhysicCube(Context &ctx, GameMap &map, const Vector3 &pos,
                              const Vector3 &size, const Vector3 &rotation,
                              Color col, uint32_t mshIndex, const PhysicMat &pm,
                              const std::optional<DynParams> &params = {}) {
  const Vector3 halfSize = Vector3Scale(size, 0.5f);
  btCollisionShape *collider_shape =
      new btBoxShape(toBtV3(halfSize) + btVector3{pm.extent, 0.0f, pm.extent});
  map.staticModels.push_back({size, pos, col, mshIndex});
  btRigidBody *rb = CreatePhysicShape(ctx, map, RbGroups::WALL, collider_shape,
                                      0, ~0u, pos, rotation, pm, params);
  map.staticBodies.push_back(rb);
  return rb;
}

void CreatePhysicCube(Context &ctx, GameMap &map, uint32_t index, DynCube &c,
                      const Vector3 &pos, const Vector3 &rotation,
                      const PhysicMat &pm,
                      const std::optional<DynParams> &params = {}) {
  const Vector3 halfSize = Vector3Scale(c.size, 0.5f);
  btCollisionShape *collider_shape =
      new btBoxShape(toBtV3(halfSize) + btVector3{pm.extent, 0.0f, pm.extent});
  c.rb = CreatePhysicShape(ctx, map, RbGroups::OBJECT, collider_shape, 0, index,
                           pos, rotation, pm, params);
}

void CreatePhysicSphere(Context &ctx, GameMap &map, uint32_t index,
                        DynSphere &s, const Vector3 &pos, const PhysicMat &pm,
                        const std::optional<DynParams> &params = {}) {
  btCollisionShape *collider_shape = new btSphereShape(s.size);
  s.rb = CreatePhysicShape(ctx, map, RbGroups::OBJECT, collider_shape, 1, index,
                           pos, {0, 0, 0}, pm, params);
}

void InitPhysicsMap(Context &ctx, GameMap &map) {
  map.collision_configuration = new btDefaultCollisionConfiguration();
  map.dispatcher = new btCollisionDispatcher(map.collision_configuration);
  map.overlapping_pair_cache = new btDbvtBroadphase();
  map.solver = new btSequentialImpulseConstraintSolver;
  map.dynamics_world =
      new btDiscreteDynamicsWorld(map.dispatcher, map.overlapping_pair_cache,
                                  map.solver, map.collision_configuration);
  map.dynamics_world->setGravity(btVector3(0, -10, 0));
}

using SliceCoord = std::array<int, 3>;
using SliceSpawns = std::vector<std::pair<SliceCoord, std::string>>;

void LoadImgMap(Context &ctx, GameMap &map, const std::string &path,
                const SliceSpawns &spawns,
                const std::function<bool(Color)> &isWall,
                const std::function<Color(Color, int)> &makeColor) {
  Image img = LoadImage(path.c_str());
  const int floors = img.width / img.height;
  const int stride = img.format == PIXELFORMAT_UNCOMPRESSED_R8G8B8 ? 3 : 4;

  TraceLog(LOG_INFO, "img %dx%d, floors %d", img.width, img.height, floors);

  std::optional<Vector3> startPos{};

  uint8_t *const pixels = static_cast<uint8_t *>(img.data);
  const float psz = 4.0f;
  const float fh = 4.0f;
  const Vector3 szc{psz, fh, psz};

  const auto GetMapAt = [&](int x, int y, int f) -> Color {
    if (x >= 0 && x < img.height && y >= 0 && y < img.height && f >= 0 &&
        f < floors) {
      const int o = y * img.width + (x + f * img.height);
      const Color cc = GetPixelColor(pixels + stride * o, img.format);
      return cc;
    }
    return {255, 255, 255, 0};
  };

  for (int f = -1; f < floors + 1; ++f) {
    bool emptyFloor = f >= 0;
    for (int y = 0; y < img.height; ++y) {
      for (int x = 0; x < img.height; ++x) {
        const Color cc = GetMapAt(x, y, f);
        const Vector3 cpos{x * psz, (f + 0.5f) * fh, y * psz};
        if (!isWall(cc)) {
          emptyFloor = false;
          const Color co = GetMapAt(x, y, f - 1);
          if (isWall(co)) {
            startPos = Vector3{cpos.x, f * fh + 0.5f, cpos.z};
          }
        } else {
          const int offsets[6][3] = {
              {1, 0, 0},  {-1, 0, 0}, {0, 1, 0},
              {0, -1, 0}, {0, 0, 1},  {0, 0, -1},
          };
          const uint32_t index = GetRandomValue(2, 3);
          for (int i = 0; i < 6; ++i) {
            const Color co = GetMapAt(x + offsets[i][0], y + offsets[i][1],
                                      f + offsets[i][2]);
            if (!isWall(co)) {
              const Color col = makeColor(cc, f);
              CreatePhysicCube(ctx, map, cpos, szc, {0, 0, 0}, col, index,
                               Wall);
              break;
            }
          }
        }
      }
    }
    if (emptyFloor)
      break;
  }

  for (const auto &spawn : spawns) {
    const auto [x, y, f] = spawn.first;
    const auto action = spawn.second;
    TraceLog(LOG_INFO, "\t(%d, %d, %d) '%s'", x, y, f, action.c_str());
    const Vector3 cpos{x * psz, f * fh + 0.5f, y * psz};
    auto *rb = CreatePhysicCube(ctx, map, cpos, Vector3{1, 1, 1},
                                Vector3{0, 0, 0}, WHITE, 2, Wall);
    rb->setUserIndex2(map.interactions.size() + 1);
    map.interactions.push_back([=](Context &ctx, GameMap &) {
      TraceLog(LOG_INFO, "interaction: %s", action.c_str());
      Tcl_Eval(ctx.tcl, action.c_str());
    });
  }

  if (startPos) {
    map.startPos = *startPos;
  }

  UnloadImage(img);
}

void LoadLayersMap(Context &ctx, GameMap &map, const std::string &path) {
  const auto isWall = [](Color c) -> bool {
    return (c.r != 0 || c.g != 0 || c.b != 0);
  };
  const auto makeColor = [](Color c, int f) {
    const int fc = f & 1;
    return RndCol(76 + fc * 30, 6);
  };
  return LoadImgMap(ctx, map, path, {}, isWall, makeColor);
}

void LoadSlicesMap(Context &ctx, GameMap &map, const std::string &path,
                   const SliceSpawns &spawns) {
  const auto isWall = [](Color c) -> bool { return (c.a < 128); };
  const auto makeColor = [](Color c, int f) {
    const int fc = f & 1;
    return RndCol(76 + fc * 30, 6);
  };
  return LoadImgMap(ctx, map, path, spawns, isWall, makeColor);
}

void LoadDbMap(Context &ctx, GameMap &map, const std::string &path) {
  sqlite3 *db = nullptr;
  SQ3(sqlite3_open(path.c_str(), &db));

  map.cubes.clear(CleanCube, ctx, map);

  {
    sqlite3_stmt *st = nullptr;
    const std::string query = "select * from cubes;";
    const char *tail = nullptr;
    SQ3(sqlite3_prepare_v3(db, query.c_str(), query.size(), 0, &st, &tail));
    while (sqlite3_step(st) == SQLITE_ROW) {
      const int s = 1;
      const float px = sqlite3_column_double(st, s + 0);
      const float py = sqlite3_column_double(st, s + 1);
      const float pz = sqlite3_column_double(st, s + 2);
      const float sx = sqlite3_column_double(st, s + 3);
      const float sy = sqlite3_column_double(st, s + 4);
      const float sz = sqlite3_column_double(st, s + 5);
      const auto *col = sqlite3_column_text(st, s + 6);
      Color c = BLUE;
      if (col) {
        int r, g, b;
        sscanf((const char *)col, "%02x%02x%02x", &r, &g, &b);
        c = Color{uint8_t(r), uint8_t(g), uint8_t(b), 255};
      }
      const Vector3 pos{px, py + 0.5f * sy, pz};
      CreatePhysicCube(ctx, map, pos, {sx, sy, sz}, {0, 0, 0}, c, 2, Wall);
    }
    SQ3(sqlite3_finalize(st));
  }

  SQ3(sqlite3_close(db));

  map.startPos = {0, 0.5, 0};
}

std::shared_ptr<GameMap> GetMap(Context &ctx, const std::string &mapName) {
  auto &map = ctx.gameMaps[mapName];
  if (!map) {
    TraceLog(LOG_INFO, "init new map");
    map = std::make_shared<GameMap>();
    InitPhysicsMap(ctx, *map);
    TclCall(ctx.tcl, "load_map", mapName);
  }
  return map;
}

int Cmd_load_map_db(ClientData data, Tcl_Interp *tcl, int argc,
                    Tcl_Obj *const *argv) {

  Context &ctx = *((Context *)data);

  const auto mapName = TclTo<std::string>(tcl, argv[1]);
  const auto mapPath = TclTo<std::string>(tcl, argv[2]);

  TraceLog(LOG_INFO, "Cmd_load_map_db %s %s", mapName.c_str(), mapPath.c_str());
  auto map = GetMap(ctx, mapName);
  LoadDbMap(ctx, *map, mapPath);

  return TCL_OK;
}

int Cmd_load_map_slices(ClientData data, Tcl_Interp *tcl, int argc,
                        Tcl_Obj *const *argv) {
  Context &ctx = *((Context *)data);

  const auto mapName = TclTo<std::string>(tcl, argv[1]);
  const auto mapPath = TclTo<std::string>(tcl, argv[2]);
  const auto spawns = TclTo<SliceSpawns>(tcl, argv[3]);

  TraceLog(LOG_INFO, "Cmd_load_map_slices %s %s", mapName.c_str(),
           mapPath.c_str());
  auto map = GetMap(ctx, mapName);
  LoadSlicesMap(ctx, *map, mapPath, spawns);

  return TCL_OK;
}

int Cmd_load_map_heightfield(ClientData data, Tcl_Interp *tcl, int argc,
                             Tcl_Obj *const *argv) {
  Context &ctx = *((Context *)data);

  const auto mapName = TclTo<std::string>(tcl, argv[1]);
  const auto mapPath = TclTo<std::string>(tcl, argv[2]);

  TraceLog(LOG_INFO, "Cmd_load_map_slices %s %s", mapName.c_str(),
           mapPath.c_str());
  auto map = GetMap(ctx, mapName);
  const float os = -0.5f * HmSize;
  const float oh = -0.5f * HmHeight;

  map->staticModels.push_back({{1, 1, 1}, {os, oh, os}, WHITE, 4});

  btCollisionShape *collider_shape = new btHeightfieldTerrainShape(
      ctx.hm.width, ctx.hm.height, (unsigned char *)(ctx.hm.data), HmHeight / 255.0f, oh, -oh, 1,
      false);
  btRigidBody *rb = CreatePhysicShape(ctx, *map, RbGroups::WALL, collider_shape,
                                      0, ~0u, {0, -0.5f * HmHeight, 0}, {0, 0, 0}, Wall, {});
  map->staticBodies.push_back(rb);
  map->startPos = {0, 0.5, 0};

  return TCL_OK;
}

int Cmd_teleport(ClientData data, Tcl_Interp *tcl, int argc,
                 Tcl_Obj *const *argv) {
  Context &ctx = *((Context *)data);

  const std::string mapName = TclTo<std::string>(tcl, argv[1]);

  TraceLog(LOG_INFO, "Cmd_teleport %s", mapName.c_str());
  auto map = GetMap(ctx, mapName);
  TeleportPlayer(ctx, ctx.player, map);
  return TCL_OK;
}

int Cmd_tracelog(ClientData data, Tcl_Interp *tcl, int argc,
                 Tcl_Obj *const *argv) {
  const std::string line = TclTo<std::string>(tcl, argv[1]);

  TraceLog(LOG_INFO, "%s", line.c_str());
  return TCL_OK;
}

// ================================================================================

void InitDB(Context &ctx) { SQ3(sqlite3_initialize()); }

void InitRender(Context &ctx) {
#ifndef DEBUG
  SetTraceLogLevel(LOG_ERROR);
#endif
  SetConfigFlags(FLAG_MSAA_4X_HINT);
  InitWindow(ctx.W, ctx.H, "main");
  SetTargetFPS(ctx.FPS);
  SetWindowState(FLAG_VSYNC_HINT);
  DisableCursor();
  SetExitKey(0);

  ctx.rt = LoadRenderTexture(ctx.W, ctx.H);
  ctx.basicShader =
      LoadShader("assets/shaders/default.vs", "assets/shaders/default.fs");
  ctx.postFxShader = LoadShader(nullptr, "assets/shaders/postfx.fs");

  const auto makeTexturedCube = [&](const char *path) -> Model {
    Texture t = LoadTexture(path);
    GenTextureMipmaps(&t);
    SetTextureFilter(t, TEXTURE_FILTER_BILINEAR);
    SetTextureFilter(t, TEXTURE_FILTER_ANISOTROPIC_16X);
    Mesh msh = GenMeshCube(1, 1, 1);
    Model m = LoadModelFromMesh(msh);
    m.materials[0].shader = ctx.basicShader;
    m.materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = t;
    return m;
  };

  const auto makeTexturedSphere = [&](const char *path) -> Model {
    Texture t = LoadTexture(path);
    GenTextureMipmaps(&t);
    SetTextureFilter(t, TEXTURE_FILTER_BILINEAR);
    SetTextureFilter(t, TEXTURE_FILTER_ANISOTROPIC_16X);
    Mesh msh = GenMeshSphere(1, 32, 32);
    Model m = LoadModelFromMesh(msh);
    m.materials[0].shader = ctx.basicShader;
    m.materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = t;
    return m;
  };

  const auto makeHm = [&](const char *path, Image &img) -> Model {
    img = LoadImage("assets/hm00.png");
    Mesh hm = GenMeshHeightmap(img, {HmSize, HmHeight, HmSize});
    Model m = LoadModelFromMesh(hm);
    m.materials[0].shader = ctx.basicShader;
    // m.materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = t;
    return m;
  };

  ctx.models = {
      makeTexturedCube("assets/crate.png"),
      makeTexturedSphere("assets/crate.png"),
      makeTexturedCube("assets/block00.png"),
      makeTexturedCube("assets/block01.png"),
      makeHm("assets/hm00.png", ctx.hm),
  };
}

void Init(Context &ctx) {
  ctx.tcl = Tcl_CreateInterp();

#define CMD(X) Tcl_CreateObjCommand(ctx.tcl, #X, Cmd_##X, &ctx, nullptr)
  CMD(load_map_heightfield);
  CMD(load_map_slices);
  CMD(load_map_db);
  CMD(tracelog);
  CMD(teleport);
#undef CMD

  InitDB(ctx);
  InitRender(ctx);
  CreatePlayerCapsule(ctx, ctx.player);

  TCL(ctx.tcl, Tcl_EvalFile(ctx.tcl, "assets/init.tcl"));
}

// ================================================================================

void ReleaseDB(Context &ctx) { SQ3(sqlite3_shutdown()); }

void ReleaseRender(Context &ctx) {
  for (Model &m : ctx.models)
    UnloadModel(m);
  ctx.models.clear();
  UnloadShader(ctx.basicShader);
  UnloadShader(ctx.postFxShader);
  CloseWindow();
}

void ReleasePhysics(Context &ctx, GameMap &map) {
  map.cubes.clear(CleanCube, ctx, map);
  map.spheres.clear(CleanSphere, ctx, map);
  for (auto *&rb : map.staticBodies)
    CleanRigidBody(ctx, map, rb);
  map.staticBodies.clear();
  delete map.dynamics_world;
  delete map.solver;
  delete map.overlapping_pair_cache;
  delete map.dispatcher;
  delete map.collision_configuration;
}

void ReleasePhysics(Context &ctx) {
  auto &player = ctx.player;
  CleanRigidBody(ctx, *player.map, player.player);
  for (const auto &kv : ctx.gameMaps)
    ReleasePhysics(ctx, *kv.second);
  ctx.gameMaps.clear();
}

void Release(Context &ctx) {
  ReleasePhysics(ctx);
  ReleaseRender(ctx);
  ReleaseDB(ctx);
  Tcl_DeleteInterp(ctx.tcl);
}

// ================================================================================

struct CResult {
  Vector3 pos{};
  Vector3 normal{};
  int index1{};
  int index2{};
  bool st{};
};

std::optional<CResult> CheckRayCollision(const Context &ctx,
                                         btDiscreteDynamicsWorld *world,
                                         Vector3 from, Vector3 to,
                                         int skipFlags = 0) {
  const btVector3 bf = toBtV3(from);
  const btVector3 bt = toBtV3(to);
  btCollisionWorld::ClosestRayResultCallback cb{bf, bt};
  cb.m_collisionFilterMask = cb.m_collisionFilterMask & ~skipFlags;
  world->rayTest(bf, bt, cb);
  if (!cb.hasHit())
    return {};
  const bool st = cb.m_collisionObject->isStaticObject();
  const int idx1 = cb.m_collisionObject->getUserIndex();
  const int idx2 = cb.m_collisionObject->getUserIndex2();
  return CResult{toRlV3(cb.m_hitPointWorld), toRlV3(cb.m_hitNormalWorld), idx1,
                 idx2, st};
}

std::optional<CResult> CheckRayCollision(const Context &ctx,
                                         btDiscreteDynamicsWorld *world,
                                         const Particle &b, float speed,
                                         Vector3 *pos = nullptr) {
  const float dt0 = ctx.pgtime - b.startTime;
  const float dt1 = ctx.gtime - b.startTime;
  const Vector3 from = Vector3Add(b.startPos, Vector3Scale(b.dir, speed * dt0));
  const Vector3 to = Vector3Add(b.startPos, Vector3Scale(b.dir, speed * dt1));
  if (pos)
    *pos = to;
  return CheckRayCollision(ctx, world, from, to, RbGroups::PLAYER);
}

void AddParticles(const Context &ctx, GameMap &map, Vector3 pos, Vector3 normal,
                  float radius, int ptype, int count) {
  for (int i = 0; i < count; ++i) {
    const float dx = float(GetRandomValue(-1000, 1000)) / 1000.0f;
    const float dy = float(GetRandomValue(-1000, 1000)) / 1000.0f;
    const float dz = float(GetRandomValue(-1000, 1000)) / 1000.0f;
    map.particles.push_back(
        {pos,
         Vector3Add(normal,
                    Vector3Scale(Vector3Normalize({dx, dy, dz}), radius)),
         ctx.gtime, ptype});
  }
}

bool MoveBullet(const Context &ctx, GameMap &map, const Particle &b) {
  const auto cpos = CheckRayCollision(ctx, map.dynamics_world, b, BulletSpeed);
  if (!cpos)
    return false;

  const Vector3 pdir = Reflect(Vector3Normalize(b.dir), cpos->normal);
  AddParticles(ctx, map, cpos->pos, pdir, 0.5f, 0, 10);

  if (!cpos->st) {
    const ID id{.v = uint32_t(cpos->index1)};
    const float force = 100.0f;
    if (id.t == 0) {
      auto *rb = map.cubes.at(id.i).rb;
      rb->setActivationState(ACTIVE_TAG);
      rb->applyImpulse(force * toBtV3(b.dir), {0, -0.1, 0});
    } else if (id.t == 1) {
      auto *rb = map.spheres.at(id.i).rb;
      rb->setActivationState(ACTIVE_TAG);
      rb->applyImpulse(force * toBtV3(b.dir).normalize(), {0, -0.1, 0});
    }
  }

  return true;
}

void UpdateBullets(Context &ctx, GameMap &map) {
  map.bullets.erase(std::remove_if(map.bullets.begin(), map.bullets.end(),
                                   [&](const Particle &b) {
                                     return MoveBullet(ctx, map, b);
                                   }),
                    map.bullets.end());
}

void UpdateMap(Context &ctx, GameMap &map) {
  map.dynamics_world->stepSimulation(1.0 / float(ctx.FPS), 1);
  UpdateBullets(ctx, map);
}

void UpdatePlayer(Context &ctx, const Camera &cam, Player &player) {
  GameMap &map = *player.map;

  if (player.grapple) {
    const Particle &grapple = std::get<Particle>(*player.grapple);
    Vector3 pos{};
    auto cpos =
        CheckRayCollision(ctx, map.dynamics_world, grapple, GrappleSpeed, &pos);
    AddParticles(ctx, map, pos, grapple.dir, 0.1f, 1, 3);
    if (cpos) {
      const float force = -200.0f;
      const ID id{.v = uint32_t(cpos->index1)};
      const auto delta = (toBtV3(cpos->pos) - toBtV3(cam.position)).normalize();
      AddParticles(ctx, map, cpos->pos, cpos->normal, 0.5f, 1, 5);
      if (!cpos->st) {
        if (id.t == 0) {
          auto *rb = map.cubes.at(id.i).rb;
          rb->setActivationState(ACTIVE_TAG);
          rb->applyImpulse(force * delta, {0, -0.1, 0});
        } else if (id.t == 1) {
          auto *rb = map.spheres.at(id.i).rb;
          rb->setActivationState(ACTIVE_TAG);
          rb->applyImpulse(force * delta, {0, -0.1, 0});
        }
      } else {
        player.player->applyCentralImpulse(-force * delta);
      }
      player.grapple = {};
    }
  }
}

void UpdatePlayerInputs(Context &ctx, const Camera &cam, Player &player) {
  GameMap &map = *player.map;

  {
    const float maxA = 0.99f * 0.5f * PI;
    const float camSpeed = 0.003f;
    player.camAngles =
        Vector2Add(player.camAngles,
                   Vector2Multiply(GetMouseDelta(), {-camSpeed, camSpeed}));
    player.camAngles.y = std::max(-maxA, std::min(maxA, player.camAngles.y));
  }

  {
    const bool sprint = IsKeyDown(KEY_LEFT_SHIFT);
    const float speed = sprint ? 5.0f : 2.5f;
    const Vector3 delta{sinf(player.camAngles.x), 0, cosf(player.camAngles.x)};
    const Vector3 side{delta.z, delta.y, -delta.x};
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

    auto *rb = player.player;
    const btVector3 speedVec = fwd * toBtV3(move) + strafe * toBtV3(side);
    const btVector3 lv = rb->getLinearVelocity();
    const btVector3 nlv = {speed * speedVec.x(), lv.y(), speed * speedVec.z()};
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
    if (ctx.targetInteraction > 0 &&
        ctx.targetInteraction <= map.interactions.size()) {
      const auto idx = ctx.targetInteraction - 1;
      ctx.targetInteraction = {};
      map.interactions[idx](ctx, map);
    }
  }

  if (IsKeyPressed(KEY_SPACE)) {
    const auto from = cam.position;
    const btVector3 bf{from.x, from.y, from.z};
    const btVector3 bt = bf + btVector3{0, -0.9, 0};
    btCollisionWorld::ClosestRayResultCallback cb{bf, bt};
    cb.m_collisionFilterMask = cb.m_collisionFilterMask & ~RbGroups::PLAYER;
    player.map->dynamics_world->rayTest(bf, bt, cb);
    if (cb.m_collisionObject != nullptr) {
      // TraceLog(LOG_INFO, "jump");
      auto *rb = player.player;
      rb->applyCentralImpulse({0, 30, 0});
    }
  }

  if (IsKeyPressed(KEY_G)) {
    const float sz = 0.5f;
    const Vector3 dir = Vector3Scale(
        Vector3Normalize(Vector3Subtract(cam.target, cam.position)), 20.0f);
    const Color col = RndCol(146, 100);
    map.cubes.push_back({{sz, sz, sz}, col, 0}, [&](size_t i, DynCube &c) {
      CreatePhysicCube(ctx, map, i, c, cam.target, {1, 0, 0}, Box,
                       DynParams{1.0f, dir});
    });
  }

  if (IsKeyPressed(KEY_F)) {
    const float sz = 0.5f * 0.5f;
    const Vector3 dir = Vector3Scale(
        Vector3Normalize(Vector3Subtract(cam.target, cam.position)), 20.0f);
    const Color col = RndCol(146, 100);
    map.spheres.push_back({sz, col, 1}, [&](size_t i, DynSphere &s) {
      CreatePhysicSphere(ctx, map, i, s, cam.target, Box, DynParams{1.0f, dir});
    });
  }

  {
    const Vector3 dir =
        Vector3Normalize(Vector3Subtract(cam.target, cam.position));
    if (IsMouseButtonPressed(0)) {
      const Vector3 gunPos = GetCameraOffset(ctx, player, {0, -0.09, -0.15});
      map.bullets.push_back({gunPos, dir, ctx.gtime});
      // shoot recoil
      player.deltaCamAngles.y -= 0.03f;
      player.deltaCamAngles.x += float(GetRandomValue(-10, 10)) / 2000.0f;
    }
    if (IsMouseButtonPressed(1)) {
      const Vector3 grapplePos = cam.position;
      player.grapple = Particle{grapplePos, dir, ctx.gtime};
    }
  }
}

bool Update(Context &ctx) {
  auto &player = ctx.player;

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
    const auto farPos =
        Vector3Add(ncam.position, Vector3Scale(Vector3Normalize(Vector3Subtract(
                                                   ncam.target, ncam.position)),
                                               200.0f));
    const auto target =
        CheckRayCollision(ctx, player.map->dynamics_world, ncam.position,
                          farPos, RbGroups::PLAYER);
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

// ================================================================================

Rectangle Inset(Rectangle r, float i = 2.0f) {
  return {r.x + i, r.y + i, r.width - 2.0f * i, r.height - 2.0f * i};
}

void DrawRope(Context &ctx, Vector3 from0, Vector3 to0, Vector3 from1,
              Vector3 to1, int segments) {
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

void RenderParticles(Context &ctx, GameMap &map) {
  const Color ctable[2] = {
      Color{253, 249, 0, 255},
      BLUE,
  };
  const float sztable[2] = {
      0.05f,
      0.01f,
  };
  map.particles.erase(
      std::remove_if(
          map.particles.begin(), map.particles.end(),
          [&](const Particle &p) -> bool {
            const float dt = ctx.gtime - p.startTime;
            const float maxt = 0.3f;
            if (dt > maxt)
              return true;
            const float speed = 3.0f;
            const float r = dt / maxt;
            const Vector3 pos = Vector3Add(
                Vector3{0.0f, -dt * dt * 2.0f, 0.0f},
                Vector3Add(p.startPos, Vector3Scale(p.dir, speed * dt)));
            const uint8_t alpha =
                static_cast<uint8_t>(std::min(255.0f, 255.0f * (1.0f - r)));
            const float sz = sztable[p.ptype] * (1 + r);
            const Color bc = ctable[p.ptype];
            DrawCube(pos, sz, sz, sz, Color{bc.r, bc.g, bc.b, alpha});
            return false;
          }),
      map.particles.end());
}

void RenderBullets(Context &ctx, GameMap &map) {
  Color col = YELLOW;
  col.a = 90;
  for (const auto &b : map.bullets) {
    const float dt0 = ctx.pgtime - b.startTime;
    const float dt1 = ctx.gtime - b.startTime;
    const float speed = BulletSpeed;
    const Vector3 from =
        Vector3Add(b.startPos, Vector3Scale(b.dir, speed * dt0));
    const Vector3 to = Vector3Add(b.startPos, Vector3Scale(b.dir, speed * dt1));
    const float sz = 0.01f;
    DrawCapsule(from, to, sz, 8, 4, col);
  }
}

void RenderViewMap(Context &ctx, const Camera &cam, Player &player,
                   GameMap &map) {
  if (player.grapple) {
    Vector3 pos{};
    const Vector3 startPos = GetCameraOffset(ctx, player, {0, -0.09, 0.15});
    const Particle &grapple = std::get<Particle>(*player.grapple);
    CheckRayCollision(ctx, map.dynamics_world, grapple, GrappleSpeed, &pos);
    DrawRope(ctx, grapple.startPos, pos, startPos, pos, 16);
  }

  for (const auto &c : map.staticModels) {
    DrawModelEx(ctx.models[c.index], c.position, {0, 1, 0}, 0, c.size, c.col);
  }

  if (ctx.targetPos)
    DrawSphere(*ctx.targetPos, 0.01f + 0.005f * sinf(20.0f * ctx.gtime),
               {255, 255, 255, 64});

  const auto getTranform =
      [](const btRigidBody *rb) -> std::tuple<Vector3, Vector3, float> {
    btTransform bt;
    rb->getMotionState()->getWorldTransform(bt);
    const auto p = bt.getOrigin();
    const auto quat = bt.getRotation();
    const auto btaxis = quat.getAxis();
    const Vector3 pos = toRlV3(p);
    const Vector3 axis = toRlV3(btaxis);
    const float radian_scale = 180.0 / PI;
    const float angle = float(quat.getAngle()) * radian_scale;
    return {pos, axis, angle};
  };

  map.cubes.forEach([&](size_t i, DynCube &c) {
    const auto &[pos, axis, angle] = getTranform(c.rb);
    DrawModelEx(ctx.models[c.index], pos, axis, angle, c.size, c.col);
  });

  map.spheres.forEach([&](size_t i, DynSphere &s) {
    const auto &[pos, axis, angle] = getTranform(s.rb);
    DrawModelEx(ctx.models[s.index], pos, axis, angle, {s.size, s.size, s.size},
                s.col);
  });

  RenderBullets(ctx, map);
  RenderParticles(ctx, map);
}

void RenderView(Context &ctx, const Camera &cam) {
  Player &player = ctx.player;
  RenderViewMap(ctx, cam, player, *player.map);
}

void RenderGui(Context &ctx) {
  if (ctx.lockCursor)
    return;
  const float o = 22.0f;
  const float y = GetScreenHeight() - o;
  const float w = GetScreenWidth();
  DrawRectangle(0, y - 0.0f, w, o, WHITE);
  GuiLabel(Inset({0, y, o, o}), "F1");
  if (GuiTextBox(Inset({o, y, w - o, o}), ctx.commandBuffer,
                 sizeof(ctx.commandBuffer), !ctx.lockCursor)) {
    ctx.command = ctx.commandBuffer;
    ctx.commandBuffer[0] = 0;
  }
}

void RenderHUD(Context &ctx) {
  if (ctx.lockCursor) {
    const float x = 0.5 * GetScreenWidth();
    const float y = 0.5 * GetScreenHeight();
    DrawRectangleRec({x - 2, y - 2, 5, 5}, BLACK);
    DrawRectangleRec({x - 1, y - 1, 3, 3}, WHITE);
  }

#ifdef DEBUG
  char buffer[2048]{};
  snprintf(buffer, sizeof(buffer), "%d fps", GetFPS());
  DrawText(buffer, 0, 0, 20, WHITE);
#endif
}

void Render(Context &ctx) {
  ctx.frame += 1;
  BeginDrawing();
  {
    BeginTextureMode(ctx.rt);
    ClearBackground(PINK);
    const Camera3D cam = GetCamera(ctx, ctx.player);
    BeginMode3D(cam);
    { RenderView(ctx, cam); }
    EndMode3D();
    RenderGui(ctx);
    RenderHUD(ctx);
    EndTextureMode();
    const float w = ctx.W;
    const float h = ctx.H;
    BeginShaderMode(ctx.postFxShader);
    {
      DrawTexturePro(ctx.rt.texture, {0, 0, w, -h}, {0, 0, w, h}, {}, 0, WHITE);
    }
    EndShaderMode();
  }
  EndDrawing();
}

// ================================================================================

int main(int, char **) {
  Context ctx{};
  Init(ctx);
  while (Update(ctx))
    Render(ctx);
  Release(ctx);
  return 0;
}
