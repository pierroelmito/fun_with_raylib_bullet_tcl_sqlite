
// STL
#include "BulletCollision/CollisionShapes/btSphereShape.h"
#include <algorithm>
#include <array>
#include <cstdint>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <tuple>
#include <vector>

// script
#include <tcl.h>

// DB
#include <sqlite3.h>

// physics
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

static_assert(sizeof(ID) == sizeof(uint32_t));

struct PhysicMat {
  float friction{};
  float frictionRoll{};
  float frictionSpin{};
  float extent{};
};

constexpr static const PhysicMat Wall{0.8f, 0.0f, 0.0f, 0.05f};
constexpr static const PhysicMat Box{0.6f, 0.0f, 0.0f, 0.0f};

struct Cube {
  Vector3 size{};
  Color col{WHITE};
  uint32_t index{};
  btRigidBody *rb{};
};

struct Sphere {
  float size{};
  Color col{WHITE};
  uint32_t index{};
  btRigidBody *rb{};
};

struct Particle {
  Vector3 startPos{};
  Vector3 dir;
  double startTime{};
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

struct Context {
  // script
  Tcl_Interp *tcl{};
  // physics
  btDefaultCollisionConfiguration *collision_configuration{};
  btCollisionDispatcher *dispatcher{};
  btBroadphaseInterface *overlapping_pair_cache{};
  btSequentialImpulseConstraintSolver *solver{};
  btDiscreteDynamicsWorld *dynamics_world{};
  std::optional<Vector3> lastHit{};
  std::set<int> selection{};
  // player
  Vector3 startPos{};
  Vector2 camAngles{};
  btRigidBody *player{};
  // GUI
  char commandBuffer[1024]{};
  std::string command{};
  std::vector<std::string> history{};
  bool lockCursor{true};
  // render
  int W{1280}, H{720};
  int FPS{60};
  int frame{};
  double gtime{};
  std::vector<Model> models{};
  std::vector<Particle> particles{};
  ObjectList<Cube> cubes;
  ObjectList<Sphere> spheres;
};

using DynParams = std::tuple<float, Vector3>;

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

void CleanRigidBody(Context &ctx, btRigidBody *&rb) {
  ctx.dynamics_world->removeRigidBody(rb);
  delete rb->getCollisionShape();
  delete rb->getMotionState();
  delete rb;
  rb = nullptr;
}

void CleanCube(Context &ctx, Cube &c) { CleanRigidBody(ctx, c.rb); }

Camera3D GetCamera(Context &ctx) {
  Camera3D r{};

  btTransform bt;
  ctx.player->getMotionState()->getWorldTransform(bt);
  const auto p = bt.getOrigin();
  const Vector3 pos = toRlV3(p + btVector3{0, 0.3, 0});
  const Vector3 delta{cosf(ctx.camAngles.y) * sinf(ctx.camAngles.x),
                      sinf(ctx.camAngles.y),
                      cosf(ctx.camAngles.y) * cosf(ctx.camAngles.x)};

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

void CreatePlayerCapsule(Context &ctx, Vector3 pos) {
  btCapsuleShape *cs = new btCapsuleShape(0.4f, 0.2f);

  const btTransform transform = MakeTransform(pos, {0, 0, 0});

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
  ctx.dynamics_world->addRigidBody(body);
  ctx.player = body;

  ctx.startPos = Vector3Subtract(pos, {0, -0.5, 0});
}

void CreatePhysicCube(Context &ctx, uint32_t index, Cube &c, const Vector3 &pos,
                      const Vector3 &rotation, const PhysicMat &pm,
                      const std::optional<DynParams> &params = {}) {
  const Vector3 halfSize = Vector3Scale(c.size, 0.5f);
  btBoxShape *collider_shape =
      new btBoxShape(toBtV3(halfSize) + btVector3{pm.extent, 0.0f, pm.extent});

  const btTransform transform = MakeTransform(pos, rotation);

  const auto np = params.value_or(DynParams{0.0f, {0.0f, 0.0f, 0.0f}});
  const float mass = 5 * std::get<0>(np);
  const btScalar object_mass(mass);
  const Vector3 li = std::get<1>(np);
  const btVector3 speed = toBtV3(li);
  const ID id{.i = index, .t = 0};

  btVector3 local_inertia{0, 0, 0};
  if (params || mass != 0.0) {
    collider_shape->calculateLocalInertia(mass, local_inertia);
  }

  btDefaultMotionState *motion_state = new btDefaultMotionState(transform);

  btRigidBody::btRigidBodyConstructionInfo rb_info{
      object_mass, motion_state, collider_shape, local_inertia};
  rb_info.m_friction = pm.friction;
  rb_info.m_rollingFriction = pm.frictionRoll;
  rb_info.m_spinningFriction = pm.frictionSpin;
  rb_info.m_linearDamping = 0.6f;
  rb_info.m_angularDamping = 0.4f;

  btRigidBody *body = new btRigidBody(rb_info);
  body->setUserIndex(id.v);
  body->setLinearVelocity(speed);
  ctx.dynamics_world->addRigidBody(body);

  c.rb = body;
}

void CreatePhysicSphere(Context &ctx, uint32_t index, Sphere &s,
                        const Vector3 &pos, const PhysicMat &pm,
                        const std::optional<DynParams> &params = {}) {
  btSphereShape *collider_shape = new btSphereShape(s.size);

  const btTransform transform = MakeTransform(pos, {0, 0, 0});

  const auto np = params.value_or(DynParams{0.0f, {0.0f, 0.0f, 0.0f}});
  const float mass = 5 * std::get<0>(np);
  const btScalar object_mass(mass);
  const Vector3 li = std::get<1>(np);
  const btVector3 speed = toBtV3(li);
  const ID id{.i = index, .t = 1};

  btVector3 local_inertia{0, 0, 0};
  if (params || mass != 0.0) {
    collider_shape->calculateLocalInertia(mass, local_inertia);
  }

  btDefaultMotionState *motion_state = new btDefaultMotionState(transform);

  btRigidBody::btRigidBodyConstructionInfo rb_info{
      object_mass, motion_state, collider_shape, local_inertia};
  rb_info.m_friction = pm.friction;
  rb_info.m_rollingFriction = pm.frictionRoll;
  rb_info.m_spinningFriction = pm.frictionSpin;
  rb_info.m_linearDamping = 0.6f;
  rb_info.m_angularDamping = 0.4f;

  btRigidBody *body = new btRigidBody(rb_info);
  body->setUserIndex(id.v);
  body->setLinearVelocity(speed);
  ctx.dynamics_world->addRigidBody(body);

  s.rb = body;
}

void LoadImgMap(Context &ctx) {
  Image img = LoadImage("assets/map00.png");
  const int floors = img.width / img.height;

  std::optional<Vector3> startPos{};

  uint8_t *const pixels = static_cast<uint8_t *>(img.data);
  const float psz = 10.0f;
  const float fh = 4.0f;
  const Vector3 szc{psz, fh, psz};

  const auto isWall = [](Color c) -> bool {
    return (c.r != 0 || c.g != 0 || c.b != 0);
  };

  const auto GetMapAt = [&](int x, int y, int f) -> Color {
    if (x >= 0 && x <= img.height && y >= 0 && y <= img.height && f >= 0 &&
        f < floors) {
      const int o = y * img.width + (x + f * img.height);
      const Color cc = GetPixelColor(pixels + 3 * o, img.format);
      return cc;
    }
    return WHITE;
  };

  for (int f = -1; f < floors + 1; ++f) {
    for (int y = 1; y < img.height - 1; ++y) {
      for (int x = 1; x < img.height - 1; ++x) {
        const Color cc = GetMapAt(x, y, f);
        const Vector3 cpos{x * psz, (f + 0.5f) * fh, y * psz};
        if (!isWall(cc)) {
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
              const Color col = RndCol(76, 10);
              ctx.cubes.push_back({szc, col, index}, [&](size_t i, Cube &c) {
                CreatePhysicCube(ctx, i, c, cpos, {0, 0, 0}, Wall);
              });
              break;
            }
          }
        }
      }
    }
  }

  if (startPos) {
    CreatePlayerCapsule(ctx, *startPos);
    // CreatePlayerCapsule(ctx, { 0, 1, 0 });
  }

  UnloadImage(img);
}

void LoadDbMap(Context &ctx) {
  sqlite3 *db = nullptr;
  SQ3(sqlite3_open("assets/test.db", &db));

  ctx.cubes.clear(CleanCube, ctx);

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
      ctx.cubes.push_back({{sx, sy, sz}, c, 1}, [&](size_t i, Cube &c) {
        CreatePhysicCube(ctx, i, c, pos, {0, 0, 0}, Wall);
      });
    }
    SQ3(sqlite3_finalize(st));
  }

  SQ3(sqlite3_close(db));
}

int Cmd_load_map(ClientData data, Tcl_Interp *, int, Tcl_Obj *const *) {
  TraceLog(LOG_INFO, "load map");
  LoadDbMap(*((Context *)data));
  return TCL_OK;
}

// ================================================================================

void InitDB(Context &ctx) { SQ3(sqlite3_initialize()); }

void InitRender(Context &ctx) {
#ifndef DEBUG
  // SetTraceLogLevel(LOG_WARNING);
#endif
  SetConfigFlags(FLAG_MSAA_4X_HINT);
  InitWindow(ctx.W, ctx.H, "main");
  SetTargetFPS(ctx.FPS);
  SetWindowState(FLAG_VSYNC_HINT);
  DisableCursor();
  SetExitKey(0);

  const auto makeTexturedCube = [](const char *path) -> Model {
    Texture t = LoadTexture(path);
    GenTextureMipmaps(&t);
    SetTextureFilter(t, TEXTURE_FILTER_BILINEAR);
    SetTextureFilter(t, TEXTURE_FILTER_ANISOTROPIC_16X);
    Mesh msh = GenMeshCube(1, 1, 1);
    Model m = LoadModelFromMesh(msh);
    m.materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = t;
    return m;
  };

  const auto makeTexturedSphere = [](const char *path) -> Model {
    Texture t = LoadTexture(path);
    GenTextureMipmaps(&t);
    SetTextureFilter(t, TEXTURE_FILTER_BILINEAR);
    SetTextureFilter(t, TEXTURE_FILTER_ANISOTROPIC_16X);
    Mesh msh = GenMeshSphere(1, 16, 16);
    Model m = LoadModelFromMesh(msh);
    m.materials[0].maps[MATERIAL_MAP_DIFFUSE].texture = t;
    return m;
  };

  ctx.models.push_back(makeTexturedCube("assets/crate.png"));
  ctx.models.push_back(makeTexturedSphere("assets/crate.png"));
  ctx.models.push_back(makeTexturedCube("assets/block00.png"));
  ctx.models.push_back(makeTexturedCube("assets/block01.png"));
}

void InitPhysics(Context &ctx) {
  ctx.collision_configuration = new btDefaultCollisionConfiguration();
  ctx.dispatcher = new btCollisionDispatcher(ctx.collision_configuration);
  ctx.overlapping_pair_cache = new btDbvtBroadphase();
  ctx.solver = new btSequentialImpulseConstraintSolver;
  ctx.dynamics_world =
      new btDiscreteDynamicsWorld(ctx.dispatcher, ctx.overlapping_pair_cache,
                                  ctx.solver, ctx.collision_configuration);
  ctx.dynamics_world->setGravity(btVector3(0, -10, 0));
}

void Init(Context &ctx) {
  ctx.tcl = Tcl_CreateInterp();
  Tcl_CreateObjCommand(ctx.tcl, "load_map", Cmd_load_map, &ctx, nullptr);
  InitDB(ctx);
  InitRender(ctx);
  InitPhysics(ctx);
  LoadDbMap(ctx);
  LoadImgMap(ctx);
}

// ================================================================================

void ReleaseDB(Context &ctx) { SQ3(sqlite3_shutdown()); }

void ReleaseRender(Context &ctx) {
  for (Model &m : ctx.models)
    UnloadModel(m);
  ctx.models.clear();
  CloseWindow();
}

void ReleasePhysics(Context &ctx) {
  ctx.cubes.clear(CleanCube, ctx);
  CleanRigidBody(ctx, ctx.player);
  delete ctx.dynamics_world;
  delete ctx.solver;
  delete ctx.overlapping_pair_cache;
  delete ctx.dispatcher;
  delete ctx.collision_configuration;
}

void Release(Context &ctx) {
  ReleasePhysics(ctx);
  ReleaseRender(ctx);
  ReleaseDB(ctx);
  Tcl_DeleteInterp(ctx.tcl);
}

// ================================================================================

bool Update(Context &ctx) {
  const bool esc = IsKeyPressed(KEY_ESCAPE);
  const bool quit = ctx.lockCursor && esc;

  ctx.gtime += 1.0 / float(ctx.FPS);

  if (IsKeyPressed(KEY_F1) || esc) {
    ctx.lockCursor = !ctx.lockCursor;
    if (ctx.lockCursor)
      DisableCursor();
    else
      EnableCursor();
  }

  Camera3D cam = GetCamera(ctx);

  if (ctx.lockCursor) {
    const float camSpeed = 0.003f;
    ctx.camAngles = Vector2Add(
        ctx.camAngles, Vector2Multiply(GetMouseDelta(), {-camSpeed, camSpeed}));

    {
      const bool sprint = IsKeyDown(KEY_LEFT_SHIFT);
      const float speed = sprint ? 5.0f : 2.5f;
      const Vector3 delta{sinf(ctx.camAngles.x), 0, cosf(ctx.camAngles.x)};
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

      auto *rb = ctx.player;
      /*if (fwd != 0.0f || strafe != 0.0f)*/ {
        const btVector3 speedVec = fwd * toBtV3(move) + strafe * toBtV3(side);
        btVector3 lv = rb->getLinearVelocity();
        lv = {speed * speedVec.x(), lv.y(), speed * speedVec.z()};
        rb->setLinearVelocity(lv);
      }
      rb->setAngularFactor(0.0);
    }

    ctx.dynamics_world->stepSimulation(1.0 / float(ctx.FPS), 1);

    if (IsKeyPressed(KEY_F5)) {
      LoadDbMap(ctx);
    }

    if (IsKeyPressed(KEY_SPACE)) {
      const auto from = cam.position;
      const btVector3 bf{from.x, from.y, from.z};
      const btVector3 bt = bf + btVector3{0, -0.9, 0};
      btCollisionWorld::AllHitsRayResultCallback cb{bf, bt};
      ctx.dynamics_world->rayTest(bf, bt, cb);
      cb.m_collisionObjects.remove(ctx.player);
      if (cb.m_collisionObjects.size() > 0) {
        // TraceLog(LOG_INFO, "jump");
        auto *rb = ctx.player;
        rb->applyCentralImpulse({0, 30, 0});
      }
    }

    if (IsKeyPressed(KEY_G)) {
      const float sz = 0.5f;
      const Vector3 dir = Vector3Scale(
          Vector3Normalize(Vector3Subtract(cam.target, cam.position)), 20.0f);
      const Color col = RndCol(146, 100);
      ctx.cubes.push_back({{sz, sz, sz}, col, 0}, [&](size_t i, Cube &c) {
        CreatePhysicCube(ctx, i, c, cam.target, {1, 0, 0}, Box,
                         DynParams{1.0f, dir});
      });
    }

    if (IsKeyPressed(KEY_F)) {
      const float sz = 0.5f * 0.5f;
      const Vector3 dir = Vector3Scale(
          Vector3Normalize(Vector3Subtract(cam.target, cam.position)), 20.0f);
      const Color col = RndCol(146, 100);
      ctx.spheres.push_back({sz, col, 1}, [&](size_t i, Sphere &s) {
        CreatePhysicSphere(ctx, i, s, cam.target, Box, DynParams{1.0f, dir});
      });
    }

    if (IsMouseButtonPressed(0) || IsMouseButtonPressed(1)) {
      const bool grapple = IsMouseButtonPressed(1);
      const float force = grapple ? -200.0f : 100.0f;
      const auto from = cam.position;
      const auto dir = Vector3Normalize(Vector3Subtract(cam.target, from));
      const auto to = Vector3Add(from, Vector3Scale(dir, 2000.0f));
      const btVector3 bf{from.x, from.y, from.z};
      const btVector3 bt{to.x, to.y, to.z};
      btCollisionWorld::ClosestRayResultCallback cb{bf, bt};
      ctx.dynamics_world->rayTest(bf, bt, cb);
      if (cb.hasHit()) {
        const auto h = cb.m_hitPointWorld;
        ctx.lastHit = toRlV3(h);
        if (cb.m_collisionObject) {
          if (!grapple) {
            for (int i = 0; i < 10; ++i) {
              const float dx = float(GetRandomValue(-1000, 1000)) / 1000.0f;
              const float dy = float(GetRandomValue(-1000, 1000)) / 1000.0f;
              const float dz = float(GetRandomValue(-1000, 1000)) / 1000.0f;
              ctx.particles.push_back(
                  {toRlV3(h),
                   toRlV3(cb.m_hitNormalWorld + btVector3{dx, dy, dz}),
                   ctx.gtime});
            }
          }
          const bool st = cb.m_collisionObject->isStaticObject();
          const int idx = cb.m_collisionObject->getUserIndex();
          const ID id{.v = uint32_t(idx)};
          if (!st) {
            if (id.t == 0) {
              ctx.selection = {id.i};
              auto *rb = ctx.cubes.at(id.i).rb;
              rb->setActivationState(ACTIVE_TAG);
              rb->applyImpulse(force * (bt - bf).normalize(), {0, -0.1, 0});
            } else if (id.t == 1) {
              ctx.selection = {};
              auto *rb = ctx.spheres.at(id.i).rb;
              rb->setActivationState(ACTIVE_TAG);
              rb->applyImpulse(force * (bt - bf).normalize(), {0, -0.1, 0});
            }
          } else {
            if (grapple)
              ctx.player->applyCentralImpulse(-force * (bt - bf).normalize());
          }
        } else {
          ctx.selection = {};
        }
      } else {
        ctx.selection = {};
        ctx.lastHit = {};
      }
    }
  } else {
    if (!ctx.command.empty()) {
      TraceLog(LOG_INFO, "console: %s", ctx.command.c_str());
      ExecCommand(ctx, ctx.command);
      ctx.history.push_back(ctx.command);
      ctx.command = {};
    }
  }

  return !WindowShouldClose() && !quit;
}

// ================================================================================

Rectangle Inset(Rectangle r, float i = 2.0f) {
  return {r.x + i, r.y + i, r.width - 2.0f * i, r.height - 2.0f * i};
}

void RenderView(Context &ctx, const Camera &cam) {
  DrawCapsule(Vector3Add(cam.position, {0, -0.2, 0}),
              ctx.lastHit.value_or(cam.target), 0.005f, 4, 4, WHITE);
#if 0
  {
    btTransform bt;
    ctx.player->getMotionState()->getWorldTransform(bt);
    const auto p = bt.getOrigin();
    const auto quat = bt.getRotation();
    const auto btaxis = quat.getAxis();
    const Vector3 pos = toRlV3(p);
    const Vector3 axis = toRlV3(btaxis);
    const float radian_scale = 57.296;
    const float angle = float(quat.getAngle()) * radian_scale;
    const float sz = 0.1f;
    DrawModelEx(ctx.models[0], pos, axis, angle, Vector3{sz, sz, sz}, WHITE);
  }
#endif

  ctx.cubes.forEach([&](size_t i, Cube &c) {
    btTransform bt;
    c.rb->getMotionState()->getWorldTransform(bt);
    const auto p = bt.getOrigin();
    const auto quat = bt.getRotation();
    const auto btaxis = quat.getAxis();
    const Vector3 pos = toRlV3(p);
    const Vector3 axis = toRlV3(btaxis);
    const float radian_scale = 57.296;
    const float angle = float(quat.getAngle()) * radian_scale;
    DrawModelEx(ctx.models[c.index], pos, axis, angle, c.size, c.col);
  });

  ctx.spheres.forEach([&](size_t i, Sphere &s) {
    btTransform bt;
    s.rb->getMotionState()->getWorldTransform(bt);
    const auto p = bt.getOrigin();
    const auto quat = bt.getRotation();
    const auto btaxis = quat.getAxis();
    const Vector3 pos = toRlV3(p);
    const Vector3 axis = toRlV3(btaxis);
    const float radian_scale = 57.296;
    const float angle = float(quat.getAngle()) * radian_scale;
    DrawModelEx(ctx.models[s.index], pos, axis, angle, {s.size, s.size, s.size},
                s.col);
  });
}

void RenderGui(Context &ctx) {
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

void RenderParticles(Context &ctx) {
  ctx.particles.erase(
      std::remove_if(
          ctx.particles.begin(), ctx.particles.end(),
          [&](const Particle &p) -> bool {
            const float dt = ctx.gtime - p.startTime;
            const float maxt = 0.3f;
            if (dt > maxt)
              return true;
            const float speed = 3.0f;
            const float r = dt / maxt;
            const Vector3 pos = Vector3Add(
                Vector3{0.0f, -dt * dt * 10.0f, 0.0f},
                Vector3Add(p.startPos, Vector3Scale(p.dir, speed * dt)));
            const uint8_t alpha =
                static_cast<uint8_t>(std::min(255.0f, 255.0f * (1.0f - r)));
            const float sz = 0.05f * (1 + r);
            DrawCube(pos, sz, sz, sz, Color{253, 249, 0, alpha});
            return false;
          }),
      ctx.particles.end());
}

void RenderHUD(Context &ctx) {
  if (ctx.lockCursor) {
    const float x = 0.5 * GetScreenWidth();
    const float y = 0.5 * GetScreenHeight();
    DrawRectangleRec({x - 2, y - 2, 5, 5}, BLACK);
    DrawRectangleRec({x - 1, y - 1, 3, 3}, WHITE);
  }

  char buffer[2048]{};
  snprintf(buffer, sizeof(buffer), "%d fps", GetFPS());
  DrawText(buffer, 0, 0, 20, WHITE);
}

void Render(Context &ctx) {
  ctx.frame += 1;
  const Camera3D cam = GetCamera(ctx);
  BeginDrawing();
  {
    ClearBackground(PINK);
    BeginMode3D(cam);
    {
      RenderView(ctx, cam);
      RenderParticles(ctx);
    }
    EndMode3D();
    RenderGui(ctx);
    RenderHUD(ctx);
  }
  EndDrawing();
}

// ================================================================================

int main(int, char **) {
  Context ctx;
  Init(ctx);
  while (Update(ctx))
    Render(ctx);
  Release(ctx);
  return 0;
}
