
#include "common.hpp"

#include <algorithm>

void InitPhysicsMap(Context&, GameMap& map)
{
	map.collision_configuration = new btDefaultCollisionConfiguration();
	map.dispatcher = new btCollisionDispatcher(map.collision_configuration);
	map.overlapping_pair_cache = new btDbvtBroadphase();
	map.solver = new btSequentialImpulseConstraintSolver;
	map.dynamics_world = new btDiscreteDynamicsWorld(map.dispatcher, map.overlapping_pair_cache, map.solver, map.collision_configuration);
	map.dynamics_world->setGravity(btVector3(0, -10, 0));
}

using SliceCoord = std::array<int, 3>;
using SliceSpawns = std::vector<std::pair<SliceCoord, std::string>>;

btCollisionShape* CreateBoxShape(Context&, const Vector3& size, const PhysicMat& pm)
{
	const Vector3 halfSize = size * 0.5f;
	btCollisionShape* collider_shape = new btBoxShape(toBtV3(halfSize) + btVector3 { pm.extent, 0.0f, pm.extent });
	return collider_shape;
}

btRigidBody* CreateRigidBody(Context& ctx, GameMap& map, const Vector3& pos, const Vector3& size, const Vector3& rotation, Color col, uint32_t mshIndex, btCollisionShape* shape, const PhysicMat& pm, const std::optional<DynParams>& params = {})
{
	map.staticModels.push_back({ size, pos, col, mshIndex });
	btRigidBody* rb = CreatePhysicShape(ctx, map, RbGroups::WALL, shape, 0, ~0u, pos, rotation, pm, params);
	map.staticBodies.push_back(rb);
	return rb;
}

btRigidBody* CreatePhysicCubeStatic(Context& ctx, GameMap& map, const Vector3& pos, const Vector3& size, const Vector3& rotation, Color col, uint32_t mshIndex, const PhysicMat& pm, const std::optional<DynParams>& params = {})
{
	btCollisionShape* shape = CreateBoxShape(ctx, size, pm);
	return CreateRigidBody(ctx, map, pos, size, rotation, col, mshIndex, shape, pm, params);
}

void LoadImgMap(Context& ctx, GameMap& map, const std::string& path, const SliceSpawns& spawns, const std::function<bool(Color)>& isWall, const std::function<Color(Color, int)>& makeColor)
{
	Image img = LoadImage(path.c_str());
	const int floors = img.width / img.height;
	const int stride = img.format == PIXELFORMAT_UNCOMPRESSED_R8G8B8 ? 3 : 4;

	TraceLog(LOG_INFO, "img %dx%d, floors %d", img.width, img.height, floors);

	std::optional<Vector3> startPos {};

	uint8_t* const pixels = static_cast<uint8_t*>(img.data);
	const float psz = 4.0f;
	const float fh = 4.0f;
	const Vector3 szc { psz, fh, psz };

	const auto GetMapAt = [&](int x, int y, int f) -> Color {
		if (x >= 0 && x < img.height && y >= 0 && y < img.height && f >= 0 && f < floors) {
			const int o = y * img.width + (x + f * img.height);
			const Color cc = GetPixelColor(pixels + stride * o, img.format);
			return cc;
		}
		return { 255, 255, 255, 0 };
	};

	btCollisionShape* shapeWall = CreateBoxShape(ctx, szc, Wall);

	for (int f = -1; f < floors + 1; ++f) {
		bool emptyFloor = f >= 0;
		for (int y = 0; y < img.height; ++y) {
			for (int x = 0; x < img.height; ++x) {
				const Color cc = GetMapAt(x, y, f);
				const Vector3 cpos { x * psz, (f + 0.5f) * fh, y * psz };
				if (!isWall(cc)) {
					emptyFloor = false;
					const Color co = GetMapAt(x, y, f - 1);
					if (isWall(co)) {
						startPos = Vector3 { cpos.x, f * fh + 0.5f, cpos.z };
					}
				} else {
					const int offsets[6][3] = {
						{ 1, 0, 0 },
						{ -1, 0, 0 },
						{ 0, 1, 0 },
						{ 0, -1, 0 },
						{ 0, 0, 1 },
						{ 0, 0, -1 },
					};
					const uint32_t index = GetRandomValue(2, 3);
					for (int i = 0; i < 6; ++i) {
						const Color co = GetMapAt(x + offsets[i][0], y + offsets[i][1], f + offsets[i][2]);
						if (!isWall(co)) {
							const Color col = makeColor(cc, f);
							CreateRigidBody(ctx, map, cpos, szc, { 0, 0, 0 }, col, index, shapeWall, Wall);
							break;
						}
					}
				}
			}
		}
		if (emptyFloor)
			break;
	}

	std::sort(map.staticModels.begin(), map.staticModels.end(), [](const StaMdl& a, const StaMdl& b) {
		return a.index < b.index;
	});

	if (!spawns.empty()) {
		btCollisionShape* shapeSpawn = CreateBoxShape(ctx, Vector3 { 1, 1, 1 }, Wall);
		for (const auto& spawn : spawns) {
			const auto [x, y, f] = spawn.first;
			const auto action = spawn.second;
			TraceLog(LOG_INFO, "\t(%d, %d, %d) '%s'", x, y, f, action.c_str());
			const Vector3 cpos { x * psz, f * fh + 0.5f, y * psz };
			auto* rb = CreateRigidBody(ctx, map, cpos, Vector3 { 1, 1, 1 }, Vector3 { 0, 0, 0 }, WHITE, 2, shapeSpawn, Wall);
			rb->setUserIndex2(map.interactions.size() + 1);
			map.interactions.push_back([=](Context& ctx, GameMap&) {
				TraceLog(LOG_INFO, "interaction: %s", action.c_str());
				Tcl_Eval(ctx.tcl, action.c_str());
			});
		}
	}

	if (startPos)
		map.startPos = *startPos;

	UnloadImage(img);
}

void LoadLayersMap(Context& ctx, GameMap& map, const std::string& path)
{
	const auto isWall = [](Color c) -> bool {
		return (c.r != 0 || c.g != 0 || c.b != 0);
	};
	const auto makeColor = [](Color, int f) {
		const int fc = f & 1;
		return RndCol(76 + fc * 30, 6);
	};
	return LoadImgMap(ctx, map, path, {}, isWall, makeColor);
}

void LoadSlicesMap(Context& ctx, GameMap& map, const std::string& path, const SliceSpawns& spawns)
{
	const auto isWall = [](Color c) -> bool { return (c.a < 128); };
	const auto makeColor = [](Color, int f) {
		const int fc = f & 1;
		return RndCol(76 + fc * 30, 6);
	};
	return LoadImgMap(ctx, map, path, spawns, isWall, makeColor);
}

void LoadDbMap(Context& ctx, GameMap& map, const std::string& path)
{
	sqlite3* db = nullptr;
	SQ3(sqlite3_open(path.c_str(), &db));

	map.cubes.clear(CleanCube, ctx, map);

	{
		sqlite3_stmt* st = nullptr;
		const std::string query = "select * from cubes;";
		const char* tail = nullptr;
		SQ3(sqlite3_prepare_v3(db, query.c_str(), query.size(), 0, &st, &tail));
		while (sqlite3_step(st) == SQLITE_ROW) {
			const int s = 1;
			const float px = sqlite3_column_double(st, s + 0);
			const float py = sqlite3_column_double(st, s + 1);
			const float pz = sqlite3_column_double(st, s + 2);
			const float sx = sqlite3_column_double(st, s + 3);
			const float sy = sqlite3_column_double(st, s + 4);
			const float sz = sqlite3_column_double(st, s + 5);
			const auto* col = sqlite3_column_text(st, s + 6);
			Color c = BLUE;
			if (col) {
				int r, g, b;
				sscanf((const char*)col, "%02x%02x%02x", &r, &g, &b);
				c = Color { uint8_t(r), uint8_t(g), uint8_t(b), 255 };
			}
			const Vector3 pos { px, py + 0.5f * sy, pz };
			CreatePhysicCubeStatic(ctx, map, pos, { sx, sy, sz }, { 0, 0, 0 }, c, 2, Wall);
		}
		SQ3(sqlite3_finalize(st));
	}

	SQ3(sqlite3_close(db));

	map.startPos = { 0, 0.5, 0 };
}

std::shared_ptr<GameMap> GetMap(Context& ctx, const std::string& mapName)
{
	auto& map = ctx.gameMaps[mapName];
	if (!map) {
		TraceLog(LOG_INFO, "init new map");
		map = std::make_shared<GameMap>();
		InitPhysicsMap(ctx, *map);
		TclCall(ctx.tcl, "load_map", mapName);
	}
	return map;
}

int Cmd_load_map_db(ClientData data, Tcl_Interp* tcl, int argc, Tcl_Obj* const* argv)
{
	if (argc != 3)
		return TCL_ERROR;

	Context& ctx = *((Context*)data);

	const auto mapName = TclTo<std::string>(tcl, argv[1]);
	const auto mapPath = TclTo<std::string>(tcl, argv[2]);

	TraceLog(LOG_INFO, "Cmd_load_map_db %s %s", mapName.c_str(), mapPath.c_str());
	auto map = GetMap(ctx, mapName);
	LoadDbMap(ctx, *map, mapPath);

	return TCL_OK;
}

int Cmd_load_map_slices(ClientData data, Tcl_Interp* tcl, int argc, Tcl_Obj* const* argv)
{
	if (argc != 4)
		return TCL_ERROR;

	Context& ctx = *((Context*)data);

	const auto mapName = TclTo<std::string>(tcl, argv[1]);
	const auto mapPath = TclTo<std::string>(tcl, argv[2]);
	const auto spawns = TclTo<SliceSpawns>(tcl, argv[3]);

	TraceLog(LOG_INFO, "Cmd_load_map_slices %s %s", mapName.c_str(), mapPath.c_str());
	auto map = GetMap(ctx, mapName);
	LoadSlicesMap(ctx, *map, mapPath, spawns);

	return TCL_OK;
}

int Cmd_load_map_heightfield(ClientData data, Tcl_Interp* tcl, int argc, Tcl_Obj* const* argv)
{
	if (argc != 3)
		return TCL_ERROR;

	Context& ctx = *((Context*)data);

	const auto mapName = TclTo<std::string>(tcl, argv[1]);
	const auto mapPath = TclTo<std::string>(tcl, argv[2]);

	TraceLog(LOG_INFO, "Cmd_load_map_slices %s %s", mapName.c_str(), mapPath.c_str());
	auto map = GetMap(ctx, mapName);
	const float os = -0.5f * HmSize;
	const float oh = -0.5f * HmHeight;

	map->staticModels.push_back({ { 1, 1, 1 }, { os, oh, os }, WHITE, 4 });

	btCollisionShape* collider_shape = new btHeightfieldTerrainShape(ctx.hm.width, ctx.hm.height, (unsigned char*)(ctx.hm.data), HmHeight / 255.0f, oh, -oh, 1, false);
	btRigidBody* rb = CreatePhysicShape(ctx, *map, RbGroups::WALL, collider_shape, 0, ~0u, { 0, -0.5f * HmHeight, 0 }, { 0, 0, 0 }, Wall, {});
	map->staticBodies.push_back(rb);
	map->startPos = { 0, 0.5, 0 };

	return TCL_OK;
}

int Cmd_teleport(ClientData data, Tcl_Interp* tcl, int argc, Tcl_Obj* const* argv)
{
	if (argc != 2)
		return TCL_ERROR;

	Context& ctx = *((Context*)data);

	const std::string mapName = TclTo<std::string>(tcl, argv[1]);

	TraceLog(LOG_INFO, "Cmd_teleport %s", mapName.c_str());
	auto map = GetMap(ctx, mapName);
	TeleportPlayer(ctx, ctx.player, map);
	return TCL_OK;
}

int Cmd_tracelog(ClientData, Tcl_Interp* tcl, int argc, Tcl_Obj* const* argv)
{
	if (argc != 2)
		return TCL_ERROR;

	const std::string line = TclTo<std::string>(tcl, argv[1]);

	TraceLog(LOG_INFO, "%s", line.c_str());
	return TCL_OK;
}

// ================================================================================

void InitDB(Context&) { SQ3(sqlite3_initialize()); }

void InitRender(Context& ctx)
{
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
	ctx.basicShader = LoadShader("assets/shaders/default.vs", "assets/shaders/default.fs");
	ctx.postFxShader = LoadShader(nullptr, "assets/shaders/postfx.fs");

	const auto makeTexturedCube = [&](const char* path) -> Model {
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

	const auto makeTexturedSphere = [&](const char* path) -> Model {
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

	const auto makeHm = [&](const char* path, Image& img) -> Model {
		img = LoadImage(path);
		Mesh hm = GenMeshHeightmap(img, { HmSize, HmHeight, HmSize });
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

void Init(Context& ctx)
{
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
