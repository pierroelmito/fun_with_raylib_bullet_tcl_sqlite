-- magick convert map00.xcf pnm:- | magick montage - -background blue -tile x1 -geometry +1+1 map00.png

add_rules("mode.debug", "mode.release", "mode.profile")

target("main")

set_kind("binary")
set_policy("build.warning", true)
set_warnings("allextra")
set_rundir(".")

add_includedirs({
  "/usr/include/bullet",
})

if is_mode("debug") then
  add_defines({
    "DEBUG"
  })
end

add_defines({
  "BT_USE_DOUBLE_PRECISION"
})

add_links({
  "tcl",
  "raylib",
  "sqlite3",
  "BulletDynamics",
  "BulletCollision",
  "LinearMath",
})

add_files({
  "src/*.cpp",
})

add_cxflags({
  "-std=c++20",
})
