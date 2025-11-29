-- magick convert map00.xcf pnm:- | magick montage - -background blue -tile x1 -geometry +1+1 map00.png

add_rules("mode.debug", "mode.release", "mode.profile")

if not is_mode("debug") then
	set_policy("build.optimization.lto", true)
end

rule("shader")
	set_extensions(".args")
	on_buildcmd_file(function (target, batchcmds, sourcefile, opt)
		batchcmds:mkdir(target:targetdir())
		local dir = path.directory(sourcefile)
		local src = path.filename(sourcefile)
		local dst = 'data/shaders/' .. path.basename(sourcefile) .. ".glsl"
		local targetfile = '../../' .. dst
		local global = path.join(dir, "global.shader")
		local common = path.join(dir, "common.glsl")
		batchcmds:show_progress(opt.progress, "${color.build.object}shader %s", targetfile)
		batchcmds:vrunv('/bin/sh', {'-c', "cd " .. dir .. "; mcpp -k `cat " .. src .. "` -P global.shader | grep -v '^$$' > " .. targetfile})
		batchcmds:add_depfiles(sourcefile, global, common)
	end)




target("main")
	set_policy("build.warning", true)
	set_warnings("all", "extra")
	set_kind("binary")
	add_files("src/*.cpp")
	add_cxflags("-std=c++20")
	add_defines({
	  "BT_USE_DOUBLE_PRECISION"
	})
	add_includedirs("/usr/include/bullet")
	add_links({
	  "tcl",
	  "raylib",
	  "sqlite3",
	  "BulletDynamics",
	  "BulletCollision",
	  "LinearMath",
	})
	set_rundir(".")
	if is_mode("debug") then
		add_defines("DEBUG")
	end

