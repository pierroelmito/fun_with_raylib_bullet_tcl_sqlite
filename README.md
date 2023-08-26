
# fun_with_raylib_bullet_tcl_sqlite
Having fun with a toy project using raylib, bullet, tcl and sqlite.

![Alt text](/screenshots/screen000.png?raw=true)

## Build and run
* dependencies:
  * bullet
  * raylib
  * tcl
  * sqlite
* Linux:
  * xmake && xmake run
* Windows MSYS2 (for some reason xmake build is failing...):
  * make && ./main.exe

## controls:
* move / sprint / jump: WASD / shift / space
* shoot: left mouse click
* grapple: right mouse click
* throw box / throw sphere: G / F

## todo
* better shaders (fog, postprocess, ...)
* use some gltf models?
* test designing maps with goxel?
