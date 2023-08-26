

CXX_FLAGS += -I/mingw64/include/bullet -m64 -Wall -Wextra -std=c++20 -O1 -g -DDEBUG
LDD_FLAGS += -ltcl -lraylib -lsqlite3 -lBulletDynamics -lBulletCollision -lLinearMath

main.exe : src/main.cpp
	g++ -o main.exe src/main.cpp $(CXX_FLAGS) $(LDD_FLAGS)

clean:
	rm -f main.exe

