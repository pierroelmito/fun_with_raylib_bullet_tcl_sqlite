
#include "common.hpp"

// DB
#include <sqlite3.h>

int main(int, char**)
{
	Context ctx {};
	Init(ctx);
	while (Update(ctx))
		Render(ctx);
	Release(ctx);
	return 0;
}
