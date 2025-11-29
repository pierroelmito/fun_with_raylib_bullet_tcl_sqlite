#pragma once

#include <algorithm>
#include <cstdint>
#include <memory>
#include <vector>

template <typename T, size_t CK>
class ChunkedFreeList {
public:
	template <typename... ARGS>
	T* add(const ARGS&... args)
	{
		checkFl();
		Cell* c = fl.back();
		fl.pop_back();
		T* r = new (&c[0]) T(args...);
		return r;
	}
	void remove(T* t)
	{
		t->~T();
		fl.push_back((Cell*)t);
	}
	void sortFl() { std::sort(fl.rbegin(), fl.rend()); }
	size_t usedCount() const { return data.size() * CK - fl.size(); }

protected:
	using Cell = std::array<uint8_t, sizeof(T)>;
	using Chunk = std::array<Cell, CK>;
	using Data = std::vector<std::unique_ptr<Chunk>>;
	using FreeList = std::vector<Cell*>;
	Data data {};
	FreeList fl {};
	void checkFl()
	{
		if (!fl.empty())
			return;
		auto& ptr = data.emplace_back(std::make_unique<Chunk>());
		fl.reserve(fl.size() + CK);
		for (auto it = ptr->rbegin(); it != ptr->rend(); ++it) {
			fl.push_back(&(*it));
		}
	}
};
