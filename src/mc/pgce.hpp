#pragma once

#include <algorithm>
#include <cstddef>
#include <optional>
#include <tuple>
#include <type_traits>
#include <utility>
#include <vector>

#define STATS 1

namespace pgce {

#if STATS
#define STAT(X) X
struct Stats {
	struct Data {
		int e { 0 };
		int d { 0 };
		int s { 0 };
	};
	static Data& Get()
	{
		static Data data;
		return data;
	}
};
#else
#define STAT(X)
#endif

template <typename Base, typename Tuple, std::size_t I = 0>
struct tuple_ref_index;

template <typename Base, typename Head, typename... Tail, std::size_t I>
struct tuple_ref_index<Base, std::tuple<Head, Tail...>, I>
	: std::conditional<std::is_base_of<Base, Head>::value, std::integral_constant<std::size_t, I>, tuple_ref_index<Base, std::tuple<Tail...>, I + 1>>::type {
};

template <typename Base, typename Tuple>
auto tuple_ref_by_inheritance(Tuple&& tuple)
	-> decltype(std::get<tuple_ref_index<Base, typename std::decay<Tuple>::type>::value>(std::forward<Tuple>(tuple)))
{
	return std::get<tuple_ref_index<Base, typename std::decay<Tuple>::type>::value>(std::forward<Tuple>(tuple));
}

template <typename T, typename Tuple>
struct has_type;
template <typename T, typename... Us>
struct has_type<T, std::tuple<Us...>>
	: std::disjunction<std::is_same<T, Us>...> { };

template <typename T, typename Tuple>
struct has_base;
template <typename T, typename... Us>
struct has_base<T, std::tuple<Us...>>
	: std::disjunction<std::is_base_of<T, Us>...> { };

template <typename... ARGS>
struct Visit {
private:
	template <typename E, typename F>
	static int VisitE(const E& e, F&& f)
	{
		if constexpr ((has_type<std::decay_t<ARGS>, E>::value && ...)) {
			STAT(Stats::Get().e++);
			f(std::get<std::decay_t<ARGS>>(e)...);
		} else if constexpr ((has_base<std::decay_t<ARGS>, E>::value && ...)) {
			STAT(Stats::Get().e++);
			f(tuple_ref_by_inheritance<std::decay_t<ARGS>>(e)...);
		}
		return 0;
	}
	template <typename E, typename F>
	static int VisitE(E& e, F&& f)
	{
		if constexpr ((has_type<std::decay_t<ARGS>, E>::value && ...)) {
			STAT(Stats::Get().e++);
			f(std::get<std::decay_t<ARGS>>(e)...);
		} else if constexpr ((has_base<std::decay_t<ARGS>, E>::value && ...)) {
			STAT(Stats::Get().e++);
			f(tuple_ref_by_inheritance<std::decay_t<ARGS>>(e)...);
		}
		return 0;
	}
	// --------------------------------------------------------------
	template <typename E, typename F>
	static int V(const std::vector<E>& es, F&& f)
	{
		if constexpr ((has_type<std::decay_t<ARGS>, E>::value && ...)) {
			STAT(Stats::Get().d++);
			for (auto& e : es)
				VisitE(e, f);
		} else if constexpr ((has_base<std::decay_t<ARGS>, E>::value && ...)) {
			STAT(Stats::Get().d++);
			for (auto& e : es)
				VisitE(e, f);
		} else {
			STAT(Stats::Get().s++);
		}
		return 0;
	}
	template <typename E, typename F>
	static int V(std::vector<E>& es, F&& f)
	{
		if constexpr ((has_type<std::decay_t<ARGS>, E>::value && ...)) {
			STAT(Stats::Get().d++);
			for (auto& e : es)
				VisitE(e, f);
		} else if constexpr ((has_base<std::decay_t<ARGS>, E>::value && ...)) {
			STAT(Stats::Get().d++);
			for (auto& e : es)
				VisitE(e, f);
		} else {
			STAT(Stats::Get().s++);
		}
		return 0;
	}
	// --------------------------------------------------------------
	template <typename E, typename F>
	static int V(const std::optional<E>& es, F&& f)
	{
		if constexpr ((has_type<std::decay_t<ARGS>, E>::value && ...)) {
			STAT(Stats::Get().d++);
			if (es)
				VisitE(*es, f);
		} else {
			STAT(Stats::Get().s++);
		}
		return 0;
	}
	template <typename E, typename F>
	static int V(std::optional<E>& es, F&& f)
	{
		if constexpr ((has_type<std::decay_t<ARGS>, E>::value && ...)) {
			STAT(Stats::Get().d++);
			if (es)
				VisitE(*es, f);
		} else {
			STAT(Stats::Get().s++);
		}
		return 0;
	}
	// --------------------------------------------------------------
	template <typename E, typename F, size_t N>
	static int V(const std::array<std::optional<E>, N>& es, F&& f)
	{
		if constexpr ((has_type<std::decay_t<ARGS>, E>::value && ...)) {
			STAT(Stats::Get().d++);
			for (auto& e : es) {
				if (e)
					VisitE(*e, f);
			}
		} else {
			STAT(Stats::Get().s++);
		}
		return 0;
	}
	template <typename E, typename F, size_t N>
	static int V(std::array<std::optional<E>, N>& es, F&& f)
	{
		if constexpr ((has_type<std::decay_t<ARGS>, E>::value && ...)) {
			STAT(Stats::Get().d++);
			for (auto& e : es) {
				if (e)
					VisitE(*e, f);
			}
		} else {
			STAT(Stats::Get().s++);
		}
		return 0;
	}
	// --------------------------------------------------------------
	template <typename E, typename F, size_t N>
	static int V(const std::array<E, N>& es, F&& f)
	{
		if constexpr ((has_type<std::decay_t<ARGS>, E>::value && ...)) {
			STAT(Stats::Get().d++);
			for (auto& e : es)
				VisitE(e, f);
		} else {
			STAT(Stats::Get().s++);
		}
		return 0;
	}
	template <typename E, typename F, size_t N>
	static int V(std::array<E, N>& es, F&& f)
	{
		if constexpr ((has_type<std::decay_t<ARGS>, E>::value && ...)) {
			STAT(Stats::Get().d++);
			for (auto& e : es)
				VisitE(e, f);
		} else {
			STAT(Stats::Get().s++);
		}
		return 0;
	}

public:
	template <typename F, typename... ES>
	static void V(std::tuple<ES...>& db, F&& f)
		requires std::is_invocable_v<F, ARGS&...>
	{
		auto _ = { (V(std::get<ES>(db), f))... };
		(void)_;
	}
	template <typename F, typename... ES>
	static void V(const std::tuple<ES...>& db, F&& f)
		requires std::is_invocable_v<F, ARGS&...>
	{
		auto _ = { (V(std::get<ES>(db), f))... };
		(void)_;
	}
};

template <typename... ARGS>
struct Remove {
private:
	template <typename E, typename F>
	static int R(std::vector<E>& es, F&& f)
	{
		if constexpr ((has_type<std::decay_t<ARGS>, E>::value && ...)) {
			STAT(Stats::Get().d++);
			es.erase(
				std::remove_if(es.begin(), es.end(), [&](E& e) -> bool {
					return f(std::get<std::decay_t<ARGS>>(e)...);
				}),
				es.end());
		} else if constexpr ((has_base<std::decay_t<ARGS>, E>::value && ...)) {
			STAT(Stats::Get().d++);
			es.erase(
				std::remove_if(es.begin(), es.end(), [&](E& e) -> bool {
					return f(tuple_ref_by_inheritance<std::decay_t<ARGS>>(e)...);
				}),
				es.end());
		} else {
			STAT(Stats::Get().s++);
		}
		return 0;
	}
	template <typename T, typename F>
	static int R(T& es, F&& f) { return 0; }

public:
	template <typename F, typename... ES>
	static void R(std::tuple<ES...>& db, F&& f)
		requires std::is_invocable_v<F, ARGS&...>
	{
		auto _ = { (R(std::get<ES>(db), f))... };
		(void)_;
	}
};

template <typename E>
struct Make {
private:
	template <typename T>
	static int Set(E& e, T&& v)
	{
		std::get<T>(e) = v;
		return 0;
	}

public:
	template <typename T>
	static E& M0(T& db, E&& e)
	{
		auto& es = std::get<std::vector<E>>(db);
		return es.emplace_back(e);
	}
	template <typename T, typename... ARGS>
	static E& M1(T& db, ARGS&&... args)
	{
		auto& es = std::get<std::vector<E>>(db);
		E& e = es.emplace_back();
		auto _ = { (Set<ARGS>(e, std::move(args)))... };
		(void)_;
		return e;
	}
};

} // namespace pgce
