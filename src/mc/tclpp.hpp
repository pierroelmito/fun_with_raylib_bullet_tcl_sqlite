#pragma once

#include <array>
#include <map>
#include <optional>
#include <set>
#include <string>
#include <tuple>
#include <vector>

#include <tcl.h>

template <typename T>
struct TclConvertTo {
	static bool To(Tcl_Interp* tcl, Tcl_Obj* obj, T* v) = delete;
};

template <>
struct TclConvertTo<std::string> {
	static bool To(Tcl_Interp*, Tcl_Obj* obj, std::string* v)
	{
		int sz {};
		*v = Tcl_GetStringFromObj(obj, &sz);
		return true;
	}
};

template <>
struct TclConvertTo<int> {
	static bool To(Tcl_Interp* tcl, Tcl_Obj* obj, int* v)
	{
		int r {};
		Tcl_GetIntFromObj(tcl, obj, &r);
		*v = r;
		return true;
	}
};

template <typename T>
struct TclConvertTo<std::vector<T>> {
	static bool To(Tcl_Interp* tcl, Tcl_Obj* obj, std::vector<T>* v)
	{
		int sz {};
		Tcl_ListObjLength(tcl, obj, &sz);
		v->resize(sz);
		for (int i = 0; i < sz; ++i) {
			Tcl_Obj* item {};
			Tcl_ListObjIndex(tcl, obj, i, &item);
			TclConvertTo<T>::To(tcl, item, &(*v)[i]);
		}
		return true;
	}
};

template <typename K, typename V>
struct TclConvertTo<std::pair<K, V>> {
	static bool To(Tcl_Interp* tcl, Tcl_Obj* obj, std::pair<K, V>* v)
	{
		int sz {};
		Tcl_ListObjLength(tcl, obj, &sz);
		if (sz != 2)
			return false;
		Tcl_Obj* vk {};
		Tcl_ListObjIndex(tcl, obj, 0, &vk);
		TclConvertTo<K>::To(tcl, vk, &v->first);
		Tcl_Obj* vv {};
		Tcl_ListObjIndex(tcl, obj, 1, &vv);
		TclConvertTo<V>::To(tcl, vv, &v->second);
		return true;
	}
};

template <typename T, size_t N>
struct TclConvertTo<std::array<T, N>> {
	static bool To(Tcl_Interp* tcl, Tcl_Obj* obj, std::array<T, N>* v)
	{
		int sz {};
		Tcl_ListObjLength(tcl, obj, &sz);
		if (sz != N)
			return false;
		for (int i = 0; i < sz; ++i) {
			Tcl_Obj* item {};
			Tcl_ListObjIndex(tcl, obj, i, &item);
			TclConvertTo<T>::To(tcl, item, &(*v)[i]);
		}
		return true;
	}
};

template <typename T>
T TclTo(Tcl_Interp* tcl, Tcl_Obj* obj)
{
	T r {};
	TclConvertTo<T>::To(tcl, obj, &r);
	return r;
}

template <typename T>
struct TclConvertFrom {
	static Tcl_Obj* From(Tcl_Interp* tcl, const T& v) = delete;
};

template <>
struct TclConvertFrom<std::string> {
	static Tcl_Obj* From(Tcl_Interp*, const std::string& v)
	{
		return Tcl_NewStringObj(v.c_str(), v.size());
	}
};

template <size_t N>
struct TclConvertFrom<char[N]> {
	static Tcl_Obj* From(Tcl_Interp*, std::string_view v)
	{
		return Tcl_NewStringObj(&v[0], v.size());
	}
};

template <typename T>
Tcl_Obj* TclFrom(Tcl_Interp* tcl, const T& v)
{
	Tcl_Obj* r {};
	r = TclConvertFrom<T>::From(tcl, v);
	return r;
}

template <typename... T>
bool TclCall(Tcl_Interp* tcl, const T&... args)
{
	Tcl_Obj* objs[sizeof...(T)] = { TclFrom(tcl, args)... };
	Tcl_EvalObjv(tcl, sizeof...(T), objs, 0);
	return true;
}
