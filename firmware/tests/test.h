// Minimal dependency-free unit test framework.
#pragma once
#include <math.h>
#include <stdio.h>

#include <functional>
#include <string>
#include <vector>

namespace t {
struct Case { std::string name; std::function<void()> fn; };
inline std::vector<Case>& registry() { static std::vector<Case> r; return r; }
inline int& failures() { static int f = 0; return f; }
inline int& checks() { static int c = 0; return c; }
struct Reg { Reg(const char* n, std::function<void()> f) { registry().push_back({n, f}); } };
}  // namespace t

#define TEST(name) static void name(); static t::Reg reg_##name(#name, name); static void name()
#define CHECK(cond) do { ++t::checks(); if (!(cond)) { ++t::failures(); \
    printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); } } while (0)
#define CHECK_NEAR(a, b, tol) do { ++t::checks(); double _a = (a), _b = (b); if (fabs(_a - _b) > (tol)) { ++t::failures(); \
    printf("  FAIL %s:%d: %s=%g vs %s=%g (tol %g)\n", __FILE__, __LINE__, #a, _a, #b, _b, (double)(tol)); } } while (0)
