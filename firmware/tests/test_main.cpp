#include "test.h"

int main() {
    int failed_cases = 0;
    for (auto& c : t::registry()) {
        const int before = t::failures();
        c.fn();
        const bool ok = t::failures() == before;
        if (!ok) ++failed_cases;
        printf("[%s] %s\n", ok ? " OK " : "FAIL", c.name.c_str());
    }
    printf("\n%zu tests, %d checks, %d failed checks, %d failed tests\n", t::registry().size(), t::checks(),
           t::failures(), failed_cases);
    return failed_cases ? 1 : 0;
}
