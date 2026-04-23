/* bigtest.cpp -- 2.3 MB DJGPP C++ binary, regression gate for "does
 * dosemu's DPMI host behave correctly under a real, nontrivial 32-bit
 * PM client with C++ STL, regex, global constructors/destructors, and
 * enough code size that memory layout stresses the MCB allocator".
 *
 * Writes "bigtest-done" on clean exit; any crash (SIGFPE recursive-
 * fault, #UD jump to bogus EIP, stack smash from buffer overflows in
 * AH=3F/40) misses the marker and the harness catches it.
 *
 * Build:  i586-pc-msdosdjgpp-g++ -O0 -o ../BIGTEST.EXE bigtest.cpp
 * (-O0 so the compiler doesn't fold out the template churn that makes
 *  the binary cross 2 MB, which is where cpp.exe-size failures lived).
 */
#include <iostream>
#include <vector>
#include <string>
#include <map>
#include <set>
#include <algorithm>
#include <sstream>
#include <fstream>
#include <memory>
#include <functional>
#include <numeric>
#include <regex>

template<int N>
void f() { f<N-1>(); }
template<> void f<0>() {}

static struct GlobalCtorDtor {
    GlobalCtorDtor()  { std::cout << "ctor\n"; }
    ~GlobalCtorDtor() { std::cout << "dtor\n"; }
} g;

int main() {
    std::vector<std::string> v;
    for (int i = 0; i < 100; i++) v.push_back("s" + std::to_string(i));
    std::sort(v.begin(), v.end());

    std::map<std::string, int> m;
    for (auto& s : v) m[s] = s.length();
    if (m.size() != v.size()) return 1;

    std::regex re("s(\\d+)");
    std::smatch sm;
    if (!std::regex_match(v[0], sm, re)) return 2;

    f<20>();

    std::stringstream ss;
    for (int i = 0; i < 1000; i++) ss << i << ",";
    if (ss.str().size() < 2000) return 3;

    std::cout << "bigtest-done\n";
    return 0;
}
