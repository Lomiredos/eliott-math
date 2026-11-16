#include "math/Vector.hpp"

using namespace ee::math;

void takeGrid(Vector<2, int>) {}

#if CASE == 1
Vector<5, float> v;
#elif CASE == 2
float z = Vector<2, float>{}.z;
#elif CASE == 3
float l = Vector<2, int>{}.length();
#elif CASE == 4
void f() {
    takeGrid({1.7, 2.9});
}
#elif CASE == 5
Vector<2, int> g = Vector<2, float>{1, 2};
#elif CASE == 6
auto c = cross(Vector<4, float>{}, Vector<4, float>{});
#elif CASE == 7
bool b = nearlyEqual(Vector<2, int>{}, Vector<2, int>{});
#endif
