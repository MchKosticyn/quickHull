#include <algorithm>
#include <array>

using namespace std;

typedef array<double, 3> coordinates;

typedef vector<coordinates> vertices;

typedef array<coordinates, 3> triangle;

typedef struct {
    triangle plane;
    vertices points;
} tFace;

bool operator ==(tFace a, tFace b)
{
    return (a.plane[0] == b.plane[0]) && (a.plane[1] == b.plane[1]) && (a.plane[2] == b.plane[2]);
}

typedef vector<tFace> tFaces;

int main() {
    return 0;
}