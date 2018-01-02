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

coordinates createVect(coordinates p1, coordinates p2) {
    return { p2[0] - p1[0], p2[1] - p1[1], p2[2] - p1[2] };
}

double vectMod (coordinates vect) {
    return sqrt(pow(vect[0], 2) + pow(vect[1], 2) + pow(vect[2], 2));
}

double scalarProd(coordinates v1, coordinates v2) {
    return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
}

coordinates vectProd(coordinates v1, coordinates v2){
    return  { v1[1] * v2[2] - v1[2] * v2[1], v1[2] * v2[0] - v1[0] * v2[2], v1[0] * v2[1] - v1[1] * v2[0] };
}

double tripleProd(coordinates v1, coordinates v2, coordinates v3) {
    return scalarProd(v1, vectProd(v2, v3));
}

double pointDist(coordinates p1, coordinates p2) {
    return vectMod(createVect(p1, p2));
}

double pointLineDist(coordinates lineP1, coordinates lineP2, coordinates point) {
    coordinates lineVect = createVect(lineP1, lineP2);
    return vectMod(vectProd(lineVect, createVect(lineP1, point))) / vectMod(lineVect);
}

double pointPlaneDist(coordinates planeP1, coordinates planeP2, coordinates planeP3, coordinates point) {
    coordinates baseV1 = createVect(planeP1, planeP2);
    coordinates baseV2 = createVect(planeP1, planeP3);
    return tripleProd(baseV1, baseV2, createVect(planeP1, point)) / vectMod(vectProd(baseV1, baseV2));
}

int main() {
    return 0;
}