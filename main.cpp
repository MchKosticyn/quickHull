#include <algorithm>
#include <array>
#include "stack"

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

vertices createSimplex(const vertices listVertices) {
    coordinates first = listVertices.front();
    coordinates EP[6] = {first, first, first, first, first, first};

    for(auto vertex : listVertices) {
        if( vertex[0] <= EP[0][0] ) { EP[0] = vertex; }
        if( vertex[0] >= EP[1][0] ) { EP[1] = vertex; }
        if( vertex[1] <= EP[2][1] ) { EP[2] = vertex; }
        if( vertex[1] >= EP[3][1] ) { EP[3] = vertex; }
        if( vertex[2] <= EP[4][2] ) { EP[4] = vertex; }
        if( vertex[2] >= EP[5][2] ) { EP[5] = vertex; }
    }

    double maxDist = 0;
    coordinates triangleP1 = EP[0], triangleP2 = EP[0], triangleP3 = EP[0];
    for (auto point1 : EP)
        for (auto point2 : EP) {
            double dist = pointDist(point1, point2);
            if (dist > maxDist) {
                maxDist = dist;
                triangleP1 = point1;
                triangleP2 = point2;
            }
        }

    maxDist = 0;
    for (auto point : EP) {
        double dist = pointLineDist(triangleP1, triangleP2, point);
        if (dist > maxDist) {
            maxDist = dist;
            triangleP3 = point;
        }
    }

    maxDist = 0;
    coordinates apex = EP[0];
    for (auto point : listVertices) {
        double dist = pointPlaneDist(triangleP1, triangleP2, triangleP3, point);
        if (dist > maxDist) {
            maxDist = dist;
            apex = point;
        }
    }
    vertices Res;
    if (pointPlaneDist(triangleP1, triangleP2, triangleP3, apex) > 0)
         Res = { triangleP1, triangleP3, triangleP2, apex };
    else Res = { triangleP1, triangleP2, triangleP3, apex };
    return Res;
};

double pointFaceDist(triangle face, coordinates point) {
    return pointPlaneDist(face[0], face[1], face[2], point);
}

bool faceIsVisible(coordinates eyePoint, tFace face) {
    return pointFaceDist(face.plane, eyePoint) > 0;
}

void addPointsToFaces(tFace* faces, unsigned long faces_count, vertices listVertices) {
    for (auto vertex : listVertices)
        for (int i = 0; i < faces_count; i++)
            if (faceIsVisible(vertex, faces[i])) {
                faces[i].points.push_back(vertex);
                break;
            }
}

int main() {
    return 0;
}