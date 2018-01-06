#include <algorithm>
#include <array>
#include "queue"

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
    auto findX = find(b.plane.begin(), b.plane.end(), a.plane[0]);
    auto findY = find(b.plane.begin(), b.plane.end(), a.plane[1]);
    auto findZ = find(b.plane.begin(), b.plane.end(), a.plane[2]);
    return (findX != b.plane.end()) && (findY != b.plane.end()) && (findZ != b.plane.end());
}

typedef vector<tFace> tFaces;

double eps = 0.0000001;

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

vertices createSimplex(const vertices& listVertices) {
    coordinates first = listVertices.front();
    coordinates EP[6] = {first, first, first, first, first, first};

    for (auto vertex : listVertices) {
        if (vertex[0] <= EP[0][0]) EP[0] = vertex;
        if (vertex[0] >= EP[1][0]) EP[1] = vertex;
        if (vertex[1] <= EP[2][1]) EP[2] = vertex;
        if (vertex[1] >= EP[3][1]) EP[3] = vertex;
        if (vertex[2] <= EP[4][2]) EP[4] = vertex;
        if (vertex[2] >= EP[5][2]) EP[5] = vertex;
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
    return pointFaceDist(face.plane, eyePoint) > eps;
}

void addPointsToFaces(tFace* faces, unsigned long faces_count, vertices listVertices) {
    for (auto vertex : listVertices)
        for (int i = 0; i < faces_count; i++)
            if (faceIsVisible(vertex, faces[i])) {
                faces[i].points.push_back(vertex);
                break;
            }
}

tFaces quickHull(const vertices& listVertices) {
    queue<tFace> Queue;
    vertices simplex = createSimplex(listVertices);
    tFaces tmpfaces = {{{simplex[0], simplex[1], simplex[2]}},
                       {{simplex[0], simplex[2], simplex[3]}},
                       {{simplex[1], simplex[3], simplex[2]}},
                       {{simplex[0], simplex[3], simplex[1]}}};
    tFaces faces;
    for (auto &tmpface : tmpfaces)
        if (find(faces.begin(), faces.end(), tmpface) == faces.end())
            faces.push_back(tmpface);
    addPointsToFaces(faces.data(), faces.size(), listVertices);
    for (auto face : faces)
        if (!face.points.empty()) Queue.push(face);

    while (!Queue.empty()) {
        tFace face = Queue.front();
        Queue.pop();
        if (!face.points.empty()) {
            double maxDist = -1;
            coordinates furthest = {0, 0, 0};
            for (auto point : face.points) {
                double dist = pointFaceDist(face.plane, point);
                if (dist > maxDist) {
                    maxDist = dist;
                    furthest = point;
                }
            }

            vertices listUnclaimedVertices;
            tFaces newFaces;
            tFaces needToDelete;
            for (auto curFace : faces)
                if (faceIsVisible(furthest, curFace)) {
                    for (auto point : curFace.points)
                        listUnclaimedVertices.push_back(point);
                    tFaces tmpFaces = {{{curFace.plane[2], furthest, curFace.plane[1]}},
                                       {{curFace.plane[0], curFace.plane[1], furthest}},
                                       {{curFace.plane[0], furthest, curFace.plane[2]}}};
                    for (auto &tmpFace : tmpFaces) {
                        auto alreadyExists = find(newFaces.begin(), newFaces.end(), tmpFace);
                        if (alreadyExists != newFaces.end())
                            newFaces.erase(alreadyExists);
                        else
                            newFaces.push_back(tmpFace);
                    }
                    needToDelete.push_back(curFace);
                }

            for (auto &iter : needToDelete)
                faces.erase(remove(faces.begin(), faces.end(), iter), faces.end());

            addPointsToFaces(newFaces.data(), newFaces.size(), listUnclaimedVertices);
            for (auto &newFace : newFaces) {
                faces.push_back(newFace);
                Queue.push(newFace);
            }
        }
    }
    return faces;
};

void testHull(vertices &verts, tFaces goldValue) {
    tFaces faces = quickHull(verts);
    bool test = true;
    if (faces.size() != goldValue.size())
        test = false;
    else {
        for (int i = 0; i < goldValue.size(); i++)
            if (goldValue[i].plane != faces[i].plane)
                test = false;
    }
    int i = 0;
    for (auto face : faces) {
        for (auto point : face.plane) {
            printf("(%F, %F, %F)\n", point[0], point[1], point[2]);
        }
        printf("\n");
        ++i;
    }
    if (test)
        printf("test completed\n");
    else
        printf("test failed\n");
    printf("number of facets: %i\n", i);
}


int main() {
    printf("First test: one dimension\n");
    vertices verts = {{1,0,0}, {1,0,0}, {1,0,0}, {1,0,0}, {1,0,0}, {1,0,0}, {1,0,0}, {1,0,0}, {1,0,0}, {1,0,0}, {1,0,0}};
    tFaces goldValue = {{{(coordinates){1,0,0}, {1,0,0}, {1,0,0}}}};
    testHull(verts, goldValue);
    printf("\nSecond test: cube with (0,0,0) point inside\n");
    verts = {{0, 0, 0}, {-1, -1, 1}, {1, -1, 1}, {-1, -1, -1}, {1, -1, -1}, {-1, 1, 1}, {1, 1, 1}, {-1, 1, -1}, {1, 1, -1}};
    goldValue = {{{(coordinates){1, -1, -1}, {1, 1, -1}, {1, 1, 1}}}, {{(coordinates){-1, 1, -1}, {1, 1, 1}, {1, 1, -1}}}, {{(coordinates){-1, 1, -1}, {1, 1, -1}, {1, -1, -1}}}, {{(coordinates){-1, -1, 1}, {-1, -1, -1}, {1, -1, -1}}}, {{(coordinates){-1, 1, -1}, {1, -1, -1}, {-1, -1, -1}}}, {{(coordinates){-1, 1, -1}, {-1, -1, -1}, {-1, -1, 1}}}, {{(coordinates){1, -1, -1}, {1, -1, 1}, {-1, -1, 1}}}, {{(coordinates){1, 1, 1}, {-1, -1, 1}, {1, -1, 1}}}, {{(coordinates){1, 1, 1}, {1, -1, 1}, {1, -1, -1}}}, {{(coordinates){1, 1, 1}, {-1, 1, 1}, {-1, -1, 1}}}, {{(coordinates){-1, 1, -1}, {-1, -1, 1}, {-1, 1, 1}}}, {{(coordinates){-1, 1, -1}, {-1, 1, 1}, {1, 1, 1}}}};
    testHull(verts, goldValue);
    printf("\nThird test: dodecahedron\n");
    verts = {{0.469, 0.469, 0.469}, {0.290, 0.000, 0.759}, {-0.759, -0.290, 0.000}, {0.759, 0.290, 0.000}, {-0.469, 0.469, -0.469}, {0.000, -0.759, -0.290}, {-0.759, 0.290, 0.000}, {0.469, -0.469, 0.469}, {-0.469, 0.469, 0.469}, {-0.469, -0.469, 0.469}, {0.469, -0.469, -0.469}, {0.290, 0.000, -0.759}, {-0.469, -0.469, -0.469}, {0.000, -0.759, 0.290}, {0.000, 0.759, -0.290}, {-0.290, 0.000, 0.759}, {0.759, -0.290, 0.000}, {-0.290, 0.000, -0.759}, {0.469, 0.469, -0.469}, {0.000, 0.759, 0.290}};
    goldValue = {{{(coordinates){0.000000, 0.759000, 0.290000}, {-0.290000, 0.000000, 0.759000}, {0.290000, 0.000000, 0.759000}}}, {{(coordinates){0.000000, 0.759000, 0.290000}, {-0.469000, 0.469000, 0.469000}, {-0.290000, 0.000000, 0.759000}}}, {{(coordinates){-0.759000, 0.290000, 0.000000}, {-0.290000, 0.000000, 0.759000}, {-0.469000, 0.469000, 0.469000}}}, {{(coordinates){-0.759000, 0.290000, 0.000000}, {-0.469000, 0.469000, 0.469000}, {0.000000, 0.759000, 0.290000}}}, {{(coordinates){0.759000, -0.290000, 0.000000}, {0.290000, 0.000000, -0.759000}, {0.759000, 0.290000, 0.000000}}}, {{(coordinates){0.759000, -0.290000, 0.000000}, {0.759000, 0.290000, 0.000000}, {0.290000, 0.000000, 0.759000}}}, {{(coordinates){0.759000, -0.290000, 0.000000}, {0.000000, -0.759000, 0.290000}, {0.000000, -0.759000, -0.290000}}}, {{(coordinates){-0.290000, 0.000000, 0.759000}, {0.000000, -0.759000, 0.290000}, {0.290000, 0.000000, 0.759000}}}, {{(coordinates){-0.759000, 0.290000, 0.000000}, {-0.759000, -0.290000, 0.000000}, {-0.290000, 0.000000, 0.759000}}}, {{(coordinates){0.000000, -0.759000, -0.290000}, {0.000000, -0.759000, 0.290000}, {-0.759000, -0.290000, 0.000000}}}, {{(coordinates){0.000000, -0.759000, -0.290000}, {-0.759000, -0.290000, 0.000000}, {-0.469000, -0.469000, -0.469000}}}, {{(coordinates){0.290000, 0.000000, -0.759000}, {0.469000, -0.469000, -0.469000}, {0.000000, -0.759000, -0.290000}}}, {{(coordinates){0.759000, -0.290000, 0.000000}, {0.000000, -0.759000, -0.290000}, {0.469000, -0.469000, -0.469000}}}, {{(coordinates){0.759000, -0.290000, 0.000000}, {0.469000, -0.469000, -0.469000}, {0.290000, 0.000000, -0.759000}}}, {{(coordinates){0.759000, 0.290000, 0.000000}, {0.469000, 0.469000, 0.469000}, {0.290000, 0.000000, 0.759000}}}, {{(coordinates){0.000000, 0.759000, 0.290000}, {0.290000, 0.000000, 0.759000}, {0.469000, 0.469000, 0.469000}}}, {{(coordinates){0.000000, 0.759000, 0.290000}, {0.469000, 0.469000, 0.469000}, {0.759000, 0.290000, 0.000000}}}, {{(coordinates){0.000000, -0.759000, 0.290000}, {-0.469000, -0.469000, 0.469000}, {-0.759000, -0.290000, 0.000000}}}, {{(coordinates){-0.290000, 0.000000, 0.759000}, {-0.759000, -0.290000, 0.000000}, {-0.469000, -0.469000, 0.469000}}}, {{(coordinates){-0.290000, 0.000000, 0.759000}, {-0.469000, -0.469000, 0.469000}, {0.000000, -0.759000, 0.290000}}}, {{(coordinates){0.000000, 0.759000, 0.290000}, {0.759000, 0.290000, 0.000000}, {0.000000, 0.759000, -0.290000}}}, {{(coordinates){-0.759000, 0.290000, 0.000000}, {0.000000, 0.759000, 0.290000}, {0.000000, 0.759000, -0.290000}}}, {{(coordinates){-0.759000, 0.290000, 0.000000}, {0.000000, 0.759000, -0.290000}, {-0.469000, 0.469000, -0.469000}}}, {{(coordinates){0.000000, -0.759000, -0.290000}, {-0.469000, -0.469000, -0.469000}, {-0.290000, 0.000000, -0.759000}}}, {{(coordinates){0.000000, -0.759000, -0.290000}, {-0.290000, 0.000000, -0.759000}, {0.290000, 0.000000, -0.759000}}}, {{(coordinates){-0.759000, 0.290000, 0.000000}, {-0.469000, 0.469000, -0.469000}, {-0.290000, 0.000000, -0.759000}}}, {{(coordinates){-0.759000, -0.290000, 0.000000}, {-0.290000, 0.000000, -0.759000}, {-0.469000, -0.469000, -0.469000}}}, {{(coordinates){-0.759000, 0.290000, 0.000000}, {-0.290000, 0.000000, -0.759000}, {-0.759000, -0.290000, 0.000000}}}, {{(coordinates){0.000000, 0.759000, -0.290000}, {-0.290000, 0.000000, -0.759000}, {-0.469000, 0.469000, -0.469000}}}, {{(coordinates){0.290000, 0.000000, -0.759000}, {-0.290000, 0.000000, -0.759000}, {0.000000, 0.759000, -0.290000}}}, {{(coordinates){0.000000, -0.759000, 0.290000}, {0.469000, -0.469000, 0.469000}, {0.290000, 0.000000, 0.759000}}}, {{(coordinates){0.759000, -0.290000, 0.000000}, {0.290000, 0.000000, 0.759000}, {0.469000, -0.469000, 0.469000}}}, {{(coordinates){0.759000, -0.290000, 0.000000}, {0.469000, -0.469000, 0.469000}, {0.000000, -0.759000, 0.290000}}}, {{(coordinates){0.759000, 0.290000, 0.000000}, {0.469000, 0.469000, -0.469000}, {0.000000, 0.759000, -0.290000}}}, {{(coordinates){0.290000, 0.000000, -0.759000}, {0.000000, 0.759000, -0.290000}, {0.469000, 0.469000, -0.469000}}}, {{(coordinates){0.290000, 0.000000, -0.759000}, {0.469000, 0.469000, -0.469000}, {0.759000, 0.290000, 0.000000}}}};
    testHull(verts, goldValue);
    return 0;
}