#include "../src/mesh/Triangulation.hpp"
#include "../src/simulation/PrimitiveSDF.hpp"
#include <glm/glm.hpp>
#include <glm/gtc/constants.hpp>
#include <cstdio>
#include <cmath>
#include <utility>
#include <vector>
#include <unordered_map>

int main() {
    const int N = 5000;
    std::vector<glm::vec3> pos, nrm;
    pos.reserve(N);
    nrm.reserve(N);
    const float R = 0.5f;
    for (int i = 0; i < N; ++i) {
        const float golden = glm::pi<float>() * (3.0f - std::sqrt(5.0f));
        const float y = 1.0f - (2.0f * i) / (N - 1.0f);
        const float radius = std::sqrt(1.0f - y * y);
        const float theta = golden * i;
        glm::vec3 d(radius * std::cos(theta), y, radius * std::sin(theta));
        pos.push_back(d * R);
        nrm.push_back(d);
    }

    SphereSDF sphere({0, 0, 0}, R);
    Triangulation::Parameters p;
    p.maxEdgeLength = 1.4f;
    p.normalThreshold = 0.3f;
    p.edgeMidpointTolerance = 0.05f;

    float spacing = std::sqrt(4.0f * glm::pi<float>() * R * R / N);
    printf("Real-Spacing: %.4f\n", spacing);

    Triangulation tri;
    tri.build(pos, nrm, spacing, sphere, p);

    auto st = tri.stats();
    const auto& tris = tri.triangles();
    printf("Triangles: %d\n", st.totalTriangles);
    printf("Rejected: length=%d normal=%d midpoint=%d\n", st.rejectedLength, st.rejectedNormal, st.rejectedMidpoint);
    printf("Degenerate: %d\n", st.degenerate);
    printf("WrongOrientation: %d\n", st.wrongOrientation);

    // Euler-Charakteristik der geschlossenen Flaeche: V - E + F = 2
    if (st.totalTriangles > 0) {
        std::vector<unsigned int> visited((size_t)N, 0);
        unsigned long long edgeHalfSum = 0;
        for (const auto& t : tris) {
            visited[t.i0] = visited[t.i1] = visited[t.i2] = 1;
            edgeHalfSum += 3; // halbe Summe: jeder Kante zaehlt doppelt bei triangulierter Sphaeren-Oberflaeche
        }
        int usedV = 0;
        for (auto v : visited) usedV += v;
        // Fuer eine geschlossene Mannigfaltigkeit gilt 3F = 2E.
        // V - E + F = 2  =>  V - 1.5F + F = 2 => V = 2 + F/2
        long long actualE = (3ll * st.totalTriangles) / 2;
        int expectV = 2 + st.totalTriangles / 2;
        printf("V=%d F=%d E=%lld, Euler erwartet V=F/2+2=%d -> %s\n",
               usedV, st.totalTriangles, actualE, expectV,
               (usedV == expectV) ? "OK" : "ABWEICHUNG");
    }

    // Randkanten zaehlen: Kanten, die nur von 1 Dreieck genutzt werden
    std::unordered_map<unsigned long long, int> edgeCount;
    auto edgeKey = [](unsigned a, unsigned b) -> unsigned long long {
        if (a > b) std::swap(a, b);
        return (static_cast<unsigned long long>(a) << 32) | b;
    };
    for (const auto& t : tris) {
        edgeCount[edgeKey(t.i0, t.i1)]++;
        edgeCount[edgeKey(t.i1, t.i2)]++;
        edgeCount[edgeKey(t.i2, t.i0)]++;
    }
    int boundaryEdges = 0, interiorEdges = 0, tooMany = 0;
    for (auto& [k, c] : edgeCount) {
        if (c == 1) boundaryEdges++;
        else if (c == 2) interiorEdges++;
        else tooMany++;
    }
    long long chi = 1000 - (long long)edgeCount.size() + st.totalTriangles;
    printf("Kanten gesamt=%zu Rand=%d innen=%d >2=%d\n",
           edgeCount.size(), boundaryEdges, interiorEdges, tooMany);
    printf("Euler chi = V-E+F = %lld\n", chi);

    bool ok = st.totalTriangles > 0 && st.degenerate == 0 && st.wrongOrientation == 0 && boundaryEdges == 0;
    printf("\n%s\n", ok ? "TEST PASS" : "TEST FAIL (mit Randkanten)");
    return ok ? 0 : 1;
}