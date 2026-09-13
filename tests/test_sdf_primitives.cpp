#include "../src/simulation/PrimitiveSDF.hpp"
#include <glm/glm.hpp>
#include <glm/gtc/constants.hpp>
#include <cmath>
#include <cstdio>

// M6: SDF-Primitiva (Kugel, Ellipsoid, Torus, Hantel/konkav).
// Prueft: phi == 0 auf der Oberfläche, |gradient| == 1 (SDF-Charakter),
// bounds enthalten die Form, surfaceArea stimmt mit den erwarteten Formeln ueberein.
static bool near(float a, float b, float tol = 1e-3f) { return std::fabs(a - b) <= tol; }

static int check(const char* name, const SDF& sdf, int samples) {
    int fails = 0;
    for (int i = 0; i < samples; ++i) {
        // Probe auf der Oberfläche: jeder Punkt p mit phi(p)=0 hat |grad|=1.
        glm::vec3 p = sdf.boundsMin() + glm::vec3(
            (i * 7 % 13) / 13.0f, (i * 11 % 17) / 17.0f, (i * 13 % 19) / 19.0f)
            * (sdf.boundsMax() - sdf.boundsMin());
        SDFSample s = sdf.sample(p);
        float gn = glm::length(s.gradient);
        if (gn < 0.99f || gn > 1.01f) {
            printf("  [%s] |grad| = %.3f an p=(%.2f,%.2f,%.2f) (kein SDF-Charakter)\n",
                name, gn, p.x, p.y, p.z);
            ++fails;
        }
    }
    return fails;
}

int main() {
    int fails = 0;

    // Kugel
    {
        SphereSDF s({0,0,0}, 1.0f);
        fails += check("Kugel", s, 40);
        if (!near(s.sample({1,0,0}).distance, 0.0f)) { printf("Kugel phi(1,0,0)!=0\n"); ++fails; }
        if (!near(s.sample({0.5f,0,0}).distance, -0.5f)) { printf("Kugel phi(0.5)!= -0.5\n"); ++fails; }
        if (!near(s.surfaceArea(), 4.0f*glm::pi<float>())) { printf("Kugel Flaeche\n"); ++fails; }
        if (s.boundsMin() != glm::vec3(-1) || s.boundsMax() != glm::vec3(1)) { printf("Kugel bounds\n"); ++fails; }
    }

    // Ellipsoid (a=b=c ist Kugel)
    {
        EllipsoidSDF s({0,0,0}, 1.0f, 1.0f, 1.0f);
        fails += check("Ellipsoid(Kugel)", s, 40);
        if (!near(s.sample({1,0,0}).distance, 0.0f)) { printf("Ellipsoid phi(1,0,0)!=0\n"); ++fails; }
        if (!near(s.surfaceArea(), 4.0f*glm::pi<float>(), 0.05f)) { printf("Ellipsoid(a=b=c) Flaeche\n"); ++fails; }
        EllipsoidSDF e({0,0,0}, 2.0f, 1.0f, 1.0f);
        // Punkt auf der Oberfläche: (2,0,0)
        if (!near(e.sample({2,0,0}).distance, 0.0f, 1e-2f)) { printf("Ellipsoid(2,1,1) phi(2,0,0)!=0\n"); ++fails; }
        // Streckung: (2,0,0) muss zur groessten Halbachse hin orientiert sein
        glm::vec3 gn = e.sample({2,0,0}).gradient;
        if (gn.x < 0.5f) { printf("Ellipsoid Gradient x\n"); ++fails; }
    }

    // Torus (R=1, r=0.5): Oberflächenpunkte (R+r,0,0) und (1,0.5,0)? Nein:
    // (R,0,0)+(0,r,0)? Der Punkt (1,0.5,0) hat lenXz=1 → g=(0,0.5) → d=0.
    {
        TorusSDF s({0,0,0}, 1.0f, 0.5f);
        fails += check("Torus", s, 40);
        if (!near(s.sample({0.5f,0,0}).distance, 0.0f)) { printf("Torus phi(0.5,0,0)!=0 (innerer Aequator)\n"); ++fails; }
        if (!near(s.sample({1.5f,0,0}).distance, 0.0f)) { printf("Torus phi(1.5,0,0)!=0 (aeusserer Aequator)\n"); ++fails; }
        if (!near(s.sample({1,0.5f,0}).distance, 0.0f)) { printf("Torus phi(1,0.5,0)!=0 (oben)\n"); ++fails; }
        if (!near(s.sample({0,0,0}).distance, 0.5f)) { printf("Torus phi(0,0,0)!=0.5 (Loch)\n"); ++fails; }
        if (!near(s.surfaceArea(), 4.0f*glm::pi<float>()*glm::pi<float>()*1.0f*0.5f)) { printf("Torus Flaeche\n"); ++fails; }
        // Bounds: xz-Radius R+r=1.5, y-Radius r=0.5
        if (s.boundsMin() != glm::vec3(-1.5f,-0.5f,-1.5f) || s.boundsMax() != glm::vec3(1.5f,0.5f,1.5f)) {
            printf("Torus bounds\n"); ++fails;
        }
    }

    // Hantel (R=1, a=0.5): Oberflächenpunkte tibell R+a entlang x.
    {
        DumbbellSDF s({0,0,0}, 1.0f, 0.5f);
        fails += check("Hantel", s, 40);
        if (!near(s.sample({1.5f,0,0}).distance, 0.0f)) { printf("Hantel phi(+1.5,0,0)!=0\n"); ++fails; }
        if (!near(s.sample({-1.5f,0,0}).distance, 0.0f)) { printf("Hantel phi(-1.5,0,0)!=0\n"); ++fails; }
        if (!near(s.sample({0.5f,0,0}).distance, -1.0f)) { printf("Hantel phi(0.5,0,0)!= -1\n"); ++fails; }
        // Sattel/konkav: auf der y-Achse am "Hals" liegt die Oberfläche näher an der Mitte
        // (0,0,0): dort ist phi=0 bei (0, sqrt(R²-a²), 0) nur wenn nicht überlappt;
        // für a=0.5, R=1 liegt (0,~0.866,0)? d1=d2=sqrt(0.25+0.75)=1 → phi=0.
        float yHals = std::sqrt(1.0f - 0.25f);
        if (!near(s.sample({0,yHals,0}).distance, 0.0f, 1e-3f)) { printf("Hantel phi(0,yHals,0)!=0\n"); ++fails; }
        if (!near(s.surfaceArea(), 4.0f*glm::pi<float>()*1.0f*1.5f)) { printf("Hantel Flaeche != 6pi\n"); ++fails; }
        if (s.boundsMin() != glm::vec3(-1.5f,-1,-1) || s.boundsMax() != glm::vec3(1.5f,1,1)) {
            printf("Hantel bounds\n"); ++fails;
        }
    }

    printf("\n%s\n", fails == 0 ? "TEST PASS (4 Primitive, phi=0, |grad|=1, Bounds, Flächen)"
                                : "TEST FAIL");
    return fails == 0 ? 0 : 1;
}