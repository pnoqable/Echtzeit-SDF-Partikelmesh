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
        // Flaches Ellipsoid: exakte Fusspunkt-Distanz (Normalisierung!).
        // Vorher (nicht normiert): d = k-1; Faktor 1./|grad phi| fehlte -> Partikel
        // sprangen weit ueber die Oberflaeche, teils aus der Bounding-Box.
        if (!near(e.sample({2.1f,0,0}).distance, 0.1f, 1e-3f)) { printf("Ellipsoid(2,1,1) Dist(2.1,0,0)!=0.1\n"); ++fails; }
        EllipsoidSDF flat({0,0,0}, 1.0f, 1.0f, 0.05f);
        if (!near(flat.sample({0,0,0.04f}).distance, -0.01f, 1e-3f)) { printf("Ellipsoid flach Dist(0,0,0.04)!= -0.01\n"); ++fails; }
        if (!near(glm::length(flat.sample({0,0,0.1f}).gradient), 1.0f, 1e-2f)) { printf("Ellipsoid flach |grad|\n"); ++fails; }
        // Projektion eines fernen Punktes landet exakt auf der Oberfläche:
        // p' = p - d·grad  (Einheitsgradient) muss phi==0 ergeben.
        {
            glm::vec3 fp(0.9f,0.9f,0.002f);
            SDFSample s = flat.sample(fp);
            glm::vec3 proj = fp - s.distance * s.gradient;
            if (std::fabs(flat.sample(proj).distance) > 1e-3f) { printf("Ellipsoid flach Projektion != Oberflaeche\n"); ++fails; }
        }
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

    // Metaball (R=1, a=0.5, k groes): weiche Verschmelzung statt harter Hantel.
    // k = 0.01 <=> harte Hantel (Grenzfall), k = 2.5 >= 4(R-a)=2 => geschlossener Blob.
    {
        // Grenzfall k->0 muss sich wie die harte Hantel verhalten.
        MetaballSDF hard({0,0,0}, 1.0f, 0.5f, 1e-4f);
        if (!near(hard.sample({1.5f,0,0}).distance, 0.0f, 1e-2f)) { printf("Metaball(k->0) phi(1.5,0,0)!=0\n"); ++fails; }
        float yHals2 = std::sqrt(1.0f - 0.25f);
        if (!near(hard.sample({0,yHals2,0}).distance, 0.0f, 1e-2f)) { printf("Metaball(k->0) phi(0,yHals,0)!=0\n"); ++fails; }

        // Weicher Blob: Sattel füllt sich auf, phi(0,0,0) wird deutlich negativ.
        MetaballSDF soft({0,0,0}, 1.0f, 0.5f, 2.5f);
        if (soft.sample({0,0,0}).distance >= -0.2f) { printf("Metaball soft phi(0,0,0) nicht verfuellt\n"); ++fails; }
        // Smooth Min weitet die Null-Isoflaeche gegenueber der harten Hantel auf:
        // der exakte aussenere Kugelpunkt (1.5,0,0) liegt jetzt bereits innen
        // (d.h. die Oberflaeche liegt weiter aussen), ein ferner Punkt (~1) bleibt
        // es ungefaehr.
        if (soft.sample({1.5f,0,0}).distance >= 0.0f) { printf("Metaball soft phi(1.5,0,0) nicht aufgeweitet\n"); ++fails; }
        if (!near(soft.sample({2.9f,0,0}).distance, 1.4f, 0.4f)) { printf("Metaball soft phi(2.9,0,0)\n"); ++fails; }

        // Analytischer Gradient gegen finite Differenzen (Kettenregel-Verifikation).
        // Stichproben: im Blendbereich (Hals + Diagonalen) und im Kerngebiet.
        glm::vec3 pts[] = {
            {0.2f, 0.7f, 0.0f}, {0.0f, 0.6f, 0.2f}, {0.4f, 0.9f, 0.0f},
            {1.0f, 0.6f, 0.0f}, {0.0f, 0.0f, 0.2f}, {0.8f, 0.8f, 0.0f},
        };
        for (const auto& q : pts) {
            SDFSample s = soft.sample(q);
            glm::vec3 gFD;
            const float eps = 1e-3f;
            for (int d = 0; d < 3; ++d) {
                glm::vec3 plus = q, minus = q;
                plus[d] += eps;  minus[d] -= eps;
                gFD[d] = (soft.sample(plus).distance - soft.sample(minus).distance) / (2.0f * eps);
            }
            float diff = glm::length(s.gradient - gFD);
            if (diff > 2e-2f || s.gradient.x != s.gradient.x) {
                printf("Metaball Gradient-Abweichung %f an p=(%.2f,%.2f,%.2f) analytisch=(%.3f,%.3f,%.3f) FD=(%.3f,%.3f,%.3f)\n",
                    diff, q.x, q.y, q.z, s.gradient.x, s.gradient.y, s.gradient.z, gFD.x, gFD.y, gFD.z);
                ++fails;
            }
        }

        // Gering k: Profil ~ Kugeln, bounds nahe dem exakten Hantel-Wert (-1.5,-1,-1).
        if (!near(hard.boundsMin().x, -1.5f, 0.05f) || !near(hard.boundsMax().x, 1.5f, 0.05f)) {
            printf("Metaball(k->0) bounds\n"); ++fails;
        }
        // Nachweis, dass die Profil-Bounds den aufgeblähten Blob umfassen.
        if (soft.sample({0.99f * soft.boundsMax().x, 0.0f, 0.0f}).distance >= 0.0f) { printf("Metaball bounds nicht umfassend\n"); ++fails; }

        // Fläche numerisch (Rotationsintegral): k→0 exakt Hantel 6π,
        // der weiche Blob ist aufgebläht und größer als die Kapsel-Näherung.
        float Ahard = hard.surfaceArea(), Asoft = soft.surfaceArea();
        if (!near(Ahard, 4.0f*glm::pi<float>()*1.0f*1.5f, 0.4f)) { printf("Metaball(k->0) Flaeche != 6pi (%.3f)\n", Ahard); ++fails; }
        if (Asoft <= Ahard) { printf("Metaball soft Flaeche nicht groesser als hart (%.3f)\n", Asoft); ++fails; }

        // Getrennte Blobs (a=1.5 > R=1, kleines k): zwei disjunkte Kugeln.
        // Die Ueberlappung darf beim Metaball >= R werden (GUI erlaubt das),
        // das Rotationsprofil muss die Lucecke zwischen den Blobs erkennen.
        MetaballSDF sep({0,0,0}, 1.0f, 1.5f, 1e-4f);
        // Hals frei: phi zwischen den Kugeln > 0, Randkante (a+R) erhalten.
        if (sep.sample({0,0,0}).distance <= 0.0f) { printf("Metaball sep Hals nicht frei\n"); ++fails; }
        if (!near(sep.sample({2.5f,0,0}).distance, 0.0f, 1e-2f)) { printf("Metaball sep Randkante a+R (!=0)\n"); ++fails; }
        // Bounds umfassen beide Kugeln bis ~a+R.
        if (!near(sep.boundsMin().x, -2.5f, 0.1f) || !near(sep.boundsMax().x, 2.5f, 0.1f)) { printf("Metaball sep bounds (%.2f..%.2f)\n", sep.boundsMin().x, sep.boundsMax().x); ++fails; }
        // Flaeche ~ 2 * 4pi R^2 (zwei getrennte Kugeln ohne Blend).
        float Asep = sep.surfaceArea();
        if (!near(Asep, 8.0f*glm::pi<float>()*1.0f*1.0f, 0.5f)) { printf("Metaball sep Flaeche != 8pi (%.3f)\n", Asep); ++fails; }
    }

    printf("\n%s\n", fails == 0 ? "TEST PASS (5 Primitive, phi=0, |grad|=1, Bounds, Flächen)"
                                : "TEST FAIL");
    return fails == 0 ? 0 : 1;
}