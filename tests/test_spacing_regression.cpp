#include "../src/mesh/Triangulation.hpp"
#include "../src/simulation/PrimitiveSDF.hpp"
#include "../src/simulation/ParticleSystem.hpp"
#include <glm/glm.hpp>
#include <glm/gtc/constants.hpp>
#include <cmath>
#include <cstdio>
#include <unordered_map>
#include <vector>

static int boundary(const std::vector<Triangle>& tris){
    std::unordered_map<unsigned long long,int> m;
    auto K=[](unsigned a,unsigned b)->unsigned long long{if(a>b)std::swap(a,b);return ((unsigned long long)a<<32)|b;};
    for(auto&t:tris){m[K(t.i0,t.i1)]++;m[K(t.i1,t.i2)]++;m[K(t.i2,t.i0)]++;}
    int b=0;for(auto&[k,c]:m)if(c==1)b++;return b;
}

int main(){
    const int N=1000;
    SphereSDF sphere({0,0,0},1.0f);
    ParticleSystem sys;
    sys.parameters.targetSpacing=0.10f;
    sys.parameters.repulsionRadius=0.35f;
    sys.parameters.repulsionStrength=0.8f;
    sys.parameters.damping=0.4f;
    sys.parameters.substeps=8;
    sys.parameters.maxStepLength=0.4f;
    sys.parameters.projectionIterations=8;
    sys.parameters.sdfTolerance=1e-4f;
    sys.initialize(N,sphere.boundsMin(),sphere.boundsMax(),42);
    sys.projectToSDF(sphere);
    const float dt=1.0f/60.0f;
    for(int f=0;f<900;++f) sys.relax(dt,sphere);

    std::vector<glm::vec3> pos,nrm;
    for(auto&p:sys.particles){pos.push_back(p.position);nrm.push_back(p.normal);}

    float A=4.0f*glm::pi<float>()*1.0f;
    float spArea=std::sqrt(A/N);          // korrekte Formel (App seit Fix)
    float spHex=std::sqrt(2.0f*A/(1.7320508f*N)); // alte fehlerhafte Formel

    Triangulation::Parameters p;
    p.maxEdgeLength=1.6f;
    Triangulation t;
    t.build(pos,nrm,spArea,sphere,p);
    int b=boundary(t.triangles());
    printf("spacing=sqrt(A/N)=%.4f: F=%d Rand=%d\n",spArea,t.stats().totalTriangles,b);

    Triangulation t2;
    t2.build(pos,nrm,spHex,sphere,p);
    int b2=boundary(t2.triangles());
    printf("spacing=sqrt(2A/sqrt3N)=%.4f: F=%d Rand=%d\n",spHex,t2.stats().totalTriangles,b2);

    bool ok = b == 0;
    printf("\n%s\n", ok ? "TEST PASS (keine Randkanten mit korrekter Spacing)" : "TEST FAIL");
    return ok ? 0 : 1;
}