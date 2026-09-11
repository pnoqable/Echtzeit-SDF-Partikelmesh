#include "SceneRenderer.hpp"
#include <raylib.h>
#include <rlgl.h>

void SceneRenderer::drawParticles(const ParticleSystem& system) {
    for (const auto& p : system.particles) {
        DrawSphereEx({p.position.x, p.position.y, p.position.z}, 0.005f, 4, 4, RED);
    }
}

void SceneRenderer::drawSDFBounds(const SDF& sdf) {
    glm::vec3 bmin = sdf.boundsMin();
    glm::vec3 bmax = sdf.boundsMax();
    Vector3 size = {
        bmax.x - bmin.x,
        bmax.y - bmin.y,
        bmax.z - bmin.z
    };
    Vector3 center = {
        (bmin.x + bmax.x) * 0.5f,
        (bmin.y + bmax.y) * 0.5f,
        (bmin.z + bmax.z) * 0.5f
    };
    DrawCubeWires(center, size.x, size.y, size.z, GRAY);
}

void SceneRenderer::drawAxes(float length) {
    // X axis (red)
    DrawLine3D({0,0,0}, {length,0,0}, RED);
    // Y axis (green)
    DrawLine3D({0,0,0}, {0,length,0}, GREEN);
    // Z axis (blue)
    DrawLine3D({0,0,0}, {0,0,length}, BLUE);
}
