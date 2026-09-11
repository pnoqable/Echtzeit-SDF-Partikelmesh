#pragma once

#include "../simulation/ParticleSystem.hpp"
#include "../simulation/SDF.hpp"
#include <glm/glm.hpp>

class SceneRenderer {
public:
    void drawParticles(const ParticleSystem& system);
    void drawSDFBounds(const SDF& sdf);
    void drawAxes(float length = 2.0f);
};
