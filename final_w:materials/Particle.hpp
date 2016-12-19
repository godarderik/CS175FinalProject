//
//  Particle.hpp
//  cs175-asst3
//
//  Created by Erik Godard on 12/18/16.
//  Copyright © 2016 cs175. All rights reserved.
//

#ifndef Particle_hpp

#define Particle_hpp

#include <vector>
#include "cvec.h"
#include <stdio.h>
#include "ParticleData.hpp"
#include "geometrymaker.h"
#include <string>
#include <memory>
#include <stdexcept>
#include "glsupport.h"
#include "Geometry.h"
#include "rigtform.h"

#endif /* Particle_hpp */

using namespace std;
using namespace tr1;

class Particle {
    public:
        void init(Cvec3 position, Cvec3 velocity, Cvec3 acceleration, Cvec3f color, float initTime);
        void update(float dt);
        void destroy();
        void addGeometry(shared_ptr<Geometry> g);
    
        shared_ptr<Geometry> getGeometry();
        Cvec3 getPosition();
        Cvec3f getColor();
    
        float getTime();
    
        bool isAlive();
    
        Particle();
    
    private:
        ParticleData m_particleData;
        shared_ptr<Geometry> m_geometry;
    
        float m_lifetime;
        float m_timeAlive;
        float m_initTime;
        bool m_alive;
    
        Cvec3 m_position;
        Cvec3 m_velocity;
        Cvec3 m_acceleration;
        Cvec3f m_color;
    
        void reset();
    
    
};
