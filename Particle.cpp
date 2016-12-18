//
//  Particle.cpp
//  cs175-asst3
//
//  Created by Erik Godard on 12/18/16.
//  Copyright © 2016 cs175. All rights reserved.
//

#include "Particle.hpp"

void Particle::addGeometry(shared_ptr<Geometry> g) {
    m_geometry = g;
}

void Particle::reset() {
    m_alive = false;
    m_lifetime = 20.0f;
    m_timeAlive = 0.0f;
    
    m_position = Cvec3(0.0f, 0.0f, 0.0f);
    m_velocity = Cvec3(0.0f, 0.0f, 0.0f);
    m_acceleration= Cvec3(0.0f, 0.0f, 0.0f);
    
}

Particle::Particle() {
    reset();
}

shared_ptr<Geometry> Particle::getGeometry() {
    return m_geometry;
}

void Particle::init(Cvec3 position, Cvec3 velocity, Cvec3 acceleration, Cvec3f color, float initTime) {
    
    m_alive = true;
    
    m_position = position;
    m_velocity = velocity;
    m_acceleration = acceleration;
    m_color = color;
    m_initTime = initTime;
}

void Particle::update(float dt) {
    m_timeAlive = dt - m_initTime;
    m_position = m_velocity * m_timeAlive + m_acceleration * 0.5f * m_timeAlive*m_timeAlive;
    if (m_timeAlive > m_lifetime || m_position[1] < -1) {
        reset();
    }
}

Cvec3 Particle::getPosition() {
    
    return m_position;
}

bool Particle::isAlive() {
    return m_alive;
}

Cvec3f Particle::getColor() {
    return m_color;
}

float Particle::getTime() {
    
    return m_timeAlive;
}
