//
//  Geometry.cpp
//  cs175-asst3
//
//  Created by Erik Godard on 12/18/16.
//  Copyright © 2016 cs175. All rights reserved.
//


#include <vector>
#include <string>
#include <memory>
#include <stdexcept>
#if __GNUG__
#   include <tr1/memory>
#endif

#ifdef __MAC__
#   include <OpenGL/gl3.h>
#   include <GLUT/glut.h>
#else
#   include <GL/glew.h>
#   include <GL/glut.h>
#endif

#include "cvec.h"
#include "matrix4.h"
#include "geometrymaker.h"
#include "ppm.h"
#include "glsupport.h"

#include <stdio.h>

//Move all of these struct definitions to their own file

// Macro used to obtain relative offset of a field within a struct
#define FIELD_OFFSET(StructType, field) &(((StructType *)0)->field)

static const bool g_Gl2Compatible = false;

struct ShaderState {
    GlProgram program;
    
    // Handles to uniform variables
    GLint h_uLight, h_uLight2;
    GLint h_uProjMatrix;
    GLint h_uModelViewMatrix;
    GLint h_uNormalMatrix;
    GLint h_uColor;
    
    // Handles to vertex attributes
    GLint h_aPosition;
    GLint h_aNormal;
    
    ShaderState(const char* vsfn, const char* fsfn) {
        readAndCompileShader(program, vsfn, fsfn);
        
        const GLuint h = program; // short hand
        
        // Retrieve handles to uniform variables
        h_uLight = safe_glGetUniformLocation(h, "uLight");
        h_uLight2 = safe_glGetUniformLocation(h, "uLight2");
        h_uProjMatrix = safe_glGetUniformLocation(h, "uProjMatrix");
        h_uModelViewMatrix = safe_glGetUniformLocation(h, "uModelViewMatrix");
        h_uNormalMatrix = safe_glGetUniformLocation(h, "uNormalMatrix");
        h_uColor = safe_glGetUniformLocation(h, "uColor");
        
        // Retrieve handles to vertex attributes
        h_aPosition = safe_glGetAttribLocation(h, "aPosition");
        h_aNormal = safe_glGetAttribLocation(h, "aNormal");
        
        if (!g_Gl2Compatible)
            glBindFragDataLocation(h, 0, "fragColor");
        checkGlErrors();
    }
    
};


// A vertex with floating point position and normal
struct VertexPN {
    Cvec3f p, n;
    
    VertexPN() {}
    VertexPN(float x, float y, float z,
             float nx, float ny, float nz)
    : p(x,y,z), n(nx, ny, nz)
    {}
    
    // Define copy constructor and assignment operator from GenericVertex so we can
    // use make* functions from geometrymaker.h
    VertexPN(const GenericVertex& v) {
        *this = v;
    }
    
    VertexPN& operator = (const GenericVertex& v) {
        p = v.pos;
        n = v.normal;
        return *this;
    }
};


struct Geometry {
    GlBufferObject vbo, ibo;
    GlArrayObject vao;
    int vboLen, iboLen;
    
    Geometry(VertexPN *vtx, unsigned short *idx, int vboLen, int iboLen) {
        this->vboLen = vboLen;
        this->iboLen = iboLen;
        
        // Now create the VBO and IBO
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        glBufferData(GL_ARRAY_BUFFER, sizeof(VertexPN) * vboLen, vtx, GL_STATIC_DRAW);
        
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, sizeof(unsigned short) * iboLen, idx, GL_STATIC_DRAW);
    }
    
    void draw(const ShaderState& curSS) {
        // bind the object's VAO
        glBindVertexArray(vao);
        
        // Enable the attributes used by our shader
        safe_glEnableVertexAttribArray(curSS.h_aPosition);
        safe_glEnableVertexAttribArray(curSS.h_aNormal);
        
        // bind vbo
        glBindBuffer(GL_ARRAY_BUFFER, vbo);
        safe_glVertexAttribPointer(curSS.h_aPosition, 3, GL_FLOAT, GL_FALSE, sizeof(VertexPN), FIELD_OFFSET(VertexPN, p));
        safe_glVertexAttribPointer(curSS.h_aNormal, 3, GL_FLOAT, GL_FALSE, sizeof(VertexPN), FIELD_OFFSET(VertexPN, n));
        
        // bind ibo
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, ibo);
        
        // draw!
        glDrawElements(GL_TRIANGLES, iboLen, GL_UNSIGNED_SHORT, 0);
        
        // Disable the attributes used by our shader
        safe_glDisableVertexAttribArray(curSS.h_aPosition);
        safe_glDisableVertexAttribArray(curSS.h_aNormal);
        
        // disable VAO
        glBindVertexArray(NULL);
    }
};
