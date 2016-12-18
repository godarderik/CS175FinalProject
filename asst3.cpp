////////////////////////////////////////////////////////////////////////
//
//   Harvard University
//   CS175 : Computer Graphics
//   Professor Steven Gortler
//
////////////////////////////////////////////////////////////////////////

#include <vector>
#include <string>
#include <memory>
#include <stdexcept>
#include <queue>
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
#include "Particle.hpp"

using namespace std;      // for string, vector, iostream, and other standard C++ stuff
using namespace tr1; // for shared_ptr

// G L O B A L S ///////////////////////////////////////////////////

static const float g_frustMinFov = 60.0;  // A minimal of 60 degree field of view
static float g_frustFovY = g_frustMinFov; // FOV in y direction (updated by updateFrustFovY)

static const float g_frustNear = -0.1;    // near plane
static const float g_frustFar = -50.0;    // far plane
static const float g_groundY = -2.0;      // y coordinate of the ground
static const float g_groundSize = 10.0;   // half the ground length

static int g_windowWidth = 512;
static int g_windowHeight = 512;
static bool g_mouseClickDown = false;    // is the mouse button pressed
static bool g_mouseLClickButton, g_mouseRClickButton, g_mouseMClickButton;
static bool g_spaceDown = false;         // space state, for middle mouse emulation
static int g_mouseClickX, g_mouseClickY; // coordinates for mouse click event
static int g_activeShader = 0;



static const int g_numShaders = 2;
static const char * const g_shaderFiles[g_numShaders][2] = {
  {"./shaders/basic-gl3.vshader", "./shaders/diffuse-gl3.fshader"},
  {"./shaders/basic-gl3.vshader", "./shaders/solid-gl3.fshader"}
};
static const char * const g_shaderFilesGl2[g_numShaders][2] = {
  {"./shaders/basic-gl2.vshader", "./shaders/diffuse-gl2.fshader"},
  {"./shaders/basic-gl2.vshader", "./shaders/solid-gl2.fshader"}
};
static vector<shared_ptr<ShaderState> > g_shaderStates; // our global shader states

int currView = 0;
int currObject = 0;
int currFrame = 0;
static const char * viewTitles [3] = {"Sky\n", "Object 0\n", "Object 1\n"};
// --------- Geometry

// Macro used to obtain relative offset of a field within a struct
#define FIELD_OFFSET(StructType, field) &(((StructType *)0)->field)

// Vertex buffer and index buffer associated with the ground and cube geometry
static shared_ptr<Geometry> g_ground, g_cube1, g_cube2;
static vector<shared_ptr<Geometry>> cubes;

//Particles
const size_t MAX_PARTICLES = 1500;
static vector<shared_ptr<Particle>> particles;
static long long time_elapsed = 0;
static long long time_since_particle = 0;
static const int particle_delay = 2;
static queue<size_t> particleQueue;
//


// --------- Scene

static const Cvec3 g_light1(2.0, 3.0, 14.0), g_light2(-2, -3.0, -5.0);  // define two lights positions in world space
static Matrix4 g_skyRbt = Matrix4::makeTranslation(Cvec3(0.0, 0.25, 6.0));
static Matrix4 g_objectRbt[2] = {Matrix4::makeTranslation(Cvec3(1,0,0)), Matrix4::makeTranslation(Cvec3(-1,0,0))};  // currently only 1 obj is defined
static Cvec3f g_objectColors[2] = {Cvec3f(1, 0, 0), Cvec3f(0, 0, 1)};

///////////////// END OF G L O B A L S //////////////////////////////////////////////////


static void initGround() {
  // A x-z plane at y = g_groundY of dimension [-g_groundSize, g_groundSize]^2
  VertexPN vtx[4] = {
    VertexPN(-g_groundSize, g_groundY, -g_groundSize, 0, 1, 0),
    VertexPN(-g_groundSize, g_groundY,  g_groundSize, 0, 1, 0),
    VertexPN( g_groundSize, g_groundY,  g_groundSize, 0, 1, 0),
    VertexPN( g_groundSize, g_groundY, -g_groundSize, 0, 1, 0),
  };
  unsigned short idx[] = {0, 1, 2, 0, 2, 3};
  g_ground.reset(new Geometry(&vtx[0], &idx[0], 4, 6));
}

double r() {
    return 2 * (double) rand() / (RAND_MAX) - 1;
}

static void initSystem() {
    int ibLen, vbLen;
    getSphereVbIbLen(20, 10, vbLen, ibLen);
    vector<VertexPN> vtx(vbLen);
    vector<unsigned short> idx(ibLen);
    makeSphere(0.04, 20, 10, vtx.begin(), idx.begin());
    
    for (int i = 0; i < MAX_PARTICLES; ++i) {
        
        shared_ptr<Particle> p = shared_ptr<Particle>(new Particle);
        
        shared_ptr<Geometry> geo;
        geo.reset(new Geometry(&vtx[0], &idx[0], vtx.size(), idx.size()));
        p->addGeometry(geo);
        
        particles.push_back(p);
        particleQueue.push(i);
    }

    
}

// takes a projection matrix and send to the the shaders
static void sendProjectionMatrix(const ShaderState& curSS, const Matrix4& projMatrix) {
  GLfloat glmatrix[16];
  projMatrix.writeToColumnMajorMatrix(glmatrix); // send projection matrix
  safe_glUniformMatrix4fv(curSS.h_uProjMatrix, glmatrix);
}

// takes MVM and its normal matrix to the shaders
static void sendModelViewNormalMatrix(const ShaderState& curSS, const Matrix4& MVM, const Matrix4& NMVM) {
  GLfloat glmatrix[16];
  MVM.writeToColumnMajorMatrix(glmatrix); // send MVM
  safe_glUniformMatrix4fv(curSS.h_uModelViewMatrix, glmatrix);

  NMVM.writeToColumnMajorMatrix(glmatrix); // send NMVM
  safe_glUniformMatrix4fv(curSS.h_uNormalMatrix, glmatrix);
}

// update g_frustFovY from g_frustMinFov, g_windowWidth, and g_windowHeight
static void updateFrustFovY() {
  if (g_windowWidth >= g_windowHeight)
    g_frustFovY = g_frustMinFov;
  else {
    const double RAD_PER_DEG = 0.5 * CS175_PI/180;
    g_frustFovY = atan2(sin(g_frustMinFov * RAD_PER_DEG) * g_windowHeight / g_windowWidth, cos(g_frustMinFov * RAD_PER_DEG)) / RAD_PER_DEG;
  }
}

static Matrix4 makeProjectionMatrix() {
  return Matrix4::makeProjection(
           g_frustFovY, g_windowWidth / static_cast <double> (g_windowHeight),
           g_frustNear, g_frustFar);
}

static void drawStuff() {
  // short hand for current shader state
  const ShaderState& curSS = *g_shaderStates[g_activeShader];

  // build & send proj. matrix to vshader
  const Matrix4 projmat = makeProjectionMatrix();
  sendProjectionMatrix(curSS, projmat);

  // use the skyRbt as the eyeRbt
  Matrix4 eyeRbt;
    
  switch (currView) {
  case 0: {
      eyeRbt = g_skyRbt;
      break;
  }
  case 1: {
      eyeRbt = g_objectRbt[0];
      break;
  }
  case 2: {
      eyeRbt = g_objectRbt[1];
      break;
  }
  }
  const Matrix4 invEyeRbt = inv(eyeRbt);

  const Cvec3 eyeLight1 = Cvec3(invEyeRbt * Cvec4(g_light1, 1)); // g_light1 position in eye coordinates
  const Cvec3 eyeLight2 = Cvec3(invEyeRbt * Cvec4(g_light2, 1)); // g_light2 position in eye coordinates
  safe_glUniform3f(curSS.h_uLight, eyeLight1[0], eyeLight1[1], eyeLight1[2]);
  safe_glUniform3f(curSS.h_uLight2, eyeLight2[0], eyeLight2[1], eyeLight2[2]);

  // draw ground
  // ===========
  //
  const Matrix4 groundRbt = Matrix4();  // identity
  Matrix4 MVM = invEyeRbt * groundRbt;
  Matrix4 NMVM = normalMatrix(MVM);
  sendModelViewNormalMatrix(curSS, MVM, NMVM);
  safe_glUniform3f(curSS.h_uColor, 0.1, 0.95, 0.1); // set color
  g_ground->draw(curSS);
    
  for (int i = 0; i < particles.size(); ++i) {
      if (!particles[i]->isAlive()) {
          continue;
      }
      Cvec3 p = particles[i]->getPosition();
      Cvec3f c = particles[i]->getColor();
      Matrix4 translation = Matrix4::makeTranslation(p);
      MVM = invEyeRbt * translation;
      NMVM = normalMatrix(MVM);
      sendModelViewNormalMatrix(curSS, MVM, NMVM);
      safe_glUniform3f(curSS.h_uColor, c[0], c[1],c[2]);
    
      particles[i]->getGeometry()->draw(curSS);
 }

}



static void display() {
  glUseProgram(g_shaderStates[g_activeShader]->program);
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);                   // clear framebuffer color&depth

  drawStuff();

  glutSwapBuffers();                                    // show the back buffer (where we rendered stuff)

  checkGlErrors();
}

void genNewParticle() {
    //printf("%lu\n", particleQueue.size());
    if (!particleQueue.empty()) {
        size_t i = particleQueue.front();
        Cvec3 position = Cvec3(0,0,0);
        Cvec3 velocity = Cvec3(r()/1.5,4 + r()/2,r()/1.5);
        Cvec3 accleration = Cvec3(r()/5,-3,r()/5);
        Cvec3f color = Cvec3f(r(),r(), r());
        
        particleQueue.pop();
        
        if (particles[i]->isAlive()) {
            return; 
        }
        
        particles[i]->init(position, velocity, accleration,color,(glutGet(GLUT_ELAPSED_TIME) - time_elapsed)/1000.0f);
    }
}

long long time_last = 0;

static void updateParticles() {
    
    long long new_time = glutGet(GLUT_ELAPSED_TIME);
    double dt = (new_time - time_elapsed);
    double dt_elapsed = (new_time - time_last);
    time_last = new_time;
    time_since_particle += dt_elapsed;
    
    for (int i = 0; i < particles.size(); ++i) {
        bool alive = particles[i]->isAlive();
        particles[i]->update(dt/1000.0f);
        if (alive && !particles[i]->isAlive()) particleQueue.push(i);
    }
    
    for (int i = 0; i < time_since_particle / particle_delay; ++i) {
        genNewParticle();
    }
    
    time_since_particle = time_since_particle % particle_delay;
    display();
    
}

static void reshape(const int w, const int h) {
  g_windowWidth = w;
  g_windowHeight = h;
  glViewport(0, 0, w, h);
  cerr << "Size of window is now " << w << "x" << h << endl;
  updateFrustFovY();
  glutPostRedisplay();
}

static Matrix4 getFrame() {
    Matrix4 retMat = Matrix4();
    
    if (currView == 0 && currObject == 1) {
        return g_objectRbt[0] * linFact(g_skyRbt);
    } else if (currView == 0 && currObject == 2) {
       return g_objectRbt[1] * linFact(g_skyRbt);
    } else if (currObject == 0 && currView == 0 && currFrame == 0) {
        return linFact(g_skyRbt);
    } else if (currObject == 0 && currView == 0 && currFrame == 1) {
        return g_skyRbt;
    }

    return retMat;
}

static void motion(const int x, const int y) {
  const double dx = x - g_mouseClickX;
  const double dy = g_windowHeight - y - 1 - g_mouseClickY;
    
  Matrix4 frame = getFrame();
    
  if (currView == 1 || currView == 2) return;

  Matrix4 m;
  if (g_mouseLClickButton && !g_mouseRClickButton && !g_spaceDown) { // left button down?
    m = Matrix4::makeXRotation(-dy) * Matrix4::makeYRotation(dx);
  }
  else if (g_mouseRClickButton && !g_mouseLClickButton) { // right button down?
    m = Matrix4::makeTranslation(Cvec3(dx, dy, 0) * 0.01);
  }
  else if (g_mouseMClickButton || (g_mouseLClickButton && g_mouseRClickButton) || (g_mouseLClickButton && !g_mouseRClickButton && g_spaceDown) ) {  // middle or (left and right, or left + space) button down?
    m = Matrix4::makeTranslation(Cvec3(0, 0, -dy) * 0.01);
  }
    
  Matrix4 *changeObject;
  Matrix4 tempObject;
    
  if (g_mouseClickDown) {
      if (currObject == 0) {
          changeObject = &g_skyRbt;
      } else if (currObject == 1) {
          changeObject  = &g_objectRbt[0];
      } else if (currObject == 2) {
        changeObject  = &g_objectRbt[1];
      }
    
      Matrix4 tempObject = *changeObject;
      Matrix4 A = inv(tempObject) * frame;
      if (currObject == 0) {
          m = inv(m);
      }
      *changeObject = tempObject * A * m * inv(A);
    glutPostRedisplay(); // we always redraw if we changed the scene
  }

  g_mouseClickX = x;
  g_mouseClickY = g_windowHeight - y - 1;
}


static void mouse(const int button, const int state, const int x, const int y) {
  g_mouseClickX = x;
  g_mouseClickY = g_windowHeight - y - 1;  // conversion from GLUT window-coordinate-system to OpenGL window-coordinate-system

  g_mouseLClickButton |= (button == GLUT_LEFT_BUTTON && state == GLUT_DOWN);
  g_mouseRClickButton |= (button == GLUT_RIGHT_BUTTON && state == GLUT_DOWN);
  g_mouseMClickButton |= (button == GLUT_MIDDLE_BUTTON && state == GLUT_DOWN);

  g_mouseLClickButton &= !(button == GLUT_LEFT_BUTTON && state == GLUT_UP);
  g_mouseRClickButton &= !(button == GLUT_RIGHT_BUTTON && state == GLUT_UP);
  g_mouseMClickButton &= !(button == GLUT_MIDDLE_BUTTON && state == GLUT_UP);

  g_mouseClickDown = g_mouseLClickButton || g_mouseRClickButton || g_mouseMClickButton;
}

static void keyboardUp(const unsigned char key, const int x, const int y) {
  switch (key) {
  case ' ':
    g_spaceDown = false;
    break;
  }
  glutPostRedisplay();
}

static void keyboard(const unsigned char key, const int x, const int y) {
  switch (key) {
  case 27:
    exit(0);                                  // ESC
  case 'h':
    cout << " ============== H E L P ==============\n\n"
    << "h\t\thelp menu\n"
    << "s\t\tsave screenshot\n"
    << "f\t\tToggle flat shading on/off.\n"
    << "o\t\tCycle object to edit\n"
    << "v\t\tCycle view\n"
    << "drag left mouse to rotate\n" << endl;
    break;
  case 's':
    glFlush();
    writePpmScreenshot(g_windowWidth, g_windowHeight, "out.ppm");
    break;
  case 'f':
    g_activeShader ^= 1;
    break;
  case 'v':
    if (currView == 2) currView = -1;
    printf("%s",viewTitles[++currView]);
    break;
  case 'o':
    if (currObject == 2) currObject = -1;
    ++currObject;
    break;
  case 'm':
    if (currFrame == 1) currFrame = 0;
    else currFrame = 1;
    break;
  case ' ':
    g_spaceDown = true;
    break;
  }
  glutPostRedisplay();
}

static void initGlutState(int argc, char * argv[]) {
  glutInit(&argc, argv);                                  // initialize Glut based on cmd-line args
#ifdef __MAC__
  glutInitDisplayMode(GLUT_3_2_CORE_PROFILE|GLUT_RGBA|GLUT_DOUBLE|GLUT_DEPTH); // core profile flag is required for GL 3.2 on Mac
#else
  glutInitDisplayMode(GLUT_RGBA|GLUT_DOUBLE|GLUT_DEPTH);  //  RGBA pixel channels and double buffering
#endif
  glutInitWindowSize(g_windowWidth, g_windowHeight);      // create a window
  glutCreateWindow("Final Project");                       // title the window

  glutIgnoreKeyRepeat(true);                              // avoids repeated keyboard calls when holding space to emulate middle mouse

  glutDisplayFunc(display);                               // display rendering callback
  glutReshapeFunc(reshape);                               // window reshape callback
  glutMotionFunc(motion);                                 // mouse movement callback
  glutMouseFunc(mouse);                                   // mouse click callback
  glutKeyboardFunc(keyboard);
  glutKeyboardUpFunc(keyboardUp);
  glutIdleFunc(updateParticles);
}

static void initGLState() {
  glClearColor(128./255., 200./255., 255./255., 0.);
  glClearDepth(0.);
  glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
  glPixelStorei(GL_PACK_ALIGNMENT, 1);
  glCullFace(GL_BACK);
  glEnable(GL_CULL_FACE);
  glEnable(GL_DEPTH_TEST);
  glDepthFunc(GL_GREATER);
  glReadBuffer(GL_BACK);
  if (!g_Gl2Compatible)
    glEnable(GL_FRAMEBUFFER_SRGB);
}

static void initShaders() {
  g_shaderStates.resize(g_numShaders);
  for (int i = 0; i < g_numShaders; ++i) {
    if (g_Gl2Compatible)
      g_shaderStates[i].reset(new ShaderState(g_shaderFilesGl2[i][0], g_shaderFilesGl2[i][1]));
    else
      g_shaderStates[i].reset(new ShaderState(g_shaderFiles[i][0], g_shaderFiles[i][1]));
  }
}

static void initGeometry() {
  initGround();
  initSystem();
}

int main(int argc, char * argv[]) {
    
  srand (time(NULL));
    
  try {
    initGlutState(argc,argv);

    // on Mac, we shouldn't use GLEW.

#ifndef __MAC__
    glewInit(); // load the OpenGL extensions
#endif

    cout << (g_Gl2Compatible ? "Will use OpenGL 2.x / GLSL 1.0" : "Will use OpenGL 3.x / GLSL 1.5") << endl;

#ifndef __MAC__
    if ((!g_Gl2Compatible) && !GLEW_VERSION_3_0)
      throw runtime_error("Error: card/driver does not support OpenGL Shading Language v1.3");
    else if (g_Gl2Compatible && !GLEW_VERSION_2_0)
      throw runtime_error("Error: card/driver does not support OpenGL Shading Language v1.0");
#endif

    initGLState();
    initShaders();
    initGeometry();

    glutMainLoop();
    return 0;
  }
  catch (const runtime_error& e) {
    cout << "Exception caught: " << e.what() << endl;
    return -1;
  }
}
