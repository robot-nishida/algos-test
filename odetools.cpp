// #include <stdio.h>
// #include <stdlib.h>
// #include <string.h>
// #include <ode/ode.h>
// #include <drawstuff/drawstuff.h>
// #include <math.h>
#include "odetools.h"

void sayhello() {
  printf("hoge");
}

static void generateObject(Object* obj, const char* type, dSpaceID space, dWorldID world) {
  dMass mass;
  dMatrix3 R;

  obj -> body = dBodyCreate(world);
  dBodySetPosition(obj->body, obj->cx[0], obj->cx[1], obj->cx[2]);
  dMassSetZero(&mass);

  if(!strcmp(type,"Cylinder")){
    dMassSetCylinderTotal(&mass, obj->m, obj->longDir, obj->r, obj->l);
    dBodySetMass(obj->body, &mass);
    obj->geom = dCreateCylinder(space, obj->r, obj->l);

  }else if(!strcmp(type,"Box")){
    dMassSetBoxTotal(&mass, obj->m, obj->lx[0], obj->lx[1], obj->lx[2]);
    dBodySetMass(obj->body, &mass);
    obj->geom = dCreateBox(space, obj->lx[0], obj->lx[1], obj->lx[2]);

  }else if(!strcmp(type,"Capsule")){
    dMassSetCapsuleTotal(&mass, obj->m, obj->longDir, obj->r, obj->l);
    dBodySetMass(obj->body, &mass);
    obj->geom = dCreateCapsule(space, obj->r, obj->l);

  }else if(!strcmp(type,"Sphere")){
    dMassSetSphereTotal(&mass, obj->m, obj->r);
    dBodySetMass(obj->body, &mass);
    obj->geom = dCreateSphere(space, obj->r);

  }else{
    printf("No type Error\n");
  }

  dGeomSetBody(obj->geom, obj->body);
  dRFromAxisAndAngle(R, obj->ax[0], obj->ax[1], obj->ax[2], obj->ax[3]);
  dBodySetRotation(obj->body, R);
}

static void generateHingeJoint(Object* obj, Object* target, dWorldID world) {
  obj->joint = dJointCreateHinge(world, 0);
  dJointAttach(obj->joint, obj->body, target->body);
  dJointSetHingeAnchor(obj->joint, obj->jcx[0], obj->jcx[1], obj->jcx[2]);
  dJointSetHingeAxis(obj->joint, obj->jax[0], obj->jax[1], obj->jax[2]);
}

static void generateFixJoint(Object* obj, dWorldID world) {
  obj->joint = dJointCreateFixed(world, 0);
  dJointAttach(obj->joint, obj->body, 0);
  dJointSetFixed(obj->joint);
}

static void drawObject(Object* obj, const char* type) {
  const dReal *pos, *rot;
  dReal side[3];

  pos = dBodyGetPosition(obj->body);
  rot = dBodyGetRotation(obj->body);
  dsSetColor(obj->color[0], obj->color[1], obj->color[2]);
  if(!strcmp(type,"Cylinder")){
    dsDrawCylinder(pos, rot, obj->l, obj->r);
  }else if(!strcmp(type,"Box")){
    for(int i=0; i<3; i++){
      side[i] = obj->lx[i];
    }
    dsDrawBox(pos, rot, side);
  }else if(!strcmp(type,"Capsule")){
    dsDrawCapsule(pos, rot, obj->l, obj->r);
  }else if(!strcmp(type,"Sphere")){
    dsDrawSphereD(pos, rot, obj->r);
  }else{
    printf("No type Error\n");
  }
  dsSetColor(0.0,0.0,0.0); //色情報をリセット
}

static void setParamColor(Object* obj, double r, double g, double b) {
  obj->color[0] = r;
  obj->color[1] = g;
  obj->color[2] = b;
}
static void setParamCenter(Object* obj, double x, double y, double z) {
  obj->cx[0] = x;
  obj->cx[1] = y;
  obj->cx[2] = z;
}
static void setParamAxis(Object* obj, double ax, double ay, double az, double axis_rad) {
  obj->ax[0] = ax;
  obj->ax[1] = ay;
  obj->ax[2] = az;
  obj->ax[3] = axis_rad;
}
static void setParamCylinder(Object* obj, double m, double l, int longDir, double r) {
  obj->l = l;
  obj->r = r;
  obj->longDir = longDir;
  obj->m = m;
}
static void setParamBox(Object* obj, double m, double lx, double ly, double lz) {
  obj->lx[0] = lx;
  obj->lx[1] = ly;
  obj->lx[2] = lz;
  obj->m = m;
}
static void setParamSphere(Object* obj, double m, double r) {
  obj->r = r;
  obj->m = m;
}
static void setParamCapsule(Object* obj, double m, double l, double r) {
  obj->l = l;
  obj->r = r;
  obj->m = m;
}
static void setParamJointCenter(Object* obj, double jx, double jy, double jz) {
  obj->jcx[0] = jx;
  obj->jcx[1] = jy;
  obj->jcx[2] = jz;
}
static void setParamJointAxis(Object* obj, double jax, double jay, double jaz) {
  obj->jax[0] = jax;
  obj->jax[1] = jay;
  obj->jax[2] = jaz;
}
