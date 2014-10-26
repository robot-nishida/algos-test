/* odetools.h */

#ifndef __INCLUDE_ODETOOLS_H__
#define __INCLUDE_ODETOOLS_H__

//-----------------------------------------------------------------
// インクルード
//-----------------------------------------------------------------

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include <math.h>

//-----------------------------------------------------------------
// 定義宣言
//-----------------------------------------------------------------

#ifdef dDOUBLE
#define dsDrawBox      dsDrawBoxD
#define dsDrawSphere   dsDrawSphereD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawCapsule  dsDrawCapsuleD
#define dsDrawLine     dsDrawLineD
#endif

typedef struct {          // Object構造体
  dBodyID body;           // ボディ(剛体)のID番号（動力学計算用）
  dGeomID geom;           // ジオメトリのID番号(衝突検出計算用）
  dJointID joint;         // 関節
  double l,r,m;           // 長さ[m], 半径[m], 質量[kg]
  double lx[3] = {0.0};   // Boxの長さ(x,y,z)
  int longDir;            // 円柱長手方向
  double color[3] = {0.8};// オブジェクトの色(r,g,b->0-1)
  double cx[3] = {0.0};   // 中心座標
  double ax[4] = {0.0};   // 中心座標での回転軸ベクトル+回転確度[rad]
  double jcx[3] = {0.0};  // 関節中心
  double jax[3] = {0.0};  // 関節回転軸ベクトル
} Object;

//-----------------------------------------------------------------
// 関数宣言
//-----------------------------------------------------------------

void sayhello();

static void generateObject(Object* obj, const char* type, dSpaceID space, dWorldID world);

static void generateHingeJoint(Object* obj, Object* target, dWorldID world);
static void generateFixJoint(Object* obj, dWorldID world);

static void drawObject(Object* obj, const char* type);

static void setParamColor(Object* obj, double r, double g, double b);
static void setParamCenter(Object* obj, double x, double y, double z);
static void setParamAxis(Object* obj, double ax, double ay, double az, double axis_rad);
static void setParamCylinder(Object* obj, double m, double l, int longDir, double r);
static void setParamBox(Object* obj, double m, double lx, double ly, double lz);
static void setParamSphere(Object* obj, double m, double r);
static void setParamCapsule(Object* obj, double m, double l, double r);
static void setParamJointCenter(Object* obj, double jx, double jy, double jz);
static void setParamJointAxis(Object* obj, double jax, double jay, double jaz);

#endif //__INCLUDE_ODETOOLS_H__