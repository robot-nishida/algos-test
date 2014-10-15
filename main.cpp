#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ode/ode.h>
#include <drawstuff/drawstuff.h>
#include <math.h>

#ifdef _MSC_VER
#pragma warning(disable:4244 4305)  // for VC++, no precision loss complaints
#endif

#ifdef dDOUBLE
#define dsDrawBox      dsDrawBoxD
#define dsDrawSphere   dsDrawSphereD
#define dsDrawCylinder dsDrawCylinderD
#define dsDrawCapsule  dsDrawCapsuleD
#define dsDrawLine     dsDrawLineD
#endif

dWorldID world;             // 動力学計算用ワールド
dSpaceID space;             // 衝突検出用スペース
dGeomID  ground;            // 地面
dJointGroupID contactgroup; // コンタクトグループ
dsFunctions fn;             // ODE関数
dMass mass;                 // 質量パラメータ
dMatrix3 R;                 // 回転行列

typedef struct {          // Object構造体
  dBodyID body;           // ボディ(剛体)のID番号（動力学計算用）
  dGeomID geom;           // ジオメトリのID番号(衝突検出計算用）
  dJointID joint;         // 関節
  double l,r,m;           // 長さ[m], 半径[m], 質量[kg]
  double lx[3] = {0.0}    // Boxの長さ(x,y,z)
  int lognDir;            // 円柱長手方向
  double color[3] = {0.0};// オブジェクトの色(r,g,b->0-1)
  double cx[3] = {0.0};   // 中心座標
  double ax[4] = {0.0};   // 中心座標での回転軸ベクトル+回転確度[rad]
  double jcx[3] = {0.0};  // 関節中心
  double jax[3] = {0.0};  // 関節回転軸ベクトル
} Object;

typedef struct {
  Object link;         // ロボットのリンク部分
  Object motor;        // ロボットのモーター部分
} RobotObject;

typedef struct {
  Object body;         // カメラの本体（球体）
  Object lens;         // カメラのレンズ（直方体で見ている方向を表示する）
} CameraObject;

static Object base;             // ロボット座標系中心にある台座
static RobotObject robot[7];    // ロボットアーム
static CameraObject camera[4];  // カメラ
static Object endeffector[10];  // ハンド部分

static void generateCylinder(Object* obj) {
  obj -> body = dBodyCreate(world);
  dBodySetPosition(obj->body, obj->cx[0], obj->cx[1], obj->cx[2]);
  dMassSetZero(&mass);
  dMassSetCylinderTotal(&mass, obj->m, obj->lognDir, obj->r, obj->l);
  dBodySetMass(obj->body, &mass);
  obj->geom = dCreateCylinder(space, obj->r, obj->l);
  dGeomSetBody(obj->geom, obj->body);
  dRFromAxisAndAngle(R, obj->ax[0], obj->ax[1], obj->ax[2], obj->ax[3]);
  dBodySetRotation(obj->body, R);
}

static void generateHingeJoint(Object* obj, Object* target) {
  obj->joint = dJointCreateHinge(world, 0);
  dJointAttach(obj->joint, obj->body, target->body);
  dJointSetHingeAnchor(obj->joint, obj->jcx[0], obj->jcx[1], obj->jcx[2]);
  dJointSetHingeAxis(obj->joint, obj->jax[0], obj->jax[1], obj->jax[2]);
}

static void generateFixJoint(Object* obj) {
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


static void createBase() {
  base.l = 1.0;
  base.r = 0.2;
  base.m = 4.0;
  base.lognDir = 3;
  base.cx[0] = 0.0;  base.cx[1] = 0.0;  base.cx[2] = 0.5;
  generateCylinder(&base);
  generateFixJoint(&base);
}
static void createRobot() {

}
static void createCamera() {

}

static void drawBase() {
  drawObject(&base, "Cylinder");

}
static void drawRobot() {

}
static void drawCamera() {

}

// ロボット構成
static void create() {
  createBase();
//  createRobot();
//  createCamera();
}

// ロボット描画
static void draw() {
  drawBase();
//  drawRobot();
//  drawCamera();
}

// ロボット制御
static void control() {

}

// シミュレーションループ関数
static void simLoop(int pause) {
  control();
  dWorldStep(world,0.0001);                // 1ステップ進める
  draw();  // ロボットの描画
}

// 視点・視線の設定
static void setView(float x,float y,float z,float h,float p,float r) {
  static float xyz[3] = {x,y,z};
  static float hpr[3] = {h,p,r};
  dsSetViewpoint(xyz,hpr);               // 視点，視線の設定
  dsSetSphereQuality(3);                 // 球の品質設定
}

static void start()
{
  setView(2.0,2.0,2.0,-180.0,0.0,0.0);
}

void setDrawStuff()           /*** 描画関数の設定 ***/
{
  fn.version = DS_VERSION;    // ドロースタッフのバージョン
  fn.start   = &start;        // 前処理 start関数のポインタ
  fn.step    = &simLoop;      // simLoop関数のポインタ
  fn.path_to_textures = "textures"; // テクスチャ
}

int main (int argc, char *argv[])
{
  dInitODE();
  setDrawStuff();
  world        = dWorldCreate();
  space        = dHashSpaceCreate(0);
  contactgroup = dJointGroupCreate(0);

  dWorldSetGravity(world,0,0,-9.8);
  dWorldSetERP(world,1.0);          // ERPの設定
  dWorldSetCFM(world,0.0);          // CFMの設定
  ground = dCreatePlane(space,0,0,1,0);
  create();
  dsSimulationLoop (argc,argv,640,480,&fn);
  dWorldDestroy (world);
  dCloseODE();

  return 0;
}
