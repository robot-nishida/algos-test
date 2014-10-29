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
dMass mass;                 // 物体質量
dMatrix3 R;                 // 回転行列

typedef struct {          // Object構造体
  dBodyID body;           // ボディ(剛体)のID番号（動力学計算用）
  dGeomID geom;           // ジオメトリのID番号(衝突検出計算用）
  dJointID joint;         // 関節
  double l = 0;           // 長さ[m], 半径[m], 質量[kg]
  double r = 0;
  double m = 0;
  double lx[3] = {};      // Boxの長さ(x,y,z)
  int longDir = 3;        // 円柱長手方向
  double color[3] = {};   // オブジェクトの色(r,g,b->0-1)
  double cx[3] = {};      // 中心座標
  double ax[4] = {};      // 中心座標での回転軸ベクトル+回転確度[rad]
  double jcx[3] = {};     // 関節中心
  double jax[3] = {};     // 関節回転軸ベクトル
} Object;

typedef struct {
  Object link;         // ロボットのリンク部分
  Object motor;        // ロボットのモーター部分
} RobotObject;

typedef struct {
  Object body;         // カメラの本体（球体）
  Object lens;         // カメラのレンズ（直方体で見ている方向を表示する）
} CameraObject;

#define LINK_NUM 7     // ロボットアームのリンク数
#define CAM_NUM 4      // カメラの台数
#define EE_NUM 10      // ハンド部分のパーツ点数

static Object base;                 // ロボット座標系中心にある台座
static RobotObject robot[LINK_NUM]; // ロボットアーム
static CameraObject camera[CAM_NUM];// カメラ
static Object endeffector[EE_NUM];  // ハンド部分

static void generateObject(Object* obj, const char* type) {
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

static void createBase() {
  setParamCylinder(&base, 1.0, 0.2, 3, 0.4);
  setParamCenter(&base, 0.0, 0.0, 0.5);
  setParamColor(&base, 0.0, 0.0, 0.0);
  setParamAxis(&base, 0.0, 0.0, 0.0, 0.0);
  setParamJointCenter(&base, 0.0, 0.0, 0.0);
  setParamJointAxis(&base, 0.0, 0.0, 0.0);
  generateObject(&base, "Cylinder");
  generateFixJoint(&base);
}

static void createRobot() {
  dReal m[LINK_NUM] =
    { 0.5, 0.5, 0.7, 0.7, 0.7, 0.5, 0.5 };
  dReal l[LINK_NUM] =
    { 0.15, 0.1, 0.2, 0.1, 0.2, 0.15, 0.15 };
  dReal r[LINK_NUM] =
    { 0.04, 0.04, 0.04, 0.04, 0.04, 0.04, 0.04 };
  int longDir[LINK_NUM] =
    { 3, 3, 3, 3, 3, 3, 3 };

  dReal cx[LINK_NUM][3] = {
    { 0.0, 0.0, 0.075 }, { 0.0, 0.0, 0.20 }, { 0.0, 0.0, 0.35 },
    { 0.0, 0.0, 0.50 }, { 0.0, 0.0, 0.65 }, { 0.0, 0.0, 0.825 },
    { 0.0, 0.0, 0.975 }
  };

  dReal ax[LINK_NUM][4] = {
    { 0.0, 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0, 0.0 },
    { 0.0, 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0, 0.0 },
    { 0.0, 0.0, 0.0, 0.0 }
  };

  dReal jcx[LINK_NUM][3] = {
    { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.15 }, { 0.0, 0.0, 0.25 },
    { 0.0, 0.0, 0.45 }, { 0.0, 0.0, 0.55 }, { 0.0, 0.0, 0.75 },
    { 0.0, 0.0, 0.90 }
  };

  dReal jax[LINK_NUM][3] = {
    { 0.0, 0.0, 1.0 }, { 0.0, 1.0, 0.0 }, { 0.0, 0.0, 1.0 },
    { 0.0, 1.0, 0.0 }, { 0.0, 0.0, 1.0 }, { 0.0, 1.0, 0.0 },
    { 0.0, 0.0, 1.0 }
  };

  dReal color[LINK_NUM][3] = {
    { 0.2, 0.2, 0.2 }, { 0.3, 0.3, 0.3 }, { 0.4, 0.4, 0.4 },
    { 0.5, 0.5, 0.5 }, { 0.6, 0.6, 0.6 }, { 0.7, 0.7, 0.7 },
    { 0.8, 0.8, 0.8 }
  };

  // リンクの生成
  for(int i=0; i<LINK_NUM; i++){
    setParamCylinder(&robot[i].link, m[i], l[i], longDir[i], r[i]);
    setParamCenter(&robot[i].link, cx[i][0], cx[i][1], cx[i][2]);
    setParamColor(&robot[i].link, color[i][0], color[i][1], color[i][2]);
    setParamAxis(&robot[i].link, ax[i][0], ax[i][1], ax[i][2], ax[i][3]);
    setParamJointCenter(&robot[i].link, jcx[i][0], jcx[i][1], jcx[i][2]);
    setParamJointAxis(&robot[i].link, jax[i][0], jax[i][1], jax[i][2]);
    generateObject(&robot[i].link, "Cylinder");
    generateFixJoint(&robot[i].link);
  }

  dReal m_motor[LINK_NUM] =
    { 0.4, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4 };
  dReal l_motor[LINK_NUM] =
    { 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 0.1 };
  dReal r_motor[LINK_NUM] =
    { 0.05, 0.05, 0.05, 0.05, 0.05, 0.05, 0.05 };
  int longDir_motor[LINK_NUM] =
    { 3, 2, 3, 2, 3, 2, 3 };

  dReal cx_motor[LINK_NUM][3] = {
    { 0.0, 0.0, 0.00 }, { 0.0, 0.0, 0.15 }, { 0.0, 0.0, 0.25 },
    { 0.0, 0.0, 0.45 }, { 0.0, 0.0, 0.55 }, { 0.0, 0.0, 0.75 },
    { 0.0, 0.0, 0.90 }
  };

  dReal ax_motor[LINK_NUM][4] = {
    { 0.0, 0.0, 0.0, 0.0 }, { 1.0, 0.0, 0.0, M_PI/2.0 }, { 0.0, 0.0, 0.0, 0.0 },
    { 1.0, 0.0, 0.0, M_PI/2.0 }, { 0.0, 0.0, 0.0, 0.0 }, { 1.0, 0.0, 0.0, M_PI/2.0 },
    { 0.0, 0.0, 0.0, 0.0 }
  };

  dReal jcx_motor[LINK_NUM][3] = {
    { 0.0, 0.0, 0.00 }, { 0.0, 0.0, 0.15 }, { 0.0, 0.0, 0.25 },
    { 0.0, 0.0, 0.45 }, { 0.0, 0.0, 0.55 }, { 0.0, 0.0, 0.75 },
    { 0.0, 0.0, 0.90 }
  };

  dReal jax_motor[LINK_NUM][3] = {
    { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 },
    { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 }, { 0.0, 0.0, 0.0 },
    { 0.0, 0.0, 0.0 }
  };

  dReal color_motor[LINK_NUM][3] = {
    { 0.2, 0.0, 0.0 }, { 0.3, 0.0, 0.0 }, { 0.4, 0.0, 0.0 },
    { 0.5, 0.0, 0.0 }, { 0.6, 0.0, 0.0 }, { 0.7, 0.0, 0.0 },
    { 0.8, 0.0, 0.0 }
  };

  // モーターの生成
  for(int i=0; i<LINK_NUM; i++){
    setParamCylinder(&robot[i].motor, m_motor[i], l_motor[i], longDir_motor[i], r_motor[i]);
    setParamCenter(&robot[i].motor, cx_motor[i][0], cx_motor[i][1], cx_motor[i][2]);
    setParamColor(&robot[i].motor, color_motor[i][0], color_motor[i][1], color_motor[i][2]);
    setParamAxis(&robot[i].motor, ax_motor[i][0], ax_motor[i][1], ax_motor[i][2], ax_motor[i][3]);
    setParamJointCenter(&robot[i].motor, jcx_motor[i][0], jcx_motor[i][1], jcx_motor[i][2]);
    setParamJointAxis(&robot[i].motor, jax_motor[i][0], jax_motor[i][1], jax_motor[i][2]);
    generateObject(&robot[i].motor, "Cylinder");
    generateFixJoint(&robot[i].motor);
  }


}
static void createCamera() {

}

// ロボット構成
static void create() {
  //createBase();
  createRobot();
  //createCamera();
}

// ロボット描画
static void draw() {
  //drawObject(&base, "Cylinder");
  // アームの描画
  for(int i=0; i<LINK_NUM; i++){
    drawObject(&robot[i].link, "Cylinder");
    drawObject(&robot[i].motor, "Cylinder");
  }
}

// ロボット制御
static void control() {

}

// シミュレーションループ関数
static void simLoop(int pause) {
  control();
  dWorldStep(world, 0.001);                // 1ステップ進める
  draw();  // ロボットの描画
}

// 視点・視線の設定
static void setView(float x,float y,float z,float h,float p,float r) {
  static float xyz[3] = {x,y,z};
  static float hpr[3] = {h,p,r};
  dsSetViewpoint(xyz,hpr);               // 視点，視線の設定
  dsSetSphereQuality(3);                 // 球の品質設定
}

static void start() {
  setView(0.6,0.6,0.6,-145.0,0.0,0.0);
}

// 描画関数の設定
void setDrawStuff() {
  fn.version = DS_VERSION;    // ドロースタッフのバージョン
  fn.start   = &start;        // 前処理 start関数のポインタ
  fn.step    = &simLoop;      // simLoop関数のポインタ
  fn.path_to_textures = "textures"; // テクスチャ
}

int main (int argc, char *argv[]) {
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
  dsSimulationLoop(argc,argv,640,480,&fn);
  dWorldDestroy(world);
  dCloseODE();

  return 0;
}