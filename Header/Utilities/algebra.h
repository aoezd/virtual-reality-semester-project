#ifndef __ALGEBRA_H_
#define __ALGEBRA_H_

// --------------- Definitions of different vector types ---------------
typedef union _Vec2 {
    struct
    {
        float x, y;
    };
    float elements[2];
} Vec2;

typedef union _Vec3 {
    struct
    {
        float x, y, z;
    };
    float elements[3];
} Vec3;

typedef union _Vec4 {
    struct
    {
        float x, y, z, w;
    };
    float elements[4];
} Vec4;

// --------------- Definitions of Matrices (column-oriented because better OpenGL interaction) ---------------
typedef struct
{
    Vec2 cols[2];
} Mat2;

typedef struct
{
    Vec3 cols[3];
} Mat3;

typedef struct
{
    Vec4 cols[4];
} Mat4;

// --------------- 2D functions ---------------
Vec2 makeVec(const float &x, const float &y);
void printVec(const Vec2 &v);
Vec2 add(const Vec2 &vl, const Vec2 &vr);
Vec2 sub(const Vec2 &vl, const Vec2 &vr);
Vec2 neg(const Vec2 &v);
Vec2 mulScal(const float &x, const Vec2 &v);
Vec2 divScal(const float &x, const Vec2 &v);
float dot(const Vec2 &vl, const Vec2 &vr);
float length(const Vec2 &v);
Vec2 normalize(const Vec2 &v);
float distance(const Vec2 &vl, const Vec2 &vr);
bool equal(const Vec2 &vl, const Vec2 &vr);

// --------------- 3D functions ---------------
Vec3 makeVec(const float &x, const float &y, const float &z);
void printVec(const Vec3 &v);
Vec3 add(const Vec3 &vl, const Vec3 &vr);
Vec3 sub(const Vec3 &vl, const Vec3 &vr);
Vec3 neg(const Vec3 &v);
Vec3 mulScal(const float &x, const Vec3 &v);
Vec3 divScal(const float &x, const Vec3 &v);
float dot(const Vec3 &vl, const Vec3 &vr);
float length(const Vec3 &v);
Vec3 normalize(const Vec3 &v);
Vec3 cross(const Vec3 &vl, const Vec3 &vr);
Vec3 unitCross(const Vec3 &vl, const Vec3 &vr);
float distance(const Vec3 &vl, const Vec3 &vr);
bool equal(const Vec3 &vl, const Vec3 &vr);

// --------------- 4D functions ---------------
Vec4 makeVec(const float &x, const float &y, const float &z, const float &w);
void printVec(const Vec4 &v);
Vec4 add(const Vec4 &vl, const Vec3 &vr);
Vec4 sub(const Vec4 &vl, const Vec3 &vr);
Vec4 neg(const Vec4 &v);
Vec4 mulScal(const float &x, const Vec4 &v);
Vec4 divScal(const float &x, const Vec4 &v);
float dot(const Vec4 &vl, const Vec4 &vr);
float length(const Vec4 &v);
Vec4 normalize(const Vec4 &v);
float distance(const Vec4 &vl, const Vec4 &vr);
bool equal(const Vec4 &vl, const Vec4 &vr);
Vec3 wDiv(const Vec4 &v);

// --------------- Matrix3 functions ---------------
Mat3 makeMatCols(const Vec3 &col0, const Vec3 &col1, const Vec3 &col2);
Mat3 makeMatRows(const Vec3 &col0, const Vec3 &col1, const Vec3 &col2);
Mat3 makeMatCols(const float &m00, const float &m01, const float &m02, const float &m10, const float &m11, const float &m12, const float &m20, const float &m21, const float &m22);
Mat3 makeMatRows(const float &m00, const float &m01, const float &m02, const float &m10, const float &m11, const float &m12, const float &m20, const float &m21, const float &m22);
Mat3 mulMatMat(const Mat3 &ml, const Mat3 &mr);
Vec3 mulMatVec(const Mat3 &m, const Vec3 &v);
Mat3 zeroM3();
Mat3 identityM3();
Mat3 inv(const Mat3 &m);
Mat3 transpose(const Mat3 &m);
void printMat(const Mat3 &m);

// --------------- Matrix4 functions ---------------
Mat4 makeMatCols(const Vec4 &col0, const Vec4 &col1, const Vec4 &col2, const Vec4 &col3);
Mat4 makeMatRows(const Vec4 &col0, const Vec4 &col1, const Vec4 &col2, const Vec4 &col3);
Mat4 makeMatCols(const float &m00, const float &m01, const float &m02, const float &m03, const float &m10, const float &m11, const float &m12, const float &m13, const float &m20, const float &m21, const float &m22, const float &m23, const float &m30, const float &m31, const float &m32, const float &m33);
Mat4 makematRows(const float &m00, const float &m01, const float &m02, const float &m03, const float &m10, const float &m11, const float &m12, const float &m13, const float &m20, const float &m21, const float &m22, const float &m23, const float &m30, const float &m31, const float &m32, const float &m33);
Mat4 mulMatMat(const Mat4 &ml, const Mat4 &mr);
Vec4 mulMatVec(const Mat4 &m, const Vec4 &v);
Mat4 zeroM4();
Mat4 identityM4();
Mat4 inv(const Mat4 &m);
Mat4 transpose(const Mat4 &m);
void printMat(const Mat4 &m);

// --------------- Transformations ---------------
Mat3 makeNormalMatrix(const Mat4 &worldMatrix);
Mat4 makeTranslationMatrix(const float &x, const float &y, const float &z);
Mat4 makeTranslationMatrix(const Vec3 &v);
Mat4 makeTranslationMatrix(const float &x);
Mat4 makeScaleMatrix(const float &x, const float &y, const float &z);
Mat4 makeScaleMatrix(const Vec3 &v);
Mat4 makeScaleMatrix(const float &x);
Mat4 makeRotationX(const float &angle);
Mat4 makeRotationY(const float &angle);
Mat4 makeRotationZ(const float &angle);
Mat3 makeRotation(const Vec3 &axis, const float &angle);
Mat4 makePerspective(const float &left, const float &right, const float &top, const float &bottom, const float &znear, const float &zfar);
Mat4 makeOrthographic(const float &left, const float &right, const float &top, const float &bottom, const float &znear, const float &zfar);
Mat4 makeLookAt(const Vec3 &center, const Vec3 &lookAt, const Vec3 &up);

#endif