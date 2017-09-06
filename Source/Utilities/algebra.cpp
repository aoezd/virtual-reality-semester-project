/**
 * algebra.cpp
 * TODO
 * 
 * Created: 2017-09-06
 * Author: Aykut Özdemir
 */

#include <iostream>

#include "../../Header/Utilities/algebra.h"
#include "../../Header/Utilities/utils.h"

// --------------- 2D functions ---------------

/**
 *
 */
Vec2 makeVec(const float &x, const float &y)
{
    Vec2 v;

    v.x = x;
    v.y = y;

    return v;
}

/**
 *
 */
void printVec(const Vec2 &v)
{
    std::cout << "[" << v.x << ", " << v.y << "]" << std::endl;
}

/**
 *
 */
Vec2 add(const Vec2 &vl, const Vec2 &vr)
{
    return makeVec(vl.x + vr.x, vl.y + vr.y);
}

/**
 *
 */
Vec2 sub(const Vec2 &vl, const Vec2 &vr)
{
    return makeVec(vl.x - vr.x, vl.y - vr.y);
}

/**
 *
 */
Vec2 neg(const Vec2 &v)
{
    return makeVec(-v.x, -v.y);
}

/**
 *
 */
Vec2 mulScal(const float &x, const Vec2 &v)
{
    return makeVec(v.x * x, v.y * x);
}

/**
 *
 */
Vec2 divScal(const float &x, const Vec2 &v)
{
    return makeVec(v.x / x, v.y / x);
}

/**
 *
 */
float dot(const Vec2 &vl, const Vec2 &vr)
{
    return vl.x * vr.x + vl.y * vr.y;
}

/**
 *
 */
float length(const Vec2 &v)
{
    return fastInvSqrt(dot(v, v));
}

/**
 *
 */
Vec2 normalize(const Vec2 &v)
{
    return divScal(length(v), v);
}

/**
 *
 */
float distance(const Vec2 &vl, const Vec2 &vr)
{
    return length(sub(vl, vr));
}

/**
 *
 */
bool equal(const Vec2 &vl, const Vec2 &vr)
{
    return floatEqual(vl.x, vr.x) && floatEqual(vl.y, vr.y);
}

// --------------- 3D functions ---------------

/**
 *
 */
Vec3 makeVec(const float &x, const float &y, const float &z)
{
    Vec3 v;

    v.x = x;
    v.y = y;
    v.z = z;

    return v;
}

/**
 *
 */
void printVec(const Vec3 &v)
{
    std::cout << "[" << v.x << ", " << v.y << ", " << v.z << "]" << std::endl;
}

/**
 *
 */
Vec3 add(const Vec3 &vl, const Vec3 &vr)
{

    return makeVec(vl.x + vr.x, vl.y + vr.y, vl.z + vr.z);
}

/**
 *
 */
Vec3 sub(const Vec3 &vl, const Vec3 &vr)
{
    return makeVec(vl.x - vr.x, vl.y - vr.y, vl.z - vr.z);
}

/**
 *
 */
Vec3 neg(const Vec3 &v)
{
    return makeVec(-v.x, -v.y, -v.z);
}

/**
 *
 */
Vec3 mulScal(const float &x, const Vec3 &v)
{
    return makeVec(x * v.x, x * v.y, x * v.z);
}

/**
 *
 */
Vec3 divScal(const float &x, const Vec3 &v)
{
    return makeVec(v.x / x, v.y / x, v.z / x);
}

/**
 *
 */
float dot(const Vec3 &vl, const Vec3 &vr)
{
    return vl.x * vr.x + vl.y * vr.y + vl.z * vr.z;
}

/**
 *
 */
float length(const Vec3 &v)
{
    return fastInvSqrt(dot(v, v));
}

/**
 *
 */
Vec3 normalize(const Vec3 &v)
{
    return divScal(length(v), v);
}

/**
 *
 */
Vec3 cross(const Vec3 &vl, const Vec3 &vr)
{
    return makeVec(vl.y * vr.z - vl.z * vr.y, vl.z * vr.x - vl.x * vr.z, vl.x * vr.y - vl.y * vr.x);
}

/**
 *
 */
Vec3 unitCross(const Vec3 &vl, const Vec3 &vr)
{
    return normalize(cross(vl, vr));
}

/**
 *
 */
float distance(const Vec3 &vl, const Vec3 &vr)
{
    return length(sub(vl, vr));
}

/**
 *
 */
bool equal(const Vec3 &vl, const Vec3 &vr)
{
    return floatEqual(vl.x, vr.x) && floatEqual(vl.y, vr.y) && floatEqual(vr.z, vl.z);
}

// --------------- 4D functions ---------------

/**
 *
 */
Vec4 makeVec(const float &x, const float &y, const float &z, const float &w)
{
    Vec4 v;

    v.x = x;
    v.y = y;
    v.z = z;
    v.w = w;

    return v;
}

/**
 *
 */
void printVec(const Vec4 &v)
{
    std::cout << "[" << v.x << ", " << v.y << ", " << v.z << ", " << v.w << "]" << std::endl;
}

/**
 *
 */
Vec4 add(const Vec4 &vl, const Vec4 &vr)
{
    return makeVec(vl.x + vr.x, vl.y + vr.y, vl.z + vr.z, vl.w + vr.w);
}

/**
 *
 */
Vec4 sub(const Vec4 &vl, const Vec4 &vr)
{
    return makeVec(vl.x - vr.x, vl.y - vr.y, vl.z - vr.z, vl.w - vr.w);
}

/**
 *
 */
Vec4 neg(const Vec4 &v)
{
    return makeVec(-v.x, -v.y, -v.z, -v.w);
}

/**
 *
 */
Vec4 mulScal(const float &x, const Vec4 &v)
{
    return makeVec(v.x * x, v.y * x, v.z * x, v.w * x);
}

/**
 *
 */
Vec4 divScal(const float &x, const Vec4 &v)
{
    return makeVec(v.x / x, v.y / x, v.z / x, v.w / x);
}

/**
 *
 */
float dot(const Vec4 &vl, const Vec4 &vr)
{
    return vl.x * vr.x + vl.y * vr.y + vl.z * vr.z + vl.w * vr.w;
}

/**
 *
 */
float length(const Vec4 &v)
{
    return fastInvSqrt(dot(v, v));
}

/**
 *
 */
Vec4 normalize(const Vec4 &v)
{
    return divScal(length(v), v);
}

/**
 *
 */
float distance(const Vec4 &vl, const Vec4 &vr)
{
    return length(sub(vl, vr));
}

/**
 *
 */
bool equal(const Vec4 &vl, const Vec4 &vr)
{
    return floatEqual(vl.x, vr.x) && floatEqual(vl.y, vr.y) && floatEqual(vl.z, vr.z) && floatEqual(vl.w, vr.w);
}

/**
 *
 */
Vec3 wDiv(const Vec4 &v)
{
    return makeVec(v.x / v.w, v.y / v.w, v.z / v.w);
}

// --------------- Matrix3 functions ---------------

/**
 *
 */
Mat3 makeMatCols(const Vec3 &col0, const Vec3 &col1, const Vec3 &col2)
{
    Mat3 res;

    res.cols[0] = col0;
    res.cols[1] = col1;
    res.cols[2] = col2;

    return res;
}

/**
 *
 */
Mat3 makeMatRows(const Vec3 &row0, const Vec3 &row1, const Vec3 &row2)
{
    Mat3 res;

    res.cols[0].x = row0.x;
    res.cols[0].y = row1.x;
    res.cols[0].z = row2.x;
    res.cols[1].x = row0.y;
    res.cols[1].y = row1.y;
    res.cols[1].z = row2.y;
    res.cols[2].x = row0.z;
    res.cols[2].y = row1.z;
    res.cols[2].z = row2.z;

    return res;
}

/**
 *
 */
Mat3 makeMatCols(const float &m00, const float &m01, const float &m02, const float &m10, const float &m11, const float &m12, const float &m20, const float &m21, const float &m22)
{
    Mat3 res;

    res.cols[0].x = m00;
    res.cols[0].y = m10;
    res.cols[0].z = m20;
    res.cols[1].x = m01;
    res.cols[1].y = m11;
    res.cols[1].z = m21;
    res.cols[2].x = m02;
    res.cols[2].y = m12;
    res.cols[2].z = m22;

    return res;
}

/**
 *
 */
Mat3 makeMatRows(const float &m00, const float &m01, const float &m02, const float &m10, const float &m11, const float &m12, const float &m20, const float &m21, const float &m22)
{
    Mat3 res;

    res.cols[0].x = m00;
    res.cols[0].y = m10;
    res.cols[0].z = m20;
    res.cols[1].x = m01;
    res.cols[1].y = m11;
    res.cols[1].z = m21;
    res.cols[2].x = m02;
    res.cols[2].y = m12;
    res.cols[2].z = m22;

    return res;
}

/**
 *
 */
Mat3 mulMatMat(const Mat3 &ml, const Mat3 &mr)
{
    Mat3 res;

    res.cols[0].x = ml.cols[0].x * mr.cols[0].x + ml.cols[1].x * mr.cols[0].y + ml.cols[2].x * mr.cols[0].z;
    res.cols[0].y = ml.cols[0].y * mr.cols[0].x + ml.cols[1].y * mr.cols[0].y + ml.cols[2].y * mr.cols[0].z;
    res.cols[0].z = ml.cols[0].z * mr.cols[0].x + ml.cols[1].z * mr.cols[0].y + ml.cols[2].z * mr.cols[0].z;
    res.cols[1].x = ml.cols[0].x * mr.cols[1].x + ml.cols[1].x * mr.cols[1].y + ml.cols[2].x * mr.cols[1].z;
    res.cols[1].y = ml.cols[0].y * mr.cols[1].x + ml.cols[1].y * mr.cols[1].y + ml.cols[2].y * mr.cols[1].z;
    res.cols[1].z = ml.cols[0].z * mr.cols[1].x + ml.cols[1].z * mr.cols[1].y + ml.cols[2].z * mr.cols[1].z;
    res.cols[2].x = ml.cols[0].x * mr.cols[2].x + ml.cols[1].x * mr.cols[2].y + ml.cols[2].x * mr.cols[2].z;
    res.cols[2].y = ml.cols[0].y * mr.cols[2].x + ml.cols[1].y * mr.cols[2].y + ml.cols[2].y * mr.cols[2].z;
    res.cols[2].z = ml.cols[0].z * mr.cols[2].x + ml.cols[1].z * mr.cols[2].y + ml.cols[2].z * mr.cols[2].z;

    return res;
}

/**
 *
 */
Vec3 mulMatVec(const Mat3 &m, const Vec3 &v)
{
    return makeVec(
        m.cols[0].x * v.x + m.cols[1].x * v.y + m.cols[2].x * v.z,
        m.cols[0].y * v.x + m.cols[1].y * v.y + m.cols[2].y * v.z,
        m.cols[0].z * v.x + m.cols[1].z * v.y + m.cols[2].z * v.z);
}

/**
 *
 */
Mat3 zeroM3()
{
    return makeMatRows(
        0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f);
}

/**
 *
 */
Mat3 identityM3()
{
    return makeMatRows(
        1.0f, 0.0f, 0.0f,
        0.0f, 1.0f, 0.0f,
        0.0f, 0.0f, 1.0f);
}

/**
 *
 */
Mat3 inv(const Mat3 &m)
{
    float oneOverDeterminant = 1.0f / (+m.cols[0].x * (m.cols[1].y * m.cols[2].z - m.cols[2].y * m.cols[1].z) - m.cols[1].x * (m.cols[0].y * m.cols[2].z - m.cols[2].y * m.cols[0].z) + m.cols[2].x * (m.cols[0].y * m.cols[1].z - m.cols[1].y * m.cols[0].z));
    Mat3 inv;

    inv.cols[0].x = +(m.cols[1].y * m.cols[2].z - m.cols[2].y * m.cols[1].z) * oneOverDeterminant;
    inv.cols[1].x = -(m.cols[1].x * m.cols[2].z - m.cols[2].x * m.cols[1].z) * oneOverDeterminant;
    inv.cols[2].x = +(m.cols[1].x * m.cols[2].y - m.cols[2].x * m.cols[1].y) * oneOverDeterminant;
    inv.cols[0].y = -(m.cols[0].y * m.cols[2].z - m.cols[2].y * m.cols[0].z) * oneOverDeterminant;
    inv.cols[1].y = +(m.cols[0].x * m.cols[2].z - m.cols[2].x * m.cols[0].z) * oneOverDeterminant;
    inv.cols[2].y = -(m.cols[0].x * m.cols[2].y - m.cols[2].x * m.cols[0].y) * oneOverDeterminant;
    inv.cols[0].z = +(m.cols[0].y * m.cols[1].z - m.cols[1].y * m.cols[0].z) * oneOverDeterminant;
    inv.cols[1].z = -(m.cols[0].x * m.cols[1].z - m.cols[1].x * m.cols[0].z) * oneOverDeterminant;
    inv.cols[2].z = +(m.cols[0].x * m.cols[1].y - m.cols[1].x * m.cols[0].y) * oneOverDeterminant;

    return inv;
}

/**
 *
 */
Mat3 transpose(const Mat3 &m)
{
    Mat3 res;

    res.cols[0].x = m.cols[0].x;
    res.cols[0].y = m.cols[1].x;
    res.cols[0].z = m.cols[2].x;
    res.cols[1].x = m.cols[0].y;
    res.cols[1].y = m.cols[1].y;
    res.cols[1].z = m.cols[2].y;
    res.cols[2].x = m.cols[0].z;
    res.cols[2].y = m.cols[1].z;
    res.cols[2].z = m.cols[2].z;

    return res;
}

/**
 *
 */
void printMat(const Mat3 &m)
{
    std::cout << "{\t" << m.cols[0].x << ", " << m.cols[1].x << ", " << m.cols[2].x << std::endl;
    std::cout << "\t" << m.cols[0].y << ", " << m.cols[1].y << ", " << m.cols[2].y << std::endl;
    std::cout << "\t" << m.cols[0].z << ", " << m.cols[1].z << ", " << m.cols[2].z << "\t}" << std::endl;
}

// --------------- Matrix4 functions ---------------

/**
 *
 */
Mat4 makeMatCols(const Vec4 &col0, const Vec4 &col1, const Vec4 &col2, const Vec4 &col3)
{
    Mat4 res;
    
    res.cols[0] = col0;
    res.cols[1] = col1;
    res.cols[2] = col2;
    res.cols[3] = col3;

    return res;
}

/**
  *
  */
Mat4 makeMatRows(const Vec4 &row0, const Vec4 &row1, const Vec4 &row2, const Vec4 &row3)
{
    Mat4 res;

    res.cols[0].x = row0.x;
    res.cols[0].y = row1.x;
    res.cols[0].z = row2.x;
    res.cols[0].w = row3.x;
    res.cols[1].x = row0.y;
    res.cols[1].y = row1.y;
    res.cols[1].z = row2.y;
    res.cols[1].w = row3.y;
    res.cols[2].x = row0.z;
    res.cols[2].y = row1.z;
    res.cols[2].z = row2.z;
    res.cols[2].w = row3.z;
    res.cols[3].x = row0.w;
    res.cols[3].y = row1.w;
    res.cols[3].z = row2.w;
    res.cols[3].w = row3.w;

    return res;
}

/**
  *
  */
Mat4 makeMatCols(const float &m00, const float &m01, const float &m02, const float &m03, const float &m10, const float &m11, const float &m12, const float &m13, const float &m20, const float &m21, const float &m22, const float &m23, const float &m30, const float &m31, const float &m32, const float &m33)
{
    Mat4 res;

    res.cols[0].x = m00;
    res.cols[0].y = m10;
    res.cols[0].z = m20;
    res.cols[0].w = m30;
    res.cols[1].x = m01;
    res.cols[1].y = m11;
    res.cols[1].z = m21;
    res.cols[1].w = m31;
    res.cols[2].x = m02;
    res.cols[2].y = m12;
    res.cols[2].z = m22;
    res.cols[2].w = m32;
    res.cols[3].x = m03;
    res.cols[3].y = m13;
    res.cols[3].z = m23;
    res.cols[3].w = m33;

    return res;
}

/**
  *
  */
Mat4 makeMatRows(const float &m00, const float &m01, const float &m02, const float &m03, const float &m10, const float &m11, const float &m12, const float &m13, const float &m20, const float &m21, const float &m22, const float &m23, const float &m30, const float &m31, const float &m32, const float &m33)
{
    Mat4 res;

    res.cols[0].x = m00;
    res.cols[0].y = m10;
    res.cols[0].z = m20;
    res.cols[0].w = m30;
    res.cols[1].x = m01;
    res.cols[1].y = m11;
    res.cols[1].z = m21;
    res.cols[1].w = m31;
    res.cols[2].x = m02;
    res.cols[2].y = m12;
    res.cols[2].z = m22;
    res.cols[2].w = m32;
    res.cols[3].x = m03;
    res.cols[3].y = m13;
    res.cols[3].z = m23;
    res.cols[3].w = m33;

    return res;
}

/**
 *
 */
Mat4 mulMatMat(const Mat4 &ml, const Mat4 &mr)
{
    Mat4 res;

    res.cols[0] = add(add(mulScal(mr.cols[0].x, ml.cols[0]), mulScal(mr.cols[0].y, ml.cols[1])), add(mulScal(mr.cols[0].z, ml.cols[2]), mulScal(mr.cols[0].w, ml.cols[3])));
    res.cols[1] = add(add(mulScal(mr.cols[1].x, ml.cols[0]), mulScal(mr.cols[1].y, ml.cols[1])), add(mulScal(mr.cols[1].z, ml.cols[2]), mulScal(mr.cols[1].w, ml.cols[3])));
    res.cols[2] = add(add(mulScal(mr.cols[2].x, ml.cols[0]), mulScal(mr.cols[2].y, ml.cols[1])), add(mulScal(mr.cols[2].z, ml.cols[2]), mulScal(mr.cols[2].w, ml.cols[3])));
    res.cols[3] = add(add(mulScal(mr.cols[3].x, ml.cols[0]), mulScal(mr.cols[3].y, ml.cols[1])), add(mulScal(mr.cols[3].z, ml.cols[2]), mulScal(mr.cols[3].w, ml.cols[3])));

    return res;
}

/**
 *
 */
Vec4 mulMatVec(const Mat4 &m, const Vec4 &v)
{
    return makeVec(
        m.cols[0].x * v.x + m.cols[1].x * v.y + m.cols[2].x * v.z + v.w * m.cols[3].x,
        m.cols[0].y * v.x + m.cols[1].y * v.y + m.cols[2].y * v.z + v.w * m.cols[3].y,
        m.cols[0].z * v.x + m.cols[1].z * v.y + m.cols[2].z * v.z + v.w * m.cols[3].z,
        m.cols[0].w * v.x + m.cols[1].w * v.y + m.cols[2].w * v.z + v.w * m.cols[3].w);
}

/**
 *
 */
Mat4 zeroM4()
{
    return makeMatRows(
        0.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 0.0f);
}

/**
 *
 */
Mat4 identityM4()
{
    return makeMatRows(
        1.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 1.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 1.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 1.0f);
}

/**
 *
 */
Mat4 inv(const Mat4 &m)
{
    float m11 = m.cols[0].x;
    float m12 = m.cols[1].x;
    float m13 = m.cols[2].x;
    float m14 = m.cols[3].x;

    float m21 = m.cols[0].y;
    float m22 = m.cols[1].y;
    float m23 = m.cols[2].y;
    float m24 = m.cols[3].y;

    float m31 = m.cols[0].z;
    float m32 = m.cols[1].z;
    float m33 = m.cols[2].z;
    float m34 = m.cols[3].z;

    float m41 = m.cols[0].w;
    float m42 = m.cols[1].w;
    float m43 = m.cols[2].w;
    float m44 = m.cols[3].w;

    float oneOverDeterminant = 1.0f /
                               (m11 * m22 * m33 * m44 + m11 * m23 * m34 * m42 + m11 * m24 * m32 * m43 +
                                m12 * m21 * m34 * m43 + m12 * m23 * m31 * m44 + m12 * m24 * m33 * m41 +
                                m13 * m21 * m32 * m44 + m13 * m22 * m34 * m41 + m13 * m24 * m31 * m42 +
                                m14 * m21 * m33 * m42 + m14 * m22 * m31 * m43 + m14 * m23 * m32 * m41 -
                                m11 * m22 * m34 * m43 - m11 * m23 * m32 * m44 - m11 * m24 * m33 * m42 -
                                m12 * m21 * m33 * m44 - m12 * m23 * m34 * m41 - m12 * m24 * m31 * m43 -
                                m13 * m21 * m34 * m42 - m13 * m22 * m31 * m44 - m13 * m24 * m32 * m41 -
                                m14 * m21 * m32 * m43 - m14 * m22 * m33 * m41 - m14 * m23 * m31 * m42);
    Mat4 inv;

    inv.cols[0].x = oneOverDeterminant * (m22 * m33 * m44 + m23 * m34 * m42 + m24 * m32 * m43 - m22 * m34 * m43 - m23 * m32 * m44 - m24 * m33 * m42);
    inv.cols[1].x = oneOverDeterminant * (m12 * m34 * m43 + m13 * m32 * m44 + m14 * m33 * m42 - m12 * m33 * m44 - m13 * m34 * m42 - m14 * m32 * m43);
    inv.cols[2].x = oneOverDeterminant * (m12 * m23 * m44 + m13 * m24 * m42 + m14 * m22 * m43 - m12 * m24 * m43 - m13 * m22 * m44 - m14 * m23 * m42);
    inv.cols[3].x = oneOverDeterminant * (m12 * m24 * m33 + m13 * m22 * m34 + m14 * m23 * m32 - m12 * m23 * m34 - m13 * m24 * m32 - m14 * m22 * m33);
    inv.cols[0].y = oneOverDeterminant * (m21 * m34 * m43 + m23 * m31 * m44 + m24 * m33 * m41 - m21 * m33 * m44 - m23 * m34 * m41 - m24 * m31 * m43);
    inv.cols[1].y = oneOverDeterminant * (m11 * m33 * m44 + m13 * m34 * m41 + m14 * m31 * m43 - m11 * m34 * m43 - m13 * m31 * m44 - m14 * m33 * m41);
    inv.cols[2].y = oneOverDeterminant * (m11 * m24 * m43 + m13 * m21 * m44 + m14 * m23 * m41 - m11 * m23 * m44 - m13 * m24 * m41 - m14 * m21 * m43);
    inv.cols[3].y = oneOverDeterminant * (m11 * m23 * m34 + m13 * m24 * m31 + m14 * m21 * m33 - m11 * m24 * m33 - m13 * m21 * m34 - m14 * m23 * m31);
    inv.cols[0].z = oneOverDeterminant * (m21 * m32 * m44 + m22 * m34 * m41 + m24 * m31 * m42 - m21 * m34 * m42 - m22 * m31 * m44 - m24 * m32 * m41);
    inv.cols[1].z = oneOverDeterminant * (m11 * m34 * m42 + m12 * m31 * m44 + m14 * m32 * m41 - m11 * m32 * m44 - m12 * m34 * m41 - m14 * m31 * m42);
    inv.cols[2].z = oneOverDeterminant * (m11 * m22 * m44 + m12 * m24 * m41 + m14 * m21 * m42 - m11 * m24 * m42 - m12 * m21 * m44 - m14 * m22 * m41);
    inv.cols[3].z = oneOverDeterminant * (m11 * m24 * m32 + m12 * m21 * m34 + m14 * m22 * m31 - m11 * m22 * m34 - m12 * m24 * m31 - m14 * m21 * m32);
    inv.cols[0].w = oneOverDeterminant * (m21 * m33 * m42 + m22 * m31 * m43 + m23 * m32 * m41 - m21 * m32 * m43 - m22 * m33 * m41 - m23 * m31 * m42);
    inv.cols[1].w = oneOverDeterminant * (m11 * m32 * m43 + m12 * m33 * m41 + m13 * m31 * m42 - m11 * m33 * m42 - m12 * m31 * m43 - m13 * m32 * m41);
    inv.cols[2].w = oneOverDeterminant * (m11 * m23 * m42 + m12 * m21 * m43 + m13 * m22 * m41 - m11 * m22 * m43 - m12 * m23 * m41 - m13 * m21 * m42);
    inv.cols[3].w = oneOverDeterminant * (m11 * m22 * m33 + m12 * m23 * m31 + m13 * m21 * m32 - m11 * m23 * m32 - m12 * m21 * m33 - m13 * m22 * m31);

    return inv;
}

/**
 *
 */
Mat4 transpose(const Mat4 &m)
{
    Mat4 res;
    
    res.cols[0].x = m.cols[0].x;
    res.cols[0].y = m.cols[1].x;
    res.cols[0].z = m.cols[2].x;
    res.cols[0].w = m.cols[3].x;
    res.cols[1].x = m.cols[0].y;
    res.cols[1].y = m.cols[1].y;
    res.cols[1].z = m.cols[2].y;
    res.cols[1].w = m.cols[3].y;
    res.cols[2].x = m.cols[0].z;
    res.cols[2].y = m.cols[1].z;
    res.cols[2].z = m.cols[2].z;
    res.cols[2].w = m.cols[3].z;
    res.cols[3].x = m.cols[0].w;
    res.cols[3].y = m.cols[1].w;
    res.cols[3].z = m.cols[2].w;
    res.cols[3].w = m.cols[3].w;

    return res;
}

/**
 *
 */
void printMat(const Mat4 &m)
{
    std::cout << "{\t" << m.cols[0].x << ", " << m.cols[1].x << ", " << m.cols[2].x << ", " << m.cols[3].x << std::endl;
    std::cout << "\t" << m.cols[0].y << ", " << m.cols[1].y << ", " << m.cols[2].y << ", " << m.cols[3].y << std::endl;
    std::cout << "\t" << m.cols[0].z << ", " << m.cols[1].z << ", " << m.cols[2].z << ", " << m.cols[3].z << std::endl;
    std::cout << "\t" << m.cols[0].w << ", " << m.cols[1].w << ", " << m.cols[2].w << ", " << m.cols[3].w << "\t}" << std::endl;
}

// --------------- Transformations ---------------

/**
 *
 */
Mat3 makeNormalMatrix(const Mat4 &worldMatrix) {    
    Mat4 m = transpose(inv(worldMatrix));
    Vec3 col0 = makeVec(m.cols[0].x, m.cols[0].y, m.cols[0].z);
    Vec3 col1 = makeVec(m.cols[1].x, m.cols[1].y, m.cols[1].z);
    Vec3 col2 = makeVec(m.cols[2].x, m.cols[2].y, m.cols[2].z);

    return makeMatCols(col0, col1, col2);
}

/**
 *
 */
Mat4 makeTranslationMatrix(const float &x, const float &y, const float &z)
{
    return makeMatRows(
        1.0f, 0.0f, 0.0f, x,
        0.0f, 1.0f, 0.0f, y,
        0.0f, 0.0f, 1.0f, z,
        0.0f, 0.0f, 0.0f, 1.0f);
}

/**
 *
 */
Mat4 makeTranslationMatrix(const Vec3 &v)
{
    return makeTranslationMatrix(v.x, v.y, v.z);
}

/**
 *
 */
Mat4 makeTranslationMatrix(const float &x)
{
    return makeTranslationMatrix(x, x, x);
}

/**
 *
 */
Mat4 makeScaleMatrix(const float &x, const float &y, const float &z)
{
    return makeMatRows(
        x, 0.0f, 0.0f, 0.0f,
        0.0f, y, 0.0f, 0.0f,
        0.0f, 0.0f, z, 0.0f,
        0.0f, 0.0f, 0.0f, 1.0f);
}

/**
 *
 */
Mat4 makeScaleMatrix(const Vec3 &v)
{
    return makeScaleMatrix(v.x, v.y, v.z);
}

/**
 *
 */
Mat4 makeScaleMatrix(const float &x)
{
    return makeScaleMatrix(x, x, x);
}

/**
 *
 */
Mat4 makeRotationX(const float &angle)
{
    return makeMatRows(
        1.0f, 0.0f, 0.0f, 0.0f,
        0.0f, cos(angle), -sin(angle), 0.0f,
        0.0f, sin(angle), cos(angle), 0.0f,
        0.0f, 0.0f, 0.0f, 1.0f);
}

/**
 *
 */
Mat4 makeRotationY(const float &angle)
{
    return makeMatRows(
        cos(angle), 0.0f, sin(angle), 0.0f,
        0.0f, 1.0f, 0.0f, 0.0f,
        -sin(angle), 0.0f, cos(angle), 0.0f,
        0.0f, 0.0f, 0.0f, 1.0f);
}

/**
 *
 */
Mat4 makeRotationZ(const float &angle)
{
    return makeMatRows(
        cos(angle), -sin(angle), 0.0f, 0.0f,
        sin(angle), cos(angle), 0.0f, 0.0f,
        0.0f, 0.0f, 1.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 1.0f);
}

/**
 *
 */
Mat3 makeRotation(const Vec3 &axis, const float &angle)
{
    return makeMatRows(
        axis.x * axis.x * (1.0f - cos(angle)) + cos(angle), axis.x * axis.y * (1.0f - cos(angle)) - axis.z * sin(angle), axis.x * axis.z * (1.0f - cos(angle)) + axis.y * sin(angle),
        axis.y * axis.x * (1.0f - cos(angle)) + axis.z * sin(angle), axis.y * axis.y * (1.0f - cos(angle)) + cos(angle), axis.y * axis.z * (1.0f - cos(angle)) - axis.x * sin(angle),
        axis.z * axis.x * (1.0f - cos(angle)) - axis.y * sin(angle), axis.z * axis.y * (1.0f - cos(angle)) + axis.x * sin(angle), axis.z * axis.z * (1.0f - cos(angle)) + cos(angle));
}

/**
 *
 */
Mat4 makePerspective(const float &left, const float &right, const float &top, const float &bottom, const float &znear, const float &zfar)
{
    return makeMatRows(
        2.0f * znear / (right - left), 0.0f, (right + left) / (right - left), 0.0f,
        0.0f, 2.0f * znear / (top - bottom), (top + bottom) / (top - bottom), 0.0f,
        0.0f, 0.0f, -(zfar + znear) / (zfar - znear), -2.0f * zfar * znear / (zfar - znear),
        0.0f, 0.0f, -1.0f, 0.0f);
}

/**
 *
 */
Mat4 makeOrthographic(const float &left, const float &right, const float &top, const float &bottom, const float &znear, const float &zfar)
{
    return makeMatRows(
        2.0f / (right - left), 0, 0, -(right + left) / (right - left),
        0, 2.0f / (top - bottom), 0, -(top + bottom) / (top - bottom),
        0, 0, -2.0f / (zfar - znear), -(zfar + znear) / (zfar - znear),
        0, 0, 0, 1.0f);
}

/**
 *
 */
Mat4 makeLookAt(const Vec3 &center, const Vec3 &lookAt, const Vec3 &up)
{
    Vec3 f = normalize(sub(lookAt, center));
    Vec3 s = unitCross(f, up);
    Vec3 u = unitCross(s, f);
    Mat4 m = makeMatRows(
        s.x, s.y, s.z, -(s.x * center.x + s.y * center.y + s.z * center.z),
        u.x, u.y, u.z, -(u.x * center.x + u.y * center.y + u.z * center.z),
        -f.x, -f.y, -f.z, (f.x * center.x + f.y * center.y + f.z * center.z),
        0.0f, 0.0f, 0.0f, 1.0f);

    return m;
}