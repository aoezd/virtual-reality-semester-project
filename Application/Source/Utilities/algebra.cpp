/**
 * algebra.cpp
 * 
 * Own libary of mathematical functions for matrix computations.
 * 
 * Created: 2017-09-06
 * Author: Aykut Özdemir
 */

#include <iostream>
#include <opencv2/core.hpp>

#include "../../Header/Utilities/algebra.h"
#include "../../Header/Utilities/utils.h"

// --------------- 2D functions ---------------

Vec2 makeVec(const float &x, const float &y)
{
    Vec2 v;

    v.data[X] = x;
    v.data[Y] = y;

    return v;
}

void printVec(const Vec2 &v)
{
    std::cout << "[" << v.data[X] << ", " << v.data[Y] << "]" << std::endl;
}

Vec2 add(const Vec2 &vl, const Vec2 &vr)
{
    return makeVec(vl.data[X] + vr.data[X], vl.data[Y] + vr.data[Y]);
}

Vec2 sub(const Vec2 &vl, const Vec2 &vr)
{
    return makeVec(vl.data[X] - vr.data[X], vl.data[Y] - vr.data[Y]);
}

Vec2 neg(const Vec2 &v)
{
    return makeVec(-v.data[X], -v.data[Y]);
}

Vec2 mulScal(const float &x, const Vec2 &v)
{
    return makeVec(v.data[X] * x, v.data[Y] * x);
}

Vec2 divScal(const float &x, const Vec2 &v)
{
    return makeVec(v.data[X] / x, v.data[Y] / x);
}

float dot(const Vec2 &vl, const Vec2 &vr)
{
    return vl.data[X] * vr.data[X] + vl.data[Y] * vr.data[Y];
}

float length(const Vec2 &v)
{
    return fastInvSqrt(dot(v, v));
}

Vec2 normalize(const Vec2 &v)
{
    return divScal(length(v), v);
}

float distance(const Vec2 &vl, const Vec2 &vr)
{
    return length(sub(vl, vr));
}

bool equal(const Vec2 &vl, const Vec2 &vr)
{
    return floatEqual(vl.data[X], vr.data[X]) && floatEqual(vl.data[Y], vr.data[Y]);
}

// --------------- 3D functions ---------------

Vec3 makeVec(const float &x, const float &y, const float &z)
{
    Vec3 v;

    v.data[X] = x;
    v.data[Y] = y;
    v.data[Z] = z;

    return v;
}

void printVec(const Vec3 &v)
{
    std::cout << "[" << v.data[X] << ", " << v.data[Y] << ", " << v.data[Z] << "]" << std::endl;
}

Vec3 add(const Vec3 &vl, const Vec3 &vr)
{
    return makeVec(vl.data[X] + vr.data[X], vl.data[Y] + vr.data[Y], vl.data[Z] + vr.data[Z]);
}

Vec3 sub(const Vec3 &vl, const Vec3 &vr)
{
    return makeVec(vl.data[X] - vr.data[X], vl.data[Y] - vr.data[Y], vl.data[Z] - vr.data[Z]);
}

Vec3 neg(const Vec3 &v)
{
    return makeVec(-v.data[X], -v.data[Y], -v.data[Z]);
}

Vec3 mulScal(const float &x, const Vec3 &v)
{
    return makeVec(x * v.data[X], x * v.data[Y], x * v.data[Z]);
}

Vec3 divScal(const float &x, const Vec3 &v)
{
    return makeVec(v.data[X] / x, v.data[Y] / x, v.data[Z] / x);
}

float dot(const Vec3 &vl, const Vec3 &vr)
{
    return vl.data[X] * vr.data[X] + vl.data[Y] * vr.data[Y] + vl.data[Z] * vr.data[Z];
}

float length(const Vec3 &v)
{
    return fastInvSqrt(dot(v, v));
}

Vec3 normalize(const Vec3 &v)
{
    return divScal(length(v), v);
}

Vec3 cross(const Vec3 &vl, const Vec3 &vr)
{
    return makeVec(vl.data[Y] * vr.data[Z] - vl.data[Z] * vr.data[Y], vl.data[Z] * vr.data[X] - vl.data[X] * vr.data[Z], vl.data[X] * vr.data[Y] - vl.data[Y] * vr.data[X]);
}

Vec3 unitCross(const Vec3 &vl, const Vec3 &vr)
{
    return normalize(cross(vl, vr));
}

float distance(const Vec3 &vl, const Vec3 &vr)
{
    return length(sub(vl, vr));
}

bool equal(const Vec3 &vl, const Vec3 &vr)
{
    return floatEqual(vl.data[X], vr.data[X]) && floatEqual(vl.data[Y], vr.data[Y]) && floatEqual(vr.data[Z], vl.data[Z]);
}

// --------------- 4D functions ---------------

Vec4 makeVec(const float &x, const float &y, const float &z, const float &w)
{
    Vec4 v;

    v.data[X] = x;
    v.data[Y] = y;
    v.data[Z] = z;
    v.data[W] = w;

    return v;
}

void printVec(const Vec4 &v)
{
    std::cout << "[" << v.data[X] << ", " << v.data[Y] << ", " << v.data[Z] << ", " << v.data[W] << "]" << std::endl;
}

Vec4 add(const Vec4 &vl, const Vec4 &vr)
{
    return makeVec(vl.data[X] + vr.data[X], vl.data[Y] + vr.data[Y], vl.data[Z] + vr.data[Z], vl.data[W] + vr.data[W]);
}

Vec4 sub(const Vec4 &vl, const Vec4 &vr)
{
    return makeVec(vl.data[X] - vr.data[X], vl.data[Y] - vr.data[Y], vl.data[Z] - vr.data[Z], vl.data[W] - vr.data[W]);
}

Vec4 neg(const Vec4 &v)
{
    return makeVec(-v.data[X], -v.data[Y], -v.data[Z], -v.data[W]);
}

Vec4 mulScal(const float &x, const Vec4 &v)
{
    return makeVec(v.data[X] * x, v.data[Y] * x, v.data[Z] * x, v.data[W] * x);
}

Vec4 divScal(const float &x, const Vec4 &v)
{
    return makeVec(v.data[X] / x, v.data[Y] / x, v.data[Z] / x, v.data[W] / x);
}

float dot(const Vec4 &vl, const Vec4 &vr)
{
    return vl.data[X] * vr.data[X] + vl.data[Y] * vr.data[Y] + vl.data[Z] * vr.data[Z] + vl.data[W] * vr.data[W];
}

float length(const Vec4 &v)
{
    return fastInvSqrt(dot(v, v));
}

Vec4 normalize(const Vec4 &v)
{
    return divScal(length(v), v);
}

float distance(const Vec4 &vl, const Vec4 &vr)
{
    return length(sub(vl, vr));
}

bool equal(const Vec4 &vl, const Vec4 &vr)
{
    return floatEqual(vl.data[X], vr.data[X]) && floatEqual(vl.data[Y], vr.data[Y]) && floatEqual(vl.data[Z], vr.data[Z]) && floatEqual(vl.data[W], vr.data[W]);
}

Vec3 wDiv(const Vec4 &v)
{
    return makeVec(v.data[X] / v.data[W], v.data[Y] / v.data[W], v.data[Z] / v.data[W]);
}

// --------------- Matrix3 functions ---------------

bool equal(const Mat3 &ml, const Mat3 &mr)
{
    bool b = true;

    for (int i = 0; i < 9 && b; i++)
    {
        b = floatEqual(ml.data[i], mr.data[i]);
    }

    return b;
}

Mat3 makeMatCols(const Vec3 &col0, const Vec3 &col1, const Vec3 &col2)
{
    Mat3 res;

    res.data[0] = col0.data[X];
    res.data[3] = col0.data[Y];
    res.data[6] = col0.data[Z];
    res.data[1] = col1.data[X];
    res.data[4] = col1.data[Y];
    res.data[7] = col1.data[Z];
    res.data[2] = col2.data[X];
    res.data[5] = col2.data[Y];
    res.data[8] = col2.data[Z];

    return res;
}

Mat3 makeMatRows(const Vec3 &row0, const Vec3 &row1, const Vec3 &row2)
{
    Mat3 res;

    res.data[0] = row0.data[X];
    res.data[1] = row0.data[Y];
    res.data[2] = row0.data[Z];
    res.data[3] = row1.data[X];
    res.data[4] = row1.data[Y];
    res.data[5] = row1.data[Z];
    res.data[6] = row2.data[X];
    res.data[7] = row2.data[Y];
    res.data[8] = row2.data[Z];

    return res;
}

Mat3 makeMatCols(const float &m00, const float &m01, const float &m02, const float &m10, const float &m11, const float &m12, const float &m20, const float &m21, const float &m22)
{
    Mat3 res;

    res.data[0] = m00;
    res.data[3] = m01;
    res.data[6] = m02;
    res.data[1] = m10;
    res.data[4] = m11;
    res.data[7] = m12;
    res.data[2] = m20;
    res.data[5] = m21;
    res.data[8] = m22;

    return res;
}

Mat3 makeMatRows(const float &m00, const float &m01, const float &m02, const float &m10, const float &m11, const float &m12, const float &m20, const float &m21, const float &m22)
{
    Mat3 res;

    res.data[0] = m00;
    res.data[1] = m01;
    res.data[2] = m02;
    res.data[3] = m10;
    res.data[4] = m11;
    res.data[5] = m12;
    res.data[6] = m20;
    res.data[7] = m21;
    res.data[8] = m22;

    return res;
}

Mat3 mulMatMat(const Mat3 &ml, const Mat3 &mr)
{
    Mat3 res;

    res.data[0] = ml.data[0] * mr.data[0] + ml.data[1] * mr.data[3] + ml.data[2] * mr.data[6];
    res.data[3] = ml.data[3] * mr.data[0] + ml.data[4] * mr.data[3] + ml.data[5] * mr.data[6];
    res.data[6] = ml.data[6] * mr.data[0] + ml.data[7] * mr.data[3] + ml.data[8] * mr.data[6];
    res.data[1] = ml.data[0] * mr.data[1] + ml.data[1] * mr.data[4] + ml.data[2] * mr.data[7];
    res.data[4] = ml.data[3] * mr.data[1] + ml.data[4] * mr.data[4] + ml.data[5] * mr.data[7];
    res.data[7] = ml.data[6] * mr.data[1] + ml.data[7] * mr.data[4] + ml.data[8] * mr.data[7];
    res.data[2] = ml.data[0] * mr.data[2] + ml.data[1] * mr.data[5] + ml.data[2] * mr.data[8];
    res.data[5] = ml.data[3] * mr.data[2] + ml.data[4] * mr.data[5] + ml.data[5] * mr.data[8];
    res.data[8] = ml.data[6] * mr.data[2] + ml.data[7] * mr.data[5] + ml.data[8] * mr.data[8];

    return res;
}

Vec3 mulMatVec(const Mat3 &m, const Vec3 &v)
{
    return makeVec(
        m.data[0] * v.data[0] + m.data[1] * v.data[1] + m.data[2] * v.data[2],
        m.data[3] * v.data[0] + m.data[4] * v.data[1] + m.data[5] * v.data[2],
        m.data[6] * v.data[0] + m.data[7] * v.data[1] + m.data[8] * v.data[2]);
}

Mat3 zeroM3(void)
{
    return makeMatRows(
        0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f);
}

Mat3 identityM3(void)
{
    return makeMatRows(
        1.0f, 0.0f, 0.0f,
        0.0f, 1.0f, 0.0f,
        0.0f, 0.0f, 1.0f);
}

Mat3 inv(const Mat3 &m)
{
    float oneOverDeterminant = 1.0f / (+m.data[0] * (m.data[4] * m.data[8] - m.data[5] * m.data[7]) - m.data[1] * (m.data[3] * m.data[8] - m.data[5] * m.data[6]) + m.data[2] * (m.data[3] * m.data[7] - m.data[4] * m.data[6]));
    Mat3 inv;

    inv.data[0] = +(m.data[4] * m.data[8] - m.data[5] * m.data[7]) * oneOverDeterminant;
    inv.data[1] = -(m.data[1] * m.data[8] - m.data[2] * m.data[7]) * oneOverDeterminant;
    inv.data[2] = +(m.data[1] * m.data[5] - m.data[2] * m.data[4]) * oneOverDeterminant;
    inv.data[3] = -(m.data[3] * m.data[8] - m.data[5] * m.data[6]) * oneOverDeterminant;
    inv.data[4] = +(m.data[0] * m.data[8] - m.data[2] * m.data[6]) * oneOverDeterminant;
    inv.data[5] = -(m.data[0] * m.data[5] - m.data[2] * m.data[3]) * oneOverDeterminant;
    inv.data[6] = +(m.data[3] * m.data[7] - m.data[4] * m.data[6]) * oneOverDeterminant;
    inv.data[7] = -(m.data[0] * m.data[7] - m.data[1] * m.data[6]) * oneOverDeterminant;
    inv.data[8] = +(m.data[0] * m.data[4] - m.data[1] * m.data[3]) * oneOverDeterminant;

    return inv;
}

Mat3 transpose(const Mat3 &m)
{
    Mat3 res;

    res.data[0] = m.data[0];
    res.data[3] = m.data[1];
    res.data[6] = m.data[2];
    res.data[1] = m.data[3];
    res.data[4] = m.data[4];
    res.data[7] = m.data[5];
    res.data[2] = m.data[6];
    res.data[5] = m.data[7];
    res.data[8] = m.data[8];

    return res;
}

void printMat(const Mat3 &m)
{
    std::cout << "{\t" << m.data[0] << ", " << m.data[1] << ", " << m.data[2] << std::endl;
    std::cout << "\t" << m.data[3] << ", " << m.data[4] << ", " << m.data[5] << std::endl;
    std::cout << "\t" << m.data[6] << ", " << m.data[7] << ", " << m.data[8] << "\t}" << std::endl;
}

// --------------- Matrix4 functions ---------------

bool equal(const Mat4 &ml, const Mat4 &mr)
{
    bool b = true;

    for (int i = 0; i < 16 && b; i++)
    {
        b = floatEqual(ml.data[i], mr.data[i]);
    }

    return b;
}

Mat4 makeMatCols(const Vec4 &col0, const Vec4 &col1, const Vec4 &col2, const Vec4 &col3)
{
    Mat4 res;

    res.data[0] = col0.data[0];
    res.data[4] = col0.data[1];
    res.data[8] = col0.data[2];
    res.data[12] = col0.data[3];
    res.data[1] = col1.data[0];
    res.data[5] = col1.data[1];
    res.data[9] = col1.data[2];
    res.data[13] = col1.data[3];
    res.data[2] = col2.data[0];
    res.data[6] = col2.data[1];
    res.data[10] = col2.data[2];
    res.data[14] = col2.data[3];
    res.data[3] = col3.data[0];
    res.data[7] = col3.data[1];
    res.data[11] = col3.data[2];
    res.data[15] = col3.data[3];

    return res;
}

Mat4 makeMatRows(const Vec4 &row0, const Vec4 &row1, const Vec4 &row2, const Vec4 &row3)
{
    Mat4 res;

    res.data[0] = row0.data[0];
    res.data[1] = row0.data[1];
    res.data[2] = row0.data[2];
    res.data[3] = row0.data[3];
    res.data[4] = row1.data[0];
    res.data[5] = row1.data[1];
    res.data[6] = row1.data[2];
    res.data[7] = row1.data[3];
    res.data[8] = row2.data[0];
    res.data[9] = row2.data[1];
    res.data[10] = row2.data[2];
    res.data[11] = row2.data[3];
    res.data[12] = row3.data[0];
    res.data[13] = row3.data[1];
    res.data[14] = row3.data[2];
    res.data[15] = row3.data[3];

    return res;
}

Mat4 makeMatCols(const float &m00, const float &m01, const float &m02, const float &m03, const float &m10, const float &m11, const float &m12, const float &m13, const float &m20, const float &m21, const float &m22, const float &m23, const float &m30, const float &m31, const float &m32, const float &m33)
{
    Mat4 res;

    res.data[0] = m00;
    res.data[4] = m01;
    res.data[8] = m02;
    res.data[12] = m03;
    res.data[1] = m10;
    res.data[5] = m11;
    res.data[9] = m12;
    res.data[13] = m13;
    res.data[2] = m20;
    res.data[6] = m21;
    res.data[10] = m22;
    res.data[14] = m23;
    res.data[3] = m30;
    res.data[7] = m31;
    res.data[11] = m32;
    res.data[15] = m33;

    return res;
}

Mat4 makeMatRows(const float &m00, const float &m01, const float &m02, const float &m03, const float &m10, const float &m11, const float &m12, const float &m13, const float &m20, const float &m21, const float &m22, const float &m23, const float &m30, const float &m31, const float &m32, const float &m33)
{
    Mat4 res;

    res.data[0] = m00;
    res.data[1] = m01;
    res.data[2] = m02;
    res.data[3] = m03;
    res.data[4] = m10;
    res.data[5] = m11;
    res.data[6] = m12;
    res.data[7] = m13;
    res.data[8] = m20;
    res.data[9] = m21;
    res.data[10] = m22;
    res.data[11] = m23;
    res.data[12] = m30;
    res.data[13] = m31;
    res.data[14] = m32;
    res.data[15] = m33;

    return res;
}

Mat4 mulMatMat(const Mat4 &ml, const Mat4 &mr)
{
    Mat4 res;

    res.data[0] = ml.data[0] * mr.data[0] + ml.data[1] * mr.data[4] + ml.data[2] * mr.data[8] + ml.data[3] * mr.data[12];
    res.data[1] = ml.data[0] * mr.data[1] + ml.data[1] * mr.data[5] + ml.data[2] * mr.data[9] + ml.data[3] * mr.data[13];
    res.data[2] = ml.data[0] * mr.data[2] + ml.data[1] * mr.data[6] + ml.data[2] * mr.data[10] + ml.data[3] * mr.data[14];
    res.data[3] = ml.data[0] * mr.data[3] + ml.data[1] * mr.data[7] + ml.data[2] * mr.data[11] + ml.data[3] * mr.data[15];
    res.data[4] = ml.data[4] * mr.data[0] + ml.data[5] * mr.data[4] + ml.data[6] * mr.data[8] + ml.data[7] * mr.data[12];
    res.data[5] = ml.data[4] * mr.data[1] + ml.data[5] * mr.data[5] + ml.data[6] * mr.data[9] + ml.data[7] * mr.data[13];
    res.data[6] = ml.data[4] * mr.data[2] + ml.data[5] * mr.data[6] + ml.data[6] * mr.data[10] + ml.data[7] * mr.data[14];
    res.data[7] = ml.data[4] * mr.data[3] + ml.data[5] * mr.data[7] + ml.data[6] * mr.data[11] + ml.data[7] * mr.data[15];
    res.data[8] = ml.data[8] * mr.data[0] + ml.data[9] * mr.data[4] + ml.data[10] * mr.data[8] + ml.data[11] * mr.data[12];
    res.data[9] = ml.data[8] * mr.data[1] + ml.data[9] * mr.data[5] + ml.data[10] * mr.data[9] + ml.data[11] * mr.data[13];
    res.data[10] = ml.data[8] * mr.data[2] + ml.data[9] * mr.data[6] + ml.data[10] * mr.data[10] + ml.data[11] * mr.data[14];
    res.data[11] = ml.data[8] * mr.data[3] + ml.data[9] * mr.data[7] + ml.data[10] * mr.data[11] + ml.data[11] * mr.data[15];
    res.data[12] = ml.data[12] * mr.data[0] + ml.data[13] * mr.data[4] + ml.data[14] * mr.data[8] + ml.data[15] * mr.data[12];
    res.data[13] = ml.data[12] * mr.data[1] + ml.data[13] * mr.data[5] + ml.data[14] * mr.data[9] + ml.data[15] * mr.data[13];
    res.data[14] = ml.data[12] * mr.data[2] + ml.data[13] * mr.data[6] + ml.data[14] * mr.data[10] + ml.data[15] * mr.data[14];
    res.data[15] = ml.data[12] * mr.data[3] + ml.data[13] * mr.data[7] + ml.data[14] * mr.data[11] + ml.data[15] * mr.data[15];

    return res;
}

Vec4 mulMatVec(const Mat4 &m, const Vec4 &v)
{
    return makeVec(
        m.data[0] * v.data[0] + m.data[1] * v.data[1] + m.data[2] * v.data[2] + v.data[3] * m.data[3],
        m.data[4] * v.data[0] + m.data[5] * v.data[1] + m.data[6] * v.data[2] + v.data[3] * m.data[7],
        m.data[8] * v.data[0] + m.data[9] * v.data[1] + m.data[10] * v.data[2] + v.data[3] * m.data[11],
        m.data[12] * v.data[0] + m.data[13] * v.data[1] + m.data[14] * v.data[2] + v.data[3] * m.data[15]);
}

Mat4 zeroM4(void)
{
    return makeMatRows(
        0.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 0.0f);
}

Mat4 identityM4(void)
{
    return makeMatRows(
        1.0f, 0.0f, 0.0f, 0.0f,
        0.0f, 1.0f, 0.0f, 0.0f,
        0.0f, 0.0f, 1.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 1.0f);
}

Mat4 inv(const Mat4 &m)
{
    float m11 = m.data[0];
    float m12 = m.data[1];
    float m13 = m.data[2];
    float m14 = m.data[3];

    float m21 = m.data[4];
    float m22 = m.data[5];
    float m23 = m.data[6];
    float m24 = m.data[7];

    float m31 = m.data[8];
    float m32 = m.data[9];
    float m33 = m.data[10];
    float m34 = m.data[11];

    float m41 = m.data[12];
    float m42 = m.data[13];
    float m43 = m.data[14];
    float m44 = m.data[15];

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

    inv.data[0] = oneOverDeterminant * (m22 * m33 * m44 + m23 * m34 * m42 + m24 * m32 * m43 - m22 * m34 * m43 - m23 * m32 * m44 - m24 * m33 * m42);
    inv.data[1] = oneOverDeterminant * (m12 * m34 * m43 + m13 * m32 * m44 + m14 * m33 * m42 - m12 * m33 * m44 - m13 * m34 * m42 - m14 * m32 * m43);
    inv.data[2] = oneOverDeterminant * (m12 * m23 * m44 + m13 * m24 * m42 + m14 * m22 * m43 - m12 * m24 * m43 - m13 * m22 * m44 - m14 * m23 * m42);
    inv.data[3] = oneOverDeterminant * (m12 * m24 * m33 + m13 * m22 * m34 + m14 * m23 * m32 - m12 * m23 * m34 - m13 * m24 * m32 - m14 * m22 * m33);
    inv.data[4] = oneOverDeterminant * (m21 * m34 * m43 + m23 * m31 * m44 + m24 * m33 * m41 - m21 * m33 * m44 - m23 * m34 * m41 - m24 * m31 * m43);
    inv.data[5] = oneOverDeterminant * (m11 * m33 * m44 + m13 * m34 * m41 + m14 * m31 * m43 - m11 * m34 * m43 - m13 * m31 * m44 - m14 * m33 * m41);
    inv.data[6] = oneOverDeterminant * (m11 * m24 * m43 + m13 * m21 * m44 + m14 * m23 * m41 - m11 * m23 * m44 - m13 * m24 * m41 - m14 * m21 * m43);
    inv.data[7] = oneOverDeterminant * (m11 * m23 * m34 + m13 * m24 * m31 + m14 * m21 * m33 - m11 * m24 * m33 - m13 * m21 * m34 - m14 * m23 * m31);
    inv.data[8] = oneOverDeterminant * (m21 * m32 * m44 + m22 * m34 * m41 + m24 * m31 * m42 - m21 * m34 * m42 - m22 * m31 * m44 - m24 * m32 * m41);
    inv.data[9] = oneOverDeterminant * (m11 * m34 * m42 + m12 * m31 * m44 + m14 * m32 * m41 - m11 * m32 * m44 - m12 * m34 * m41 - m14 * m31 * m42);
    inv.data[10] = oneOverDeterminant * (m11 * m22 * m44 + m12 * m24 * m41 + m14 * m21 * m42 - m11 * m24 * m42 - m12 * m21 * m44 - m14 * m22 * m41);
    inv.data[11] = oneOverDeterminant * (m11 * m24 * m32 + m12 * m21 * m34 + m14 * m22 * m31 - m11 * m22 * m34 - m12 * m24 * m31 - m14 * m21 * m32);
    inv.data[12] = oneOverDeterminant * (m21 * m33 * m42 + m22 * m31 * m43 + m23 * m32 * m41 - m21 * m32 * m43 - m22 * m33 * m41 - m23 * m31 * m42);
    inv.data[13] = oneOverDeterminant * (m11 * m32 * m43 + m12 * m33 * m41 + m13 * m31 * m42 - m11 * m33 * m42 - m12 * m31 * m43 - m13 * m32 * m41);
    inv.data[14] = oneOverDeterminant * (m11 * m23 * m42 + m12 * m21 * m43 + m13 * m22 * m41 - m11 * m22 * m43 - m12 * m23 * m41 - m13 * m21 * m42);
    inv.data[15] = oneOverDeterminant * (m11 * m22 * m33 + m12 * m23 * m31 + m13 * m21 * m32 - m11 * m23 * m32 - m12 * m21 * m33 - m13 * m22 * m31);

    return inv;
}

Mat4 transpose(const Mat4 &m)
{
    Mat4 res;

    res.data[0] = m.data[0];
    res.data[4] = m.data[1];
    res.data[8] = m.data[2];
    res.data[12] = m.data[3];
    res.data[1] = m.data[4];
    res.data[5] = m.data[5];
    res.data[9] = m.data[6];
    res.data[13] = m.data[7];
    res.data[2] = m.data[8];
    res.data[6] = m.data[9];
    res.data[10] = m.data[10];
    res.data[14] = m.data[11];
    res.data[3] = m.data[12];
    res.data[7] = m.data[13];
    res.data[11] = m.data[14];
    res.data[15] = m.data[15];

    return res;
}

void printMat(const Mat4 &m)
{
    std::cout << "{\t" << m.data[0] << ", " << m.data[1] << ", " << m.data[2] << ", " << m.data[3] << std::endl;
    std::cout << "\t" << m.data[4] << ", " << m.data[5] << ", " << m.data[6] << ", " << m.data[7] << std::endl;
    std::cout << "\t" << m.data[8] << ", " << m.data[9] << ", " << m.data[10] << ", " << m.data[11] << std::endl;
    std::cout << "\t" << m.data[12] << ", " << m.data[13] << ", " << m.data[14] << ", " << m.data[15] << "\t}" << std::endl;
}

// --------------- Transformations ---------------

Mat3 makeNormalMatrix(const Mat4 &worldMatrix)
{
    Mat4 m = transpose(inv(worldMatrix));
    Vec3 col0 = makeVec(m.data[0], m.data[4], m.data[8]);
    Vec3 col1 = makeVec(m.data[1], m.data[5], m.data[9]);
    Vec3 col2 = makeVec(m.data[2], m.data[6], m.data[10]);

    return makeMatCols(col0, col1, col2);
}

Mat4 makeTranslationMatrix(const float &x, const float &y, const float &z)
{
    return makeMatRows(
        1.0f, 0.0f, 0.0f, x,
        0.0f, 1.0f, 0.0f, y,
        0.0f, 0.0f, 1.0f, z,
        0.0f, 0.0f, 0.0f, 1.0f);
}

Mat4 makeTranslationMatrix(const Vec3 &v)
{
    return makeTranslationMatrix(v.data[X], v.data[Y], v.data[Z]);
}

Mat4 makeTranslationMatrix(const float &x)
{
    return makeTranslationMatrix(x, x, x);
}

Mat4 makeScaleMatrix(const float &x, const float &y, const float &z)
{
    return makeMatRows(
        x, 0.0f, 0.0f, 0.0f,
        0.0f, y, 0.0f, 0.0f,
        0.0f, 0.0f, z, 0.0f,
        0.0f, 0.0f, 0.0f, 1.0f);
}

Mat4 makeScaleMatrix(const Vec3 &v)
{
    return makeScaleMatrix(v.data[X], v.data[Y], v.data[Z]);
}

Mat4 makeScaleMatrix(const float &x)
{
    return makeScaleMatrix(x, x, x);
}

Mat4 makeRotationX(const float &angle)
{
    return makeMatRows(
        1.0f, 0.0f, 0.0f, 0.0f,
        0.0f, cos(angle), -sin(angle), 0.0f,
        0.0f, sin(angle), cos(angle), 0.0f,
        0.0f, 0.0f, 0.0f, 1.0f);
}

Mat4 makeRotationY(const float &angle)
{
    return makeMatRows(
        cos(angle), 0.0f, sin(angle), 0.0f,
        0.0f, 1.0f, 0.0f, 0.0f,
        -sin(angle), 0.0f, cos(angle), 0.0f,
        0.0f, 0.0f, 0.0f, 1.0f);
}

Mat4 makeRotationZ(const float &angle)
{
    return makeMatRows(
        cos(angle), -sin(angle), 0.0f, 0.0f,
        sin(angle), cos(angle), 0.0f, 0.0f,
        0.0f, 0.0f, 1.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 1.0f);
}

Mat3 makeRotation(const Vec3 &axis, const float &angle)
{
    return makeMatRows(
        axis.data[X] * axis.data[X] * (1.0f - cos(angle)) + cos(angle), axis.data[X] * axis.data[Y] * (1.0f - cos(angle)) - axis.data[Z] * sin(angle), axis.data[X] * axis.data[Z] * (1.0f - cos(angle)) + axis.data[Y] * sin(angle),
        axis.data[Y] * axis.data[X] * (1.0f - cos(angle)) + axis.data[Z] * sin(angle), axis.data[Y] * axis.data[Y] * (1.0f - cos(angle)) + cos(angle), axis.data[Y] * axis.data[Z] * (1.0f - cos(angle)) - axis.data[X] * sin(angle),
        axis.data[Z] * axis.data[X] * (1.0f - cos(angle)) - axis.data[Y] * sin(angle), axis.data[Z] * axis.data[Y] * (1.0f - cos(angle)) + axis.data[X] * sin(angle), axis.data[Z] * axis.data[Z] * (1.0f - cos(angle)) + cos(angle));
}

Mat4 makePerspective(const float &left, const float &right, const float &top, const float &bottom, const float &znear, const float &zfar)
{
    return makeMatRows(
        2.0f * znear / (right - left), 0.0f, (right + left) / (right - left), 0.0f,
        0.0f, 2.0f * znear / (top - bottom), (top + bottom) / (top - bottom), 0.0f,
        0.0f, 0.0f, -(zfar + znear) / (zfar - znear), -2.0f * zfar * znear / (zfar - znear),
        0.0f, 0.0f, -1.0f, 0.0f);
}

Mat4 makeOrthographic(const float &left, const float &right, const float &top, const float &bottom, const float &znear, const float &zfar)
{
    return makeMatRows(
        2.0f / (right - left), 0, 0, -(right + left) / (right - left),
        0, 2.0f / (top - bottom), 0, -(top + bottom) / (top - bottom),
        0, 0, -2.0f / (zfar - znear), -(zfar + znear) / (zfar - znear),
        0, 0, 0, 1.0f);
}

Mat4 makeLookAt(const Vec3 &center, const Vec3 &lookAt, const Vec3 &up)
{
    Vec3 f = normalize(sub(lookAt, center));
    Vec3 s = unitCross(f, up);
    Vec3 u = unitCross(s, f);
    Mat4 m = makeMatRows(
        s.data[X], s.data[Y], s.data[Z], -(s.data[X] * center.data[X] + s.data[Y] * center.data[Y] + s.data[Z] * center.data[Z]),
        u.data[X], u.data[Y], u.data[Z], -(u.data[X] * center.data[X] + u.data[Y] * center.data[Y] + u.data[Z] * center.data[Z]),
        -f.data[X], -f.data[Y], -f.data[Z], (f.data[X] * center.data[X] + f.data[Y] * center.data[Y] + f.data[Z] * center.data[Z]),
        0.0f, 0.0f, 0.0f, 1.0f);

    return m;
}

// --------------- OpenCV Matrices ---------------

Mat4 makeOrthographic(int w, int h)
{
    return makeMatRows(0.0f, -2.0f / static_cast<float>(w), 0.0f, 0.0f,
                       -2.0f / static_cast<float>(h), 0.0f, 0.0f, 0.0f,
                       0.0f, 0.0f, 1.0f, 0.0f,
                       1.0f, 1.0f, 0.0f, 1.0f);
}

Mat4 makePerspective(const float &fx, const float &fy, const float &cx, const float &cy, const int &w, const int &h, const float &near, const float &far)
{
    return makeMatRows(-2.0f * fx / w, 0.0f, 2.0f * cx / w - 1.0f, 0.0f,
                       0.0f, 2.0f * fy / h, 2.0f * cy / h - 1.0f, 0.0f,
                       0.0f, 0.0f, -(far + near) / (far - near), -2.0f * far * near / (far - near),
                       0.0f, 0.0f, -1.0f, 1.0f);
}