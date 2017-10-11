#include "../../Header/catch.h"

#include "../../../Application/Header/Utilities/algebra.h"
#include "../../../Application/Header/Utilities/utils.h"

TEST_CASE("VECTOR")
{
    SECTION("2D")
    {
        Vec2 v2l = makeVec(5.0f, 2.0f);
        Vec2 v2r = makeVec(4.0f, 3.0f);

        SECTION("makeVec")
        {
            REQUIRE(equal(v2l, v2l));
            REQUIRE(!equal(v2l, v2r));
        }

        SECTION("add")
        {
            Vec2 vadd = add(v2l, v2r);
            REQUIRE(floatEqual(vadd.data[X], 9.0f));
            REQUIRE(floatEqual(vadd.data[Y], 5.0f));
        }

        SECTION("sub")
        {
            Vec2 vsub = sub(v2l, v2r);
            REQUIRE(floatEqual(vsub.data[X], 1.0f));
            REQUIRE(floatEqual(vsub.data[Y], -1.0f));
        }

        SECTION("neg")
        {
            Vec2 vneg = neg(v2l);
            REQUIRE(floatEqual(vneg.data[X], -5.0f));
            REQUIRE(floatEqual(vneg.data[Y], -2.0f));
        }

        SECTION("mulScal")
        {
            Vec2 vmulScal = mulScal(1.0f, v2l);
            REQUIRE(equal(v2l, vmulScal));
        }

        SECTION("divScal")
        {
            Vec2 vdivScal = divScal(1.0f, v2l);
            REQUIRE(equal(v2l, vdivScal));
        }

        SECTION("dot")
        {
        }

        SECTION("length")
        {
        }

        SECTION("normalize")
        {
        }

        SECTION("distance")
        {
        }

        SECTION("equal")
        {
            v2l = makeVec(0.111112f, 2.12312f);
            v2r = makeVec(0.111111f, 2.12312f);
            REQUIRE(equal(v2l, v2l));
            REQUIRE(!equal(v2l, v2r));
        }
    }

    SECTION("3D")
    {
        Vec3 v3l, v3r;

        SECTION("makeVec")
        {
            v3l = makeVec(5, 2, 2);

            REQUIRE(floatEqual(v3l.data[X], 5));
            REQUIRE(floatEqual(v3l.data[Y], 2));
            REQUIRE(floatEqual(v3l.data[Z], 2));
        }

        SECTION("add")
        {
        }

        SECTION("sub")
        {
        }

        SECTION("neg")
        {
        }

        SECTION("mulScal")
        {
        }

        SECTION("divScal")
        {
        }

        SECTION("dot")
        {
        }

        SECTION("length")
        {
        }

        SECTION("normalize")
        {
        }

        SECTION("distance")
        {
        }

        SECTION("equal")
        {
        }
    }

    SECTION("4D")
    {
        Vec4 v4l, v4r;

        SECTION("makeVec")
        {
            v4l = makeVec(5, 2, 2, 3);

            REQUIRE(floatEqual(v4l.data[X], 5));
            REQUIRE(floatEqual(v4l.data[Y], 2));
            REQUIRE(floatEqual(v4l.data[Z], 2));
            REQUIRE(floatEqual(v4l.data[W], 3));
        }

        SECTION("add")
        {
        }

        SECTION("sub")
        {
        }

        SECTION("neg")
        {
        }

        SECTION("mulScal")
        {
        }

        SECTION("divScal")
        {
        }

        SECTION("dot")
        {
        }

        SECTION("length")
        {
        }

        SECTION("normalize")
        {
        }

        SECTION("distance")
        {
        }

        SECTION("equal")
        {
        }
    }
}

TEST_CASE("MATRIX")
{
    SECTION("3D")
    {
        Mat3 m3l = makeMatRows(0.0f, 1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f, 8.0f);
        Mat3 m3r = makeMatCols(0.0f, 1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f, 8.0f);
        Vec3 v3a = makeVec(0.0f, 1.0f, 2.0f);
        Vec3 v3aa = makeVec(3.0f, 4.0f, 5.0f);
        Vec3 v3aaa = makeVec(6.0f, 7.0f, 8.0f);

        SECTION("equal")
        {
            REQUIRE(equal(m3l, m3l));
            REQUIRE(!equal(m3r, m3l));
        }

        SECTION("makeMatCols")
        {
            Mat3 m3col = makeMatCols(v3a, v3aa, v3aaa);
            REQUIRE(equal(m3l, transpose(m3col)));
        }

        SECTION("makeMatRows")
        {
            Mat3 m3row = makeMatRows(v3a, v3aa, v3aaa);
            REQUIRE(equal(m3r, transpose(m3row)));
        }

        SECTION("mulMatMat")
        {
            Mat3 m3res = makeMatRows(15.0f, 18.0f, 21.0f, 42.0f, 54.0f, 66.0f, 69.0f, 90.0f, 111.0f);
            REQUIRE(equal(mulMatMat(m3l, m3l), m3res));
        }

        SECTION("mulMatVec")
        {
        }

        SECTION("zeroM3")
        {
            m3l = makeMatRows(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
            REQUIRE(equal(m3l, zeroM3()));
        }

        SECTION("identityM3")
        {
            m3l = makeMatRows(1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f);
            REQUIRE(equal(m3l, identityM3()));
        }

        SECTION("inv")
        {
            m3l = makeMatRows(0.0f, 2.0f, 1.0f, -5.0f, 1.0f, 6.0f, -3.0f, 0.0f, 2.0f);
            Mat3 m3inv = makeMatRows(-2.0f / 13.0f, 4.0f / 13.0f, -11.0f / 13.0f, 8.0f / 13.0f, -3.0f / 13.0f, 5.0f / 13.0f, -3.0f / 13.0f, 6.0f / 13.0f, -10.0f / 13.0f);
            REQUIRE(equal(inv(m3l), m3inv));
        }

        SECTION("transpose")
        {
            REQUIRE(equal(identityM3(), transpose(identityM3())));
        }
    }

    SECTION("4D")
    {
        Mat4 m4l = makeMatRows(0.0f, 1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f, 8.0f, 9.0f, 10.0f, 11.0f, 12.0f, 13.0f, 14.0f, 15.0f);
        Mat4 m4r = makeMatCols(0.0f, 1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f, 8.0f, 9.0f, 10.0f, 11.0f, 12.0f, 13.0f, 14.0f, 15.0f);
        Vec4 v4a = makeVec(0.0f, 1.0f, 2.0f, 3.0f);
        Vec4 v4aa = makeVec(4.0f, 5.0f, 6.0f, 7.0f);
        Vec4 v4aaa = makeVec(8.0f, 9.0f, 10.0f, 11.0f);
        Vec4 v4aaaa = makeVec(12.0f, 13.0f, 14.0f, 15.0f);

        SECTION("equal")
        {
            REQUIRE(equal(m4l, m4l));
            REQUIRE(!equal(m4r, m4l));
        }

        SECTION("makeMatCols")
        {
            Mat4 m4col = makeMatCols(v4a, v4aa, v4aaa, v4aaaa);
            REQUIRE(equal(m4l, transpose(m4col)));
        }

        SECTION("makeMatRows")
        {
            Mat4 m4row = makeMatRows(v4a, v4aa, v4aaa, v4aaaa);
            REQUIRE(equal(m4r, transpose(m4row)));
        }

        SECTION("mulMatMat")
        {
            Mat4 m4res = makeMatRows(56.0f, 62.0f, 68.0f, 74.0f, 152.0f, 174.0f, 196.0f, 218.0f, 248.0f, 286.0f, 324.0f, 362.0f, 344.0f, 398.0f, 452.0f, 506.0f);
            REQUIRE(equal(mulMatMat(m4l, m4l), m4res));
        }

        SECTION("mulMatVec")
        {
        }

        SECTION("zeroM4")
        {
            m4l = makeMatRows(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);
            REQUIRE(equal(m4l, zeroM4()));
        }

        SECTION("identityM4")
        {
            m4l = makeMatRows(1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 0.0f, 1.0f);
            REQUIRE(equal(m4l, identityM4()));
        }

        SECTION("inv")
        {
            m4l = makeMatRows(0.0f, 2.0f, -1.0f, 5.0f, -5.0f, 4.0f, 4.0f, 0.0f, 2.0f, 0.0f, 0.0f, 0.0f, 2.0f, -1.0f, -1.0f, -2.0f);
            Mat4 m4inv = makeMatRows(0.0f, 0.0f, 0.5f, 0.0f, 1.0f / 3.0f, 7.0f / 24.0f, -5.0f / 48.0f, 5.0f / 6.0f, -1.0f / 3.0f, -1.0f / 24.0f, 35.0f / 48.0f, -5.0f / 6.0f, 0.0f, -1.0f / 8.0f, 3.0f / 16.0f, -0.5f);
            REQUIRE(equal(inv(m4l), m4inv));
        }

        SECTION("transpose")
        {
            REQUIRE(equal(identityM4(), transpose(identityM4())));
        }
    }
}

TEST_CASE("TRANSFORMATION")
{
}