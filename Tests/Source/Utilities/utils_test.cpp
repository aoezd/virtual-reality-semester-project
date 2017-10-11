#include "../../Header/catch.h"

#include "../../../Application/Header/Utilities/utils.h"

TEST_CASE("UTILS")
{
    SECTION("Algebra Stuff")
    {
        SECTION("floatEqual")
        {
            REQUIRE(floatEqual(5.1212121f, 5.1212121f));
            REQUIRE(!floatEqual(5.1212f, 5.12121f));
        }

        SECTION("doubleEqual")
        {
            REQUIRE(doubleEqual(5.12121211231231, 5.12121211231231));
            REQUIRE(!doubleEqual(5.1212121131, 5.12121231233));
        }

        SECTION("sqrt")
        {
            REQUIRE(floatEqual(5.0f, sqrt(25.0f)));
        }
    }
}