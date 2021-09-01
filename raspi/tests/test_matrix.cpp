#include <catch.h>
#include <catch_utils.h>

#include <matrix.h>

TEST_CASE("construct")
{
    SECTION("Invalid dimension")
    {
        REQUIRE_THROWS_WITH(Matrix({{1}, {1, 0}}), "Matrix dimension error (not MxN)");
    }
}

TEST_CASE("equal")
{
    Matrix A = Matrix({{1, 3}, {1, 3}});

    SECTION("should be equal")
    {
        Matrix B = Matrix({{1, 3}, {1, 3}});
        REQUIRE(A.equal(B));
    }
    SECTION("same dimension but not content")
    {
        Matrix B = Matrix({{3, 1}, {1, 3}});
        REQUIRE_FALSE(A.equal(B));
    }
    SECTION("different dimension")
    {
        Matrix B = Matrix({{3, 1, 3}, {1, 3, 1}});
        REQUIRE_FALSE(A.equal(B));
    }
}

TEST_CASE("inverse")
{
    LogLevel org_level = logger.get_level();
    catchutils::PatchStdCout patched_cout;
    logger.set_level(LogLevel::info);

    SECTION("square")
    {
        Matrix A = Matrix({{1, 2}, {1, 3}});
        Matrix A_exp_inv = Matrix({{3, -2}, {-1, 1}});

        A.inverse();
        REQUIRE(A.equal(A_exp_inv));
    }
    SECTION("not square")
    {
        Matrix A = Matrix({{1, 2, 0}, {2, 3, -1}});
        A.inverse();
        REQUIRE(patched_cout.get().find("Can't compute inverse: matrix is not square") != std::string::npos);
    }
    SECTION("singular")
    {
        Matrix A = Matrix({{1, 2}, {2, 4}});
        A.inverse();
        REQUIRE(patched_cout.get().find("Can't compute inverse: matrix is singular") != std::string::npos);
    }

    logger.set_level(org_level);
}
