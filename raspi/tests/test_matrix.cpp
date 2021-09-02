#include <catch.h>
#include <catch_utils.h>

#include <matrix.h>

TEST_CASE("constructor")
{
    SECTION("Invalid dimension")
    {
        REQUIRE_THROWS_WITH(Matrix({{1}, {1, 0}}), "Matrix dimension error (not MxN)");
    }
}

TEST_CASE("equality operator")
{
    Matrix A = Matrix({{1, 3}, {1, 3}});

    SECTION("should be equal")
    {
        Matrix B = Matrix({{1, 3}, {1, 3}});
        REQUIRE(A == B);
    }
    SECTION("same dimension but not content")
    {
        Matrix B = Matrix({{3, 1}, {1, 3}});
        REQUIRE(A != B);
    }
    SECTION("different dimension")
    {
        Matrix B = Matrix({{3, 1, 3}, {1, 3, 1}});
        REQUIRE(A != B);
    }
}

TEST_CASE("subscript operator")
{
    Matrix A = Matrix({{-1, -9}, {6, 7}});

    REQUIRE(A[0][1] == -9);
    REQUIRE(A[1][0] == 6);
}

TEST_CASE("inverse")
{
    SECTION("square")
    {
        Matrix A = Matrix({{1, 2}, {1, 3}});
        A.inverse();
        REQUIRE(A == Matrix({{3, -2}, {-1, 1}}));
    }
    SECTION("not square")
    {
        Matrix A = Matrix({{1, 2, 0}, {2, 3, -1}});
        REQUIRE_THROWS_WITH(A.inverse(), "Can't compute inverse: matrix is not square");
    }
    SECTION("singular")
    {
        Matrix A = Matrix({{1, 2}, {2, 4}});

        LogLevel org_level = logger.get_level();
        catchutils::PatchStdCout patched_cout;
        logger.set_level(LogLevel::info);

        A.inverse();
        REQUIRE(patched_cout.get().find("Can't compute inverse: matrix is singular") != std::string::npos);

        logger.set_level(org_level);
    }
}

TEST_CASE("transpose")
{
    SECTION("1x3")
    {
        Matrix A = Matrix({{7}, {1}, {1}});
        A.transpose();
        REQUIRE(A == Matrix({{7, 1, 1}}));
    }
    SECTION("2x1")
    {
        Matrix A = Matrix({{4, 5}});
        A.transpose();
        REQUIRE(A == Matrix({{4}, {5}}));
    }
    SECTION("2x2")
    {
        Matrix A = Matrix({{3, 2}, {1, 0}});
        A.transpose();
        REQUIRE(A == Matrix({{3, 1}, {2, 0}}));
    }
    SECTION("2x3")
    {
        Matrix A = Matrix({{1, 2, 3}, {4, 5, 6}});
        A.transpose();
        REQUIRE(A == Matrix({{1, 4}, {2, 5}, {3, 6}}));
    }
}
