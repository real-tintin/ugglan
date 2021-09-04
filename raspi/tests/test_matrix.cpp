#include <catch.h>
#include <catch_utils.h>

#include <matrix.h>

void almost_equal(Matrix A, Matrix B, double abs_tol = 1e-6)
{
    REQUIRE(A.get_m() == B.get_m());
    REQUIRE(A.get_n() == B.get_n());

    for (size_t i = 0; i < A.get_m(); i++)
    {
        for (size_t j = 0; j < A.get_n(); j++)
        {
            REQUIRE((A[i][j] - B[i][j]) < abs_tol);
        }
    }
}

TEST_CASE("constructor")
{
    SECTION("Default")
    {
        Matrix A;
    }
    SECTION("MatrixContent")
    {
        MatrixContent content = {{0}};
        Matrix A(content);
    }
    SECTION("Initializer list")
    {
        Matrix A({{1}});
    }
    SECTION("Invalid dimension")
    {
        REQUIRE_THROWS_WITH(Matrix({{1}, {1, 0}}), "Matrix dimension error (not MxN)");
    }
}

TEST_CASE("equality operator")
{
    Matrix A({{1, 3}, {1, 3}});

    SECTION("should be equal")
    {
        Matrix B({{1, 3}, {1, 3}});
        REQUIRE(A == B);
    }
    SECTION("same dimension but not content")
    {
        Matrix B({{3, 1}, {1, 3}});
        REQUIRE(A != B);
    }
    SECTION("different dimension")
    {
        Matrix B({{3, 1, 3}, {1, 3, 1}});
        REQUIRE(A != B);
    }
}

TEST_CASE("subscript operator")
{
    Matrix A({{-1, -9}, {6, 7}});

    REQUIRE(A[0][1] == -9);
    REQUIRE(A[1][0] == 6);
}

TEST_CASE("inverse")
{
    SECTION("square")
    {
        Matrix A({{1, 2}, {1, 3}});
        REQUIRE(A.inverse() == Matrix({{3, -2}, {-1, 1}}));
    }
    SECTION("not square")
    {
        Matrix A({{1, 2, 0}, {2, 3, -1}});
        REQUIRE_THROWS_WITH(A.inverse(), "Can't compute inverse: matrix is not square");
    }
    SECTION("singular")
    {
        Matrix A({{1, 2}, {2, 4}});

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
    SECTION("1x1")
    {
        Matrix A({{8}});
        REQUIRE(A.transpose() == Matrix({{8}}));
    }
    SECTION("1x3")
    {
        Matrix A({{7}, {1}, {1}});
        REQUIRE(A.transpose() == Matrix({{7, 1, 1}}));
    }
    SECTION("2x1")
    {
        Matrix A({{4, 5}});
        REQUIRE(A.transpose() == Matrix({{4}, {5}}));
    }
    SECTION("2x2")
    {
        Matrix A({{3, 2}, {1, 0}});
        REQUIRE(A.transpose() == Matrix({{3, 1}, {2, 0}}));
    }
    SECTION("2x3")
    {
        Matrix A({{1, 2, 3}, {4, 5, 6}});
        REQUIRE(A.transpose() == Matrix({{1, 4}, {2, 5}, {3, 6}}));
    }
}

TEST_CASE("multiplication")
{
    SECTION("scalar * 3x1")
    {
        Matrix A({{3}, {2}, {1}});
        REQUIRE((3 * A) == Matrix({{9}, {6}, {3}}));
    }
    SECTION("2x2 * scalar")
    {
        Matrix A({{1, 2}, {2, 1}});
        REQUIRE((A * 2) == Matrix({{2, 4}, {4, 2}}));
    }
    SECTION("1x1 * 1x1")
    {
        Matrix A({{-2}});
        Matrix B({{-5}});
        REQUIRE((A * B) == Matrix({{10}}));
    }
    SECTION("2x2 * 2x2")
    {
        Matrix A({{1, 2}, {2, 1}});
        Matrix B({{1, 3}, {0, 1}});
        REQUIRE((A * B) == Matrix({{1, 5}, {2, 7}}));
    }
    SECTION("1x3 * 3x1")
    {
        Matrix A({{1, 0, 1}});
        Matrix B({{3}, {2}, {1}});
        REQUIRE((A * B) == Matrix({{4}}));
    }
    SECTION("2x1 * 1x2")
    {
        Matrix A({{3}, {4}});
        Matrix B({{2, -1}});
        REQUIRE((A * B) == Matrix({{6, -3}, {8, -4}}));
    }
    SECTION("3x2 * 2x3")
    {
        Matrix A({{2, 1}, {3, 1}, {4, 0}});
        Matrix B({{2, -1, 1}, {3, 1, 2}});
        REQUIRE((A * B) == Matrix({{7, -1, 4}, {9, -2, 5}, {8, -4, 4}}));
    }
    SECTION("3x2 * 2x2")
    {
        Matrix A({{1, 1}, {2, 2}, {3, 3}});
        Matrix B({{1, -1}, {2, 3}});
        REQUIRE((A * B) == Matrix({{3, 2}, {6, 4}, {9, 6}}));
    }
    SECTION("1x2 * 3x1 (dim mismatch)")
    {
        Matrix A({{1, 2}});
        Matrix B({{1}, {2}, {3}});
        REQUIRE_THROWS_WITH(A * B, "Can't multiply matrices: dimension mismatch");
    }
}

TEST_CASE("addition")
{
    SECTION("1x1 * 1x1")
    {
        Matrix A({{2}});
        Matrix B({{-5}});
        REQUIRE((A + B) == Matrix({{-3}}));
    }
    SECTION("2x2 * 2x2")
    {
        Matrix A({{1, 2}, {2, 1}});
        Matrix B({{1, 3}, {0, 1}});
        REQUIRE((A + B) == Matrix({{2, 5}, {2, 2}}));
    }
    SECTION("1x2 * 3x1 (dim mismatch)")
    {
        Matrix A({{1, 2}});
        Matrix B({{1}, {2}, {3}});
        REQUIRE_THROWS_WITH(A + B, "Can't add/subtract matrices: dimension mismatch");
    }
}

TEST_CASE("subtraction")
{
    SECTION("1x1 * 1x1")
    {
        Matrix A({{1}});
        Matrix B({{6}});
        REQUIRE((A - B) == Matrix({{-5}}));
    }
    SECTION("1x2 * 1x2")
    {
        Matrix A({{5}, {-2}});
        Matrix B({{-1}, {3}});
        REQUIRE((A - B) == Matrix({{6}, {-5}}));
    }
    SECTION("1x2 * 2x1 (dim mismatch)")
    {
        Matrix A({{1, 2}});
        Matrix B({{1}, {2}});
        REQUIRE_THROWS_WITH(A - B, "Can't add/subtract matrices: dimension mismatch");
    }
}

TEST_CASE("operator chaining")
{
    Matrix P({{0, 2, 4}, {2, 6, 8}, {4, 6, 10}});
    Matrix H({{1, 0, 0}, {0, 1, 0}});
    Matrix R({{1, 0}, {0, 1}});

    Matrix S = H * P * H.transpose() + R;
    Matrix K = P * H.transpose() * S.inverse();

    almost_equal(K, Matrix({{-4.0/3.0, 2.0/3.0}, {2.0/3.0, 2.0/3.0}, {16.0/3.0, -2.0/3.0}}));
}
