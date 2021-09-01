#include <cstdint>
#include <cstring>
#include <vector>
#include <stdexcept>
#include <logger.h>

// https://stackoverflow.com/questions/3519959/computing-the-inverse-of-a-matrix-using-lapack-in-c
extern "C" {
    // LU decomoposition of a general matrix.
    void dgetrf_(int* M, int *N, double* A, int* LDA, int* IPIV, int* INFO);

    // Inverse of a matrix given its LU decomposition.
    void dgetri_(int* N, double* A, int* LDA, int* IPIV, double* WORK, int* LWORK, int* INFO);
}

typedef std::vector<std::vector<double>> MatrixContent;

class Matrix
{
public:
    Matrix(MatrixContent content);

    void inverse();
    // TODO: transpose
    // TODO: overload multiplication operator
    // TODO: get content => overload []?
    const MatrixContent get_content();

    bool equal(Matrix other);

    uint32_t get_m() { return _m; }
    uint32_t get_n() { return _n; }
private:
    MatrixContent _content;

    uint32_t _m;
    uint32_t _n;

    bool _is_square() { return _m == _n; }

    uint32_t _check_and_get_n();

    void _to_lapack(MatrixContent& src, double* dst, uint32_t m, uint32_t n);
    void _from_lapack(double* src, MatrixContent& dst, uint32_t m, uint32_t n);
};
