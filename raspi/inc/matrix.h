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

    friend bool operator == (const Matrix& lhs, const Matrix& rhs) { return lhs._content == rhs._content; }
    friend bool operator != (const Matrix& lhs, const Matrix& rhs) { return lhs._content != rhs._content; }

    std::vector<double>& operator [] (size_t index) { return _content[index]; }

    friend Matrix operator * (const Matrix& mat, double scalar);
    friend Matrix operator * (double scalar, const Matrix& mat);
    // TODO: friend Matrix operator * (const Matrix& mat_0, const Matrix& mat_1);

    void inverse();
    void transpose();

    std::size_t get_m() { return _m; }
    std::size_t get_n() { return _n; }
private:
    MatrixContent _content;

    std::size_t _m;
    std::size_t _n;

    bool _is_square() { return _m == _n; }

    std::size_t _check_and_get_n();

    void _to_lapack(MatrixContent& src, double* dst, std::size_t m, std::size_t n);
    void _from_lapack(double* src, MatrixContent& dst, std::size_t m, std::size_t n);
};
