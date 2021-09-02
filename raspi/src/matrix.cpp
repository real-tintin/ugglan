#include <matrix.h>

Matrix::Matrix(std::vector<std::vector<double>> content) :
    _content(content)
{
    _m = _content.size();
    _n = _check_and_get_n();
}

void Matrix::inverse()
{
    if (_is_square())
    {
        int m = static_cast<int>(_m);
        int ipiv[m];
        int lwork = m * m;
        double work[lwork];
        int info;

        double lapack_content[m * m] = {};
        _to_lapack(_content, lapack_content, _m, _n);

        dgetrf_(&m, &m, lapack_content, &m, ipiv, &info);
        dgetri_(&m, lapack_content, &m, ipiv, work, &lwork, &info);

        if (info > 0) { logger.error("Can't compute inverse: matrix is singular"); }
        if (info < 0) { logger.error("Can't compute inverse: invalid arg to dgetri"); }

        _from_lapack(lapack_content, _content, _m, _n);
    }
    else
    {
        logger.error("Can't compute inverse: matrix is not square");
    }
}

std::size_t Matrix::_check_and_get_n()
{
    std::size_t exp_n = _content[0].size();

    for (auto const& row: _content)
    {
        if (row.size() != exp_n)
        {
            throw std::logic_error("Matrix dimension error (not MxN)");
        }
    }

    return exp_n;
}

void Matrix::_to_lapack(MatrixContent& src, double* dst, std::size_t m, std::size_t n)
{
    for (std::size_t i = 0; i < m; i++)
    {
        for (std::size_t j = 0; j < n; j++)
        {
            dst[m * i + j] = src[i][j];
        }
    }
}

void Matrix::_from_lapack(double* src, MatrixContent& dst, std::size_t m, std::size_t n)
{
    for (std::size_t i = 0; i < m; i++)
    {
        for (std::size_t j = 0; j < n; j++)
        {
            dst[i][j] = src[m * i + j];
        }
    }
}
