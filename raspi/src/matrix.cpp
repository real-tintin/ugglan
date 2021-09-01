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
        int ipiv[_m];
        int lwork = _m * _m;
        double work[lwork];
        int info;

        double lapack_content[_m * _m] = {};
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

const MatrixContent Matrix::get_content()
{
    return (static_cast<const MatrixContent>(_content));
}

bool Matrix::equal(Matrix other)
{
    return (_content == other.get_content());
}

uint32_t Matrix::_check_and_get_n()
{
    std::vector<int>::size_type exp_n = _content[0].size();

    for (auto const& row: _content)
    {
        if (row.size() != exp_n)
        {
            throw std::logic_error("Matrix dimension error (not MxN)");
        }
    }

    return static_cast<uint32_t>(exp_n);
}

void Matrix::_to_lapack(MatrixContent& src, double* dst, uint32_t m, uint32_t n)
{
    for (uint32_t i = 0; i < m; i++)
    {
        for (uint32_t j = 0; j < n; j++)
        {
            dst[m * i + j] = src[i][j];
        }
    }
}

void Matrix::_from_lapack(double* src, MatrixContent& dst, uint32_t m, uint32_t n)
{
    for (uint32_t i = 0; i < m; i++)
    {
        for (uint32_t j = 0; j < n; j++)
        {
            dst[i][j] = src[m * i + j];
        }
    }
}
