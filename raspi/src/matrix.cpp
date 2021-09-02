#include <matrix.h>

static MatrixContent _scale_content(MatrixContent content, double scalar);

Matrix::Matrix(std::vector<std::vector<double>> content) :
    _content(content)
{
    _m = _content.size();
    _n = _check_and_get_n();
}

Matrix operator * (const Matrix& mat, double scalar)
{
    return Matrix(_scale_content(mat._content, scalar));
}

Matrix operator * (double scalar, const Matrix& mat)
{
    return Matrix(_scale_content(mat._content, scalar));
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

        if (info > 0) { logger.warn("Can't compute inverse: matrix is singular"); }
        if (info < 0) { logger.warn("Can't compute inverse: invalid arg to dgetri"); }

        _from_lapack(lapack_content, _content, _m, _n);
    }
    else
    {
        throw std::logic_error("Can't compute inverse: matrix is not square");
    }
}

void Matrix::transpose()
{
    size_t m_t = _n;
    size_t n_t = _m;
    MatrixContent content_t;
    std::vector<double> col_j;

    for (std::size_t j = 0; j < _n; j++)
    {
        col_j.clear();
        for (std::size_t i = 0; i < _m; i++)
        {
            col_j.push_back(_content[i][j]);
        }
        content_t.push_back(col_j);
    }

    _m = m_t;
    _n = n_t;
    _content = content_t;
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

static MatrixContent _scale_content(MatrixContent content, double scalar)
{
    for (std::size_t i = 0; i < content.size(); i++)
    {
        for (std::size_t j = 0; j < content[i].size(); j++)
        {
            content[i][j] *= scalar;
        }
    }

    return content;
}
