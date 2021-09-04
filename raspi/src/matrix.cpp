#include <matrix.h>

static MatrixContent _scale_content(MatrixContent content, double scalar);
static MatrixContent _init_content(size_t m, size_t n, double value = 0);

static double _dot_product(std::vector<double> v_0, std::vector<double> v_1);

Matrix::Matrix(std::vector<std::vector<double>> content) : _content(content)
{
    _check_and_set_size();
}

Matrix::Matrix(std::initializer_list<std::initializer_list<double>> lst)
{
    for (auto const& row : lst) { _content.push_back(row); }

    _check_and_set_size();
}

Matrix operator * (const Matrix& mat, double scalar)
{
    return Matrix(_scale_content(mat._content, scalar));
}

Matrix operator * (double scalar, const Matrix& mat)
{
    return Matrix(_scale_content(mat._content, scalar));
}

Matrix operator * (const Matrix& mat_0, const Matrix& mat_1)
{
    if (mat_0._n != mat_1._m)
    {
        throw std::logic_error("Can't multiply matrices: dimension mismatch");
    }

    MatrixContent content_m = _init_content(mat_0._m, mat_1._n);

    Matrix mat_1_t = mat_1;
    mat_1_t = mat_1_t.transpose();

    for (std::size_t i = 0; i < mat_0._m; i++)
    {
        for (std::size_t j = 0; j < mat_1._n; j++)
        {
            content_m[i][j] = _dot_product(mat_0._content[i], mat_1_t._content[j]);
        }
    }

    return Matrix(content_m);
}

Matrix operator + (const Matrix& mat_0, const Matrix& mat_1)
{
    if ((mat_0._m != mat_1._m) || (mat_0._n != mat_1._n))
    {
        throw std::logic_error("Can't add/subtract matrices: dimension mismatch");
    }

    MatrixContent content_a = _init_content(mat_0._m, mat_0._n);

    for (std::size_t i = 0; i < mat_0._m; i++)
    {
        for (std::size_t j = 0; j < mat_0._n; j++)
        {
            content_a[i][j] = mat_0._content[i][j] + mat_1._content[i][j];
        }
    }

    return Matrix(content_a);
}

Matrix operator - (const Matrix& mat_0, const Matrix& mat_1)
{
    return mat_0 + (-1 * mat_1);
}

Matrix Matrix::inverse()
{
    MatrixContent content_i = _content;

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

        _from_lapack(lapack_content, content_i, _m, _n);
    }
    else
    {
        throw std::logic_error("Can't compute inverse: matrix is not square");
    }

    return Matrix(content_i);
}

Matrix Matrix::transpose()
{
    size_t m_t = _n;
    size_t n_t = _m;
    MatrixContent content_t = _init_content(m_t, n_t);

    for (std::size_t i = 0; i < _m; i++)
    {
        for (std::size_t j = 0; j < _n; j++)
        {
            content_t[j][i] = _content[i][j];
        }
    }

    return Matrix(content_t);
}

void Matrix::debug_print(std::string name)
{
    std::cout << name << " = " << std::endl;

    for (auto row : _content)
    {
        for (auto x : row)
        {
            std::cout << x << "\t";
        }
        std::cout << std::endl;
    }

    std::cout << "-----------------" << std::endl;
}

void Matrix::_check_and_set_size()
{
    _m = _content.size();
    _n = _check_and_get_n();
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

static MatrixContent _init_content(size_t m, size_t n, double value)
{
    std::vector<double> row(n, value);
    MatrixContent mat(m, row);

    return mat;
}

static double _dot_product(std::vector<double> v_0, std::vector<double> v_1)
{
    double dp = 0;

    for (std::size_t i = 0; i < v_0.size(); i++)
    {
        dp += v_0[i] * v_1[i];
    }

    return dp;
}
