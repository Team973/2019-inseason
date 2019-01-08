#include "lib/util/Matrix.h"

namespace frc973 {

Matrix::Matrix(int nRows, int nCols) {
    int i;
    m_width = nCols;
    m_height = nRows;
    m_data = new double[m_width * m_height];
    for (i = 0; i < nRows * nCols; i++) {
        m_data[i] = 0.0;
    }
}

Matrix::~Matrix() {
    delete[] m_data;
}

double *Matrix::GetData() {
    return m_data;
}

double Matrix::Get(int y, int x) {
    if (x >= m_width || y >= m_height) {
        printf("Array out of bounds: wanted (%d, %d) from (%d, %d)\n", y, x,
               m_height, m_width);
        return NAN;
    }
    return m_data[x + (m_width * y)];
}

double Matrix::Get(int i) {
    if (i >= m_width * m_height) {
        printf("Array out of bounds: wanted %d from (%d, %d)\n", i, m_height,
               m_width);
        return NAN;
    }
    return m_data[i];
}

void Matrix::Set(int y, int x, double val) {
    if (x >= m_width || y >= m_height) {
        printf("Array out of bounds: wanted (%d, %d) from (%d, %d)\n", y, x,
               m_height, m_width);
    }
    m_data[x + (m_width * y)] = val;
}

void Matrix::Set(int i, double val) {
    if (i >= m_width * m_height) {
        printf("Array out of bounds: wanted %d from (%d, %d)\n", i, m_height,
               m_width);
    }
    m_data[i] = val;
}

int Matrix::GetWidth() {
    return m_width;
}

int Matrix::GetHeight() {
    return m_height;
}

bool Matrix::SameSize(Matrix *m) {
    return m && (GetWidth() == m->GetWidth()) &&
           (GetHeight() == m->GetHeight());
}

Matrix *Matrix::Subtract(Matrix *mat1, Matrix *mat2) {
    if (!mat1 || !mat2 || !mat1->SameSize(mat2)) {
        printf("Tried to subtract matrices of not the same size");
        return nullptr;
    }
    Matrix *result = new Matrix(mat2->GetHeight(), mat1->GetWidth());
    int comp = mat1->GetWidth() * mat2->GetHeight();

    for (int i = 0; i < comp; i++) {
        result->Set(i, mat1->Get(i) - mat2->Get(i));
    }
    return result;
}

Matrix *Matrix::Add(Matrix *mat1, Matrix *mat2) {
    if (!mat1 || !mat2 || !mat1->SameSize(mat2)) {
        printf("Tried to multiple matrices of not the same size");
        return nullptr;
    }

    Matrix *result = new Matrix(mat1->GetHeight(), mat1->GetWidth());
    int comp = mat1->GetWidth() * mat1->GetHeight();
    int i;
    for (i = 0; i < comp; i++) {
        result->Set(i, mat1->Get(i) + mat2->Get(i));
    }
    return result;
}

Matrix *Matrix::Multiply(Matrix *mat1, Matrix *mat2) {
    if (!mat1 || !mat2 || mat1->GetWidth() != mat2->GetHeight()) {
        printf("Cant multiply matrices of disparit dimensions\n");
        return nullptr;
    }

    int destHeight = mat1->GetHeight();
    int destWidth = mat2->GetWidth();

    Matrix *result = new Matrix(destHeight, destWidth);

    int pMax = mat1->GetWidth();
    int width1 = mat1->GetWidth();
    int width2 = mat2->GetWidth();
    for (int i = 0; i < destWidth; i++) {
        for (int j = 0; j < destHeight; j++) {
            double tmp = 0.0;
            for (int p = 0; p < pMax; p++) {
                tmp +=
                    mat2->m_data[i + width2 * p] * mat1->m_data[p + width1 * j];
            }
            result->m_data[i + destWidth * j] = tmp;
        }
    }
    return result;
}

/**
 * Replace the first |n| values in this matrix with the first |n|
 * values in the array defined by d
 */
void Matrix::Flash(const double *d, const int N) {
    for (int i = 0; i < N; i++) {
        m_data[i] = d[i];
    }
}

bool Matrix::Equals(Matrix *m) {
    if (!SameSize(m)) {
        return false;
    }

    for (int i = 0; i < GetHeight(); ++i) {
        for (int j = 0; j < GetWidth(); ++j) {
            if (Get(i, j) != m->Get(i, j)) {
                return false;
            }
        }
    }

    return true;
}

void Matrix::Display() {
    printf("[ ");

    for (int x = 0; x < GetHeight(); x++) {
        for (int y = 0; y < GetWidth(); y++) {
            printf("%lf ", Get(x, y));
        }
        printf(";");
    }

    printf("]\n");
}
}
