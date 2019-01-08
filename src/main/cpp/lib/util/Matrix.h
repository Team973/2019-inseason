/*
 * Matrix.h
 *
 *  Created on: Jan 16, 2016
 *      Author: Andrew
 */

#pragma once

#include "math.h"
#include <stdio.h>

namespace frc973 {

/**
 * Matrix implementation. All fields are doubles.
 */
class Matrix {
public:
    /**
     * Create a new matrix with nRows rows and nCols cols.
     * @param nRows The number of rows.
     * @param nCols The number of columns.
     */
    Matrix(int nRows, int nCols);
    virtual ~Matrix();

    /**
     * Get all the data in the form of a double pointer. Use at your own risk.
     * Please don't screw it up!
     * @return The data.
     */
    double *GetData();

    /**
     * Get the element in the (x,y) position, not (y, x).
     * @param y specifies the row to get from.
     * @param x specifies the column to get from.
     */
    double Get(int y, int x);

    /**
     * Get the ith element (matlab bracket notation).
     * @param i The linear index into this matrix.
     */
    double Get(int i);

    /**
     * Set the (x,y)th element to |val|. Note it's y,x
     *
     * @param y The row to set into.
     * @param x The col to set into.
     * @param val The val to put there.
     */
    void Set(int y, int x, double val);

    /**
     * Set the ith element to |val|. Note it's linear index.
     *
     * @param i The linear index of cell to set.
     * @param val The value to write there.
     */
    void Set(int i, double val);

    /**
     * Get the width of this matrix.
     * @return The width.
     */
    int GetWidth();

    /**
     * Get the height of this matrix.
     * @return The height.
     */
    int GetHeight();

    /**
     * Check whether this matrix is the same size as that other matrix.
     */
    bool SameSize(Matrix *m);

    /**
     * Generate a new matrix that is the difference between the two given
     * matrices.
     * @param mat1 The left operand of the subtraction operation.
     * @param mat2 The right operand of the subtraction operation.
     * @return The new matrix object which you have to remember to clean up
     * after.
     */
    static Matrix *Subtract(Matrix *mat1, Matrix *mat2);

    /**
     * Generate a new matrix that is the sum of the two given matrices.
     * @param mat1 The left operand of the sum operation.
     * @param mat2 The right operand of the sum operation.
     * @return The new matrix object which you have to remember to clean up
     * after.
     */
    static Matrix *Add(Matrix *mat1, Matrix *mat2);

    /**
     * Generate a new matrix that is the product of the two given matrices.
     * @param mat1 The left operand of the multiplication operation.
     * @param mat2 The right operand of the multiplication operation.
     * @return The new matrix object which you have to remember to clean up
     * after.
     */
    static Matrix *Multiply(Matrix *mat1, Matrix *mat2);

    /**
     * Replace the first |n| values in this matrix with the first |n| values in
     * the array defined by d.
     * @param d The values in the array.
     * @param N The number of values.
     */
    void Flash(const double *d, const int N);

    /**
     * Check whether this matrix equals that matrix. This means same size and
     * same value in every column.
     * @param m The matrix.
     */
    bool Equals(Matrix *m);

    /**
     * Print this matrix to stdout.
     */
    void Display();

private:
    double *m_data;
    int m_width;
    int m_height;
};
}
