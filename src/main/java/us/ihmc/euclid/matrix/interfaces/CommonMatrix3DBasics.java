package us.ihmc.euclid.matrix.interfaces;


import org.ejml.data.DMatrixRMaj;

import us.ihmc.euclid.interfaces.Clearable;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tools.Matrix3DTools;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;

/**
 * Read interface for a 3-by-3 matrix object with a restricted writing interface.
 * <p>
 * In this interface, the matrix is assumed to be generic purpose. Therefore, the algorithms used
 * here are limited to generic applications without violating potential constraints of more specific
 * matrices such as a rotation matrix.
 * </p>
 *
 * @author Sylvain Bertrand
 */
public interface CommonMatrix3DBasics extends Matrix3DReadOnly, Clearable
{
   /**
    * Sets the 9 coefficients of this matrix to the given ones.
    *
    * @param m00 the new 1st row 1st column coefficient for this matrix.
    * @param m01 the new 1st row 2nd column coefficient for this matrix.
    * @param m02 the new 1st row 3rd column coefficient for this matrix.
    * @param m10 the new 2nd row 1st column coefficient for this matrix.
    * @param m11 the new 2nd row 2nd column coefficient for this matrix.
    * @param m12 the new 2nd row 3rd column coefficient for this matrix.
    * @param m20 the new 3rd row 1st column coefficient for this matrix.
    * @param m21 the new 3rd row 2nd column coefficient for this matrix.
    * @param m22 the new 3rd row 3rd column coefficient for this matrix.
    */
   public void set(double m00, double m01, double m02, double m10, double m11, double m12, double m20, double m21, double m22);

   @Override
   default void setToZero()
   {
      set(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
   }

   /**
    * Sets this matrix to contain only {@linkplain Double#NaN}:
    *
    * <pre>
    *     / NaN  NaN  NaN \
    * m = | NaN  NaN  NaN |
    *     \ NaN  NaN  NaN /
    * </pre>
    */
   @Override
   default void setToNaN()
   {
      set(Double.NaN, Double.NaN, Double.NaN, Double.NaN, Double.NaN, Double.NaN, Double.NaN, Double.NaN, Double.NaN);
   }

   @Override
   default boolean containsNaN()
   {
      return Matrix3DReadOnly.super.containsNaN();
   }

   /**
    * Orthonormalization of this matrix using the
    * <a href="https://en.wikipedia.org/wiki/Gram%E2%80%93Schmidt_process"> Gram-Schmidt method</a>.
    */
   default void normalize()
   {
      Matrix3DTools.normalize(this);
   }

   /**
    * Sets this matrix to identity:
    *
    * <pre>
    *     / 1  0  0 \
    * m = | 0  1  0 |
    *     \ 0  0  1 /
    * </pre>
    */
   default void setIdentity()
   {
      set(1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0);
   }

   /**
    * Sets this matrix to {@code other}.
    *
    * @param other the other matrix to copy the values of. Not modified.
    */
   default void set(Matrix3DReadOnly other)
   {
      set(other.getM00(), other.getM01(), other.getM02(), other.getM10(), other.getM11(), other.getM12(), other.getM20(), other.getM21(), other.getM22());
   }

   /**
    * Copies the values in the given array into this matrix as follows:
    *
    * <pre>
    *     / matrixArray[0]  matrixArray[1]  matrixArray[2] \
    * m = | matrixArray[3]  matrixArray[4]  matrixArray[5] |
    *     \ matrixArray[6]  matrixArray[7]  matrixArray[8] /
    * </pre>
    *
    * @param matrixArray the array containing the new values for this matrix. Not modified.
    */
   default void set(double[] matrixArray)
   {
      double m00 = matrixArray[0];
      double m01 = matrixArray[1];
      double m02 = matrixArray[2];
      double m10 = matrixArray[3];
      double m11 = matrixArray[4];
      double m12 = matrixArray[5];
      double m20 = matrixArray[6];
      double m21 = matrixArray[7];
      double m22 = matrixArray[8];
      set(m00, m01, m02, m10, m11, m12, m20, m21, m22);
   }

   /**
    * Copies the values in the given array into this matrix as follows:
    *
    * <pre>
    *     / matrixArray[startIndex + 0]  matrixArray[startIndex + 1]  matrixArray[startIndex + 2] \
    * m = | matrixArray[startIndex + 3]  matrixArray[startIndex + 4]  matrixArray[startIndex + 5] |
    *     \ matrixArray[startIndex + 6]  matrixArray[startIndex + 7]  matrixArray[startIndex + 8] /
    * </pre>
    *
    * @param startIndex  the first index to start reading from in the array.
    * @param matrixArray the array containing the new values for this matrix. Not modified.
    */
   default void set(int startIndex, double[] matrixArray)
   {
      double m00 = matrixArray[startIndex + 0];
      double m01 = matrixArray[startIndex + 1];
      double m02 = matrixArray[startIndex + 2];
      double m10 = matrixArray[startIndex + 3];
      double m11 = matrixArray[startIndex + 4];
      double m12 = matrixArray[startIndex + 5];
      double m20 = matrixArray[startIndex + 6];
      double m21 = matrixArray[startIndex + 7];
      double m22 = matrixArray[startIndex + 8];
      set(m00, m01, m02, m10, m11, m12, m20, m21, m22);
   }

   /**
    * Copies the values in the given dense-matrix into this matrix.
    *
    * @param matrix the dense-matrix containing the new values for this matrix. Not modified.
    */
   default void set(DMatrixRMaj matrix)
   {
      EuclidCoreTools.checkMatrixMinimumSize(3, 3, matrix);

      double m00 = matrix.unsafe_get(0, 0);
      double m01 = matrix.unsafe_get(0, 1);
      double m02 = matrix.unsafe_get(0, 2);
      double m10 = matrix.unsafe_get(1, 0);
      double m11 = matrix.unsafe_get(1, 1);
      double m12 = matrix.unsafe_get(1, 2);
      double m20 = matrix.unsafe_get(2, 0);
      double m21 = matrix.unsafe_get(2, 1);
      double m22 = matrix.unsafe_get(2, 2);
      set(m00, m01, m02, m10, m11, m12, m20, m21, m22);
   }

   /**
    * Copies the values in the given dense-matrix into this matrix given index offsets for the row and
    * column.
    *
    * @param startRow    the first row index to start reading from in the dense-matrix.
    * @param startColumn the first column index to start reading from in the dense-matrix.
    * @param matrix      the dense-matrix containing the new values for this matrix. Not modified.
    */
   default void set(int startRow, int startColumn, DMatrixRMaj matrix)
   {
      EuclidCoreTools.checkMatrixMinimumSize(startRow + 3, startColumn + 3, matrix);

      int row = startRow;
      int column = startColumn;

      double m00 = matrix.unsafe_get(row, column++);
      double m01 = matrix.unsafe_get(row, column++);
      double m02 = matrix.unsafe_get(row, column);
      row++;
      column = startColumn;
      double m10 = matrix.unsafe_get(row, column++);
      double m11 = matrix.unsafe_get(row, column++);
      double m12 = matrix.unsafe_get(row, column);
      row++;
      column = startColumn;
      double m20 = matrix.unsafe_get(row, column++);
      double m21 = matrix.unsafe_get(row, column++);
      double m22 = matrix.unsafe_get(row, column);
      set(m00, m01, m02, m10, m11, m12, m20, m21, m22);
   }

   /**
    * Sets this matrix to equal the other matrix and then normalizes this, see {@link #normalize()}.
    *
    * @param other the other matrix used to update this matrix. Not modified.
    */
   default void setAndNormalize(Matrix3DReadOnly other)
   {
      set(other);
      normalize();
   }

   /**
    * Sets this matrix from the given tuples each holding on the values of each row.
    *
    * <pre>
    *        /  firstRow.getX()  firstRow.getY()  firstRow.getZ() \
    * this = | secondRow.getX() secondRow.getY() secondRow.getZ() |
    *        \  thirdRow.getX()  thirdRow.getY()  thirdRow.getZ() /
    * </pre>
    *
    * @param firstRow  the tuple holding onto the values of the first row. Not modified.
    * @param secondRow the tuple holding onto the values of the second row. Not modified.
    * @param thirdRow  the tuple holding onto the values of the third row. Not modified.
    */
   default void setRows(Tuple3DReadOnly firstRow, Tuple3DReadOnly secondRow, Tuple3DReadOnly thirdRow)
   {
      set(firstRow.getX(),
          firstRow.getY(),
          firstRow.getZ(),

          secondRow.getX(),
          secondRow.getY(),
          secondRow.getZ(),

          thirdRow.getX(),
          thirdRow.getY(),
          thirdRow.getZ());
   }

   /**
    * Sets this matrix from the given tuples each holding on the values of each column.
    *
    * <pre>
    *        / firstColumn.getX() secondColumn.getX() thirdColumn.getX() \
    * this = | firstColumn.getY() secondColumn.getY() thirdColumn.getY() |
    *        \ firstColumn.getZ() secondColumn.getZ() thirdColumn.getZ() /
    * </pre>
    *
    * @param firstColumn  the tuple holding onto the values of the first column. Not modified.
    * @param secondColumn the tuple holding onto the values of the second column. Not modified.
    * @param thirdColumn  the tuple holding onto the values of the third column. Not modified.
    */
   default void setColumns(Tuple3DReadOnly firstColumn, Tuple3DReadOnly secondColumn, Tuple3DReadOnly thirdColumn)
   {
      set(firstColumn.getX(),
          secondColumn.getX(),
          thirdColumn.getX(),

          firstColumn.getY(),
          secondColumn.getY(),
          thirdColumn.getY(),

          firstColumn.getZ(),
          secondColumn.getZ(),
          thirdColumn.getZ());
   }
}