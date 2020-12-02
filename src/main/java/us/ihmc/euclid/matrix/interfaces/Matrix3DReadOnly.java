package us.ihmc.euclid.matrix.interfaces;

import org.ejml.data.DMatrix;

import us.ihmc.euclid.exceptions.NotAMatrix2DException;
import us.ihmc.euclid.exceptions.NotARotationMatrixException;
import us.ihmc.euclid.exceptions.SingularMatrixException;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tools.Matrix3DFeatures;
import us.ihmc.euclid.tools.Matrix3DTools;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.Vector4DBasics;
import us.ihmc.euclid.tuple4D.interfaces.Vector4DReadOnly;

/**
 * Read-only interface for any type of 3-by-3 matrices.
 *
 * @author Sylvain Bertrand
 */
public interface Matrix3DReadOnly
{
   /**
    * Gets the 1st row 1st column coefficient of this matrix.
    *
    * @return the 1st row 1st column coefficient.
    */
   double getM00();

   /**
    * Gets the 1st row 2nd column coefficient of this matrix.
    *
    * @return the 1st row 2nd column coefficient.
    */
   double getM01();

   /**
    * Gets the 1st row 3rd column coefficient of this matrix.
    *
    * @return the 1st row 3rd column coefficient.
    */
   double getM02();

   /**
    * Gets the 2nd row 1st column coefficient of this matrix.
    *
    * @return the 2nd row 1st column coefficient.
    */
   double getM10();

   /**
    * Gets the 2nd row 2nd column coefficient of this matrix.
    *
    * @return the 2nd row 2nd column coefficient.
    */
   double getM11();

   /**
    * Gets the 2nd row 3rd column coefficient of this matrix.
    *
    * @return the 2nd row 3rd column coefficient.
    */
   double getM12();

   /**
    * Gets the 3rd row 1st column coefficient of this matrix.
    *
    * @return the 3rd row 1st column coefficient.
    */
   double getM20();

   /**
    * Gets the 3rd row 2nd column coefficient of this matrix.
    *
    * @return the 3rd row 2nd column coefficient.
    */
   double getM21();

   /**
    * Gets the 3rd row 3rd column coefficient of this matrix.
    *
    * @return the 3rd row 3rd column coefficient.
    */
   double getM22();

   /**
    * Retrieves and returns a coefficient of this matrix given its row and column indices.
    *
    * @param row    the row of the coefficient to return.
    * @param column the column of the coefficient to return.
    * @return the coefficient's value.
    * @throws ArrayIndexOutOfBoundsException if either {@code row} &notin; [0, 2] or {@code column}
    *                                        &notin; [0, 2].
    */
   default double getElement(int row, int column)
   {
      switch (row)
      {
         case 0:
            switch (column)
            {
               case 0:
                  return getM00();
               case 1:
                  return getM01();
               case 2:
                  return getM02();
               default:
                  throw Matrix3DTools.columnOutOfBoundsException(2, column);
            }
         case 1:
            switch (column)
            {
               case 0:
                  return getM10();
               case 1:
                  return getM11();
               case 2:
                  return getM12();
               default:
                  throw Matrix3DTools.columnOutOfBoundsException(2, column);
            }

         case 2:
            switch (column)
            {
               case 0:
                  return getM20();
               case 1:
                  return getM21();
               case 2:
                  return getM22();
               default:
                  throw Matrix3DTools.columnOutOfBoundsException(2, column);
            }

         default:
            throw Matrix3DTools.rowOutOfBoundsException(2, row);
      }
   }

   /**
    * Packs the coefficients of this matrix into a row-major 1D array.
    *
    * @param matrixArrayToPack the array in which the coefficients of this matrix are stored. Modified.
    */
   default void get(double[] matrixArrayToPack)
   {
      matrixArrayToPack[0] = getM00();
      matrixArrayToPack[1] = getM01();
      matrixArrayToPack[2] = getM02();
      matrixArrayToPack[3] = getM10();
      matrixArrayToPack[4] = getM11();
      matrixArrayToPack[5] = getM12();
      matrixArrayToPack[6] = getM20();
      matrixArrayToPack[7] = getM21();
      matrixArrayToPack[8] = getM22();
   }

   /**
    * Packs the coefficients of this matrix into a row-major 1D array starting at the given index
    * {@code startIndex}.
    *
    * @param startIndex        index in the array to store the first coefficient of this matrix.
    * @param matrixArrayToPack the array in which the coefficients of this matrix are stored. Modified.
    */
   default void get(int startIndex, double[] matrixArrayToPack)
   {
      matrixArrayToPack[startIndex++] = getM00();
      matrixArrayToPack[startIndex++] = getM01();
      matrixArrayToPack[startIndex++] = getM02();
      matrixArrayToPack[startIndex++] = getM10();
      matrixArrayToPack[startIndex++] = getM11();
      matrixArrayToPack[startIndex++] = getM12();
      matrixArrayToPack[startIndex++] = getM20();
      matrixArrayToPack[startIndex++] = getM21();
      matrixArrayToPack[startIndex] = getM22();
   }

   /**
    * Packs the coefficients of this matrix into a matrix.
    *
    * @param matrixToPack the matrix in which the coefficients of this matrix are stored. Modified.
    */
   default void get(DMatrix matrixToPack)
   {
      EuclidCoreTools.checkMatrixMinimumSize(3, 3, matrixToPack);

      matrixToPack.unsafe_set(0, 0, getM00());
      matrixToPack.unsafe_set(0, 1, getM01());
      matrixToPack.unsafe_set(0, 2, getM02());
      matrixToPack.unsafe_set(1, 0, getM10());
      matrixToPack.unsafe_set(1, 1, getM11());
      matrixToPack.unsafe_set(1, 2, getM12());
      matrixToPack.unsafe_set(2, 0, getM20());
      matrixToPack.unsafe_set(2, 1, getM21());
      matrixToPack.unsafe_set(2, 2, getM22());
   }

   /**
    * Packs the coefficients of this matrix into a matrix given index offsets for the row and the
    * column.
    *
    * @param startRow     the first row index to start writing in the matrix.
    * @param startColumn  the first column index to start writing in the matrix.
    * @param matrixToPack the matrix in which the coefficients of this matrix are stored. Modified.
    */
   default void get(int startRow, int startColumn, DMatrix matrixToPack)
   {
      EuclidCoreTools.checkMatrixMinimumSize(startRow + 3, startColumn + 3, matrixToPack);

      int row = startRow;
      int column = startColumn;
      matrixToPack.unsafe_set(row, column++, getM00());
      matrixToPack.unsafe_set(row, column++, getM01());
      matrixToPack.unsafe_set(row++, column, getM02());
      column = startColumn;
      matrixToPack.unsafe_set(row, column++, getM10());
      matrixToPack.unsafe_set(row, column++, getM11());
      matrixToPack.unsafe_set(row++, column, getM12());
      column = startColumn;
      matrixToPack.unsafe_set(row, column++, getM20());
      matrixToPack.unsafe_set(row, column++, getM21());
      matrixToPack.unsafe_set(row, column, getM22());
   }

   /**
    * Packs a column of this matrix into an array.
    *
    * @param column            the index of the column to pack.
    * @param columnArrayToPack the array in which the column of this matrix is stored. Modified.
    * @throws ArrayIndexOutOfBoundsException if {@code column} &notin; [0, 2].
    */
   default void getColumn(int column, double columnArrayToPack[])
   {
      switch (column)
      {
         case 0:
            columnArrayToPack[0] = getM00();
            columnArrayToPack[1] = getM10();
            columnArrayToPack[2] = getM20();
            return;
         case 1:
            columnArrayToPack[0] = getM01();
            columnArrayToPack[1] = getM11();
            columnArrayToPack[2] = getM21();
            return;
         case 2:
            columnArrayToPack[0] = getM02();
            columnArrayToPack[1] = getM12();
            columnArrayToPack[2] = getM22();
            return;
         default:
            throw Matrix3DTools.columnOutOfBoundsException(2, column);
      }
   }

   /**
    * Packs a column of this matrix into a 3D tuple.
    *
    * @param column       the index of the column to pack.
    * @param columnToPack the tuple in which the column of this matrix is stored. Modified.
    * @throws ArrayIndexOutOfBoundsException if {@code column} &notin; [0, 2].
    */
   default void getColumn(int column, Tuple3DBasics columnToPack)
   {
      switch (column)
      {
         case 0:
            columnToPack.setX(getM00());
            columnToPack.setY(getM10());
            columnToPack.setZ(getM20());
            return;
         case 1:
            columnToPack.setX(getM01());
            columnToPack.setY(getM11());
            columnToPack.setZ(getM21());
            return;
         case 2:
            columnToPack.setX(getM02());
            columnToPack.setY(getM12());
            columnToPack.setZ(getM22());
            return;
         default:
            throw Matrix3DTools.columnOutOfBoundsException(2, column);
      }
   }

   /**
    * Packs a row of this matrix into an array.
    *
    * @param row            the index of the row to pack.
    * @param rowArrayToPack the array in which the row of this matrix is stored. Modified.
    * @throws ArrayIndexOutOfBoundsException if {@code row} &notin; [0, 2].
    */
   default void getRow(int row, double rowArrayToPack[])
   {
      switch (row)
      {
         case 0:
            rowArrayToPack[0] = getM00();
            rowArrayToPack[1] = getM01();
            rowArrayToPack[2] = getM02();
            return;
         case 1:
            rowArrayToPack[0] = getM10();
            rowArrayToPack[1] = getM11();
            rowArrayToPack[2] = getM12();
            return;
         case 2:
            rowArrayToPack[0] = getM20();
            rowArrayToPack[1] = getM21();
            rowArrayToPack[2] = getM22();
            return;
         default:
            throw Matrix3DTools.rowOutOfBoundsException(2, row);
      }
   }

   /**
    * Packs a row of this matrix into a 3D tuple.
    *
    * @param row       the index of the row to pack.
    * @param rowToPack the array in which the row of this matrix is stored. Modified.
    * @throws ArrayIndexOutOfBoundsException if {@code row} &notin; [0, 2].
    */
   default void getRow(int row, Tuple3DBasics rowToPack)
   {
      switch (row)
      {
         case 0:
            rowToPack.setX(getM00());
            rowToPack.setY(getM01());
            rowToPack.setZ(getM02());
            return;
         case 1:
            rowToPack.setX(getM10());
            rowToPack.setY(getM11());
            rowToPack.setZ(getM12());
            return;
         case 2:
            rowToPack.setX(getM20());
            rowToPack.setY(getM21());
            rowToPack.setZ(getM22());
            return;
         default:
            throw Matrix3DTools.rowOutOfBoundsException(2, row);
      }
   }

   /**
    * Tests if at least one element of this matrix is equal to {@linkplain Double#NaN}.
    *
    * @return {@code true} if at least one element of this matrix is equal to {@linkplain Double#NaN},
    *         {@code false} otherwise.
    */
   default boolean containsNaN()
   {
      return EuclidCoreTools.containsNaN(getM00(), getM01(), getM02(), getM10(), getM11(), getM12(), getM20(), getM21(), getM22());
   }

   /**
    * Computes the determinant of this matrix.
    *
    * @return the determinant of this matrix.
    */
   default double determinant()
   {
      return Matrix3DFeatures.determinant(getM00(), getM01(), getM02(), getM10(), getM11(), getM12(), getM20(), getM21(), getM22());
   }

   /**
    * Asserts that this matrix is a rotation matrix.
    * <p>
    * This matrix is a rotation matrix if:
    * <ul>
    * <li>the length of each row vector is equal to 1.0 +/-
    * {@link Matrix3DFeatures#EPS_CHECK_ROTATION},
    * <li>the dot product of each pair of row vectors is equal to 0.0 +/-
    * {@link Matrix3DFeatures#EPS_CHECK_ROTATION},
    * <li>the determinant of the matrix is equal to 1.0 +/-
    * {@link Matrix3DFeatures#EPS_CHECK_ROTATION}.
    * </ul>
    * </p>
    *
    * @throws NotARotationMatrixException if the matrix is not a rotation matrix.
    */
   default void checkIfRotationMatrix()
   {
      if (!isRotationMatrix())
         throw new NotARotationMatrixException(this);
   }

   /**
    * Asserts that this matrix describes transformation in the XY plane.
    * <p>
    * This matrix is considered to be a 2D transformation in the XY plane if:
    * <ul>
    * <li>the last diagonal coefficient m22 is equal to 1.0 +/- {@link Matrix3DFeatures#EPS_CHECK_2D},
    * <li>the coefficients {@code m20}, {@code m02}, {@code m21}, and {@code m12} are equal to 0.0 +/-
    * {@link Matrix3DFeatures#EPS_CHECK_2D}.
    * </ul>
    * </p>
    *
    * @throws NotAMatrix2DException if the matrix represents a 3D transformation.
    */
   default void checkIfMatrix2D()
   {
      if (!isMatrix2D())
         throw new NotAMatrix2DException(this);
   }

   /**
    * Tests if this matrix is equal to the identity matrix.
    * <p>
    * The assertion is done on a per coefficient basis using
    * {@link Matrix3DFeatures#EPS_CHECK_IDENTITY} as the tolerance.
    * </p>
    *
    * @param epsilon the tolerance as shown above.
    * @return {@code true} if the given matrix is considered to be equal to the identity matrix,
    *         {@code false} otherwise.
    */
   default boolean isIdentity()
   {
      return isIdentity(Matrix3DFeatures.EPS_CHECK_IDENTITY);
   }

   /**
    * Tests if this matrix is equal to the identity matrix.
    * <p>
    * The assertion is done on a per coefficient basis using {@code epsilon} as the tolerance.
    * </p>
    *
    * @param epsilon the tolerance as shown above.
    * @return {@code true} if the given matrix is considered to be equal to the identity matrix,
    *         {@code false} otherwise.
    */
   default boolean isIdentity(double epsilon)
   {
      return Matrix3DFeatures.isIdentity(getM00(), getM01(), getM02(), getM10(), getM11(), getM12(), getM20(), getM21(), getM22(), epsilon);
   }

   /**
    * Tests if this matrix is equal to the zero matrix.
    * <p>
    * The assertion is done on a per coefficient basis using
    * {@link Matrix3DFeatures#EPS_CHECK_IDENTITY} as the tolerance.
    * </p>
    *
    * @param epsilon the tolerance as shown above.
    * @return {@code true} if the given matrix is considered to be equal to the zero matrix,
    *         {@code false} otherwise.
    */
   default boolean isZero()
   {
      return isZero(Matrix3DFeatures.EPS_CHECK_IDENTITY);
   }

   /**
    * Tests if this matrix is equal to the zero matrix.
    * <p>
    * The assertion is done on a per coefficient basis using {@code epsilon} as the tolerance.
    * </p>
    *
    * @param epsilon the tolerance as shown above.
    * @return {@code true} if the given matrix is considered to be equal to the zero matrix,
    *         {@code false} otherwise.
    */
   default boolean isZero(double epsilon)
   {
      return Matrix3DFeatures.isZero(getM00(), getM01(), getM02(), getM10(), getM11(), getM12(), getM20(), getM21(), getM22(), epsilon);
   }

   /**
    * Tests if this matrix is a rotation matrix.
    * <p>
    * This matrix is a rotation matrix if:
    * <ul>
    * <li>the length of each row vector is equal to 1.0 +/-
    * {@link Matrix3DFeatures#EPS_CHECK_ROTATION},
    * <li>the dot product of each pair of row vectors is equal to 0.0 +/-
    * {@link Matrix3DFeatures#EPS_CHECK_ROTATION},
    * <li>the determinant of the matrix is equal to 1.0 +/-
    * {@link Matrix3DFeatures#EPS_CHECK_ROTATION}.
    * </ul>
    * </p>
    *
    * @return {@code true} if this matrix is a rotation matrix, {@code false} otherwise.
    */
   default boolean isRotationMatrix()
   {
      return isRotationMatrix(Matrix3DFeatures.EPS_CHECK_ROTATION);
   }

   /**
    * Tests if this matrix is a rotation matrix given a tolerance {@code epsilon}.
    * <p>
    * This matrix is a rotation matrix if:
    * <ul>
    * <li>the length of each row vector is equal to 1.0 +/- {@code epsilon},
    * <li>the dot product of each pair of row vectors is equal to 0.0 +/- {@code epsilon},
    * <li>the determinant of the matrix is equal to 1.0 +/- {@code epsilon}.
    * </ul>
    * </p>
    *
    * @param epsilon the tolerance to use.
    * @return {@code true} if this matrix is a rotation matrix, {@code false} otherwise.
    */
   default boolean isRotationMatrix(double epsilon)
   {
      return Matrix3DFeatures.isRotationMatrix(getM00(), getM01(), getM02(), getM10(), getM11(), getM12(), getM20(), getM21(), getM22(), epsilon);
   }

   /**
    * Tests if this matrix describes transformation in the XY plane.
    * <p>
    * This matrix is considered to be a 2D transformation in the XY plane if:
    * <ul>
    * <li>the last diagonal coefficient m22 is equal to 1.0 +/- {@link Matrix3DFeatures#EPS_CHECK_2D},
    * <li>the coefficients {@code m20}, {@code m02}, {@code m21}, and {@code m12} are equal to 0.0 +/-
    * {@link Matrix3DFeatures#EPS_CHECK_2D}.
    * </ul>
    * </p>
    *
    * @return {@code true} if this matrix describes a 2D transformation in the XY plane, {@code false}
    *         otherwise.
    */
   default boolean isMatrix2D()
   {
      return isMatrix2D(Matrix3DFeatures.EPS_CHECK_2D);
   }

   /**
    * Tests if this matrix describes transformation in the XY plane.
    * <p>
    * This matrix is considered to be a 2D transformation in the XY plane if:
    * <ul>
    * <li>the last diagonal coefficient m22 is equal to 1.0 +/- {@code epsilon},
    * <li>the coefficients {@code m20}, {@code m02}, {@code m21}, and {@code m12} are equal to 0.0 +/-
    * {@code epsilon}.
    * </ul>
    * </p>
    *
    * @param epsilon the tolerance to use.
    * @return {@code true} if this matrix describes a 2D transformation in the XY plane, {@code false}
    *         otherwise.
    */
   default boolean isMatrix2D(double epsilon)
   {
      return Matrix3DFeatures.isMatrix2D(getM00(), getM01(), getM02(), getM10(), getM11(), getM12(), getM20(), getM21(), getM22(), epsilon);
   }

   /**
    * Tests if this matrix is skew symmetric:
    *
    * <pre>
    *     |  0 -z  y |
    * m = |  z  0 -x |
    *     | -y  x  0 |
    * </pre>
    * <p>
    * This matrix is considered to be skew symmetric if:
    * <ul>
    * <li>each diagonal coefficient is equal to 0.0 +/- {@link Matrix3DFeatures#EPS_CHECK_SKEW},
    * <li>the sums of each pair of cross-diagonal coefficients ({@code m10}, {@code m01}),
    * ({@code m12}, {@code m21}), and ({@code m20}, {@code m02}) are equal to 0.0 +/-
    * {@link Matrix3DFeatures#EPS_CHECK_SKEW}.
    * </ul>
    * </p>
    *
    * @return {@code true} if the matrix is skew symmetric, {@code false} otherwise.
    */
   default boolean isMatrixSkewSymmetric()
   {
      return isMatrixSkewSymmetric(Matrix3DFeatures.EPS_CHECK_SKEW);
   }

   /**
    * Tests if this matrix is skew symmetric:
    *
    * <pre>
    *     |  0 -z  y |
    * m = |  z  0 -x |
    *     | -y  x  0 |
    * </pre>
    * <p>
    * This matrix is considered to be skew symmetric if:
    * <ul>
    * <li>each diagonal coefficient is equal to 0.0 +/- {@code epsilon},
    * <li>the sums of each pair of cross-diagonal coefficients ({@code m10}, {@code m01}),
    * ({@code m12}, {@code m21}), and ({@code m20}, {@code m02}) are equal to 0.0 +/- {@code epsilon}.
    * </ul>
    * </p>
    *
    * @param epsilon the tolerance to use.
    * @return {@code true} if the matrix is skew symmetric, {@code false} otherwise.
    */
   default boolean isMatrixSkewSymmetric(double epsilon)
   {
      return Matrix3DFeatures.isMatrixSkewSymmetric(getM00(), getM01(), getM02(), getM10(), getM11(), getM12(), getM20(), getM21(), getM22(), epsilon);
   }

   /**
    * Transforms the given tuple by this matrix.
    * <p>
    * tupleToTransform = this * tupleToTransform
    * </p>
    *
    * @param tupleToTransform the tuple to transform. Modified.
    */
   default void transform(Tuple3DBasics tupleToTransform)
   {
      transform(tupleToTransform, tupleToTransform);
   }

   /**
    * Transforms the given tuple {@code tupleOriginal} by this matrix and stores the result in
    * {@code tupleTransformed}.
    * <p>
    * tupleTransformed = this * tupleOriginal
    * </p>
    *
    * @param tupleOriginal    the tuple to transform. Not modified.
    * @param tupleTransformed the tuple to store the result. Modified.
    */
   default void transform(Tuple3DReadOnly tupleOriginal, Tuple3DBasics tupleTransformed)
   {
      Matrix3DTools.transform(this, tupleOriginal, tupleTransformed);
   }

   /**
    * Transforms the given tuple by this matrix and adds the result to the tuple.
    * <p>
    * tupleToTransform = tupleToTransform + this * tupleToTransform
    * </p>
    *
    * @param tupleToTransform the tuple to transform. Modified.
    */
   default void addTransform(Tuple3DBasics tupleToTransform)
   {
      addTransform(tupleToTransform, tupleToTransform);
   }

   /**
    * Transforms the given tuple {@code tupleOriginal} by this matrix and adds the result to
    * {@code tupleTransformed}.
    * <p>
    * tupleTransformed = tupleTransformed + this * tupleOriginal
    * </p>
    *
    * @param tupleOriginal    the tuple to transform. Not modified.
    * @param tupleTransformed the tuple to add the result to. Modified.
    */
   default void addTransform(Tuple3DReadOnly tupleOriginal, Tuple3DBasics tupleTransformed)
   {
      Matrix3DTools.addTransform(this, tupleOriginal, tupleTransformed);
   }

   /**
    * Transforms the given tuple by this matrix.
    * <p>
    * tupleToTransform = this * tupleToTransform
    * </p>
    *
    * @param tupleToTransform the tuple to transform. Modified.
    * @throws NotAMatrix2DException if this matrix does not represent a transformation in the XY plane.
    */
   default void transform(Tuple2DBasics tupleToTransform)
   {
      transform(tupleToTransform, tupleToTransform, true);
   }

   /**
    * Transforms the given tuple {@code tupleOriginal} by this matrix and stores the result in
    * {@code tupleTransformed}.
    * <p>
    * tupleTransformed = this * tupleOriginal
    * </p>
    *
    * @param tupleOriginal    the tuple to transform. Not modified.
    * @param tupleTransformed the tuple to store the result. Modified.
    * @throws NotAMatrix2DException if this matrix does not represent a transformation in the XY plane.
    */
   default void transform(Tuple2DReadOnly tupleOriginal, Tuple2DBasics tupleTransformed)
   {
      transform(tupleOriginal, tupleTransformed, true);
   }

   /**
    * Transforms the given tuple by this matrix.
    * <p>
    * tupleToTransform = this * tupleToTransform
    * </p>
    *
    * @param tupleToTransform          the tuple to transform. Modified.
    * @param checkIfTransformInXYPlane whether this method should assert that this matrix represents a
    *                                  transformation in the XY plane.
    * @throws NotAMatrix2DException if {@code checkIfTransformInXYPlane == true} and this matrix does
    *                               not represent a transformation in the XY plane.
    */
   default void transform(Tuple2DBasics tupleToTransform, boolean checkIfTransformInXYPlane)
   {
      transform(tupleToTransform, tupleToTransform, checkIfTransformInXYPlane);
   }

   /**
    * Transforms the given tuple {@code tupleOriginal} by this matrix and stores the result in
    * {@code tupleTransformed}.
    * <p>
    * tupleTransformed = this * tupleOriginal
    * </p>
    *
    * @param tupleOriginal            the tuple to transform. Not modified.
    * @param tupleTransformed         the tuple to store the result. Modified.
    * @param checkIfRotationInXYPlane whether this method should assert that this matrix represents a
    *                                 transformation in the XY plane.
    * @throws NotAMatrix2DException if {@code checkIfTransformInXYPlane == true} and this matrix does
    *                               not represent a transformation in the XY plane.
    */
   default void transform(Tuple2DReadOnly tupleOriginal, Tuple2DBasics tupleTransformed, boolean checkIfRotationInXYPlane)
   {
      Matrix3DTools.transform(this, tupleOriginal, tupleTransformed, checkIfRotationInXYPlane);
   }

   /**
    * Transforms the given 3D matrix by this matrix.
    * <p>
    * matrixToTransform = this * matrixToTransform * this<sup>-1</sup>
    * </p>
    *
    * @param matrixToTransform the matrix to transform. Modified.
    * @throws SingularMatrixException if this matrix is not invertible.
    */
   default void transform(Matrix3DBasics matrixToTransform)
   {
      transform(matrixToTransform, matrixToTransform);
   }

   /**
    * Transforms the given 3D matrix {@code matrixOriginal} by this matrix and stores the result in
    * {@code matrixTransformed}.
    * <p>
    * matrixTransformed = this * matrixOriginal * this<sup>-1</sup>
    * </p>
    *
    * @param matrixOriginal    the matrix to transform. Not modified.
    * @param matrixTransformed the matrix in which the result is stored. Modified.
    * @throws SingularMatrixException if this matrix is not invertible.
    */
   default void transform(Matrix3DReadOnly matrixOriginal, Matrix3DBasics matrixTransformed)
   {
      Matrix3DTools.transform(this, matrixOriginal, matrixTransformed);
   }

   /**
    * Transforms the vector part of the given 4D vector.
    * <p>
    * vectorToTransform.s = vectorToTransform.s <br>
    * vectorToTransform.xyz = this * vectorToTransform.xyz
    * </p>
    *
    * @param vectorToTransform the vector to transform. Modified.
    */
   default void transform(Vector4DBasics vectorToTransform)
   {
      transform(vectorToTransform, vectorToTransform);
   }

   /**
    * Transforms the vector part of the given 4D vector {@code vectorOriginal} and stores the result
    * into {@code vectorTransformed}.
    * <p>
    * vectorTransformed.s = vectorOriginal.s <br>
    * vectorTransformed.xyz = this * vectorOriginal.xyz
    * </p>
    *
    * @param vectorOriginal    the vector to transform. Not modified.
    * @param vectorTransformed the vector in which the result is stored. Modified.
    */
   default void transform(Vector4DReadOnly vectorOriginal, Vector4DBasics vectorTransformed)
   {
      Matrix3DTools.transform(this, vectorOriginal, vectorTransformed);
   }

   /**
    * Performs the inverse of the transform to the given tuple by this matrix.
    * <p>
    * tupleToTransform = this<sup>-1</sup> * tupleToTransform
    * </p>
    *
    * @param tupleToTransform the tuple to transform. Modified.
    * @throws SingularMatrixException if this matrix is not invertible.
    */
   default void inverseTransform(Tuple3DBasics tupleToTransform)
   {
      inverseTransform(tupleToTransform, tupleToTransform);
   }

   /**
    * Performs the inverse of the transform to the given tuple {@code tupleOriginal} by this matrix and
    * stores the result in {@code tupleTransformed}.
    * <p>
    * tupleTransformed = this<sup>-1</sup> * tupleOriginal
    * </p>
    *
    * @param tupleOriginal    the tuple to transform. Not modified.
    * @param tupleTransformed the tuple in which the result is stored. Modified.
    * @throws SingularMatrixException if this matrix is not invertible.
    */
   default void inverseTransform(Tuple3DReadOnly tupleOriginal, Tuple3DBasics tupleTransformed)
   {
      Matrix3DTools.inverseTransform(this, tupleOriginal, tupleTransformed);
   }

   /**
    * Performs the inverse of the transform to the given tuple by this matrix.
    * <p>
    * tupleToTransform = this<sup>-1</sup> * tupleToTransform
    * </p>
    *
    * @param tupleToTransform the tuple to transform. Modified.
    * @throws NotAMatrix2DException   if this matrix does not represent a transformation in the XY
    *                                 plane.
    * @throws SingularMatrixException if this matrix is not invertible.
    */
   default void inverseTransform(Tuple2DBasics tupleToTransform)
   {
      inverseTransform(tupleToTransform, tupleToTransform, true);
   }

   /**
    * Performs the inverse of the transform to the given tuple {@code tupleOriginal} by this matrix and
    * stores the result in {@code tupleTransformed}.
    * <p>
    * tupleTransformed = this<sup>-1</sup> * tupleOriginal
    * </p>
    *
    * @param tupleOriginal    the tuple to transform. Not modified.
    * @param tupleTransformed the tuple in which the result is stored. Modified.
    * @throws NotAMatrix2DException   if this matrix does not represent a transformation in the XY
    *                                 plane.
    * @throws SingularMatrixException if this matrix is not invertible.
    */
   default void inverseTransform(Tuple2DReadOnly tupleOriginal, Tuple2DBasics tupleTransformed)
   {
      inverseTransform(tupleOriginal, tupleTransformed, true);
   }

   /**
    * Performs the inverse of the transform to the given tuple by this matrix.
    * <p>
    * tupleToTransform = this<sup>-1</sup> * tupleToTransform
    * </p>
    *
    * @param tupleToTransform          the tuple to transform. Modified.
    * @param checkIfTransformInXYPlane whether this method should assert that this matrix represents a
    *                                  transformation in the XY plane.
    * @throws NotAMatrix2DException   if {@code checkIfTransformInXYPlane == true} and this matrix does
    *                                 not represent a transformation in the XY plane.
    * @throws SingularMatrixException if this matrix is not invertible.
    */
   default void inverseTransform(Tuple2DBasics tupleToTransform, boolean checkIfTransformInXYPlane)
   {
      inverseTransform(tupleToTransform, tupleToTransform, checkIfTransformInXYPlane);
   }

   /**
    * Performs the inverse of the transform to the given tuple {@code tupleOriginal} by this matrix and
    * stores the result in {@code tupleTransformed}.
    * <p>
    * tupleTransformed = this<sup>-1</sup> * tupleOriginal
    * </p>
    *
    * @param tupleOriginal             the tuple to transform. Not modified.
    * @param tupleTransformed          the tuple in which the result is stored. Modified.
    * @param checkIfTransformInXYPlane whether this method should assert that this matrix represents a
    *                                  transformation in the XY plane.
    * @throws NotAMatrix2DException   if {@code checkIfTransformInXYPlane == true} and this matrix does
    *                                 not represent a transformation in the XY plane.
    * @throws SingularMatrixException if this matrix is not invertible.
    */
   default void inverseTransform(Tuple2DReadOnly tupleOriginal, Tuple2DBasics tupleTransformed, boolean checkIfTransformInXYPlane)
   {
      Matrix3DTools.inverseTransform(this, tupleOriginal, tupleTransformed, checkIfTransformInXYPlane);
   }

   /**
    * Performs the inverse of the transforms to the given 3D matrix {@code matrixOriginal} by this
    * matrix.
    * <p>
    * matrixToTransform = this<sup>-1</sup> * matrixToTransform * this
    * </p>
    *
    * @param matrixToTransform the matrix to transform. Not modified.
    * @throws SingularMatrixException if this matrix is not invertible.
    */
   default void inverseTransform(Matrix3DBasics matrixToTransform)
   {
      inverseTransform(matrixToTransform, matrixToTransform);
   }

   /**
    * Performs the inverse of the transforms to the given 3D matrix {@code matrixOriginal} by this
    * matrix and stores the result in {@code matrixTransformed}.
    * <p>
    * matrixTransformed = this<sup>-1</sup> * matrixOriginal * this
    * </p>
    *
    * @param matrixOriginal    the matrix to transform. Not modified.
    * @param matrixTransformed the matrix in which the result is stored. Modified.
    * @throws SingularMatrixException if this matrix is not invertible.
    */
   default void inverseTransform(Matrix3DReadOnly matrixOriginal, Matrix3DBasics matrixTransformed)
   {
      Matrix3DTools.inverseTransform(this, matrixOriginal, matrixTransformed);
   }

   /**
    * Performs the inverse of the transform to the vector part the given 4D vector by this matrix.
    * <p>
    * vectorToTransform.s = vectorToTransform.s <br>
    * vectorToTransform.xyz = this<sup>-1</sup> * vectorToTransform.xyz
    * </p>
    *
    * @param vectorToTransform the vector to transform. Modified.
    * @throws SingularMatrixException if this matrix is not invertible.
    */
   default void inverseTransform(Vector4DBasics vectorToTransform)
   {
      inverseTransform(vectorToTransform, vectorToTransform);
   }

   /**
    * Performs the inverse of the transform to the vector part the given 4D vector
    * {@code vectorOriginal} by this matrix and stores the result in {@code vectorTransformed}.
    * <p>
    * vectorTransformed.s = vectorOriginal.s <br>
    * vectorTransformed.xyz = this<sup>-1</sup> * vectorOriginal.xyz
    * </p>
    *
    * @param vectorOriginal    the vector to transform. Not modified.
    * @param vectorTransformed the vector in which the result is stored. Modified.
    * @throws SingularMatrixException if this matrix is not invertible.
    */
   default void inverseTransform(Vector4DReadOnly vectorOriginal, Vector4DBasics vectorTransformed)
   {
      Matrix3DTools.inverseTransform(this, vectorOriginal, vectorTransformed);
   }

   /**
    * Tests on a per component basis if this matrix is exactly equal to {@code other}.
    * <p>
    * The method returns {@code false} if the given matrix is {@code null}.
    * </p>
    *
    * @param other the other matrix to compare against this. Not modified.
    * @return {@code true} if the two matrices are exactly equal component-wise, {@code false}
    *         otherwise.
    */
   default boolean equals(Matrix3DReadOnly other)
   {
      return Matrix3DFeatures.equals(this, other);
   }

   /**
    * Tests on a per coefficient basis if this matrix is equal to the given {@code other} to an
    * {@code epsilon}.
    *
    * @param other   the other matrix to compare against this. Not modified.
    * @param epsilon the tolerance to use when comparing each component.
    * @return {@code true} if the two matrices are equal, {@code false} otherwise.
    */
   default boolean epsilonEquals(Matrix3DReadOnly other, double epsilon)
   {
      if (!EuclidCoreTools.epsilonEquals(getM00(), other.getM00(), epsilon))
         return false;
      if (!EuclidCoreTools.epsilonEquals(getM01(), other.getM01(), epsilon))
         return false;
      if (!EuclidCoreTools.epsilonEquals(getM02(), other.getM02(), epsilon))
         return false;
      if (!EuclidCoreTools.epsilonEquals(getM10(), other.getM10(), epsilon))
         return false;
      if (!EuclidCoreTools.epsilonEquals(getM11(), other.getM11(), epsilon))
         return false;
      if (!EuclidCoreTools.epsilonEquals(getM12(), other.getM12(), epsilon))
         return false;
      if (!EuclidCoreTools.epsilonEquals(getM20(), other.getM20(), epsilon))
         return false;
      if (!EuclidCoreTools.epsilonEquals(getM21(), other.getM21(), epsilon))
         return false;
      if (!EuclidCoreTools.epsilonEquals(getM22(), other.getM22(), epsilon))
         return false;

      return true;
   }
}