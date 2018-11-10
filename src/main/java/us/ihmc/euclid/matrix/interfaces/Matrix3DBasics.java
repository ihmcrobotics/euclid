package us.ihmc.euclid.matrix.interfaces;

import us.ihmc.euclid.exceptions.SingularMatrixException;
import us.ihmc.euclid.interfaces.Transformable;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tools.Matrix3DTools;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.Vector4DBasics;
import us.ihmc.euclid.tuple4D.interfaces.Vector4DReadOnly;

/**
 * Write and read interface for generic matrix 3D.
 * <p>
 * In contrast to {@link CommonMatrix3DBasics}, this interface implements the features and algebra
 * tools for a general matrix 3D. A matrix 3D implementing this interface cannot guarantee any
 * properties, e.g. each individual element is accessible and modifiable by user.
 * </p>
 * 
 * @author Sylvain Bertrand
 */
public interface Matrix3DBasics extends CommonMatrix3DBasics, Transformable
{
   /**
    * Sets the value of the 1st row 1st column component.
    *
    * @param m00 the new value of the component.
    */
   void setM00(double m00);

   /**
    * Sets the value of the 1st row 2nd column component.
    *
    * @param m01 the new value of the component.
    */
   void setM01(double m01);

   /**
    * Sets the value of the 1st row 3rd column component.
    *
    * @param m02 the new value of the component.
    */
   void setM02(double m02);

   /**
    * Sets the value of the 2nd row 1st column component.
    *
    * @param m10 the new value of the component.
    */
   void setM10(double m10);

   /**
    * Sets the value of the 2nd row 2nd column component.
    *
    * @param m11 the new value of the component.
    */
   void setM11(double m11);

   /**
    * Sets the value of the 2nd row 3rd column component.
    *
    * @param m12 the new value of the component.
    */
   void setM12(double m12);

   /**
    * Sets the value of the 3rd row 1st column component.
    *
    * @param m20 the new value of the component.
    */
   void setM20(double m20);

   /**
    * Sets the value of the 3rd row 2nd column component.
    *
    * @param m21 the new value of the component.
    */
   void setM21(double m21);

   /**
    * Sets the value of the 3rd row 3rd column component.
    *
    * @param m22 the new value of the component.
    */
   void setM22(double m22);

   /**
    * Sets the value of the component of this matrix located by its row and column indices.
    *
    * @param row the index of the component's row.
    * @param column the index of the component's column.
    * @param value the new value of the component.
    * @throws ArrayIndexOutOfBoundsException if {@code row} &notin; [0, 2] or {@code column} &notin;
    *            [0, 2]
    */
   default void setElement(int row, int column, double value)
   {
      switch (row)
      {
      case 0:
         switch (column)
         {
         case 0:
            setM00(value);
            return;
         case 1:
            setM01(value);
            return;
         case 2:
            setM02(value);
            return;
         default:
            throw Matrix3DTools.columnOutOfBoundsException(2, column);
         }

      case 1:
         switch (column)
         {
         case 0:
            setM10(value);
            return;
         case 1:
            setM11(value);
            return;
         case 2:
            setM12(value);
            return;
         default:
            throw Matrix3DTools.columnOutOfBoundsException(2, column);
         }

      case 2:
         switch (column)
         {
         case 0:
            setM20(value);
            return;
         case 1:
            setM21(value);
            return;
         case 2:
            setM22(value);
            return;
         default:
            throw Matrix3DTools.columnOutOfBoundsException(2, column);
         }

      default:
         throw Matrix3DTools.rowOutOfBoundsException(2, row);
      }
   }

   /** {@inheritDoc} */
   @Override
   default void set(double m00, double m01, double m02, double m10, double m11, double m12, double m20, double m21, double m22)
   {
      setM00(m00);
      setM01(m01);
      setM02(m02);

      setM10(m10);
      setM11(m11);
      setM12(m12);

      setM20(m20);
      setM21(m21);
      setM22(m22);
   }

   /**
    * Sets all the coefficients of this matrix to zero.
    */
   @Override
   default void setToZero()
   {
      setM00(0.0);
      setM01(0.0);
      setM02(0.0);

      setM10(0.0);
      setM11(0.0);
      setM12(0.0);

      setM20(0.0);
      setM21(0.0);
      setM22(0.0);
   }

   /**
    * Transposes this matrix: m = m<sup>T</sup>.
    */
   default void transpose()
   {
      set(getM00(), getM10(), getM20(), getM01(), getM11(), getM21(), getM02(), getM12(), getM22());
   }

   /**
    * Sets this matrix to be equal to its outer-product.
    * <p>
    * this = this * this<sup>T<sup>
    * </p>
    */
   default void multiplyOuter()
   {
      Matrix3DTools.multiplyTransposeRight(this, this, this);
   }

   /**
    * Invert this matrix.
    * <p>
    * this = this<sup>-1</sup>
    * </p>
    *
    * @throws SingularMatrixException if the matrix is not invertible.
    */
   default void invert()
   {
      boolean success = Matrix3DTools.invert(this);
      if (!success)
         throw new SingularMatrixException(this);
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
    * Negates each component of this matrix.
    */
   default void negate()
   {
      setM00(-getM00());
      setM01(-getM01());
      setM02(-getM02());

      setM10(-getM10());
      setM11(-getM11());
      setM12(-getM12());

      setM20(-getM20());
      setM21(-getM21());
      setM22(-getM22());
   }

   /**
    * Converts a vector to tilde form (matrix implementation of cross product).
    *
    * <pre>
    *        /  0 -z  y \
    * this = |  z  0 -x |
    *        \ -y  x  0 /
    * </pre>
    *
    * @param tuple the tuple to use to create its tilde form. Not modified.
    */
   default void setToTildeForm(Tuple3DReadOnly tuple)
   {
      double x = tuple.getX();
      double y = tuple.getY();
      double z = tuple.getZ();
      setM00(0.0);
      setM01(-z);
      setM02(y);

      setM10(z);
      setM11(0.0);
      setM12(-x);

      setM20(-y);
      setM21(x);
      setM22(0.0);
   }

   /**
    * Sets this matrix to be a diagonal matrix as follows:
    * 
    * <pre>
    *        / m00  0   0  \
    * this = |  0  m11  0  |
    *        \  0   0  m22 /
    * </pre>
    * 
    * @param m00 the first diagonal element.
    * @param m11 the second diagonal element.
    * @param m22 the third diagonal element.
    */
   default void setToDiagonal(double m00, double m11, double m22)
   {
      set(m00, 0.0, 0.0, 0.0, m11, 0.0, 0.0, 0.0, m22);
   }

   /**
    * Sets this matrix to be a diagonal matrix as follows:
    * 
    * <pre>
    *        / x 0 0 \
    * this = | 0 y 0 |
    *        \ 0 0 z /
    * </pre>
    * 
    * where x, y, and z are the components of the given tuple.
    * 
    * @param tuple the tuple used to set this matrix diagonal elements. Not modified.
    */
   default void setToDiagonal(Tuple3DReadOnly tuple)
   {
      setToDiagonal(tuple.getX(), tuple.getY(), tuple.getZ());
   }

   /**
    * Sets this matrix to be equal to the outer-product of {@code other}.
    * <p>
    * this = other * other<sup>T<sup>
    * </p>
    * 
    * @param other the other matrix used for this operation. Not modified.
    */
   default void setAndMultiplyOuter(Matrix3DReadOnly other)
   {
      set(other);
      multiplyOuter();
   }

   /**
    * Set this matrix to the inverse of the other matrix.
    * <p>
    * this = other<sup>-1</sup>
    * </p>
    *
    * @param other the other matrix. Not modified.
    * @throws SingularMatrixException if the matrix is not invertible.
    */
   default void setAndInvert(Matrix3DReadOnly other)
   {
      set(other);
      invert();
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
    * Sets this matrix to equal the other matrix and then transposes this.
    * <p>
    * this = other<sup>T</sup>
    * </p>
    *
    * @param other the other matrix used to update this matrix. Not modified.
    */
   default void setAndTranspose(Matrix3DReadOnly other)
   {
      set(other);
      transpose();
   }

   /**
    * Sets this matrix to {@code other} and then calls {@link #negate()}.
    *
    * @param other the other matrix used to update this matrix. Not modified.
    */
   default void setAndNegate(Matrix3DReadOnly other)
   {
      set(other);
      negate();
   }

   /**
    * Sets this matrix to represent to represent a counter clockwise rotation around the z-axis of
    * an angle {@code yaw}.
    *
    * <pre>
    *        / cos(yaw) -sin(yaw) 0 \
    * this = | sin(yaw)  cos(yaw) 0 |
    *        \    0         0     1 /
    * </pre>
    *
    * @param yaw the angle to rotate about the z-axis.
    */
   default void setToYawMatrix(double yaw)
   {
      double sinYaw = Math.sin(yaw);
      double cosYaw = Math.cos(yaw);

      setM00(cosYaw);
      setM01(-sinYaw);
      setM02(0.0);

      setM10(sinYaw);
      setM11(cosYaw);
      setM12(0.0);

      setM20(0.0);
      setM21(0.0);
      setM22(1.0);
   }

   /**
    * Sets this matrix to represent a counter clockwise rotation around the y-axis of an angle
    * {@code pitch}.
    *
    * <pre>
    *        /  cos(pitch) 0 sin(pitch) \
    * this = |      0      1     0      |
    *        \ -sin(pitch) 0 cos(pitch) /
    * </pre>
    *
    * @param pitch the angle to rotate about the y-axis.
    */
   default void setToPitchMatrix(double pitch)
   {
      double sinPitch = Math.sin(pitch);
      double cosPitch = Math.cos(pitch);

      setM00(cosPitch);
      setM01(0.0);
      setM02(sinPitch);

      setM10(0.0);
      setM11(1.0);
      setM12(0.0);

      setM20(-sinPitch);
      setM21(0.0);
      setM22(cosPitch);
   }

   /**
    * Sets this matrix to represent a counter clockwise rotation around the x-axis of an angle
    * {@code roll}.
    *
    * <pre>
    *        / 1     0          0     \
    * this = | 0 cos(roll) -sin(roll) |
    *        \ 0 sin(roll)  cos(roll) /
    * </pre>
    *
    * @param roll the angle to rotate about the x-axis.
    */
   default void setToRollMatrix(double roll)
   {
      double sinRoll = Math.sin(roll);
      double cosRoll = Math.cos(roll);
      setM00(1.0);
      setM01(0.0);
      setM02(0.0);

      setM10(0.0);
      setM11(cosRoll);
      setM12(-sinRoll);

      setM20(0.0);
      setM21(sinRoll);
      setM22(cosRoll);
   }

   /**
    * Sets the {@code row}<sup>th</sup> row components to the values contained in the given array
    * {@code rowArray}.
    *
    * @param row the index of the row to set the values of.
    * @param rowArray the array containing the new values for the row. Not modified.
    * @throws ArrayIndexOutOfBoundsException if {@code row} &notin; [0, 2].
    */
   default void setRow(int row, double rowArray[])
   {
      setRow(row, rowArray[0], rowArray[1], rowArray[2]);
   }

   /**
    * Sets the {@code row}<sup>th</sup> row components to the values contained in the given tuple
    * {@code rowValues}.
    *
    * @param row the index of the row to set the values of.
    * @param rowValues the tuple containing the new values for the row. Not modified.
    * @throws ArrayIndexOutOfBoundsException if {@code row} &notin; [0, 2].
    */
   default void setRow(int row, Tuple3DReadOnly rowValues)
   {
      setRow(row, rowValues.getX(), rowValues.getY(), rowValues.getZ());
   }

   /**
    * Sets the {@code row}<sup>th</sup> row components to the given values.
    *
    * @param row the index of the row to set the values of.
    * @param x the new value of the first component in the row.
    * @param y the new value of the second component in the row.
    * @param z the new value of the third component in the row.
    * @throws ArrayIndexOutOfBoundsException if {@code row} &notin; [0, 2].
    */
   default void setRow(int row, double x, double y, double z)
   {
      switch (row)
      {
      case 0:
         setM00(x);
         setM01(y);
         setM02(z);
         return;

      case 1:
         setM10(x);
         setM11(y);
         setM12(z);
         return;

      case 2:
         setM20(x);
         setM21(y);
         setM22(z);
         return;

      default:
         throw Matrix3DTools.rowOutOfBoundsException(2, row);
      }
   }

   /**
    * Sets the {@code column}<sup>th</sup> column components to the values contained in the given
    * array {@code columnArray}.
    *
    * @param column the index of the column to set the values of.
    * @param columnArray the array containing the new values for the column. Not modified.
    * @throws ArrayIndexOutOfBoundsException if {@code column} &notin; [0, 2].
    */
   default void setColumn(int column, double columnArray[])
   {
      setColumn(column, columnArray[0], columnArray[1], columnArray[2]);
   }

   /**
    * Sets the {@code column}<sup>th</sup> column components to the values contained in the given
    * tuple {@code columnValues}.
    *
    * @param column the index of the column to set the values of.
    * @param columnValues the tuple containing the new values for the column. Not modified.
    * @throws ArrayIndexOutOfBoundsException if {@code column} &notin; [0, 2].
    */
   default void setColumn(int column, Tuple3DReadOnly columnValues)
   {
      setColumn(column, columnValues.getX(), columnValues.getY(), columnValues.getZ());
   }

   /**
    * Sets the {@code column}<sup>th</sup> column components to the given values.
    *
    * @param column the index of the column to set the values of.
    * @param x the new value of the first component in the column.
    * @param y the new value of the second component in the column.
    * @param z the new value of the third component in the column.
    * @throws ArrayIndexOutOfBoundsException if {@code column} &notin; [0, 2].
    */
   default void setColumn(int column, double x, double y, double z)
   {
      switch (column)
      {
      case 0:
         setM00(x);
         setM10(y);
         setM20(z);
         break;

      case 1:
         setM01(x);
         setM11(y);
         setM21(z);
         break;

      case 2:
         setM02(x);
         setM12(y);
         setM22(z);
         break;

      default:
         throw Matrix3DTools.columnOutOfBoundsException(2, column);
      }
   }

   /**
    * Adds the given value to the 1st row 1st column component.
    *
    * @param m00 the value to be added.
    */
   default void addM00(double m00)
   {
      setM00(getM00() + m00);
   }

   /**
    * Adds the given value to the 1st row 2nd column component.
    *
    * @param m01 the value to be added.
    */
   default void addM01(double m01)
   {
      setM01(getM01() + m01);
   }

   /**
    * Adds the given value to the 1st row 3rd column component.
    *
    * @param m02 the value to be added.
    */
   default void addM02(double m02)
   {
      setM02(getM02() + m02);
   }

   /**
    * Adds the given value to the 2nd row 1st column component.
    *
    * @param m10 the value to be added.
    */
   default void addM10(double m10)
   {
      setM10(getM10() + m10);
   }

   /**
    * Adds the given value to the 2nd row 2nd column component.
    *
    * @param m11 the value to be added.
    */
   default void addM11(double m11)
   {
      setM11(getM11() + m11);
   }

   /**
    * Adds the given value to the 2nd row 3rd column component.
    *
    * @param m12 the value to be added.
    */
   default void addM12(double m12)
   {
      setM12(getM12() + m12);
   }

   /**
    * Adds the given value to the 3rd row 1st column component.
    *
    * @param m20 the value to be added.
    */
   default void addM20(double m20)
   {
      setM20(getM20() + m20);
   }

   /**
    * Adds the given value to the 3rd row 2nd column component.
    *
    * @param m21 the value to be added.
    */
   default void addM21(double m21)
   {
      setM21(getM21() + m21);
   }

   /**
    * Adds the given value to the 3rd row 3rd column component.
    *
    * @param m22 the value to be added.
    */
   default void addM22(double m22)
   {
      setM22(getM22() + m22);
   }

   /**
    * Performs a per-component addition onto the coefficients of this matrix.
    * <p>
    * this = this + other
    * </p>
    *
    * @param other the other matrix to use for the addition. Not modified.
    */
   default void add(Matrix3DReadOnly other)
   {
      addM00(other.getM00());
      addM01(other.getM01());
      addM02(other.getM02());

      addM10(other.getM10());
      addM11(other.getM11());
      addM12(other.getM12());

      addM20(other.getM20());
      addM21(other.getM21());
      addM22(other.getM22());
   }

   /**
    * Sets this matrix coefficients to the per-component sum of the two given matrices.
    * <p>
    * this = matrix1 + matrix2
    * </p>
    *
    * @param matrix1 the first term of the addition. Not modified.
    * @param matrix2 the second term of the addition. Not modified.
    */
   default void add(Matrix3DReadOnly matrix1, Matrix3DReadOnly matrix2)
   {
      setM00(matrix1.getM00() + matrix2.getM00());
      setM01(matrix1.getM01() + matrix2.getM01());
      setM02(matrix1.getM02() + matrix2.getM02());

      setM10(matrix1.getM10() + matrix2.getM10());
      setM11(matrix1.getM11() + matrix2.getM11());
      setM12(matrix1.getM12() + matrix2.getM12());

      setM20(matrix1.getM20() + matrix2.getM20());
      setM21(matrix1.getM21() + matrix2.getM21());
      setM22(matrix1.getM22() + matrix2.getM22());
   }

   /**
    * Subtracts the given value to the 1st row 1st column component.
    *
    * @param m00 the value to be subtracted.
    */
   default void subM00(double m00)
   {
      setM00(getM00() - m00);
   }

   /**
    * Subtracts the given value to the 1st row 2nd column component.
    *
    * @param m01 the value to be subtracted.
    */
   default void subM01(double m01)
   {
      setM01(getM01() - m01);
   }

   /**
    * Subtracts the given value to the 1st row 3rd column component.
    *
    * @param m02 the value to be subtracted.
    */
   default void subM02(double m02)
   {
      setM02(getM02() - m02);
   }

   /**
    * Subtracts the given value to the 2nd row 1st column component.
    *
    * @param m10 the value to be subtracted.
    */
   default void subM10(double m10)
   {
      setM10(getM10() - m10);
   }

   /**
    * Subtracts the given value to the 2nd row 2nd column component.
    *
    * @param m11 the value to be subtracted.
    */
   default void subM11(double m11)
   {
      setM11(getM11() - m11);
   }

   /**
    * Subtracts the given value to the 2nd row 3rd column component.
    *
    * @param m12 the value to be subtracted.
    */
   default void subM12(double m12)
   {
      setM12(getM12() - m12);
   }

   /**
    * Subtracts the given value to the 3rd row 1st column component.
    *
    * @param m20 the value to be subtracted.
    */
   default void subM20(double m20)
   {
      setM20(getM20() - m20);
   }

   /**
    * Subtracts the given value to the 3rd row 2nd column component.
    *
    * @param m21 the value to be subtracted.
    */
   default void subM21(double m21)
   {
      setM21(getM21() - m21);
   }

   /**
    * Subtracts the given value to the 3rd row 3rd column component.
    *
    * @param m22 the value to be subtracted.
    */
   default void subM22(double m22)
   {
      setM22(getM22() - m22);
   }

   /**
    * Performs a per-component subtraction onto the coefficients of this matrix.
    * <p>
    * this = this - other
    * </p>
    *
    * @param other the other matrix to use for the subtraction. Not modified.
    */
   default void sub(Matrix3DReadOnly other)
   {
      subM00(other.getM00());
      subM01(other.getM01());
      subM02(other.getM02());

      subM10(other.getM10());
      subM11(other.getM11());
      subM12(other.getM12());

      subM20(other.getM20());
      subM21(other.getM21());
      subM22(other.getM22());
   }

   /**
    * Sets this matrix coefficients to the per-component difference of the two given matrices.
    * <p>
    * this = matrix1 - matrix2
    * </p>
    *
    * @param matrix1 the first term of the addition. Not modified.
    * @param matrix2 the second term of the addition. Not modified.
    */
   default void sub(Matrix3DReadOnly matrix1, Matrix3DReadOnly matrix2)
   {
      setM00(matrix1.getM00() - matrix2.getM00());
      setM01(matrix1.getM01() - matrix2.getM01());
      setM02(matrix1.getM02() - matrix2.getM02());

      setM10(matrix1.getM10() - matrix2.getM10());
      setM11(matrix1.getM11() - matrix2.getM11());
      setM12(matrix1.getM12() - matrix2.getM12());

      setM20(matrix1.getM20() - matrix2.getM20());
      setM21(matrix1.getM21() - matrix2.getM21());
      setM22(matrix1.getM22() - matrix2.getM22());
   }

   /**
    * Sets all the coefficients of this matrix to be equal to {@code scalar}.
    *
    * <pre>
    *        / scalar scalar scalar \
    * this = | scalar scalar scalar |
    *        \ scalar scalar scalar /
    * </pre>
    *
    * @param scalar the scalar value to fill this matrix with.
    */
   default void fill(double scalar)
   {
      setM00(scalar);
      setM01(scalar);
      setM02(scalar);

      setM10(scalar);
      setM11(scalar);
      setM12(scalar);

      setM20(scalar);
      setM21(scalar);
      setM22(scalar);
   }

   /**
    * Scales the value of the 1st row 1st column component.
    *
    * @param scalar the scale factor to apply.
    */
   default void scaleM00(double scalar)
   {
      setM00(scalar * getM00());
   }

   /**
    * Scales the value of the 1st row 2nd column component.
    *
    * @param scalar the scale factor to apply.
    */
   default void scaleM01(double scalar)
   {
      setM01(scalar * getM01());
   }

   /**
    * Scales the value of the 1st row 3rd column component.
    *
    * @param scalar the scale factor to apply.
    */
   default void scaleM02(double scalar)
   {
      setM02(scalar * getM02());
   }

   /**
    * Scales the value of the 2nd row 1st column component.
    *
    * @param scalar the scale factor to apply.
    */
   default void scaleM10(double scalar)
   {
      setM10(scalar * getM10());
   }

   /**
    * Scales the value of the 2nd row 2nd column component.
    *
    * @param scalar the scale factor to apply.
    */
   default void scaleM11(double scalar)
   {
      setM11(scalar * getM11());
   }

   /**
    * Scales the value of the 2nd row 3rd column component.
    *
    * @param scalar the scale factor to apply.
    */
   default void scaleM12(double scalar)
   {
      setM12(scalar * getM12());
   }

   /**
    * Scales the value of the 3rd row 1st column component.
    *
    * @param scalar the scale factor to apply.
    */
   default void scaleM20(double scalar)
   {
      setM20(scalar * getM20());
   }

   /**
    * Scales the value of the 3rd row 2nd column component.
    *
    * @param scalar the scale factor to apply.
    */
   default void scaleM21(double scalar)
   {
      setM21(scalar * getM21());
   }

   /**
    * Scales the value of the 3rd row 3rd column component.
    *
    * @param scalar the scale factor to apply.
    */
   default void scaleM22(double scalar)
   {
      setM22(scalar * getM22());
   }

   /**
    * Performs a per-component scale on this matrix.
    * <p>
    * this = scalar * this
    * </p>
    *
    * @param scalar the scale factor to use on the components of this matrix.
    */
   default void scale(double scalar)
   {
      scaleM00(scalar);
      scaleM01(scalar);
      scaleM02(scalar);

      scaleM10(scalar);
      scaleM11(scalar);
      scaleM12(scalar);

      scaleM20(scalar);
      scaleM21(scalar);
      scaleM22(scalar);
   }

   /**
    * Scales individually each row of this matrix.
    *
    * <pre>
    *        / scalarRow0 * m00 scalarRow0 * m01 scalarRow0 * m02 \
    * this = | scalarRow1 * m10 scalarRow1 * m11 scalarRow1 * m12 |
    *        \ scalarRow2 * m20 scalarRow2 * m21 scalarRow2 * m22 /
    * </pre>
    * <p>
    * This operation is equivalent to pre-multiplying this matrix, i.e. this = D * this, by the
    * following diagonal matrix D:
    *
    * <pre>
    *     / scaleRow0     0         0     \
    * D = |     0     scaleRow1     0     |
    *     \     0         0     scaleRow2 /
    * </pre>
    * </p>
    *
    * @param scalarRow0 the scale factor to use on the components of the 1st row.
    * @param scalarRow1 the scale factor to use on the components of the 2nd row.
    * @param scalarRow2 the scale factor to use on the components of the 3rd row.
    */
   default void scaleRows(double scalarRow0, double scalarRow1, double scalarRow2)
   {
      scaleM00(scalarRow0);
      scaleM01(scalarRow0);
      scaleM02(scalarRow0);

      scaleM10(scalarRow1);
      scaleM11(scalarRow1);
      scaleM12(scalarRow1);

      scaleM20(scalarRow2);
      scaleM21(scalarRow2);
      scaleM22(scalarRow2);
   }

   /**
    * Scales individually each column of this matrix.
    *
    * <pre>
    *        / scalarColumn0 * m00 scalarColumn1 * m01 scalarColumn2 * m02 \
    * this = | scalarColumn0 * m10 scalarColumn1 * m11 scalarColumn2 * m12 |
    *        \ scalarColumn0 * m20 scalarColumn1 * m21 scalarColumn2 * m22 /
    * </pre>
    * <p>
    * This operation is equivalent to multiplying this matrix, i.e. this = this * D, by the
    * following diagonal matrix D:
    *
    * <pre>
    *     / scalarColumn0       0             0       \
    * D = |       0       scalarColumn1       0       |
    *     \       0             0       scalarColumn2 /
    * </pre>
    * </p>
    *
    * @param scalarColumn0 the scale factor to use on the components of the 1st column.
    * @param scalarColumn1 the scale factor to use on the components of the 2nd column.
    * @param scalarColumn2 the scale factor to use on the components of the 3rd column.
    */
   default void scaleColumns(double scalarColumn0, double scalarColumn1, double scalarColumn2)
   {
      scaleM00(scalarColumn0);
      scaleM01(scalarColumn1);
      scaleM02(scalarColumn2);

      scaleM10(scalarColumn0);
      scaleM11(scalarColumn1);
      scaleM12(scalarColumn2);

      scaleM20(scalarColumn0);
      scaleM21(scalarColumn1);
      scaleM22(scalarColumn2);
   }

   /**
    * Scales the components of the {@code row}<sup>th</sup> row of this matrix.
    *
    * @param row the index of the row to scale.
    * @param scalar the scale factor to apply.
    * @throws ArrayIndexOutOfBoundsException if {@code row} &notin; [0, 2].
    */
   default void scaleRow(int row, double scalar)
   {
      switch (row)
      {
      case 0:
         scaleM00(scalar);
         scaleM01(scalar);
         scaleM02(scalar);
         return;

      case 1:
         scaleM10(scalar);
         scaleM11(scalar);
         scaleM12(scalar);
         return;

      case 2:
         scaleM20(scalar);
         scaleM21(scalar);
         scaleM22(scalar);
         return;

      default:
         throw Matrix3DTools.rowOutOfBoundsException(2, row);
      }
   }

   /**
    * Scales the components of the {@code column}<sup>th</sup> column of this matrix.
    *
    * @param column the index of the column to scale.
    * @param scalar the scale factor to apply.
    * @throws ArrayIndexOutOfBoundsException if {@code column} &notin; [0, 2].
    */
   default void scaleColumn(int column, double scalar)
   {
      switch (column)
      {
      case 0:
         scaleM00(scalar);
         scaleM10(scalar);
         scaleM20(scalar);
         break;

      case 1:
         scaleM01(scalar);
         scaleM11(scalar);
         scaleM21(scalar);
         break;

      case 2:
         scaleM02(scalar);
         scaleM12(scalar);
         scaleM22(scalar);
         break;

      default:
         throw Matrix3DTools.columnOutOfBoundsException(2, column);
      }
   }

   /**
    * {@inheritDoc}
    * <p>
    * this = R * this * R<sup>T</sup><br>
    * where 'R' is the 3-by-3 matrix representing the rotation part of the {@code transform}.
    * </p>
    * <p>
    * Note: the transformation of a {@code Matrix3D} strongly differs from the transformation of a
    * {@link RotationMatrix}.
    * </p>
    */
   @Override
   default void applyTransform(Transform transform)
   {
      transform.transform(this);
   }

   /**
    * {@inheritDoc}
    * <p>
    * this = R<sup>T</sup> * this * R<br>
    * where 'R' is the 3-by-3 matrix representing the rotation part of the {@code transform}.
    * </p>
    * <p>
    * Note: the transformation of a {@code Matrix3D} strongly differs from the transformation of a
    * {@link RotationMatrix}.
    * </p>
    */
   @Override
   default void applyInverseTransform(Transform transform)
   {
      transform.inverseTransform(this);
   }

   /**
    * Performs a matrix multiplication on this.
    * <p>
    * this = this * other
    * </p>
    *
    * @param other the other matrix to multiply this by. Not modified.
    */
   default void multiply(Matrix3DReadOnly other)
   {
      Matrix3DTools.multiply(this, other, this);
   }

   /**
    * Performs a matrix multiplication on this.
    * <p>
    * this = this<sup>T</sup> * other
    * </p>
    *
    * @param other the other matrix to multiply this by. Not modified.
    */
   default void multiplyTransposeThis(Matrix3DReadOnly other)
   {
      Matrix3DTools.multiplyTransposeLeft(this, other, this);
   }

   /**
    * Performs a matrix multiplication on this.
    * <p>
    * this = this * other<sup>T</sup>
    * </p>
    *
    * @param other the other matrix to multiply this by. Not modified.
    */
   default void multiplyTransposeOther(Matrix3DReadOnly other)
   {
      Matrix3DTools.multiplyTransposeRight(this, other, this);
   }

   /**
    * Performs a matrix multiplication on this.
    * <p>
    * this = this<sup>T</sup> * other<sup>T</sup>
    * </p>
    *
    * @param other the other matrix to multiply this by. Not modified.
    */
   default void multiplyTransposeBoth(Matrix3DReadOnly other)
   {
      Matrix3DTools.multiplyTransposeBoth(this, other, this);
   }

   /**
    * Performs a matrix multiplication on this.
    * <p>
    * this = this<sup>-1</sup> * other
    * </p>
    *
    * @param other the other matrix to multiply this by. Not modified.
    * @throws SingularMatrixException if {@code this} is not invertible.
    */
   default void multiplyInvertThis(Matrix3DReadOnly other)
   {
      Matrix3DTools.multiplyInvertLeft(this, other, this);
   }

   /**
    * Performs a matrix multiplication on this.
    * <p>
    * this = this * other<sup>-1</sup>
    * </p>
    *
    * @param other the other matrix to multiply this by. Not modified.
    * @throws SingularMatrixException if {@code other} is not invertible.
    */
   default void multiplyInvertOther(Matrix3DReadOnly other)
   {
      Matrix3DTools.multiplyInvertRight(this, other, this);
   }

   /**
    * Performs a matrix multiplication on this.
    * <p>
    * this = this * other<sup>-1</sup>
    * </p>
    * <p>
    * This operation uses the property: <br>
    * R<sup>-1</sup> = R<sup>T</sup> </br>
    * of the rotation matrix preventing to actually compute its inverse.
    * </p>
    *
    * @param other the other matrix to multiply this by. Not modified.
    */
   default void multiplyInvertOther(RotationMatrixReadOnly other)
   {
      Matrix3DTools.multiplyInvertRight(this, other, this);
   }

   /**
    * Performs a matrix multiplication on this.
    * <p>
    * this = this * other<sup>-1</sup>
    * </p>
    * <p>
    * This operation uses the property: <br>
    * (R * S)<sup>-1</sup> = S<sup>-1</sup> * R<sup>T</sup> </br>
    * of the rotation-scale matrix preventing to actually compute its inverse.
    * </p>
    *
    * @param other the other matrix to multiply this by. Not modified.
    */
   default void multiplyInvertOther(RotationScaleMatrixReadOnly other)
   {
      Matrix3DTools.multiplyInvertRight(this, other, this);
   }

   /**
    * Performs a matrix multiplication on this.
    * <p>
    * this = other * this
    * </p>
    *
    * @param other the other matrix to multiply this by. Not modified.
    */
   default void preMultiply(Matrix3DReadOnly other)
   {
      Matrix3DTools.multiply(other, this, this);
   }

   /**
    * Performs a matrix multiplication on this.
    * <p>
    * this = other * this<sup>T</sup>
    * </p>
    *
    * @param other the other matrix to multiply this by. Not modified.
    */
   default void preMultiplyTransposeThis(Matrix3DReadOnly other)
   {
      Matrix3DTools.multiplyTransposeRight(other, this, this);
   }

   /**
    * Performs a matrix multiplication on this.
    * <p>
    * this = other<sup>T</sup> * this
    * </p>
    *
    * @param other the other matrix to multiply this by. Not modified.
    */
   default void preMultiplyTransposeOther(Matrix3DReadOnly other)
   {
      Matrix3DTools.multiplyTransposeLeft(other, this, this);
   }

   /**
    * Performs a matrix multiplication on this.
    * <p>
    * this = other<sup>T</sup> * this<sup>T</sup>
    * </p>
    *
    * @param other the other matrix to multiply this by. Not modified.
    */
   default void preMultiplyTransposeBoth(Matrix3DReadOnly other)
   {
      Matrix3DTools.multiplyTransposeBoth(other, this, this);
   }

   /**
    * Performs a matrix multiplication on this.
    * <p>
    * this = other * this<sup>-1</sup>
    * </p>
    *
    * @param other the other matrix to multiply this by. Not modified.
    * @throws SingularMatrixException if {@code this} is not invertible.
    */
   default void preMultiplyInvertThis(Matrix3DReadOnly other)
   {
      Matrix3DTools.multiplyInvertRight(other, this, this);
   }

   /**
    * Performs a matrix multiplication on this.
    * <p>
    * this = other<sup>-1</sup> * this
    * </p>
    *
    * @param other the other matrix to multiply this by. Not modified.
    * @throws SingularMatrixException if {@code other} is not invertible.
    */
   default void preMultiplyInvertOther(Matrix3DReadOnly other)
   {
      Matrix3DTools.multiplyInvertLeft(other, this, this);
   }

   /**
    * Performs a matrix multiplication on this.
    * <p>
    * this = other<sup>-1</sup> * this
    * </p>
    * <p>
    * This operation uses the property: <br>
    * R<sup>-1</sup> = R<sup>T</sup> </br>
    * of the rotation matrix preventing to actually compute its inverse.
    * </p>
    *
    * @param other the other matrix to multiply this by. Not modified.
    */
   default void preMultiplyInvertOther(RotationMatrixReadOnly other)
   {
      Matrix3DTools.multiplyInvertLeft(other, this, this);
   }

   /**
    * Performs a matrix multiplication on this.
    * <p>
    * this = other<sup>-1</sup> * this
    * </p>
    * <p>
    * This operation uses the property: <br>
    * (R * S)<sup>-1</sup> = S<sup>-1</sup> * R<sup>T</sup> </br>
    * of the rotation-scale matrix preventing to actually compute its inverse.
    * </p>
    *
    * @param other the other matrix to multiply this by. Not modified.
    */
   default void preMultiplyInvertOther(RotationScaleMatrixReadOnly other)
   {
      Matrix3DTools.multiplyInvertLeft(other, this, this);
   }

   /** {@inheritDoc} */
   @Override
   default void transform(Matrix3DReadOnly matrixOriginal, Matrix3DBasics matrixTransformed)
   {
      Matrix3DTools.transform(this, matrixOriginal, matrixTransformed);
   }

   /** {@inheritDoc} */
   @Override
   default void inverseTransform(Tuple3DReadOnly tupleOriginal, Tuple3DBasics tupleTransformed)
   {
      Matrix3DTools.inverseTransform(this, tupleOriginal, tupleTransformed);
   }

   /** {@inheritDoc} */
   @Override
   default void inverseTransform(Tuple2DReadOnly tupleOriginal, Tuple2DBasics tupleTransformed, boolean checkIfTransformInXYPlane)
   {
      Matrix3DTools.inverseTransform(this, tupleOriginal, tupleTransformed, checkIfTransformInXYPlane);
   }

   /** {@inheritDoc} */
   @Override
   default void inverseTransform(Vector4DReadOnly vectorOriginal, Vector4DBasics vectorTransformed)
   {
      Matrix3DTools.inverseTransform(this, vectorOriginal, vectorTransformed);
   }

   /** {@inheritDoc} */
   @Override
   default void inverseTransform(Matrix3DReadOnly matrixOriginal, Matrix3DBasics matrixTransformed)
   {
      Matrix3DTools.inverseTransform(this, matrixOriginal, matrixTransformed);
   }

}
