package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.exceptions.SingularMatrixException;
import us.ihmc.euclid.interfaces.Transformable;
import us.ihmc.euclid.matrix.interfaces.Matrix3DBasics;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.tools.Matrix3DTools;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;

/**
 * Write and read interface for generic matrix 3D expressed in a constant reference frame, i.e. the
 * reference frame of this object cannot be changed via this interface.
 * <p>
 * In addition to representing a {@link Matrix3DBasics}, a {@link ReferenceFrame} is associated to a
 * {@code FixedFrameMatrix3DBasics}. This allows, for instance, to enforce, at runtime, that
 * operations on matrices occur in the same coordinate system.
 * </p>
 * <p>
 * Because a {@code FixedFrameMatrix3DBasics} extends {@code Matrix3DBasics}, it is compatible with
 * methods only requiring {@code Matrix3DBasics}. However, these methods do NOT assert that the
 * operation occur in the proper coordinate system. Use this feature carefully and always prefer
 * using methods requiring {@code FixedFrameMatrix3DBasics}.
 * </p>
 *
 * @author Sylvain Bertrand
 */
public interface FixedFrameMatrix3DBasics extends FrameMatrix3DReadOnly, Matrix3DBasics, Transformable, FixedFrameCommonMatrix3DBasics
{
   /**
    * Sets the 9 coefficients of this matrix to the given ones.
    *
    * @param referenceFrame the reference frame in which the given coefficients are expressed.
    * @param m00            the new 1st row 1st column coefficient for this matrix.
    * @param m01            the new 1st row 2nd column coefficient for this matrix.
    * @param m02            the new 1st row 3rd column coefficient for this matrix.
    * @param m10            the new 2nd row 1st column coefficient for this matrix.
    * @param m11            the new 2nd row 2nd column coefficient for this matrix.
    * @param m12            the new 2nd row 3rd column coefficient for this matrix.
    * @param m20            the new 3rd row 1st column coefficient for this matrix.
    * @param m21            the new 3rd row 2nd column coefficient for this matrix.
    * @param m22            the new 3rd row 3rd column coefficient for this matrix.
    * @throws ReferenceFrameMismatchException if the coefficients are not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default void set(ReferenceFrame referenceFrame, double m00, double m01, double m02, double m10, double m11, double m12, double m20, double m21, double m22)
   {
      checkReferenceFrameMatch(referenceFrame);
      set(m00, m01, m02, m10, m11, m12, m20, m21, m22);
   }

   /**
    * Sets this matrix to {@code matrix3DReadOnly}.
    * <p>
    * If {@code matrix3DReadOnly} is expressed in the frame as {@code this}, then this method is
    * equivalent to {@link #set(ReferenceFrame, Matrix3DReadOnly)}.
    * </p>
    * <p>
    * If {@code matrix3DReadOnly} is expressed in a different frame than {@code this}, then
    * {@code this} is set to {@code matrix3DReadOnly} once transformed to be expressed in
    * {@code this.getReferenceFrame()}.
    * </p>
    *
    * @param referenceFrame   the reference frame in which {@code matrix3DReadOnly} is expressed.
    * @param matrix3DReadOnly the other frame matrix to set this to. Not modified.
    */
   default void setMatchingFrame(ReferenceFrame referenceFrame, Matrix3DReadOnly matrix3DReadOnly)
   {
      Matrix3DBasics.super.set(matrix3DReadOnly);
      referenceFrame.transformFromThisToDesiredFrame(getReferenceFrame(), this);
   }

   /**
    * Sets this matrix to {@code other}.
    * <p>
    * If {@code other} is expressed in the frame as {@code this}, then this method is equivalent to
    * {@link #set(FrameMatrix3DReadOnly)}.
    * </p>
    * <p>
    * If {@code other} is expressed in a different frame than {@code this}, then {@code this} is set to
    * {@code other} once transformed to be expressed in {@code this.getReferenceFrame()}.
    * </p>
    *
    * @param other the other frame matrix to set this to. Not modified.
    */
   default void setMatchingFrame(FrameMatrix3DReadOnly other)
   {
      setMatchingFrame(other.getReferenceFrame(), other);
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
    * @param referenceFrame  the reference frame in which {@code tuple3DReadOnly} is expressed.
    * @param tuple3DReadOnly the tuple to use to create its tilde form. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this.referenceFrame != referenceFrame}.
    */
   default void setToTildeForm(ReferenceFrame referenceFrame, Tuple3DReadOnly tuple3DReadOnly)
   {
      checkReferenceFrameMatch(referenceFrame);
      setToTildeForm(tuple3DReadOnly);
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
    * @param frameTuple3DReadOnly the tuple to use to create its tilde form. Not modified.
    * @throws ReferenceFrameMismatchException if {@code frameTuple3DReadOnly} is not expressed in the
    *                                         same reference frame as {@code this}.
    */
   default void setToTildeForm(FrameTuple3DReadOnly frameTuple3DReadOnly)
   {
      setToTildeForm(frameTuple3DReadOnly.getReferenceFrame(), frameTuple3DReadOnly);
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
    * @param referenceFrame  the reference frame in which {@code tuple3DReadOnly} is expressed.
    * @param tuple3DReadOnly the tuple used to set this matrix diagonal elements. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this.referenceFrame != referenceFrame}.
    */
   default void setToDiagonal(ReferenceFrame referenceFrame, Tuple3DReadOnly tuple3DReadOnly)
   {
      checkReferenceFrameMatch(referenceFrame);
      setToDiagonal(tuple3DReadOnly);
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
    * @param frameTuple3DReadOnly the tuple used to set this matrix diagonal elements. Not modified.
    * @throws ReferenceFrameMismatchException if {@code frameTuple3DReadOnly} is not expressed in the
    *                                         same reference frame as {@code this}.
    */
   default void setToDiagonal(FrameTuple3DReadOnly frameTuple3DReadOnly)
   {
      setToDiagonal(frameTuple3DReadOnly.getReferenceFrame(), frameTuple3DReadOnly);
   }

   /**
    * Sets this matrix to be equal to the outer-product of {@code matrix3DReadOnly}.
    * <p>
    * this = matrix3DReadOnly * matrix3DReadOnly<sup>T<sup>
    * </p>
    *
    * @param referenceFrame   the reference frame in which {@code matrix3DReadOnly} is expressed.
    * @param matrix3DReadOnly the other matrix used for this operation. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this.referenceFrame != referenceFrame}.
    */
   default void setAndMultiplyOuter(ReferenceFrame referenceFrame, Matrix3DReadOnly matrix3DReadOnly)
   {
      checkReferenceFrameMatch(referenceFrame);
      setAndMultiplyOuter(matrix3DReadOnly);
   }

   /**
    * Sets this matrix to be equal to the outer-product of {@code other}.
    * <p>
    * this = other * other<sup>T<sup>
    * </p>
    *
    * @param other the other matrix used for this operation. Not modified.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void setAndMultiplyOuter(FrameMatrix3DReadOnly other)
   {
      setAndMultiplyOuter(other.getReferenceFrame(), other);
   }

   /**
    * Sets this matrix to be equal to the inner-product of {@code matrix3DReadOnly}.
    * <p>
    * this = matrix3DReadOnly<sup>T<sup> * matrix3DReadOnly
    * </p>
    *
    * @param referenceFrame   the reference frame in which {@code matrix3DReadOnly} is expressed.
    * @param matrix3DReadOnly the other matrix used for this operation. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this.referenceFrame != referenceFrame}.
    */
   default void setAndMultiplyInner(ReferenceFrame referenceFrame, Matrix3DReadOnly matrix3DReadOnly)
   {
      checkReferenceFrameMatch(referenceFrame);
      setAndMultiplyInner(matrix3DReadOnly);
   }

   /**
    * Sets this matrix to be equal to the inner-product of {@code other}.
    * <p>
    * this = other<sup>T<sup> * other
    * </p>
    *
    * @param other the other matrix used for this operation. Not modified.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void setAndMultiplyInner(FrameMatrix3DReadOnly other)
   {
      setAndMultiplyInner(other.getReferenceFrame(), other);
   }

   /**
    * Sets this matrix to {@code matrix3DReadOnly} and then calls {@link #negate()}.
    *
    * @param referenceFrame   the reference frame in which {@code matrix3DReadOnly} is expressed.
    * @param matrix3DReadOnly the other matrix used to update this matrix. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this.referenceFrame != referenceFrame}.
    */
   default void setAndNegate(ReferenceFrame referenceFrame, Matrix3DReadOnly matrix3DReadOnly)
   {
      checkReferenceFrameMatch(referenceFrame);
      setAndNegate(matrix3DReadOnly);
   }

   /**
    * Sets this matrix to {@code other} and then calls {@link #negate()}.
    *
    * @param other the other matrix used to update this matrix. Not modified.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void setAndNegate(FrameMatrix3DReadOnly other)
   {
      setAndNegate(other.getReferenceFrame(), other);
   }

   /**
    * Sets the {@code row}<sup>th</sup> row components to the values contained in the given tuple
    * {@code rowValues}.
    *
    * @param row            the index of the row to set the values of.
    * @param referenceFrame the reference frame in which {@code rowValues} is expressed.
    * @param rowValues      the tuple containing the new values for the row. Not modified.
    * @throws ArrayIndexOutOfBoundsException  if {@code row} &notin; [0, 2].
    * @throws ReferenceFrameMismatchException if {@code this.referenceFrame != referenceFrame}.
    */
   default void setRow(int row, ReferenceFrame referenceFrame, Tuple3DReadOnly rowValues)
   {
      checkReferenceFrameMatch(referenceFrame);
      setRow(row, rowValues);
   }

   /**
    * Sets the {@code row}<sup>th</sup> row components to the values contained in the given tuple
    * {@code rowValues}.
    *
    * @param row       the index of the row to set the values of.
    * @param rowValues the tuple containing the new values for the row. Not modified.
    * @throws ArrayIndexOutOfBoundsException  if {@code row} &notin; [0, 2].
    * @throws ReferenceFrameMismatchException if {@code rowValues} is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default void setRow(int row, FrameTuple3DReadOnly rowValues)
   {
      setRow(row, rowValues.getReferenceFrame(), rowValues);
   }

   /**
    * Sets the {@code column}<sup>th</sup> column components to the values contained in the given tuple
    * {@code columnValues}.
    *
    * @param column         the index of the column to set the values of.
    * @param referenceFrame the reference frame in which {@code other} is expressed.
    * @param columnValues   the tuple containing the new values for the column. Not modified.
    * @throws ArrayIndexOutOfBoundsException  if {@code column} &notin; [0, 2].
    * @throws ReferenceFrameMismatchException if {@code this.referenceFrame != referenceFrame}.
    */
   default void setColumn(int column, ReferenceFrame referenceFrame, Tuple3DReadOnly columnValues)
   {
      checkReferenceFrameMatch(referenceFrame);
      setColumn(column, columnValues);
   }

   /**
    * Sets the {@code column}<sup>th</sup> column components to the values contained in the given tuple
    * {@code columnValues}.
    *
    * @param column       the index of the column to set the values of.
    * @param columnValues the tuple containing the new values for the column. Not modified.
    * @throws ArrayIndexOutOfBoundsException  if {@code column} &notin; [0, 2].
    * @throws ReferenceFrameMismatchException if {@code columnValues} is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default void setColumn(int column, FrameTuple3DReadOnly columnValues)
   {
      setColumn(column, columnValues.getReferenceFrame(), columnValues);
   }

   /**
    * Performs a per-component addition onto the coefficients of this matrix.
    * <p>
    * this = this + other
    * </p>
    *
    * @param other the other matrix to use for the addition. Not modified.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void add(FrameMatrix3DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      Matrix3DBasics.super.add(other);
   }

   /**
    * Sets this matrix coefficients to the per-component sum of the two given matrices.
    * <p>
    * this = matrix1 + matrix2
    * </p>
    *
    * @param matrix1 the first term of the addition. Not modified.
    * @param matrix2 the second term of the addition. Not modified.
    * @throws ReferenceFrameMismatchException if {@code matrix1} is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void add(FrameMatrix3DReadOnly matrix1, Matrix3DReadOnly matrix2)
   {
      checkReferenceFrameMatch(matrix1);
      Matrix3DBasics.super.add(matrix1, matrix2);
   }

   /**
    * Sets this matrix coefficients to the per-component sum of the two given matrices.
    * <p>
    * this = matrix1 + matrix2
    * </p>
    *
    * @param matrix1 the first term of the addition. Not modified.
    * @param matrix2 the second term of the addition. Not modified.
    * @throws ReferenceFrameMismatchException if {@code matrix2} is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void add(Matrix3DReadOnly matrix1, FrameMatrix3DReadOnly matrix2)
   {
      checkReferenceFrameMatch(matrix2);
      Matrix3DBasics.super.add(matrix1, matrix2);
   }

   /**
    * Sets this matrix coefficients to the per-component sum of the two given matrices.
    * <p>
    * this = matrix1 + matrix2
    * </p>
    *
    * @param matrix1 the first term of the addition. Not modified.
    * @param matrix2 the second term of the addition. Not modified.
    * @throws ReferenceFrameMismatchException if either {@code matrix1} or {@code matrix2} is not
    *                                         expressed in the same reference frame as {@code this}.
    */
   default void add(FrameMatrix3DReadOnly matrix1, FrameMatrix3DReadOnly matrix2)
   {
      checkReferenceFrameMatch(matrix1, matrix2);
      Matrix3DBasics.super.add(matrix1, matrix2);
   }

   /**
    * Performs a per-component subtraction onto the coefficients of this matrix.
    * <p>
    * this = this - other
    * </p>
    *
    * @param other the other matrix to use for the subtraction. Not modified.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void sub(FrameMatrix3DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      Matrix3DBasics.super.sub(other);
   }

   /**
    * Sets this matrix coefficients to the per-component difference of the two given matrices.
    * <p>
    * this = matrix1 - matrix2
    * </p>
    *
    * @param matrix1 the first term of the addition. Not modified.
    * @param matrix2 the second term of the addition. Not modified.
    * @throws ReferenceFrameMismatchException if {@code matrix1} is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void sub(FrameMatrix3DReadOnly matrix1, Matrix3DReadOnly matrix2)
   {
      checkReferenceFrameMatch(matrix1);
      Matrix3DBasics.super.sub(matrix1, matrix2);
   }

   /**
    * Sets this matrix coefficients to the per-component difference of the two given matrices.
    * <p>
    * this = matrix1 - matrix2
    * </p>
    *
    * @param matrix1 the first term of the addition. Not modified.
    * @param matrix2 the second term of the addition. Not modified.
    * @throws ReferenceFrameMismatchException if {@code matrix2} is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void sub(Matrix3DReadOnly matrix1, FrameMatrix3DReadOnly matrix2)
   {
      checkReferenceFrameMatch(matrix2);
      Matrix3DBasics.super.sub(matrix1, matrix2);
   }

   /**
    * Sets this matrix coefficients to the per-component difference of the two given matrices.
    * <p>
    * this = matrix1 - matrix2
    * </p>
    *
    * @param matrix1 the first term of the addition. Not modified.
    * @param matrix2 the second term of the addition. Not modified.
    * @throws ReferenceFrameMismatchException if either {@code matrix1} or {@code matrix2} is not
    *                                         expressed in the same reference frame as {@code this}.
    */
   default void sub(FrameMatrix3DReadOnly matrix1, FrameMatrix3DReadOnly matrix2)
   {
      checkReferenceFrameMatch(matrix1, matrix2);
      Matrix3DBasics.super.sub(matrix1, matrix2);
   }

   /**
    * Performs a matrix multiplication on this.
    * <p>
    * this = this * other
    * </p>
    *
    * @param other the other matrix to multiply this by. Not modified.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void multiply(FrameMatrix3DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      Matrix3DBasics.super.multiply(other);
   }

   /**
    * Performs a matrix multiplication on this.
    * <p>
    * this = this<sup>T</sup> * other
    * </p>
    *
    * @param other the other matrix to multiply this by. Not modified.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void multiplyTransposeThis(FrameMatrix3DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      Matrix3DBasics.super.multiplyTransposeThis(other);
   }

   /**
    * Performs a matrix multiplication on this.
    * <p>
    * this = this * other<sup>T</sup>
    * </p>
    *
    * @param other the other matrix to multiply this by. Not modified.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void multiplyTransposeOther(FrameMatrix3DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      Matrix3DBasics.super.multiplyTransposeOther(other);
   }

   /**
    * Performs a matrix multiplication on this.
    * <p>
    * this = this<sup>T</sup> * other<sup>T</sup>
    * </p>
    *
    * @param other the other matrix to multiply this by. Not modified.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void multiplyTransposeBoth(FrameMatrix3DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      Matrix3DBasics.super.multiplyTransposeBoth(other);
   }

   /**
    * Performs a matrix multiplication on this.
    * <p>
    * this = this<sup>-1</sup> * other
    * </p>
    *
    * @param other the other matrix to multiply this by. Not modified.
    * @throws SingularMatrixException         if {@code this} is not invertible.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void multiplyInvertThis(FrameMatrix3DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      Matrix3DBasics.super.multiplyInvertThis(other);
   }

   /**
    * Performs a matrix multiplication on this.
    * <p>
    * this = this * other<sup>-1</sup>
    * </p>
    *
    * @param other the other matrix to multiply this by. Not modified.
    * @throws SingularMatrixException         if {@code other} is not invertible.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void multiplyInvertOther(FrameMatrix3DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      Matrix3DBasics.super.multiplyInvertOther(other);
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
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void multiplyInvertOther(FrameRotationMatrixReadOnly other)
   {
      checkReferenceFrameMatch(other);
      Matrix3DBasics.super.multiplyInvertOther(other);
   }

   /**
    * Performs a matrix multiplication on this.
    * <p>
    * this = other * this
    * </p>
    *
    * @param other the other matrix to multiply this by. Not modified.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void preMultiply(FrameMatrix3DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      Matrix3DBasics.super.preMultiply(other);
   }

   /**
    * Performs a matrix multiplication on this.
    * <p>
    * this = other * this<sup>T</sup>
    * </p>
    *
    * @param other the other matrix to multiply this by. Not modified.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void preMultiplyTransposeThis(FrameMatrix3DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      Matrix3DBasics.super.preMultiplyTransposeThis(other);
   }

   /**
    * Performs a matrix multiplication on this.
    * <p>
    * this = other<sup>T</sup> * this
    * </p>
    *
    * @param other the other matrix to multiply this by. Not modified.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void preMultiplyTransposeOther(FrameMatrix3DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      Matrix3DBasics.super.preMultiplyTransposeOther(other);
   }

   /**
    * Performs a matrix multiplication on this.
    * <p>
    * this = other<sup>T</sup> * this<sup>T</sup>
    * </p>
    *
    * @param other the other matrix to multiply this by. Not modified.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void preMultiplyTransposeBoth(FrameMatrix3DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      Matrix3DBasics.super.preMultiplyTransposeBoth(other);
   }

   /**
    * Performs a matrix multiplication on this.
    * <p>
    * this = other * this<sup>-1</sup>
    * </p>
    *
    * @param other the other matrix to multiply this by. Not modified.
    * @throws SingularMatrixException         if {@code this} is not invertible.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void preMultiplyInvertThis(FrameMatrix3DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      Matrix3DBasics.super.preMultiplyInvertThis(other);
   }

   /**
    * Performs a matrix multiplication on this.
    * <p>
    * this = other<sup>-1</sup> * this
    * </p>
    *
    * @param other the other matrix to multiply this by. Not modified.
    * @throws SingularMatrixException         if {@code other} is not invertible.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void preMultiplyInvertOther(FrameMatrix3DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      Matrix3DBasics.super.preMultiplyInvertOther(other);
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
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void preMultiplyInvertOther(FrameRotationMatrixReadOnly other)
   {
      checkReferenceFrameMatch(other);
      Matrix3DTools.multiplyInvertLeft(other, this, this);
   }
}
