package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.exceptions.NotARotationMatrixException;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixBasics;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;

/**
 * Write and read interface used for a 3-by-3 rotation matrix expressed in a constant reference
 * frame, i.e. the reference frame of this object cannot be changed via this interface.
 * <p>
 * A rotation matrix is used to represent a 3D orientation through its 9 coefficients. A rotation
 * matrix has to comply to several constraints:
 * <ul>
 * <li>each column of the matrix represents a unitary vector,
 * <li>each row of the matrix represents a unitary vector,
 * <li>every pair of columns of the matrix represents two orthogonal vectors,
 * <li>every pair of rows of the matrix represents two orthogonal vectors,
 * <li>the matrix determinant is equal to {@code 1}.
 * </ul>
 * A rotation matrix has the nice property <i>R<sup>T</sup> = R<sup>-1</sup></i>.
 * </p>
 *
 * @author Sylvain Bertrand
 */
public interface FixedFrameRotationMatrixBasics
      extends FrameRotationMatrixReadOnly, RotationMatrixBasics, FixedFrameCommonMatrix3DBasics, FixedFrameOrientation3DBasics
{
   /**
    * Sets this rotation matrix to equal the 3D matrix {@code matrix} and then normalizes {@code this}.
    *
    * @param matrix the matrix to copy the values from. Not modified.
    * @throws NotARotationMatrixException     if the normalization failed.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   @Override
   default void setAndNormalize(FrameMatrix3DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      RotationMatrixBasics.super.setAndNormalize(other);
   }

   /**
    * Sets this rotation matrix to equal the given {@code other} and then normalizes {@code this}.
    *
    * @param other the rotation matrix to copy the values from. Not modified.
    * @throws NotARotationMatrixException     if the normalization failed.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void setAndNormalize(FrameRotationMatrixReadOnly other)
   {
      checkReferenceFrameMatch(other);
      RotationMatrixBasics.super.setAndNormalize(other);
   }

   /**
    * {@inheritDoc}
    * <p>
    * If the argument implements {@link RotationMatrixReadOnly}, a redirection
    * {@link #set(RotationMatrixReadOnly)} is done.
    * </p>
    *
    * @throws NotARotationMatrixException if the argument is not a rotation matrix.
    */
   @Override
   default void set(ReferenceFrame referenceFrame, Matrix3DReadOnly other)
   {
      checkReferenceFrameMatch(referenceFrame);
      RotationMatrixBasics.super.set(other);
   }

   /**
    * {@inheritDoc}
    * <p>
    * If the argument implements {@link FrameRotationMatrixReadOnly}, a redirection
    * {@link #set(FrameRotationMatrixReadOnly)} is done.
    * </p>
    *
    * @throws NotARotationMatrixException if the argument is not a rotation matrix.
    */
   @Override
   default void set(FrameMatrix3DReadOnly other)
   {
      set(other.getReferenceFrame(), other);
   }

   /**
    * {@inheritDoc}
    * <p>
    * If the argument implements {@link RotationMatrixReadOnly}, a redirection
    * {@link #set(RotationMatrixReadOnly)} is done.
    * </p>
    */
   @Override
   default void set(ReferenceFrame referenceFrame, Orientation3DReadOnly orientation)
   {
      checkReferenceFrameMatch(referenceFrame);
      RotationMatrixBasics.super.set(orientation);
   }

   /**
    * {@inheritDoc}
    * <p>
    * If the argument implements {@link FrameRotationMatrixReadOnly}, a redirection
    * {@link #set(FrameRotationMatrixReadOnly)} is done.
    * </p>
    */
   @Override
   default void set(FrameOrientation3DReadOnly orientation)
   {
      set(orientation.getReferenceFrame(), orientation);
   }

   /**
    * Sets this rotation matrix to {@code other} and copies the dirty and identity flags from the other
    * matrix.
    * 
    * @param referenceFrame the reference frame in which the argument is expressed.
    * @param other          the other rotation matrix to copy. Not modified.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void set(ReferenceFrame referenceFrame, RotationMatrixReadOnly other)
   {
      checkReferenceFrameMatch(referenceFrame);
      set(other);
   }

   /**
    * Sets this rotation matrix to {@code other} and copies the dirty and identity flags from the other
    * matrix.
    * 
    * @param other the other rotation matrix to copy. Not modified.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void set(FrameRotationMatrixReadOnly other)
   {
      set(other.getReferenceFrame(), (RotationMatrixReadOnly) other);
   }

   /**
    * Sets this rotation matrix to the invert of the given {@code matrix}.
    * <p>
    * This operation uses the property: <br>
    * R<sup>-1</sup> = R<sup>T</sup> </br>
    * of a rotation matrix preventing to actually compute the inverse of the matrix.
    * </p>
    *
    * @param matrix the matrix to copy the values from. Not modified.
    * @throws NotARotationMatrixException     if {@code matrix} is not a rotation matrix.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void setAndInvert(FrameMatrix3DReadOnly matrix)
   {
      checkReferenceFrameMatch(matrix);
      RotationMatrixBasics.super.setAndInvert(matrix);
   }

   /**
    * Sets this rotation matrix to the invert of the given one {@code other}.
    * <p>
    * This operation uses the property: <br>
    * R<sup>-1</sup> = R<sup>T</sup> </br>
    * of a rotation matrix preventing to actually compute the inverse of the matrix.
    * </p>
    *
    * @param other the matrix to copy the values from. Not modified.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void setAndInvert(FrameRotationMatrixReadOnly other)
   {
      checkReferenceFrameMatch(other);
      RotationMatrixBasics.super.setAndInvert(other);
   }

   /**
    * Sets this matrix to equal the other matrix and then transposes this.
    * <p>
    * this = other<sup>T</sup>
    * </p>
    *
    * @param other the other matrix used to update this matrix. Not modified.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void setAndTranspose(FrameMatrix3DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      RotationMatrixBasics.super.setAndTranspose(other);
   }

   /**
    * Sets this matrix to {@code other}.
    * <p>
    * If {@code other} is expressed in the frame as {@code this}, then this method is equivalent to
    * {@link #set(ReferenceFrame, Matrix3DReadOnly)}.
    * </p>
    * <p>
    * If {@code other} is expressed in a different frame than {@code this}, then {@code this} is set to
    * {@code other} and then transformed to be expressed in {@code this.getReferenceFrame()}.
    * </p>
    * <p>
    * If the argument implements {@link RotationMatrixReadOnly}, a redirection
    * {@link #set(RotationMatrixReadOnly)} is done.
    * </p>
    *
    * @param referenceFrame the reference frame in which the argument is expressed.
    * @param matrix         the matrix to copy the values of. Not modified.
    * @throws NotARotationMatrixException if the argument is not a rotation matrix.
    */
   default void setMatchingFrame(ReferenceFrame referenceFrame, Matrix3DReadOnly matrix)
   {
      set(matrix);
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
    * {@code other} and then transformed to be expressed in {@code this.getReferenceFrame()}.
    * </p>
    * <p>
    * If the argument implements {@link FrameRotationMatrixReadOnly}, a redirection
    * {@link #set(FrameRotationMatrixReadOnly)} is done.
    * </p>
    *
    * @param matrix the matrix to copy the values of. Not modified.
    * @throws NotARotationMatrixException if the argument is not a rotation matrix.
    */
   default void setMatchingFrame(FrameMatrix3DReadOnly matrix)
   {
      setMatchingFrame(matrix.getReferenceFrame(), matrix);
   }

   /**
    * Sets this rotation matrix to {@code other} and copies the dirty and identity flags from the other
    * matrix.
    * <p>
    * If {@code other} is expressed in the frame as {@code this}, then this method is equivalent to
    * {@link #set(RotationMatrixReadOnly)}.
    * </p>
    * <p>
    * If {@code other} is expressed in a different frame than {@code this}, then {@code this} is set to
    * {@code other} and then transformed to be expressed in {@code this.getReferenceFrame()}.
    * </p>
    * 
    * @param referenceFrame the reference frame in which the argument is expressed.
    * @param other          the other rotation matrix to copy. Not modified.
    */
   default void setMatchingFrame(ReferenceFrame referenceFrame, RotationMatrixReadOnly other)
   {
      set(other);
      referenceFrame.transformFromThisToDesiredFrame(getReferenceFrame(), this);
   }

   /**
    * Sets this rotation matrix to {@code other} and copies the dirty and identity flags from the other
    * matrix.
    * <p>
    * If {@code other} is expressed in the frame as {@code this}, then this method is equivalent to
    * {@link #set(FrameRotationMatrixReadOnly)}.
    * </p>
    * <p>
    * If {@code other} is expressed in a different frame than {@code this}, then {@code this} is set to
    * {@code other} and then transformed to be expressed in {@code this.getReferenceFrame()}.
    * </p>
    * 
    * @param other the other rotation matrix to copy. Not modified.
    */
   default void setMatchingFrame(FrameRotationMatrixReadOnly other)
   {
      setMatchingFrame(other.getReferenceFrame(), other);
   }

   /**
    * Sets this frame orientation to {@code orientation}.
    * <p>
    * If {@code orientation} is expressed in the frame as {@code this}, then this method is equivalent
    * to {@link #set(FrameOrientation3DReadOnly)}.
    * </p>
    * <p>
    * If {@code orientation} is expressed in a different frame than {@code this}, then {@code this} is
    * set to {@code orientation} and then transformed to be expressed in
    * {@code this.getReferenceFrame()}.
    * </p>
    *
    * @param orientation the other orientation to copy the values from. Not modified.
    */
   default void setMatchingFrame(ReferenceFrame referenceFrame, Orientation3DReadOnly orientation)
   {
      RotationMatrixBasics.super.set(orientation);
      referenceFrame.transformFromThisToDesiredFrame(getReferenceFrame(), this);
   }

   /**
    * Sets this frame orientation to {@code orientation}.
    * <p>
    * If {@code orientation} is expressed in the frame as {@code this}, then this method is equivalent
    * to {@link #set(FrameOrientation3DReadOnly)}.
    * </p>
    * <p>
    * If {@code orientation} is expressed in a different frame than {@code this}, then {@code this} is
    * set to {@code orientation} and then transformed to be expressed in
    * {@code this.getReferenceFrame()}.
    * </p>
    *
    * @param orientation the other orientation to copy the values from. Not modified.
    */
   default void setMatchingFrame(FrameOrientation3DReadOnly orientation)
   {
      set((Orientation3DReadOnly) orientation);
      orientation.getReferenceFrame().transformFromThisToDesiredFrame(getReferenceFrame(), this);
   }

   /**
    * Performs a matrix multiplication on this.
    * <p>
    * this = this * other
    * </p>
    *
    * @param other the other matrix to multiply this by. Not modified.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void multiply(FrameRotationMatrixReadOnly other)
   {
      checkReferenceFrameMatch(other);
      RotationMatrixBasics.super.multiply(other);
   }

   /**
    * Performs a matrix multiplication on this.
    * <p>
    * this = this<sup>T</sup> * other
    * </p>
    *
    * @param other the other matrix to multiply this by. Not modified.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void multiplyTransposeThis(FrameRotationMatrixReadOnly other)
   {
      checkReferenceFrameMatch(other);
      RotationMatrixBasics.super.multiplyTransposeThis(other);
   }

   /**
    * Performs a matrix multiplication on this.
    * <p>
    * this = this * other<sup>T</sup>
    * </p>
    *
    * @param other the other matrix to multiply this by. Not modified.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void multiplyTransposeOther(FrameRotationMatrixReadOnly other)
   {
      checkReferenceFrameMatch(other);
      RotationMatrixBasics.super.multiplyTransposeOther(other);
   }

   /**
    * Performs a matrix multiplication on this.
    * <p>
    * this = this<sup>T</sup> * other<sup>T</sup>
    * </p>
    *
    * @param other the other matrix to multiply this by. Not modified.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void multiplyTransposeBoth(FrameRotationMatrixReadOnly other)
   {
      checkReferenceFrameMatch(other);
      RotationMatrixBasics.super.multiplyTransposeBoth(other);
   }

   /**
    * Performs a matrix multiplication on this.
    * <p>
    * this = other * this
    * </p>
    *
    * @param other the other matrix to multiply this by. Not modified.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void preMultiply(FrameRotationMatrixReadOnly other)
   {
      checkReferenceFrameMatch(other);
      RotationMatrixBasics.super.preMultiply(other);
   }

   /**
    * Performs a matrix multiplication on this.
    * <p>
    * this = other * this<sup>T</sup>
    * </p>
    *
    * @param other the other matrix to multiply this by. Not modified.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void preMultiplyTransposeThis(FrameRotationMatrixReadOnly other)
   {
      checkReferenceFrameMatch(other);
      RotationMatrixBasics.super.preMultiplyTransposeThis(other);
   }

   /**
    * Performs a matrix multiplication on this.
    * <p>
    * this = other<sup>T</sup> * this
    * </p>
    *
    * @param other the other matrix to multiply this by. Not modified.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void preMultiplyTransposeOther(FrameRotationMatrixReadOnly other)
   {
      checkReferenceFrameMatch(other);
      RotationMatrixBasics.super.preMultiplyTransposeOther(other);
   }

   /**
    * Performs a matrix multiplication on this.
    * <p>
    * this = other<sup>T</sup> * this<sup>T</sup>
    * </p>
    *
    * @param other the other matrix to multiply this by. Not modified.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void preMultiplyTransposeBoth(FrameRotationMatrixReadOnly other)
   {
      checkReferenceFrameMatch(other);
      RotationMatrixBasics.super.preMultiplyTransposeBoth(other);
   }

   /**
    * Performs a linear interpolation in SO(3) from {@code this} to {@code rf} given the percentage
    * {@code alpha}.
    * <p>
    * This is equivalent to but much more computationally expensive than the <i>Spherical Linear
    * Interpolation</i> performed with quaternions.
    * </p>
    *
    * @param rf    the other rotation matrix used for the interpolation. Not modified.
    * @param alpha the percentage used for the interpolation. A value of 0 will result in not modifying
    *              this rotation matrix, while a value of 1 is equivalent to setting this rotation
    *              matrix to {@code rf}.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void interpolate(FrameRotationMatrixReadOnly rf, double alpha)
   {
      checkReferenceFrameMatch(rf);
      RotationMatrixBasics.super.interpolate(rf, alpha);
   }

   /**
    * Performs a linear interpolation in SO(3) from {@code r0} to {@code rf} given the percentage
    * {@code alpha}.
    * <p>
    * This is equivalent to but much more computationally expensive than the <i>Spherical Linear
    * Interpolation</i> performed with quaternions.
    * </p>
    *
    * @param r0    the first rotation matrix used in the interpolation. Not modified.
    * @param rf    the second rotation matrix used in the interpolation. Not modified.
    * @param alpha the percentage to use for the interpolation. A value of 0 will result in setting
    *              this rotation matrix to {@code r0}, while a value of 1 is equivalent to setting this
    *              rotation matrix to {@code rf}.
    * @throws ReferenceFrameMismatchException if the frame argument is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default void interpolate(RotationMatrixReadOnly r0, FrameRotationMatrixReadOnly rf, double alpha)
   {
      checkReferenceFrameMatch(rf);
      RotationMatrixBasics.super.interpolate(r0, rf, alpha);
   }

   /**
    * Performs a linear interpolation in SO(3) from {@code r0} to {@code rf} given the percentage
    * {@code alpha}.
    * <p>
    * This is equivalent to but much more computationally expensive than the <i>Spherical Linear
    * Interpolation</i> performed with quaternions.
    * </p>
    *
    * @param r0    the first rotation matrix used in the interpolation. Not modified.
    * @param rf    the second rotation matrix used in the interpolation. Not modified.
    * @param alpha the percentage to use for the interpolation. A value of 0 will result in setting
    *              this rotation matrix to {@code r0}, while a value of 1 is equivalent to setting this
    *              rotation matrix to {@code rf}.
    * @throws ReferenceFrameMismatchException if the frame argument is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default void interpolate(FrameRotationMatrixReadOnly r0, RotationMatrixReadOnly rf, double alpha)
   {
      checkReferenceFrameMatch(r0);
      RotationMatrixBasics.super.interpolate(r0, rf, alpha);
   }

   /**
    * Performs a linear interpolation in SO(3) from {@code r0} to {@code rf} given the percentage
    * {@code alpha}.
    * <p>
    * This is equivalent to but much more computationally expensive than the <i>Spherical Linear
    * Interpolation</i> performed with quaternions.
    * </p>
    *
    * @param r0    the first rotation matrix used in the interpolation. Not modified.
    * @param rf    the second rotation matrix used in the interpolation. Not modified.
    * @param alpha the percentage to use for the interpolation. A value of 0 will result in setting
    *              this rotation matrix to {@code r0}, while a value of 1 is equivalent to setting this
    *              rotation matrix to {@code rf}.
    * @throws ReferenceFrameMismatchException if either of the frame arguments is not expressed in the
    *                                         same reference frame as {@code this}.
    */
   default void interpolate(FrameRotationMatrixReadOnly r0, FrameRotationMatrixReadOnly rf, double alpha)
   {
      checkReferenceFrameMatch(r0, rf);
      RotationMatrixBasics.super.interpolate(r0, rf, alpha);
   }
}
