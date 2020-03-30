package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.exceptions.NotARotationMatrixException;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;

/**
 * Write and read interface used for a 3-by-3 rotation matrix expressed in a changeable reference
 * frame, i.e. the reference frame in which this rotation matrix is expressed can be changed.
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
public interface FrameRotationMatrixBasics extends FixedFrameRotationMatrixBasics, FrameCommonMatrix3DBasics, FrameOrientation3DBasics
{
   /**
    * Sets this matrix to {@code other} and sets its current frame to {@code referenceFrame}.
    * <p>
    * If the argument implements {@link RotationMatrixReadOnly}, a redirection
    * {@link #set(RotationMatrixReadOnly)} is done.
    * </p>
    *
    * @param referenceFrame the reference frame in which the argument is expressed.
    * @param matrix         the matrix to copy the values of. Not modified.
    * @throws NotARotationMatrixException if the argument is not a rotation matrix.
    */
   @Override
   default void setIncludingFrame(ReferenceFrame referenceFrame, Matrix3DReadOnly matrix)
   {
      setReferenceFrame(referenceFrame);
      set(matrix);
   }

   /**
    * Sets this matrix to {@code other}.
    * <p>
    * If the argument implements {@link FrameRotationMatrixReadOnly}, a redirection
    * {@link #set(FrameRotationMatrixReadOnly)} is done.
    * </p>
    *
    * @param matrix the matrix to copy the values of. Not modified.
    * @throws NotARotationMatrixException if the argument is not a rotation matrix.
    */
   @Override
   default void setIncludingFrame(FrameMatrix3DReadOnly matrix)
   {
      setIncludingFrame(matrix.getReferenceFrame(), matrix);
   }

   /**
    * Sets this rotation matrix to {@code rotationMatrix} and copies the dirty and identity flags from
    * the other matrix and sets its reference frame.
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
    * @param rotationMatrix the other rotation matrix to copy. Not modified.
    */
   default void setIncludingFrame(ReferenceFrame referenceFrame, RotationMatrixReadOnly rotationMatrix)
   {
      setReferenceFrame(referenceFrame);
      set(rotationMatrix);
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
   default void setIncludingFrame(FrameRotationMatrixReadOnly other)
   {
      setIncludingFrame(other.getReferenceFrame(), other);
   }
}
