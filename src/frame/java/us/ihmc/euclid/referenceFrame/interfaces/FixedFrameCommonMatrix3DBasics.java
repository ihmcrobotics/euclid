package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.exceptions.SingularMatrixException;
import us.ihmc.euclid.matrix.interfaces.CommonMatrix3DBasics;
import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;

/**
 * Read interface for a 3-by-3 matrix object with a restricted writing interface.
 * <p>
 * The matrix is expressed in a constant reference frame, i.e. the reference frame of this object
 * cannot be changed via this interface.
 * </p>
 * <p>
 * In this interface, the matrix is assumed to be generic purpose. Therefore, the algorithms used
 * here are limited to generic applications without violating potential constraints of more specific
 * matrices such as a rotation matrix.
 * </p>
 *
 * @author Sylvain Bertrand
 */
public interface FixedFrameCommonMatrix3DBasics extends FrameMatrix3DReadOnly, CommonMatrix3DBasics
{
   /**
    * Sets this matrix to {@code other}.
    *
    * @param referenceFrame the reference frame in which {@code other} is expressed.
    * @param other          the other frame matrix to copy the values of. Not modified.
    * @throws ReferenceFrameMismatchException if the matrix is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void set(ReferenceFrame referenceFrame, Matrix3DReadOnly other)
   {
      checkReferenceFrameMatch(referenceFrame);
      set(other);
   }

   /**
    * Sets this matrix to {@code other}.
    *
    * @param other the other frame matrix to copy the values of. Not modified.
    * @throws ReferenceFrameMismatchException if the matrix is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void set(FrameMatrix3DReadOnly other)
   {
      set(other.getReferenceFrame(), other);
   }

   /**
    * Sets this matrix to equal the other matrix and then normalizes this, see {@link #normalize()}.
    *
    * @param referenceFrame the reference frame in which {@code other} is expressed.
    * @param other          the other matrix used to update this matrix. Not modified.
    * @throws ReferenceFrameMismatchException if the matrix is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void setAndNormalize(ReferenceFrame referenceFrame, Matrix3DReadOnly other)
   {
      checkReferenceFrameMatch(referenceFrame);
      setAndNormalize(other);
   }

   /**
    * Sets this matrix to equal the other matrix and then normalizes this, see {@link #normalize()}.
    *
    * @param other the other matrix used to update this matrix. Not modified.
    * @throws ReferenceFrameMismatchException if the matrix is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void setAndNormalize(FrameMatrix3DReadOnly other)
   {
      setAndNormalize(other.getReferenceFrame(), other);
   }

   /**
    * Set this matrix to the inverse of the other matrix.
    * <p>
    * this = other<sup>-1</sup>
    * </p>
    *
    * @param referenceFrame the reference frame in which {@code other} is expressed.
    * @param other          the other matrix. Not modified.
    * @throws SingularMatrixException         if the matrix is not invertible.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void setAndInvert(ReferenceFrame referenceFrame, Matrix3DReadOnly other)
   {
      checkReferenceFrameMatch(referenceFrame);
      setAndInvert(other);
   }

   /**
    * Set this matrix to the inverse of the other matrix.
    * <p>
    * this = other<sup>-1</sup>
    * </p>
    *
    * @param other the other matrix. Not modified.
    * @throws SingularMatrixException         if the matrix is not invertible.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void setAndInvert(FrameMatrix3DReadOnly other)
   {
      setAndInvert(other.getReferenceFrame(), other);
   }

   /**
    * Sets this matrix to equal the other matrix and then transposes this.
    * <p>
    * this = other<sup>T</sup>
    * </p>
    *
    * @param referenceFrame the reference frame in which {@code other} is expressed.
    * @param other          the other matrix used to update this matrix. Not modified.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void setAndTranspose(ReferenceFrame referenceFrame, Matrix3DReadOnly other)
   {
      checkReferenceFrameMatch(referenceFrame);
      setAndTranspose(other);
   }

   /**
    * Sets this matrix to equal the other matrix and then transposes this.
    * <p>
    * this = other<sup>T</sup>
    * </p>
    *
    * @param other the other matrix used to update this matrix. Not modified.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void setAndTranspose(FrameMatrix3DReadOnly other)
   {
      setAndTranspose(other.getReferenceFrame(), other);
   }

   /**
    * Sets this frame quaternion to {@code orientation} and checks that its current frame equals
    * {@code referenceFrame}.
    *
    * @param referenceFrame the coordinate system in which the given {@code quaternionReadOnly} is
    *                       expressed.
    * @param orientation    the orientation to copy the values from. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this.referenceFrame != referenceFrame}.
    */
   default void set(ReferenceFrame referenceFrame, Orientation3DReadOnly orientation)
   {
      checkReferenceFrameMatch(referenceFrame);
      set(orientation);
   }

   /**
    * Converts, if necessary, and sets this orientation to represents the same orientation as
    * {@code orientation}.
    *
    * @param orientation the new orientation. Not modified.
    * @throws ReferenceFrameMismatchException if {@code orientation} is not expressed in the same
    *                                         reference frame as {@code this}.
    */
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
      set(other.getReferenceFrame(), other);
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
    * @param referenceFrame the reference frame in which the arguments are expressed.
    * @param firstRow       the tuple holding onto the values of the first row. Not modified.
    * @param secondRow      the tuple holding onto the values of the second row. Not modified.
    * @param thirdRow       the tuple holding onto the values of the third row. Not modified.
    * @throws ReferenceFrameMismatchException if the arguments are not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void setRows(ReferenceFrame referenceFrame, Tuple3DReadOnly firstRow, Tuple3DReadOnly secondRow, Tuple3DReadOnly thirdRow)
   {
      checkReferenceFrameMatch(referenceFrame);
      setRows(firstRow, secondRow, thirdRow);
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
    * @throws ReferenceFrameMismatchException if any of the arguments is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default void setRows(FrameTuple3DReadOnly firstRow, FrameTuple3DReadOnly secondRow, FrameTuple3DReadOnly thirdRow)
   {
      firstRow.checkReferenceFrameMatch(secondRow, thirdRow);
      setRows(firstRow.getReferenceFrame(), firstRow, secondRow, thirdRow);
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
    * @param referenceFrame the reference frame in which the arguments are expressed.
    * @param firstColumn    the tuple holding onto the values of the first column. Not modified.
    * @param secondColumn   the tuple holding onto the values of the second column. Not modified.
    * @param thirdColumn    the tuple holding onto the values of the third column. Not modified.
    * @throws ReferenceFrameMismatchException if the arguments are not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void setColumns(ReferenceFrame referenceFrame, Tuple3DReadOnly firstColumn, Tuple3DReadOnly secondColumn, Tuple3DReadOnly thirdColumn)
   {
      checkReferenceFrameMatch(referenceFrame);
      setColumns(firstColumn, secondColumn, thirdColumn);
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
    * @throws ReferenceFrameMismatchException if any of the arguments is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default void setColumns(FrameTuple3DReadOnly firstColumn, FrameTuple3DReadOnly secondColumn, FrameTuple3DReadOnly thirdColumn)
   {
      firstColumn.checkReferenceFrameMatch(secondColumn, thirdColumn);
      setColumns(firstColumn.getReferenceFrame(), firstColumn, secondColumn, thirdColumn);
   }
}
