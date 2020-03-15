package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.matrix.interfaces.CommonMatrix3DBasics;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;

public interface FixedFrameCommonMatrix3DBasics extends FrameMatrix3DReadOnly, CommonMatrix3DBasics
{
   /**
    * Sets this matrix to {@code other}.
    *
    * @param other the other frame matrix to copy the values of. Not modified.
    * @throws ReferenceFrameMismatchException if the matrix is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void set(FrameMatrix3DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      CommonMatrix3DBasics.super.set(other);
   }

   /**
    * Sets this matrix to equal the other matrix and then normalizes this, see {@link #normalize()}.
    *
    * @param other the other matrix used to update this matrix. Not modified.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void setAndNormalize(FrameMatrix3DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      CommonMatrix3DBasics.super.setAndNormalize(other);
   }

   default void setRows(FrameTuple3DReadOnly firstRow, FrameTuple3DReadOnly secondRow, FrameTuple3DReadOnly thirdRow)
   {
      checkReferenceFrameMatch(firstRow, secondRow, thirdRow);
      CommonMatrix3DBasics.super.setRows(firstRow, secondRow, thirdRow);
   }

   default void setColumns(FrameTuple3DReadOnly firstColumn, FrameTuple3DReadOnly secondColumn, FrameTuple3DReadOnly thirdColumn)
   {
      checkReferenceFrameMatch(firstColumn, secondColumn, thirdColumn);
      CommonMatrix3DBasics.super.setColumns(firstColumn, secondColumn, thirdColumn);
   }
}
