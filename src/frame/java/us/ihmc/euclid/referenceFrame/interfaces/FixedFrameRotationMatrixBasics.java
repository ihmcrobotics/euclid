package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.matrix.interfaces.Matrix3DReadOnly;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixBasics;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;

public interface FixedFrameRotationMatrixBasics
      extends FrameRotationMatrixReadOnly, RotationMatrixBasics, FixedFrameCommonMatrix3DBasics, FixedFrameOrientation3DBasics
{
   default void set(FrameRotationMatrixReadOnly other)
   {
      checkReferenceFrameMatch(other);
      set((RotationMatrixReadOnly) other);
   }

   default void setMatchingFrame(FrameMatrix3DReadOnly matrix)
   {
      set((Matrix3DReadOnly) matrix);
      matrix.getReferenceFrame().transformFromThisToDesiredFrame(getReferenceFrame(), this);
   }

   default void setMatchingFrame(FrameRotationMatrixReadOnly other)
   {
      set((RotationMatrixReadOnly) other);
      other.getReferenceFrame().transformFromThisToDesiredFrame(getReferenceFrame(), this);
   }
}
