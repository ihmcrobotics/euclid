package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;

public interface FrameRotationMatrixBasics extends FixedFrameRotationMatrixBasics, FrameCommonMatrix3DBasics, FrameOrientation3DBasics
{
   default void setIncludingFrame(ReferenceFrame referenceFrame, RotationMatrixReadOnly rotationMatrix)
   {
      setReferenceFrame(referenceFrame);
      set(rotationMatrix);
   }

   default void setIncludingFrame(FrameRotationMatrixReadOnly other)
   {
      setIncludingFrame(other.getReferenceFrame(), other);
   }
}
