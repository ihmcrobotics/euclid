package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;

public interface FramePose3DBasics extends FixedFramePose3DBasics
{
   /**
    * Sets the reference frame of this orientation 3D without updating or modifying its position or
    * orientation.
    * 
    * @param referenceFrame the new reference frame for this frame orientation 3D.
    */
   void setReferenceFrame(ReferenceFrame referenceFrame);

   /**
    * Sets the position and orientation parts of this pose 3D to zero and sets the current reference
    * frame to {@code referenceFrame}.
    * 
    * @param referenceFrame the new reference frame to be associated with this pose 3D.
    */
   default void setToZero(ReferenceFrame referenceFrame)
   {
      setReferenceFrame(referenceFrame);
      setToZero();
   }

   /**
    * Sets the position and position parts of this pose 3D to {@link Double#NaN} and sets the
    * current reference frame to {@code referenceFrame}.
    * 
    * @param referenceFrame the new reference frame to be associated with this pose 3D.
    */
   default void setToNaN(ReferenceFrame referenceFrame)
   {
      setReferenceFrame(referenceFrame);
      setToNaN();
   }

   default void setIncludingFrame(ReferenceFrame referenceFrame, Pose3DReadOnly pose3DReadOnly)
   {
      setReferenceFrame(referenceFrame);
      set(pose3DReadOnly);
   }

   default void setIncludingFrame(FramePose3DReadOnly other)
   {
      setIncludingFrame(other.getReferenceFrame(), other);
   }
}
