package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.geometry.interfaces.Pose2DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;

public interface FramePose2DBasics extends FixedFramePose2DBasics
{
   /**
    * Sets the reference frame of this orientation 2D without updating or modifying its yaw angle.
    * 
    * @param referenceFrame the new reference frame for this frame orientation 2D.
    */
   void setReferenceFrame(ReferenceFrame referenceFrame);

   /**
    * Sets the yaw angle of this orientation 2D to zero and sets the current reference frame to
    * {@code referenceFrame}.
    * 
    * @param referenceFrame the new reference frame to be associated with this orientation 2D.
    */
   default void setToZero(ReferenceFrame referenceFrame)
   {
      setReferenceFrame(referenceFrame);
      setToZero();
   }

   /**
    * Sets the yaw angle of this orientation 2D to {@link Double#NaN} and sets the current reference
    * frame to {@code referenceFrame}.
    * 
    * @param referenceFrame the new reference frame to be associated with this orientation 2D.
    */
   default void setToNaN(ReferenceFrame referenceFrame)
   {
      setReferenceFrame(referenceFrame);
      setToNaN();
   }

   default void setIncludingFrame(ReferenceFrame referenceFrame, double x, double y, double yaw)
   {
      setReferenceFrame(referenceFrame);
      set(x, y, yaw);
   }

   default void setIncludingFrame(ReferenceFrame referenceFrame, Pose2DReadOnly pose2DReadOnly)
   {
      setReferenceFrame(referenceFrame);
      FixedFramePose2DBasics.super.set(pose2DReadOnly);
   }

   default void setIncludingFrame(FramePose2DReadOnly other)
   {
      setIncludingFrame(other.getReferenceFrame(), other);
   }
}
