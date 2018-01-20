package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.geometry.interfaces.Orientation2DReadOnly;
import us.ihmc.euclid.geometry.interfaces.Pose2DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;

public interface FramePose2DBasics extends FixedFramePose2DBasics
{
   /**
    * Sets the reference frame of this orientation 2D without updating or modifying its position or
    * orientation.
    * 
    * @param referenceFrame the new reference frame for this frame pose 2D.
    */
   void setReferenceFrame(ReferenceFrame referenceFrame);

   /**
    * Sets the position and orientation parts of this pose 2D to zero and sets the current reference
    * frame to {@code referenceFrame}.
    * 
    * @param referenceFrame the new reference frame to be associated with this pose 2D.
    */
   default void setToZero(ReferenceFrame referenceFrame)
   {
      setReferenceFrame(referenceFrame);
      setToZero();
   }

   /**
    * Sets the position and position parts of this pose 2D to {@link Double#NaN} and sets the
    * current reference frame to {@code referenceFrame}.
    * 
    * @param referenceFrame the new reference frame to be associated with this pose 2D.
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
      set(pose2DReadOnly);
   }

   default void setIncludingFrame(ReferenceFrame referenceFrame, Tuple2DReadOnly position, Orientation2DReadOnly orientation)
   {
      setReferenceFrame(referenceFrame);
      set(position, orientation);
   }

   default void setIncludingFrame(FrameTuple2DReadOnly position, FrameOrientation2DReadOnly orientation)
   {
      position.checkReferenceFrameMatch(orientation);
      setIncludingFrame(position.getReferenceFrame(), position, orientation);
   }

   default void setIncludingFrame(FramePose2DReadOnly other)
   {
      setIncludingFrame(other.getReferenceFrame(), other);
   }
}
