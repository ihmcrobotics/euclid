package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.geometry.interfaces.Orientation2DReadOnly;
import us.ihmc.euclid.geometry.interfaces.Pose2DReadOnly;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;

public interface FramePose2DBasics extends FixedFramePose2DBasics, FrameChangeable
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

   default void setIncludingFrame(ReferenceFrame referenceFrame, Pose3DReadOnly pose3DReadOnly)
   {
      setReferenceFrame(referenceFrame);
      set(pose3DReadOnly);
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

   default void setIncludingFrame(ReferenceFrame referenceFrame, RigidBodyTransform rigidBodyTransform)
   {
      setIncludingFrame(referenceFrame, rigidBodyTransform, true);
   }

   default void setIncludingFrame(ReferenceFrame referenceFrame, RigidBodyTransform rigidBodyTransform, boolean checkIsTransform2D)
   {
      setReferenceFrame(referenceFrame);
      set(rigidBodyTransform, checkIsTransform2D);
   }

   default void setIncludingFrame(ReferenceFrame referenceFrame, FrameTuple2DReadOnly position, double yaw)
   {
      position.checkReferenceFrameMatch(referenceFrame);
      setIncludingFrame(referenceFrame, (Tuple2DReadOnly) position, yaw);
   }

   default void setIncludingFrame(ReferenceFrame referenceFrame, Tuple2DReadOnly position, double yaw)
   {
      setReferenceFrame(referenceFrame);
      set(position, yaw);
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

   default void setIncludingFrame(FramePose3DReadOnly other)
   {
      setIncludingFrame(other.getReferenceFrame(), other);
   }
}
