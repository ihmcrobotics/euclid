package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.axisAngle.interfaces.AxisAngleReadOnly;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;

public interface FramePose3DBasics extends FixedFramePose3DBasics, FrameChangeable
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

   default void setIncludingFrame(ReferenceFrame referenceFrame, RigidBodyTransform rigidBodyTransform)
   {
      setReferenceFrame(referenceFrame);
      set(rigidBodyTransform);
   }

   default void setIncludingFrame(ReferenceFrame referenceFrame, Tuple3DReadOnly position, QuaternionReadOnly orientation)
   {
      setReferenceFrame(referenceFrame);
      set(position, orientation);
   }

   default void setIncludingFrame(ReferenceFrame referenceFrame, Tuple3DReadOnly position, AxisAngleReadOnly orientation)
   {
      setReferenceFrame(referenceFrame);
      set(position, orientation);
   }

   default void setIncludingFrame(FrameTuple3DReadOnly position, FrameQuaternionReadOnly orientation)
   {
      position.checkReferenceFrameMatch(orientation);
      setReferenceFrame(position.getReferenceFrame());
      set((Tuple3DReadOnly) position, (QuaternionReadOnly) orientation);
   }

   default void setIncludingFrame(FramePose3DReadOnly other)
   {
      setIncludingFrame(other.getReferenceFrame(), other);
   }
}
