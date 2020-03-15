package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;

public interface FrameShape3DPoseBasics extends FixedFrameShape3DPoseBasics, FrameChangeable
{
   default void setToZero(ReferenceFrame referenceFrame)
   {
      setReferenceFrame(referenceFrame);
      setToZero();
   }

   default void setToNaN(ReferenceFrame referenceFrame)
   {
      setReferenceFrame(referenceFrame);
      setToNaN();
   }

   default void setIncludingFrame(ReferenceFrame referenceFrame, Pose3DReadOnly pose)
   {
      setReferenceFrame(referenceFrame);
      set(pose);
   }

   default void setIncludingFrame(FramePose3DReadOnly pose)
   {
      setIncludingFrame(pose.getReferenceFrame(), pose);
   }

   default void setIncludingFrame(ReferenceFrame referenceFrame, RigidBodyTransformReadOnly pose)
   {
      setReferenceFrame(referenceFrame);
      set(pose);
   }

   default void setIncludingFrame(FrameShape3DPoseReadOnly pose)
   {
      setIncludingFrame(pose.getReferenceFrame(), pose);
   }
}
