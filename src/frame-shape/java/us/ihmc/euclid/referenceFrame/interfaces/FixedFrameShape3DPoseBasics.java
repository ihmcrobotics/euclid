package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.shape.primitives.interfaces.Shape3DPoseBasics;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;

public interface FixedFrameShape3DPoseBasics extends Shape3DPoseBasics, FrameShape3DPoseReadOnly
{
   @Override
   FixedFrameRotationMatrixBasics getShapeOrientation();

   @Override
   FixedFramePoint3DBasics getShapePosition();

   default void set(ReferenceFrame referenceFrame, Pose3DReadOnly pose)
   {
      checkReferenceFrameMatch(referenceFrame);
      set(pose);
   }

   default void set(FramePose3DReadOnly pose)
   {
      set(pose.getReferenceFrame(), pose);
   }

   default void set(ReferenceFrame referenceFrame, RigidBodyTransformReadOnly pose)
   {
      checkReferenceFrameMatch(referenceFrame);
      set(pose);
   }

   default void set(FrameShape3DPoseReadOnly pose)
   {
      set(pose.getReferenceFrame(), pose);
   }

   default void setMatchingFrame(ReferenceFrame referenceFrame, Pose3DReadOnly pose)
   {
      set(pose);
      referenceFrame.transformFromThisToDesiredFrame(getReferenceFrame(), this);
   }

   default void setMatchingFrame(FramePose3DReadOnly pose)
   {
      setMatchingFrame(pose.getReferenceFrame(), pose);
   }

   default void setMatchingFrame(ReferenceFrame referenceFrame, RigidBodyTransformReadOnly pose)
   {
      set(pose);
      referenceFrame.transformFromThisToDesiredFrame(getReferenceFrame(), this);
   }

   default void setMatchingFrame(FrameShape3DPoseReadOnly pose)
   {
      setMatchingFrame(pose.getReferenceFrame(), pose);
   }
}
