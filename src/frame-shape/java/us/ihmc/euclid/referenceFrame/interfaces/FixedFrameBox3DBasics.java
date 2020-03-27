package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.shape.primitives.interfaces.Box3DBasics;
import us.ihmc.euclid.shape.primitives.interfaces.Box3DReadOnly;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

public interface FixedFrameBox3DBasics extends Box3DBasics, FrameBox3DReadOnly, FixedFrameShape3DBasics
{
   @Override
   FixedFrameVector3DBasics getSize();

   @Override
   FixedFrameShape3DPoseBasics getPose();

   @Override
   default FixedFrameRotationMatrixBasics getOrientation()
   {
      return getPose().getShapeOrientation();
   }

   @Override
   default FixedFramePoint3DBasics getPosition()
   {
      return getPose().getShapePosition();
   }

   default void set(ReferenceFrame referenceFrame, Box3DReadOnly other)
   {
      checkReferenceFrameMatch(referenceFrame);
      set(other);
   }

   default void set(FrameBox3DReadOnly other)
   {
      set(other.getReferenceFrame(), other);
   }

   default void set(ReferenceFrame referenceFrame, Point3DReadOnly position, Orientation3DReadOnly orientation, double sizeX, double sizeY, double sizeZ)
   {
      checkReferenceFrameMatch(referenceFrame);
      set(position, orientation, sizeX, sizeY, sizeZ);
   }

   default void set(FramePoint3DReadOnly position, FrameOrientation3DReadOnly orientation, double sizeX, double sizeY, double sizeZ)
   {
      position.checkReferenceFrameMatch(orientation);
      set(position.getReferenceFrame(), position, orientation, sizeX, sizeY, sizeZ);
   }

   default void set(ReferenceFrame referenceFrame, Pose3DReadOnly pose, double sizeX, double sizeY, double sizeZ)
   {
      checkReferenceFrameMatch(referenceFrame);
      set(pose, sizeX, sizeY, sizeZ);
   }

   default void set(FramePose3DReadOnly pose, double sizeX, double sizeY, double sizeZ)
   {
      set(pose.getReferenceFrame(), pose, sizeX, sizeY, sizeZ);
   }

   default void set(ReferenceFrame referenceFrame, RigidBodyTransformReadOnly pose, double sizeX, double sizeY, double sizeZ)
   {
      checkReferenceFrameMatch(referenceFrame);
      set(pose, sizeX, sizeY, sizeZ);
   }

   default void set(FrameShape3DPoseReadOnly pose, double sizeX, double sizeY, double sizeZ)
   {
      set(pose.getReferenceFrame(), pose, sizeX, sizeY, sizeZ);
   }

   default void set(ReferenceFrame referenceFrame, RigidBodyTransformReadOnly pose, double[] size)
   {
      checkReferenceFrameMatch(referenceFrame);
      set(pose, size);
   }

   default void set(FrameShape3DPoseReadOnly pose, double[] size)
   {
      set(pose.getReferenceFrame(), pose, size);
   }

   default void setMatchingFrame(ReferenceFrame referenceFrame, Box3DReadOnly other)
   {
      set(other);
      referenceFrame.transformFromThisToDesiredFrame(getReferenceFrame(), this);
   }

   default void setMatchingFrame(FrameBox3DReadOnly other)
   {
      setMatchingFrame(other.getReferenceFrame(), other);
   }

   default void setMatchingFrame(ReferenceFrame referenceFrame, Point3DReadOnly position, Orientation3DReadOnly orientation, double sizeX, double sizeY,
                                 double sizeZ)
   {
      set(position, orientation, sizeX, sizeY, sizeZ);
      referenceFrame.transformFromThisToDesiredFrame(getReferenceFrame(), this);
   }

   default void setMatchingFrame(FramePoint3DReadOnly position, FrameOrientation3DReadOnly orientation, double sizeX, double sizeY, double sizeZ)
   {
      position.checkReferenceFrameMatch(orientation);
      setMatchingFrame(position.getReferenceFrame(), position, orientation, sizeX, sizeY, sizeZ);
   }

   default void setMatchingFrame(ReferenceFrame referenceFrame, Pose3DReadOnly pose, double sizeX, double sizeY, double sizeZ)
   {
      set(pose, sizeX, sizeY, sizeZ);
      referenceFrame.transformFromThisToDesiredFrame(getReferenceFrame(), this);
   }

   default void setMatchingFrame(FramePose3DReadOnly pose, double sizeX, double sizeY, double sizeZ)
   {
      setMatchingFrame(pose.getReferenceFrame(), pose, sizeX, sizeY, sizeZ);
   }

   default void setMatchingFrame(ReferenceFrame referenceFrame, RigidBodyTransformReadOnly pose, double sizeX, double sizeY, double sizeZ)
   {
      set(pose, sizeX, sizeY, sizeZ);
      referenceFrame.transformFromThisToDesiredFrame(getReferenceFrame(), this);
   }

   default void setMatchingFrame(FrameShape3DPoseReadOnly pose, double sizeX, double sizeY, double sizeZ)
   {
      setMatchingFrame(pose.getReferenceFrame(), pose, sizeX, sizeY, sizeZ);
   }

   default void setMatchingFrame(ReferenceFrame referenceFrame, RigidBodyTransformReadOnly pose, double[] size)
   {
      set(pose, size);
      referenceFrame.transformFromThisToDesiredFrame(getReferenceFrame(), this);
   }

   default void setMatchingFrame(FrameShape3DPoseReadOnly pose, double[] size)
   {
      setMatchingFrame(pose.getReferenceFrame(), pose, size);
   }
}
