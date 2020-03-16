package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.shape.primitives.interfaces.Ramp3DReadOnly;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

public interface FrameRamp3DBasics extends FixedFrameRamp3DBasics, FrameShape3DBasics
{
   default void setToNaN(ReferenceFrame referenceFrame)
   {
      setReferenceFrame(referenceFrame);
      setToNaN();
   }

   default void setToZero(ReferenceFrame referenceFrame)
   {
      setReferenceFrame(referenceFrame);
      setToZero();
   }

   default void setIncludingFrame(ReferenceFrame referenceFrame, Ramp3DReadOnly other)
   {
      setReferenceFrame(referenceFrame);
      set(other);
   }

   default void setIncludingFrame(FrameRamp3DReadOnly other)
   {
      setIncludingFrame(other.getReferenceFrame(), other);
   }

   default void setIncludingFrame(ReferenceFrame referenceFrame, Point3DReadOnly position, Orientation3DReadOnly orientation, double sizeX, double sizeY,
                                  double sizeZ)
   {
      setReferenceFrame(referenceFrame);
      set(position, orientation, sizeX, sizeY, sizeZ);
   }

   default void setIncludingFrame(FramePoint3DReadOnly position, FrameOrientation3DReadOnly orientation, double sizeX, double sizeY, double sizeZ)
   {
      position.checkReferenceFrameMatch(orientation);
      setIncludingFrame(position.getReferenceFrame(), position, orientation, sizeX, sizeY, sizeZ);
   }

   default void setIncludingFrame(ReferenceFrame referenceFrame, Pose3DReadOnly pose, double sizeX, double sizeY, double sizeZ)
   {
      setReferenceFrame(referenceFrame);
      set(pose, sizeX, sizeY, sizeZ);
   }

   default void setIncludingFrame(FramePose3DReadOnly pose, double sizeX, double sizeY, double sizeZ)
   {
      setIncludingFrame(pose.getReferenceFrame(), pose, sizeX, sizeY, sizeZ);
   }

   default void setIncludingFrame(ReferenceFrame referenceFrame, RigidBodyTransformReadOnly pose, double sizeX, double sizeY, double sizeZ)
   {
      setReferenceFrame(referenceFrame);
      set(pose, sizeX, sizeY, sizeZ);
   }

   default void setIncludingFrame(FrameShape3DPoseReadOnly pose, double sizeX, double sizeY, double sizeZ)
   {
      setIncludingFrame(pose.getReferenceFrame(), pose, sizeX, sizeY, sizeZ);
   }

   default void setIncludingFrame(ReferenceFrame referenceFrame, RigidBodyTransformReadOnly pose, double[] size)
   {
      setReferenceFrame(referenceFrame);
      set(pose, size);
   }

   default void setIncludingFrame(FrameShape3DPoseReadOnly pose, double[] size)
   {
      setIncludingFrame(pose.getReferenceFrame(), pose, size);
   }
}
