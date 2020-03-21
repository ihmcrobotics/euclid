package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.shape.primitives.interfaces.Ellipsoid3DBasics;
import us.ihmc.euclid.shape.primitives.interfaces.Ellipsoid3DReadOnly;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

public interface FixedFrameEllipsoid3DBasics extends Ellipsoid3DBasics, FrameEllipsoid3DReadOnly, FixedFrameShape3DBasics
{
   @Override
   FixedFrameVector3DBasics getRadii();

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

   default void set(ReferenceFrame referenceFrame, Ellipsoid3DReadOnly other)
   {
      checkReferenceFrameMatch(referenceFrame);
      set(other);
   }

   default void set(FrameEllipsoid3DReadOnly other)
   {
      set(other.getReferenceFrame(), other);
   }

   default void set(ReferenceFrame referenceFrame, Point3DReadOnly position, Orientation3DReadOnly orientation, double radiusX, double radiusY, double radiusZ)
   {
      checkReferenceFrameMatch(referenceFrame);
      set(position, orientation, radiusX, radiusY, radiusZ);
   }

   default void set(FramePoint3DReadOnly position, FrameOrientation3DReadOnly orientation, double radiusX, double radiusY, double radiusZ)
   {
      position.checkReferenceFrameMatch(orientation);
      set(position.getReferenceFrame(), position, orientation, radiusX, radiusY, radiusZ);
   }

   default void set(ReferenceFrame referenceFrame, Pose3DReadOnly pose, double radiusX, double radiusY, double radiusZ)
   {
      checkReferenceFrameMatch(referenceFrame);
      set(pose, radiusX, radiusY, radiusZ);
   }

   default void set(FramePose3DReadOnly pose, double radiusX, double radiusY, double radiusZ)
   {
      set(pose.getReferenceFrame(), pose, radiusX, radiusY, radiusZ);
   }

   default void set(ReferenceFrame referenceFrame, RigidBodyTransformReadOnly pose, double radiusX, double radiusY, double radiusZ)
   {
      checkReferenceFrameMatch(referenceFrame);
      set(pose, radiusX, radiusY, radiusZ);
   }

   default void set(FrameShape3DPoseReadOnly pose, double radiusX, double radiusY, double radiusZ)
   {
      set(pose.getReferenceFrame(), pose, radiusX, radiusY, radiusZ);
   }

   default void set(ReferenceFrame referenceFrame, RigidBodyTransformReadOnly pose, double[] radii)
   {
      checkReferenceFrameMatch(referenceFrame);
      set(pose, radii);
   }

   default void set(FrameShape3DPoseReadOnly pose, double[] radii)
   {
      set(pose.getReferenceFrame(), pose, radii);
   }

   default void setMatchingFrame(ReferenceFrame referenceFrame, Ellipsoid3DReadOnly other)
   {
      set(other);
      referenceFrame.transformFromThisToDesiredFrame(getReferenceFrame(), this);
   }

   default void setMatchingFrame(FrameEllipsoid3DReadOnly other)
   {
      setMatchingFrame(other.getReferenceFrame(), other);
   }

   default void setMatchingFrame(ReferenceFrame referenceFrame, Point3DReadOnly position, Orientation3DReadOnly orientation, double radiusX, double radiusY,
                                 double radiusZ)
   {
      set(position, orientation, radiusX, radiusY, radiusZ);
      referenceFrame.transformFromThisToDesiredFrame(getReferenceFrame(), this);
   }

   default void setMatchingFrame(FramePoint3DReadOnly position, FrameOrientation3DReadOnly orientation, double radiusX, double radiusY, double radiusZ)
   {
      position.checkReferenceFrameMatch(orientation);
      setMatchingFrame(position.getReferenceFrame(), position, orientation, radiusX, radiusY, radiusZ);
   }

   default void setMatchingFrame(ReferenceFrame referenceFrame, Pose3DReadOnly pose, double radiusX, double radiusY, double radiusZ)
   {
      set(pose, radiusX, radiusY, radiusZ);
      referenceFrame.transformFromThisToDesiredFrame(getReferenceFrame(), this);
   }

   default void setMatchingFrame(FramePose3DReadOnly pose, double radiusX, double radiusY, double radiusZ)
   {
      setMatchingFrame(pose.getReferenceFrame(), pose, radiusX, radiusY, radiusZ);
   }

   default void setMatchingFrame(ReferenceFrame referenceFrame, RigidBodyTransformReadOnly pose, double radiusX, double radiusY, double radiusZ)
   {
      set(pose, radiusX, radiusY, radiusZ);
      referenceFrame.transformFromThisToDesiredFrame(getReferenceFrame(), this);
   }

   default void setMatchingFrame(FrameShape3DPoseReadOnly pose, double radiusX, double radiusY, double radiusZ)
   {
      setMatchingFrame(pose.getReferenceFrame(), pose, radiusX, radiusY, radiusZ);
   }

   default void setMatchingFrame(ReferenceFrame referenceFrame, RigidBodyTransformReadOnly pose, double[] radii)
   {
      set(pose, radii);
      referenceFrame.transformFromThisToDesiredFrame(getReferenceFrame(), this);
   }

   default void setMatchingFrame(FrameShape3DPoseReadOnly pose, double[] radii)
   {
      setMatchingFrame(pose.getReferenceFrame(), pose, radii);
   }
}
