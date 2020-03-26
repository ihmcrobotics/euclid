package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.shape.primitives.interfaces.Ellipsoid3DReadOnly;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

public interface FrameEllipsoid3DBasics extends FixedFrameEllipsoid3DBasics, FrameShape3DBasics
{
   default void setIncludingFrame(ReferenceFrame referenceFrame, Ellipsoid3DReadOnly other)
   {
      setReferenceFrame(referenceFrame);
      set(other);
   }

   default void setIncludingFrame(FrameEllipsoid3DReadOnly other)
   {
      setIncludingFrame(other.getReferenceFrame(), other);
   }

   default void setIncludingFrame(ReferenceFrame referenceFrame, Point3DReadOnly position, Orientation3DReadOnly orientation, double radiusX, double radiusY,
                                  double radiusZ)
   {
      setReferenceFrame(referenceFrame);
      set(position, orientation, radiusX, radiusY, radiusZ);
   }

   default void setIncludingFrame(FramePoint3DReadOnly position, FrameOrientation3DReadOnly orientation, double radiusX, double radiusY, double radiusZ)
   {
      position.checkReferenceFrameMatch(orientation);
      setIncludingFrame(position.getReferenceFrame(), position, orientation, radiusX, radiusY, radiusZ);
   }

   default void setIncludingFrame(ReferenceFrame referenceFrame, Pose3DReadOnly pose, double radiusX, double radiusY, double radiusZ)
   {
      setReferenceFrame(referenceFrame);
      set(pose, radiusX, radiusY, radiusZ);
   }

   default void setIncludingFrame(FramePose3DReadOnly pose, double radiusX, double radiusY, double radiusZ)
   {
      setIncludingFrame(pose.getReferenceFrame(), pose, radiusX, radiusY, radiusZ);
   }

   default void setIncludingFrame(ReferenceFrame referenceFrame, RigidBodyTransformReadOnly pose, double radiusX, double radiusY, double radiusZ)
   {
      setReferenceFrame(referenceFrame);
      set(pose, radiusX, radiusY, radiusZ);
   }

   default void setIncludingFrame(FrameShape3DPoseReadOnly pose, double radiusX, double radiusY, double radiusZ)
   {
      setIncludingFrame(pose.getReferenceFrame(), pose, radiusX, radiusY, radiusZ);
   }

   default void setIncludingFrame(ReferenceFrame referenceFrame, RigidBodyTransformReadOnly pose, double[] radii)
   {
      setReferenceFrame(referenceFrame);
      set(pose, radii);
   }

   default void setIncludingFrame(FrameShape3DPoseReadOnly pose, double[] radii)
   {
      setIncludingFrame(pose.getReferenceFrame(), pose, radii);
   }

   @Override
   FrameEllipsoid3DBasics copy();
}
