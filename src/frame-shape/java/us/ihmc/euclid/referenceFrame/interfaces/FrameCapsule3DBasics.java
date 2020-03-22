package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.shape.primitives.interfaces.Capsule3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public interface FrameCapsule3DBasics extends FixedFrameCapsule3DBasics, FrameShape3DBasics
{
   default void setIncludingFrame(ReferenceFrame referenceFrame, Capsule3DReadOnly other)
   {
      setReferenceFrame(referenceFrame);
      set(other);
   }

   default void setIncludingFrame(FrameCapsule3DReadOnly other)
   {
      setIncludingFrame(other.getReferenceFrame(), other);
   }

   default void setIncludingFrame(ReferenceFrame referenceFrame, Point3DReadOnly position, Vector3DReadOnly axis, double length, double radius)
   {
      setReferenceFrame(referenceFrame);
      set(position, axis, length, radius);
   }

   default void setIncludingFrame(FramePoint3DReadOnly position, FrameVector3DReadOnly axis, double length, double radius)
   {
      position.checkReferenceFrameMatch(axis);
      setIncludingFrame(position.getReferenceFrame(), position, axis, length, radius);
   }
}
