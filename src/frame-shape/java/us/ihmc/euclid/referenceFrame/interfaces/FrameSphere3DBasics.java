package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.shape.primitives.interfaces.Sphere3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

public interface FrameSphere3DBasics extends FixedFrameSphere3DBasics, FrameShape3DBasics
{
   default void setIncludingFrame(ReferenceFrame referenceFrame, Sphere3DReadOnly other)
   {
      setReferenceFrame(referenceFrame);
      set(other);
   }

   default void setIncludingFrame(FrameSphere3DReadOnly other)
   {
      setIncludingFrame(other.getReferenceFrame(), other);
   }

   default void setIncludingFrame(ReferenceFrame referenceFrame, double centerX, double centerY, double centerZ, double radius)
   {
      setReferenceFrame(referenceFrame);
      set(centerX, centerY, centerZ, radius);
   }

   default void setIncludingFrame(ReferenceFrame referenceFrame, Point3DReadOnly center, double radius)
   {
      setReferenceFrame(referenceFrame);
      set(center, radius);
   }

   default void setIncludingFrame(FramePoint3DReadOnly center, double radius)
   {
      setIncludingFrame(center.getReferenceFrame(), center, radius);
   }
}
