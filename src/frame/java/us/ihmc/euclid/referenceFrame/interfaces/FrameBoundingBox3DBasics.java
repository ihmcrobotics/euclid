package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.geometry.interfaces.BoundingBox3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public interface FrameBoundingBox3DBasics extends FixedFrameBoundingBox3DBasics
{
   void setReferenceFrame(ReferenceFrame referenceFrame);

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

   default void setIncludingFrame(FramePoint3DReadOnly min, FramePoint3DReadOnly max)
   {
      min.checkReferenceFrameMatch(max);
      setIncludingFrame(min.getReferenceFrame(), min, max);
   }

   default void setIncludingFrame(ReferenceFrame referenceFrame, Point3DReadOnly min, Point3DReadOnly max)
   {
      setReferenceFrame(referenceFrame);
      set(min, max);
   }

   default void setIncludingFrame(FramePoint3DReadOnly center, FrameVector3DReadOnly halfSize)
   {
      center.checkReferenceFrameMatch(halfSize);
      setIncludingFrame(center.getReferenceFrame(), center, halfSize);
   }

   default void setIncludingFrame(ReferenceFrame referenceFrame, Point3DReadOnly center, Vector3DReadOnly halfSize)
   {
      setReferenceFrame(referenceFrame);
      set(center, halfSize);
   }

   default void setIncludingFrame(FrameBoundingBox3DReadOnly other)
   {
      setIncludingFrame(other.getReferenceFrame(), other);
   }

   default void setIncludingFrame(ReferenceFrame referenceFrame, BoundingBox3DReadOnly boundingBox3DReadOnly)
   {
      setReferenceFrame(referenceFrame);
      set(boundingBox3DReadOnly);
   }
}
