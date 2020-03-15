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
      setReferenceFrame(min.getReferenceFrame());
      set((Point3DReadOnly) min, (Point3DReadOnly) max);
   }

   default void setIncludingFrame(ReferenceFrame referenceFrame, Point3DReadOnly min, Point3DReadOnly max)
   {
      setReferenceFrame(referenceFrame);
      set((Point3DReadOnly) min, (Point3DReadOnly) max);
   }

   default void setIncludingFrame(FramePoint3DReadOnly center, FrameVector3DReadOnly halfSize)
   {
      center.checkReferenceFrameMatch(halfSize);
      setReferenceFrame(center.getReferenceFrame());
      set((Point3DReadOnly) center, (Vector3DReadOnly) halfSize);
   }

   default void setIncludingFrame(ReferenceFrame referenceFrame, Point3DReadOnly center, Vector3DReadOnly halfSize)
   {
      setReferenceFrame(referenceFrame);
      set((Point3DReadOnly) center, (Vector3DReadOnly) halfSize);
   }

   default void setIncludingFrame(FrameBoundingBox3DReadOnly other)
   {
      setReferenceFrame(other.getReferenceFrame());
      set((BoundingBox3DReadOnly) other);
   }

   default void setIncludingFrame(ReferenceFrame referenceFrame, BoundingBox3DReadOnly boundingBox3DReadOnly)
   {
      setReferenceFrame(referenceFrame);
      set((BoundingBox3DReadOnly) boundingBox3DReadOnly);
   }
}
