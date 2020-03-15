package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.geometry.interfaces.BoundingBox2DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;

public interface FrameBoundingBox2DBasics extends FixedFrameBoundingBox2DBasics
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

   default void setIncludingFrame(FramePoint2DReadOnly min, FramePoint2DReadOnly max)
   {
      min.checkReferenceFrameMatch(max);
      setReferenceFrame(min.getReferenceFrame());
      set((Point2DReadOnly) min, (Point2DReadOnly) max);
   }

   default void setIncludingFrame(ReferenceFrame referenceFrame, Point2DReadOnly min, Point2DReadOnly max)
   {
      setReferenceFrame(referenceFrame);
      set((Point2DReadOnly) min, (Point2DReadOnly) max);
   }

   default void setIncludingFrame(FramePoint2DReadOnly center, FrameVector2DReadOnly halfSize)
   {
      center.checkReferenceFrameMatch(halfSize);
      setReferenceFrame(center.getReferenceFrame());
      set((Point2DReadOnly) center, (Vector2DReadOnly) halfSize);
   }

   default void setIncludingFrame(ReferenceFrame referenceFrame, Point2DReadOnly center, Vector2DReadOnly halfSize)
   {
      setReferenceFrame(referenceFrame);
      set((Point2DReadOnly) center, (Vector2DReadOnly) halfSize);
   }

   default void setIncludingFrame(FrameBoundingBox2DReadOnly other)
   {
      setReferenceFrame(other.getReferenceFrame());
      set((BoundingBox2DReadOnly) other);
   }

   default void setIncludingFrame(ReferenceFrame referenceFrame, BoundingBox2DReadOnly boundingBox2DReadOnly)
   {
      setReferenceFrame(referenceFrame);
      set((BoundingBox2DReadOnly) boundingBox2DReadOnly);
   }
}
