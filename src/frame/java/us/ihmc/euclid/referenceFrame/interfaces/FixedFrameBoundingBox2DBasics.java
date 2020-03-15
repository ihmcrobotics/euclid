package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.geometry.interfaces.BoundingBox2DBasics;
import us.ihmc.euclid.geometry.interfaces.BoundingBox2DReadOnly;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;

public interface FixedFrameBoundingBox2DBasics extends BoundingBox2DBasics, FrameBoundingBox2DReadOnly
{
   @Override
   FixedFramePoint2DBasics getMinPoint();

   @Override
   FixedFramePoint2DBasics getMaxPoint();

   default void setMin(FramePoint2DReadOnly min)
   {
      checkReferenceFrameMatch(min);
      setMin((Point2DReadOnly) min);
   }

   default void setMax(FramePoint2DReadOnly max)
   {
      checkReferenceFrameMatch(max);
      setMax((Point2DReadOnly) max);
   }

   default void set(FramePoint2DReadOnly min, FramePoint2DReadOnly max)
   {
      checkReferenceFrameMatch(min, max);
      set((Point2DReadOnly) min, (Point2DReadOnly) max);
   }

   default void set(FramePoint2DReadOnly center, Vector2DReadOnly halfSize)
   {
      checkReferenceFrameMatch(center);
      set((Point2DReadOnly) center, halfSize);
   }

   default void set(FramePoint2DReadOnly center, FrameVector2DReadOnly halfSize)
   {
      checkReferenceFrameMatch(halfSize);
      set(center, (Vector2DReadOnly) halfSize);
   }

   default void set(FrameBoundingBox2DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      set((BoundingBox2DReadOnly) other);
   }

   default void combine(FrameBoundingBox2DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      combine((BoundingBox2DReadOnly) other);
   }

   default void combine(FrameBoundingBox2DReadOnly boundingBoxOne, BoundingBox2DReadOnly boundingBoxTwo)
   {
      checkReferenceFrameMatch(boundingBoxOne);
      combine((BoundingBox2DReadOnly) boundingBoxOne, boundingBoxTwo);
   }

   default void combine(BoundingBox2DReadOnly boundingBoxOne, FrameBoundingBox2DReadOnly boundingBoxTwo)
   {
      checkReferenceFrameMatch(boundingBoxTwo);
      combine(boundingBoxOne, (BoundingBox2DReadOnly) boundingBoxTwo);
   }

   default void combine(FrameBoundingBox2DReadOnly boundingBoxOne, FrameBoundingBox2DReadOnly boundingBoxTwo)
   {
      checkReferenceFrameMatch(boundingBoxTwo);
      combine(boundingBoxOne, (BoundingBox2DReadOnly) boundingBoxTwo);
   }

   default void updateToIncludePoints(FrameVertex2DSupplier vertex2DSupplier)
   {
      checkReferenceFrameMatch(vertex2DSupplier);
      updateToIncludePoints((Vertex2DSupplier) vertex2DSupplier);
   }

   default void updateToIncludePoint(FramePoint2DReadOnly point)
   {
      checkReferenceFrameMatch(point);
      updateToIncludePoint((Point2DReadOnly) point);
   }
}
