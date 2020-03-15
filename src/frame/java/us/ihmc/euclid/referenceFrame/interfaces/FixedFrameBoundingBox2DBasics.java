package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.geometry.interfaces.BoundingBox2DBasics;
import us.ihmc.euclid.geometry.interfaces.BoundingBox2DReadOnly;
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
      BoundingBox2DBasics.super.setMin(min);
   }

   default void setMax(FramePoint2DReadOnly max)
   {
      checkReferenceFrameMatch(max);
      BoundingBox2DBasics.super.setMax(max);
   }

   default void set(FramePoint2DReadOnly min, FramePoint2DReadOnly max)
   {
      checkReferenceFrameMatch(min, max);
      BoundingBox2DBasics.super.set(min, max);
   }

   default void set(FramePoint2DReadOnly center, Vector2DReadOnly halfSize)
   {
      checkReferenceFrameMatch(center);
      BoundingBox2DBasics.super.set(center, halfSize);
   }

   default void set(FramePoint2DReadOnly center, FrameVector2DReadOnly halfSize)
   {
      checkReferenceFrameMatch(center, halfSize);
      BoundingBox2DBasics.super.set(center, halfSize);
   }

   default void set(FrameBoundingBox2DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      BoundingBox2DBasics.super.set(other);
   }

   default void combine(FrameBoundingBox2DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      BoundingBox2DBasics.super.combine(other);
   }

   default void combine(FrameBoundingBox2DReadOnly boundingBoxOne, BoundingBox2DReadOnly boundingBoxTwo)
   {
      checkReferenceFrameMatch(boundingBoxOne);
      BoundingBox2DBasics.super.combine(boundingBoxOne, boundingBoxTwo);
   }

   default void combine(BoundingBox2DReadOnly boundingBoxOne, FrameBoundingBox2DReadOnly boundingBoxTwo)
   {
      checkReferenceFrameMatch(boundingBoxTwo);
      BoundingBox2DBasics.super.combine(boundingBoxOne, boundingBoxTwo);
   }

   default void combine(FrameBoundingBox2DReadOnly boundingBoxOne, FrameBoundingBox2DReadOnly boundingBoxTwo)
   {
      checkReferenceFrameMatch(boundingBoxOne, boundingBoxTwo);
      BoundingBox2DBasics.super.combine(boundingBoxOne, boundingBoxTwo);
   }

   default void updateToIncludePoints(FrameVertex2DSupplier vertex2DSupplier)
   {
      checkReferenceFrameMatch(vertex2DSupplier);
      BoundingBox2DBasics.super.updateToIncludePoints(vertex2DSupplier);
   }

   default void updateToIncludePoint(FramePoint2DReadOnly point)
   {
      checkReferenceFrameMatch(point);
      BoundingBox2DBasics.super.updateToIncludePoint(point);
   }
}
