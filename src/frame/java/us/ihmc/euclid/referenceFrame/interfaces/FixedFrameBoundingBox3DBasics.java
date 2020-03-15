package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.geometry.interfaces.BoundingBox3DBasics;
import us.ihmc.euclid.geometry.interfaces.BoundingBox3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public interface FixedFrameBoundingBox3DBasics extends BoundingBox3DBasics, FrameBoundingBox3DReadOnly
{
   @Override
   FixedFramePoint3DBasics getMinPoint();

   @Override
   FixedFramePoint3DBasics getMaxPoint();

   default void setMin(FramePoint3DReadOnly min)
   {
      checkReferenceFrameMatch(min);
      BoundingBox3DBasics.super.setMin(min);
   }

   default void setMax(FramePoint3DReadOnly max)
   {
      checkReferenceFrameMatch(max);
      BoundingBox3DBasics.super.setMax(max);
   }

   default void set(FramePoint3DReadOnly min, FramePoint3DReadOnly max)
   {
      checkReferenceFrameMatch(min, max);
      BoundingBox3DBasics.super.set(min, max);
   }

   default void set(FramePoint3DReadOnly center, Vector3DReadOnly halfSize)
   {
      checkReferenceFrameMatch(center);
      BoundingBox3DBasics.super.set(center, halfSize);
   }

   default void set(FramePoint3DReadOnly center, FrameVector3DReadOnly halfSize)
   {
      checkReferenceFrameMatch(center, halfSize);
      BoundingBox3DBasics.super.set(center, halfSize);
   }

   default void set(FrameBoundingBox3DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      BoundingBox3DBasics.super.set(other);
   }

   default void combine(FrameBoundingBox3DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      BoundingBox3DBasics.super.combine(other);
   }

   default void combine(FrameBoundingBox3DReadOnly boundingBoxOne, BoundingBox3DReadOnly boundingBoxTwo)
   {
      checkReferenceFrameMatch(boundingBoxOne);
      BoundingBox3DBasics.super.combine(boundingBoxOne, boundingBoxTwo);
   }

   default void combine(BoundingBox3DReadOnly boundingBoxOne, FrameBoundingBox3DReadOnly boundingBoxTwo)
   {
      checkReferenceFrameMatch(boundingBoxTwo);
      BoundingBox3DBasics.super.combine(boundingBoxOne, boundingBoxTwo);
   }

   default void combine(FrameBoundingBox3DReadOnly boundingBoxOne, FrameBoundingBox3DReadOnly boundingBoxTwo)
   {
      checkReferenceFrameMatch(boundingBoxOne, boundingBoxTwo);
      BoundingBox3DBasics.super.combine(boundingBoxOne, boundingBoxTwo);
   }

   default void updateToIncludePoint(FramePoint3DReadOnly point)
   {
      checkReferenceFrameMatch(point);
      BoundingBox3DBasics.super.updateToIncludePoint(point);
   }
}
