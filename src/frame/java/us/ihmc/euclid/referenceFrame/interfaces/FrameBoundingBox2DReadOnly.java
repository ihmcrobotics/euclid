package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.geometry.interfaces.BoundingBox2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;

public interface FrameBoundingBox2DReadOnly extends BoundingBox2DReadOnly, ReferenceFrameHolder
{
   @Override
   FramePoint2DReadOnly getMinPoint();

   @Override
   FramePoint2DReadOnly getMaxPoint();

   default void getCenterPoint(FixedFramePoint2DBasics centerToPack)
   {
      checkReferenceFrameMatch(centerToPack);
      BoundingBox2DReadOnly.super.getCenterPoint(centerToPack);
   }

   default void getCenterPoint(FramePoint2DBasics centerToPack)
   {
      centerToPack.setReferenceFrame(getReferenceFrame());
      BoundingBox2DReadOnly.super.getCenterPoint(centerToPack);
   }

   default void getPointGivenParameters(double xParameter, double yParameter, FixedFramePoint2DBasics pointToPack)
   {
      checkReferenceFrameMatch(pointToPack);
      BoundingBox2DReadOnly.super.getPointGivenParameters(xParameter, yParameter, pointToPack);
   }

   default void getPointGivenParameters(double xParameter, double yParameter, FramePoint2DBasics pointToPack)
   {
      pointToPack.setReferenceFrame(getReferenceFrame());
      BoundingBox2DReadOnly.super.getPointGivenParameters(xParameter, yParameter, pointToPack);
   }

   default boolean isInsideExclusive(FramePoint2DReadOnly query)
   {
      checkReferenceFrameMatch(query);
      return BoundingBox2DReadOnly.super.isInsideExclusive(query);
   }

   default boolean isInsideInclusive(FramePoint2DReadOnly query)
   {
      checkReferenceFrameMatch(query);
      return BoundingBox2DReadOnly.super.isInsideInclusive(query);
   }

   default boolean isInsideEpsilon(FramePoint2DReadOnly query, double epsilon)
   {
      checkReferenceFrameMatch(query);
      return BoundingBox2DReadOnly.super.isInsideEpsilon(query, epsilon);
   }

   default boolean intersectsExclusive(FrameBoundingBox2DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      return BoundingBox2DReadOnly.super.intersectsExclusive(other);
   }

   default boolean intersectsInclusive(FrameBoundingBox2DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      return BoundingBox2DReadOnly.super.intersectsInclusive(other);
   }

   default boolean intersectsEpsilon(FrameBoundingBox2DReadOnly other, double epsilon)
   {
      checkReferenceFrameMatch(other);
      return BoundingBox2DReadOnly.super.intersectsEpsilon(other, epsilon);
   }

   default boolean doesIntersectWithLine2D(FramePoint2DReadOnly pointOnLine, FrameVector2DReadOnly lineDirection)
   {
      checkReferenceFrameMatch(pointOnLine, lineDirection);
      return BoundingBox2DReadOnly.super.doesIntersectWithLine2D(pointOnLine, lineDirection);
   }

   default boolean doesIntersectWithLineSegment2D(FramePoint2DReadOnly lineSegmentStart, FramePoint2DReadOnly lineSegmentEnd)
   {
      checkReferenceFrameMatch(lineSegmentStart, lineSegmentEnd);
      return BoundingBox2DReadOnly.super.doesIntersectWithLineSegment2D(lineSegmentStart, lineSegmentEnd);
   }

   default boolean doesIntersectWithRay2D(FramePoint2DReadOnly rayOrigin, FrameVector2DReadOnly rayDirection)
   {
      checkReferenceFrameMatch(rayOrigin, rayDirection);
      return BoundingBox2DReadOnly.super.doesIntersectWithRay2D(rayOrigin, rayDirection);
   }

   default int intersectionWithLine2D(FramePoint2DReadOnly pointOnLine, FrameVector2DReadOnly lineDirection, Point2DBasics firstIntersectionToPack,
                                      Point2DBasics secondIntersectionToPack)
   {
      checkReferenceFrameMatch(pointOnLine, lineDirection);
      return BoundingBox2DReadOnly.super.intersectionWithLine2D(pointOnLine, lineDirection, firstIntersectionToPack, secondIntersectionToPack);
   }

   default int intersectionWithLine2D(Point2DReadOnly pointOnLine, Vector2DReadOnly lineDirection, FixedFramePoint2DBasics firstIntersectionToPack,
                                      FixedFramePoint2DBasics secondIntersectionToPack)
   {
      if (firstIntersectionToPack != null)
         checkReferenceFrameMatch(firstIntersectionToPack);
      if (secondIntersectionToPack != null)
         checkReferenceFrameMatch(secondIntersectionToPack);
      return BoundingBox2DReadOnly.super.intersectionWithLine2D(pointOnLine, lineDirection, firstIntersectionToPack, secondIntersectionToPack);
   }

   default int intersectionWithLine2D(Point2DReadOnly pointOnLine, Vector2DReadOnly lineDirection, FramePoint2DBasics firstIntersectionToPack,
                                      FramePoint2DBasics secondIntersectionToPack)
   {
      if (firstIntersectionToPack != null)
         firstIntersectionToPack.setReferenceFrame(getReferenceFrame());
      if (secondIntersectionToPack != null)
         secondIntersectionToPack.setReferenceFrame(getReferenceFrame());
      return BoundingBox2DReadOnly.super.intersectionWithLine2D(pointOnLine, lineDirection, firstIntersectionToPack, secondIntersectionToPack);
   }

   default int intersectionWithLine2D(FramePoint2DReadOnly pointOnLine, FrameVector2DReadOnly lineDirection, FixedFramePoint2DBasics firstIntersectionToPack,
                                      FixedFramePoint2DBasics secondIntersectionToPack)
   {
      checkReferenceFrameMatch(pointOnLine, lineDirection);
      if (firstIntersectionToPack != null)
         checkReferenceFrameMatch(firstIntersectionToPack);
      if (secondIntersectionToPack != null)
         checkReferenceFrameMatch(secondIntersectionToPack);
      return BoundingBox2DReadOnly.super.intersectionWithLine2D(pointOnLine, lineDirection, firstIntersectionToPack, secondIntersectionToPack);
   }

   default int intersectionWithLine2D(FramePoint2DReadOnly pointOnLine, FrameVector2DReadOnly lineDirection, FramePoint2DBasics firstIntersectionToPack,
                                      FramePoint2DBasics secondIntersectionToPack)
   {
      checkReferenceFrameMatch(pointOnLine, lineDirection);
      if (firstIntersectionToPack != null)
         firstIntersectionToPack.setReferenceFrame(getReferenceFrame());
      if (secondIntersectionToPack != null)
         secondIntersectionToPack.setReferenceFrame(getReferenceFrame());
      return BoundingBox2DReadOnly.super.intersectionWithLine2D(pointOnLine, lineDirection, firstIntersectionToPack, secondIntersectionToPack);
   }

   default int intersectionWithLineSegment2D(FramePoint2DReadOnly lineSegmentStart, FramePoint2DReadOnly lineSegmentEnd, Point2DBasics firstIntersectionToPack,
                                             Point2DBasics secondIntersectionToPack)
   {
      checkReferenceFrameMatch(lineSegmentStart, lineSegmentEnd);
      return BoundingBox2DReadOnly.super.intersectionWithLineSegment2D(lineSegmentStart, lineSegmentEnd, firstIntersectionToPack, secondIntersectionToPack);
   }

   default int intersectionWithLineSegment2D(Point2DReadOnly lineSegmentStart, Point2DReadOnly lineSegmentEnd, FixedFramePoint2DBasics firstIntersectionToPack,
                                             FixedFramePoint2DBasics secondIntersectionToPack)
   {
      if (firstIntersectionToPack != null)
         checkReferenceFrameMatch(firstIntersectionToPack);
      if (secondIntersectionToPack != null)
         checkReferenceFrameMatch(secondIntersectionToPack);
      return BoundingBox2DReadOnly.super.intersectionWithLineSegment2D(lineSegmentStart, lineSegmentEnd, firstIntersectionToPack, secondIntersectionToPack);
   }

   default int intersectionWithLineSegment2D(Point2DReadOnly lineSegmentStart, Point2DReadOnly lineSegmentEnd, FramePoint2DBasics firstIntersectionToPack,
                                             FramePoint2DBasics secondIntersectionToPack)
   {
      if (firstIntersectionToPack != null)
         firstIntersectionToPack.setReferenceFrame(getReferenceFrame());
      if (secondIntersectionToPack != null)
         secondIntersectionToPack.setReferenceFrame(getReferenceFrame());
      return BoundingBox2DReadOnly.super.intersectionWithLineSegment2D(lineSegmentStart, lineSegmentEnd, firstIntersectionToPack, secondIntersectionToPack);
   }

   default int intersectionWithLineSegment2D(FramePoint2DReadOnly lineSegmentStart, FramePoint2DReadOnly lineSegmentEnd,
                                             FixedFramePoint2DBasics firstIntersectionToPack, FixedFramePoint2DBasics secondIntersectionToPack)
   {
      checkReferenceFrameMatch(lineSegmentStart, lineSegmentEnd);
      if (firstIntersectionToPack != null)
         checkReferenceFrameMatch(firstIntersectionToPack);
      if (secondIntersectionToPack != null)
         checkReferenceFrameMatch(secondIntersectionToPack);
      return BoundingBox2DReadOnly.super.intersectionWithLineSegment2D(lineSegmentStart, lineSegmentEnd, firstIntersectionToPack, secondIntersectionToPack);
   }

   default int intersectionWithLineSegment2D(FramePoint2DReadOnly lineSegmentStart, FramePoint2DReadOnly lineSegmentEnd,
                                             FramePoint2DBasics firstIntersectionToPack, FramePoint2DBasics secondIntersectionToPack)
   {
      checkReferenceFrameMatch(lineSegmentStart, lineSegmentEnd);
      if (firstIntersectionToPack != null)
         firstIntersectionToPack.setReferenceFrame(getReferenceFrame());
      if (secondIntersectionToPack != null)
         secondIntersectionToPack.setReferenceFrame(getReferenceFrame());
      return BoundingBox2DReadOnly.super.intersectionWithLineSegment2D(lineSegmentStart, lineSegmentEnd, firstIntersectionToPack, secondIntersectionToPack);
   }

   default int intersectionWithRay2D(FramePoint2DReadOnly rayOrigin, FrameVector2DReadOnly rayDirection, Point2DBasics firstIntersectionToPack,
                                     Point2DBasics secondIntersectionToPack)
   {
      checkReferenceFrameMatch(rayOrigin, rayDirection);
      return BoundingBox2DReadOnly.super.intersectionWithRay2D(rayOrigin, rayDirection, firstIntersectionToPack, secondIntersectionToPack);
   }

   default int intersectionWithRay2D(Point2DReadOnly rayOrigin, Vector2DReadOnly rayDirection, FramePoint2DBasics firstIntersectionToPack,
                                     FramePoint2DBasics secondIntersectionToPack)
   {
      if (firstIntersectionToPack != null)
         firstIntersectionToPack.setReferenceFrame(getReferenceFrame());
      if (secondIntersectionToPack != null)
         secondIntersectionToPack.setReferenceFrame(getReferenceFrame());
      return BoundingBox2DReadOnly.super.intersectionWithRay2D(rayOrigin, rayDirection, firstIntersectionToPack, secondIntersectionToPack);
   }

   default int intersectionWithRay2D(Point2DReadOnly rayOrigin, Vector2DReadOnly rayDirection, FixedFramePoint2DBasics firstIntersectionToPack,
                                     FixedFramePoint2DBasics secondIntersectionToPack)
   {
      if (firstIntersectionToPack != null)
         checkReferenceFrameMatch(firstIntersectionToPack);
      if (secondIntersectionToPack != null)
         checkReferenceFrameMatch(secondIntersectionToPack);
      return BoundingBox2DReadOnly.super.intersectionWithRay2D(rayOrigin, rayDirection, firstIntersectionToPack, secondIntersectionToPack);
   }

   default int intersectionWithRay2D(FramePoint2DReadOnly rayOrigin, FrameVector2DReadOnly rayDirection, FixedFramePoint2DBasics firstIntersectionToPack,
                                     FixedFramePoint2DBasics secondIntersectionToPack)
   {
      checkReferenceFrameMatch(rayOrigin, rayDirection);
      if (firstIntersectionToPack != null)
         checkReferenceFrameMatch(firstIntersectionToPack);
      if (secondIntersectionToPack != null)
         checkReferenceFrameMatch(secondIntersectionToPack);
      return BoundingBox2DReadOnly.super.intersectionWithRay2D(rayOrigin, rayDirection, firstIntersectionToPack, secondIntersectionToPack);
   }

   default boolean equals(FrameBoundingBox2DReadOnly other)
   {
      if (other == this)
         return true;
      else if (other == null || getReferenceFrame() != other.getReferenceFrame())
         return false;
      else
         return getMinPoint().equals(other.getMinPoint()) && getMaxPoint().equals(other.getMaxPoint());
   }

   default boolean epsilonEquals(FrameBoundingBox2DReadOnly other, double epsilon)
   {
      if (getReferenceFrame() != other.getReferenceFrame())
         return false;
      return BoundingBox2DReadOnly.super.epsilonEquals(other, epsilon);
   }

   default boolean geometricallyEquals(FrameBoundingBox2DReadOnly other, double epsilon)
   {
      checkReferenceFrameMatch(other);
      return BoundingBox2DReadOnly.super.geometricallyEquals(other, epsilon);
   }
}
