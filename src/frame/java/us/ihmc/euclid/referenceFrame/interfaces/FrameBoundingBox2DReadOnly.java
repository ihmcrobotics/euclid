package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.geometry.interfaces.BoundingBox2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;

public interface FrameBoundingBox2DReadOnly extends BoundingBox2DReadOnly, ReferenceFrameHolder
{
   FramePoint2DReadOnly getMinPoint();

   FramePoint2DReadOnly getMaxPoint();

   default void getMinPoint(FixedFramePoint2DBasics minToPack)
   {
      minToPack.set(getMinPoint());
   }

   default void getMinPoint(FramePoint2DBasics minToPack)
   {
      minToPack.setIncludingFrame(getMinPoint());
   }

   default void getMaxPoint(FixedFramePoint2DBasics maxToPack)
   {
      maxToPack.set(getMaxPoint());
   }

   default void getMaxPoint(FramePoint2DBasics maxToPack)
   {
      maxToPack.setIncludingFrame(getMaxPoint());
   }

   default void getCenterPoint(FixedFramePoint2DBasics centerToPack)
   {
      checkReferenceFrameMatch(centerToPack);
      getCenterPoint((Point2DBasics) centerToPack);
   }

   default void getCenterPoint(FramePoint2DBasics centerToPack)
   {
      centerToPack.setReferenceFrame(getReferenceFrame());
      getCenterPoint((Point2DBasics) centerToPack);
   }

   default void getPointGivenParameters(double xParameter, double yParameter, FixedFramePoint2DBasics pointToPack)
   {
      checkReferenceFrameMatch(pointToPack);
      getPointGivenParameters(xParameter, yParameter, (Point2DBasics) pointToPack);
   }

   default void getPointGivenParameters(double xParameter, double yParameter, FramePoint2DBasics pointToPack)
   {
      pointToPack.setReferenceFrame(getReferenceFrame());
      getPointGivenParameters(xParameter, yParameter, (Point2DBasics) pointToPack);
   }

   default boolean isInsideExclusive(FramePoint2DReadOnly query)
   {
      checkReferenceFrameMatch(query);
      return isInsideExclusive((Point2DReadOnly) query);
   }

   default boolean isInsideInclusive(FramePoint2DReadOnly query)
   {
      checkReferenceFrameMatch(query);
      return isInsideInclusive((Point2DReadOnly) query);
   }

   default boolean isInsideEpsilon(FramePoint2DReadOnly query, double epsilon)
   {
      checkReferenceFrameMatch(query);
      return isInsideEpsilon((Point2DReadOnly) query, epsilon);
   }

   default boolean intersectsExclusive(FrameBoundingBox2DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      return intersectsExclusive((BoundingBox2DReadOnly) other);
   }

   default boolean intersectsInclusive(FrameBoundingBox2DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      return intersectsInclusive((BoundingBox2DReadOnly) other);
   }

   default boolean intersectsEpsilon(FrameBoundingBox2DReadOnly other, double epsilon)
   {
      checkReferenceFrameMatch(other);
      return intersectsEpsilon((BoundingBox2DReadOnly) other, epsilon);
   }

   default boolean doesIntersectWithLine2D(FramePoint2DReadOnly pointOnLine, FrameVector2DReadOnly lineDirection)
   {
      checkReferenceFrameMatch(pointOnLine);
      return doesIntersectWithLine2D((Point2DReadOnly) pointOnLine, lineDirection);
   }

   default boolean doesIntersectWithLineSegment2D(FramePoint2DReadOnly lineSegmentStart, FramePoint2DReadOnly lineSegmentEnd)
   {
      checkReferenceFrameMatch(lineSegmentStart);
      return doesIntersectWithLineSegment2D((Point2DReadOnly) lineSegmentStart, lineSegmentEnd);
   }

   default boolean doesIntersectWithRay2D(FramePoint2DReadOnly rayOrigin, FrameVector2DReadOnly rayDirection)
   {
      checkReferenceFrameMatch(rayOrigin);
      return doesIntersectWithRay2D((Point2DReadOnly) rayOrigin, rayDirection);
   }

   default int intersectionWithLine2D(FramePoint2DReadOnly pointOnLine, FrameVector2DReadOnly lineDirection, Point2DBasics firstIntersectionToPack,
                                      Point2DBasics secondIntersectionToPack)
   {
      checkReferenceFrameMatch(pointOnLine, lineDirection);
      return intersectionWithLine2D((Point2DReadOnly) pointOnLine, (Vector2DReadOnly) lineDirection, firstIntersectionToPack, secondIntersectionToPack);
   }

   default int intersectionWithLine2D(Point2DReadOnly pointOnLine, Vector2DReadOnly lineDirection, FixedFramePoint2DBasics firstIntersectionToPack,
                                      FixedFramePoint2DBasics secondIntersectionToPack)
   {
      if (firstIntersectionToPack != null)
         checkReferenceFrameMatch(firstIntersectionToPack);
      if (secondIntersectionToPack != null)
         checkReferenceFrameMatch(secondIntersectionToPack);
      return intersectionWithLine2D(pointOnLine, lineDirection, (Point2DBasics) firstIntersectionToPack, (Point2DBasics) secondIntersectionToPack);
   }

   default int intersectionWithLine2D(Point2DReadOnly pointOnLine, Vector2DReadOnly lineDirection, FramePoint2DBasics firstIntersectionToPack,
                                      FramePoint2DBasics secondIntersectionToPack)
   {
      if (firstIntersectionToPack != null)
         firstIntersectionToPack.setReferenceFrame(getReferenceFrame());
      if (secondIntersectionToPack != null)
         secondIntersectionToPack.setReferenceFrame(getReferenceFrame());
      return intersectionWithLine2D(pointOnLine, lineDirection, (Point2DBasics) firstIntersectionToPack, (Point2DBasics) secondIntersectionToPack);
   }

   default int intersectionWithLine2D(FramePoint2DReadOnly pointOnLine, FrameVector2DReadOnly lineDirection, FixedFramePoint2DBasics firstIntersectionToPack,
                                      FixedFramePoint2DBasics secondIntersectionToPack)
   {
      checkReferenceFrameMatch(pointOnLine, lineDirection);
      return intersectionWithLine2D((Point2DReadOnly) pointOnLine, (Vector2DReadOnly) lineDirection, firstIntersectionToPack, secondIntersectionToPack);
   }

   default int intersectionWithLine2D(FramePoint2DReadOnly pointOnLine, FrameVector2DReadOnly lineDirection, FramePoint2DBasics firstIntersectionToPack,
                                      FramePoint2DBasics secondIntersectionToPack)
   {
      checkReferenceFrameMatch(pointOnLine, lineDirection);
      return intersectionWithLine2D((Point2DReadOnly) pointOnLine, (Vector2DReadOnly) lineDirection, firstIntersectionToPack, secondIntersectionToPack);
   }

   default int intersectionWithLineSegment2D(FramePoint2DReadOnly lineSegmentStart, FramePoint2DReadOnly lineSegmentEnd, Point2DBasics firstIntersectionToPack,
                                             Point2DBasics secondIntersectionToPack)
   {
      checkReferenceFrameMatch(lineSegmentStart, lineSegmentEnd);
      return intersectionWithLineSegment2D((Point2DReadOnly) lineSegmentStart,
                                           (Point2DReadOnly) lineSegmentEnd,
                                           firstIntersectionToPack,
                                           secondIntersectionToPack);
   }

   default int intersectionWithLineSegment2D(Point2DReadOnly lineSegmentStart, Point2DReadOnly lineSegmentEnd, FixedFramePoint2DBasics firstIntersectionToPack,
                                             FixedFramePoint2DBasics secondIntersectionToPack)
   {
      if (firstIntersectionToPack != null)
         checkReferenceFrameMatch(firstIntersectionToPack);
      if (secondIntersectionToPack != null)
         checkReferenceFrameMatch(secondIntersectionToPack);
      return intersectionWithLineSegment2D(lineSegmentStart, lineSegmentEnd, (Point2DBasics) firstIntersectionToPack, (Point2DBasics) secondIntersectionToPack);
   }

   default int intersectionWithLineSegment2D(Point2DReadOnly lineSegmentStart, Point2DReadOnly lineSegmentEnd, FramePoint2DBasics firstIntersectionToPack,
                                             FramePoint2DBasics secondIntersectionToPack)
   {
      if (firstIntersectionToPack != null)
         firstIntersectionToPack.setReferenceFrame(getReferenceFrame());
      if (secondIntersectionToPack != null)
         secondIntersectionToPack.setReferenceFrame(getReferenceFrame());
      return intersectionWithLineSegment2D(lineSegmentStart, lineSegmentEnd, (Point2DBasics) firstIntersectionToPack, (Point2DBasics) secondIntersectionToPack);
   }

   default int intersectionWithLineSegment2D(FramePoint2DReadOnly lineSegmentStart, FramePoint2DReadOnly lineSegmentEnd,
                                             FixedFramePoint2DBasics firstIntersectionToPack, FixedFramePoint2DBasics secondIntersectionToPack)
   {
      checkReferenceFrameMatch(lineSegmentStart, lineSegmentEnd);
      return intersectionWithLineSegment2D((Point2DReadOnly) lineSegmentStart,
                                           (Point2DReadOnly) lineSegmentEnd,
                                           firstIntersectionToPack,
                                           secondIntersectionToPack);
   }

   default int intersectionWithLineSegment2D(FramePoint2DReadOnly lineSegmentStart, FramePoint2DReadOnly lineSegmentEnd,
                                             FramePoint2DBasics firstIntersectionToPack, FramePoint2DBasics secondIntersectionToPack)
   {
      checkReferenceFrameMatch(lineSegmentStart, lineSegmentEnd);
      return intersectionWithLineSegment2D((Point2DReadOnly) lineSegmentStart,
                                           (Point2DReadOnly) lineSegmentEnd,
                                           firstIntersectionToPack,
                                           secondIntersectionToPack);
   }

   default int intersectionWithRay2D(FramePoint2DReadOnly rayOrigin, FrameVector2DReadOnly rayDirection, Point2DBasics firstIntersectionToPack,
                                     Point2DBasics secondIntersectionToPack)
   {
      checkReferenceFrameMatch(rayOrigin, rayDirection);
      return intersectionWithRay2D((Point2DReadOnly) rayOrigin, (Vector2DReadOnly) rayDirection, firstIntersectionToPack, secondIntersectionToPack);
   }

   default int intersectionWithRay2D(Point2DReadOnly rayOrigin, Vector2DReadOnly rayDirection, FramePoint2DBasics firstIntersectionToPack,
                                     FramePoint2DBasics secondIntersectionToPack)
   {
      if (firstIntersectionToPack != null)
         firstIntersectionToPack.setReferenceFrame(getReferenceFrame());
      if (secondIntersectionToPack != null)
         secondIntersectionToPack.setReferenceFrame(getReferenceFrame());
      return intersectionWithRay2D(rayOrigin, rayDirection, (Point2DBasics) firstIntersectionToPack, (Point2DBasics) secondIntersectionToPack);
   }

   default int intersectionWithRay2D(Point2DReadOnly rayOrigin, Vector2DReadOnly rayDirection, FixedFramePoint2DBasics firstIntersectionToPack,
                                     FixedFramePoint2DBasics secondIntersectionToPack)
   {
      if (firstIntersectionToPack != null)
         checkReferenceFrameMatch(firstIntersectionToPack);
      if (secondIntersectionToPack != null)
         checkReferenceFrameMatch(secondIntersectionToPack);
      return intersectionWithRay2D(rayOrigin, rayDirection, (Point2DBasics) firstIntersectionToPack, (Point2DBasics) secondIntersectionToPack);
   }

   default int intersectionWithRay2D(FramePoint2DReadOnly rayOrigin, FrameVector2DReadOnly rayDirection, FixedFramePoint2DBasics firstIntersectionToPack,
                                     FixedFramePoint2DBasics secondIntersectionToPack)
   {
      checkReferenceFrameMatch(rayOrigin, rayDirection);
      return intersectionWithRay2D((Point2DReadOnly) rayOrigin, (Vector2DReadOnly) rayDirection, firstIntersectionToPack, secondIntersectionToPack);
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
      return epsilonEquals((BoundingBox2DReadOnly) other, epsilon);
   }

   default boolean geometricallyEquals(FrameBoundingBox2DReadOnly other, double epsilon)
   {
      checkReferenceFrameMatch(other);
      return geometricallyEquals((BoundingBox2DReadOnly) other, epsilon);
   }
}
