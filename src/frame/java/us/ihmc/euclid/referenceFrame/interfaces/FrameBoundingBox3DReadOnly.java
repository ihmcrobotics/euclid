package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.geometry.interfaces.BoundingBox3DReadOnly;
import us.ihmc.euclid.geometry.interfaces.Line3DReadOnly;
import us.ihmc.euclid.geometry.interfaces.LineSegment3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public interface FrameBoundingBox3DReadOnly extends BoundingBox3DReadOnly, ReferenceFrameHolder
{
   @Override
   FramePoint3DReadOnly getMinPoint();

   @Override
   FramePoint3DReadOnly getMaxPoint();

   default void getCenterPoint(FramePoint3DBasics centerToPack)
   {
      centerToPack.setReferenceFrame(getReferenceFrame());
      BoundingBox3DReadOnly.super.getCenterPoint(centerToPack);
   }

   default void getCenterPoint(FixedFramePoint3DBasics centerToPack)
   {
      checkReferenceFrameMatch(centerToPack);
      BoundingBox3DReadOnly.super.getCenterPoint(centerToPack);
   }

   default void getPointGivenParameters(double xParameter, double yParameter, double zParameter, FixedFramePoint3DBasics pointToPack)
   {
      checkReferenceFrameMatch(pointToPack);
      BoundingBox3DReadOnly.super.getPointGivenParameters(xParameter, yParameter, zParameter, pointToPack);
   }

   default void getPointGivenParameters(double xParameter, double yParameter, double zParameter, FramePoint3DBasics pointToPack)
   {
      pointToPack.setReferenceFrame(getReferenceFrame());
      BoundingBox3DReadOnly.super.getPointGivenParameters(xParameter, yParameter, zParameter, pointToPack);
   }

   default boolean isInsideExclusive(FramePoint3DReadOnly query)
   {
      checkReferenceFrameMatch(query);
      return BoundingBox3DReadOnly.super.isInsideExclusive(query);
   }

   default boolean isInsideInclusive(FramePoint3DReadOnly query)
   {
      checkReferenceFrameMatch(query);
      return BoundingBox3DReadOnly.super.isInsideInclusive(query);
   }

   default boolean isInsideEpsilon(FramePoint3DReadOnly query, double epsilon)
   {
      checkReferenceFrameMatch(query);
      return BoundingBox3DReadOnly.super.isInsideEpsilon(query, epsilon);
   }

   default boolean isXYInsideExclusive(FramePoint2DReadOnly query)
   {
      checkReferenceFrameMatch(query);
      return BoundingBox3DReadOnly.super.isXYInsideExclusive(query);
   }

   default boolean isXYInsideInclusive(FramePoint2DReadOnly query)
   {
      checkReferenceFrameMatch(query);
      return BoundingBox3DReadOnly.super.isXYInsideInclusive(query);
   }

   default boolean isXYInsideEpsilon(FramePoint2DReadOnly query, double epsilon)
   {
      checkReferenceFrameMatch(query);
      return BoundingBox3DReadOnly.super.isXYInsideEpsilon(query, epsilon);
   }

   default boolean intersectsExclusive(FrameBoundingBox3DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      return BoundingBox3DReadOnly.super.intersectsExclusive(other);
   }

   default boolean intersectsInclusive(FrameBoundingBox3DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      return BoundingBox3DReadOnly.super.intersectsInclusive(other);
   }

   default boolean intersectsEpsilon(FrameBoundingBox3DReadOnly other, double epsilon)
   {
      checkReferenceFrameMatch(other);
      return BoundingBox3DReadOnly.super.intersectsEpsilon(other, epsilon);
   }

   default boolean intersectsExclusiveInXYPlane(FrameBoundingBox2DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      return BoundingBox3DReadOnly.super.intersectsExclusiveInXYPlane(other);
   }

   default boolean intersectsInclusiveInXYPlane(FrameBoundingBox2DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      return BoundingBox3DReadOnly.super.intersectsInclusiveInXYPlane(other);
   }

   default boolean intersectsEpsilonInXYPlane(FrameBoundingBox2DReadOnly other, double epsilon)
   {
      checkReferenceFrameMatch(other);
      return BoundingBox3DReadOnly.super.intersectsEpsilonInXYPlane(other, epsilon);
   }

   default boolean doesIntersectWithLine3D(FrameLine3DReadOnly line3D)
   {
      checkReferenceFrameMatch(line3D);
      return BoundingBox3DReadOnly.super.doesIntersectWithLine3D(line3D);
   }

   default boolean doesIntersectWithLine3D(FramePoint3DReadOnly pointOnLine, FrameVector3DReadOnly lineDirection)
   {
      checkReferenceFrameMatch(pointOnLine, lineDirection);
      return BoundingBox3DReadOnly.super.doesIntersectWithLine3D(pointOnLine, lineDirection);
   }

   default boolean doesIntersectWithLineSegment3D(FrameLineSegment3DReadOnly lineSegment3D)
   {
      checkReferenceFrameMatch(lineSegment3D);
      return BoundingBox3DReadOnly.super.doesIntersectWithLineSegment3D(lineSegment3D);
   }

   default boolean doesIntersectWithLineSegment3D(FramePoint3DReadOnly lineSegmentStart, FramePoint3DReadOnly lineSegmentEnd)
   {
      checkReferenceFrameMatch(lineSegmentStart, lineSegmentEnd);
      return BoundingBox3DReadOnly.super.doesIntersectWithLineSegment3D(lineSegmentStart, lineSegmentEnd);
   }

   default boolean doesIntersectWithRay3D(FramePoint3DReadOnly rayOrigin, FrameVector3DReadOnly rayDirection)
   {
      checkReferenceFrameMatch(rayOrigin, rayDirection);
      return BoundingBox3DReadOnly.super.doesIntersectWithRay3D(rayOrigin, rayDirection);
   }

   default int intersectionWithLine3D(FrameLine3DReadOnly line3d, Point3DBasics firstIntersectionToPack, Point3DBasics secondIntersectionToPack)
   {
      checkReferenceFrameMatch(line3d);
      return BoundingBox3DReadOnly.super.intersectionWithLine3D(line3d, firstIntersectionToPack, secondIntersectionToPack);
   }

   default int intersectionWithLine3D(Line3DReadOnly line3d, FixedFramePoint3DBasics firstIntersectionToPack, FixedFramePoint3DBasics secondIntersectionToPack)
   {
      if (firstIntersectionToPack != null)
         checkReferenceFrameMatch(firstIntersectionToPack);
      if (secondIntersectionToPack != null)
         checkReferenceFrameMatch(secondIntersectionToPack);
      return BoundingBox3DReadOnly.super.intersectionWithLine3D(line3d, firstIntersectionToPack, secondIntersectionToPack);
   }

   default int intersectionWithLine3D(Line3DReadOnly line3d, FramePoint3DBasics firstIntersectionToPack, FramePoint3DBasics secondIntersectionToPack)
   {
      if (firstIntersectionToPack != null)
         firstIntersectionToPack.setReferenceFrame(getReferenceFrame());
      if (secondIntersectionToPack != null)
         secondIntersectionToPack.setReferenceFrame(getReferenceFrame());
      return BoundingBox3DReadOnly.super.intersectionWithLine3D(line3d, firstIntersectionToPack, secondIntersectionToPack);
   }

   default int intersectionWithLine3D(FrameLine3DReadOnly line3d, FixedFramePoint3DBasics firstIntersectionToPack,
                                      FixedFramePoint3DBasics secondIntersectionToPack)
   {
      checkReferenceFrameMatch(line3d);
      if (firstIntersectionToPack != null)
         checkReferenceFrameMatch(firstIntersectionToPack);
      if (secondIntersectionToPack != null)
         checkReferenceFrameMatch(secondIntersectionToPack);
      return BoundingBox3DReadOnly.super.intersectionWithLine3D(line3d, firstIntersectionToPack, secondIntersectionToPack);
   }

   default int intersectionWithLine3D(FrameLine3DReadOnly line3d, FramePoint3DBasics firstIntersectionToPack, FramePoint3DBasics secondIntersectionToPack)
   {
      checkReferenceFrameMatch(line3d);
      if (firstIntersectionToPack != null)
         firstIntersectionToPack.setReferenceFrame(getReferenceFrame());
      if (secondIntersectionToPack != null)
         secondIntersectionToPack.setReferenceFrame(getReferenceFrame());
      return BoundingBox3DReadOnly.super.intersectionWithLine3D(line3d, firstIntersectionToPack, secondIntersectionToPack);
   }

   default int intersectionWithLine3D(FramePoint3DReadOnly pointOnLine, FrameVector3DReadOnly lineDirection, Point3DBasics firstIntersectionToPack,
                                      Point3DBasics secondIntersectionToPack)
   {
      checkReferenceFrameMatch(pointOnLine, lineDirection);
      return BoundingBox3DReadOnly.super.intersectionWithLine3D(pointOnLine, lineDirection, firstIntersectionToPack, secondIntersectionToPack);
   }

   default int intersectionWithLine3D(Point3DReadOnly pointOnLine, Vector3DReadOnly lineDirection, FixedFramePoint3DBasics firstIntersectionToPack,
                                      FixedFramePoint3DBasics secondIntersectionToPack)
   {
      if (firstIntersectionToPack != null)
         checkReferenceFrameMatch(firstIntersectionToPack);
      if (secondIntersectionToPack != null)
         checkReferenceFrameMatch(secondIntersectionToPack);
      return BoundingBox3DReadOnly.super.intersectionWithLine3D(pointOnLine, lineDirection, firstIntersectionToPack, secondIntersectionToPack);
   }

   default int intersectionWithLine3D(Point3DReadOnly pointOnLine, Vector3DReadOnly lineDirection, FramePoint3DBasics firstIntersectionToPack,
                                      FramePoint3DBasics secondIntersectionToPack)
   {
      if (firstIntersectionToPack != null)
         firstIntersectionToPack.setReferenceFrame(getReferenceFrame());
      if (secondIntersectionToPack != null)
         secondIntersectionToPack.setReferenceFrame(getReferenceFrame());
      return BoundingBox3DReadOnly.super.intersectionWithLine3D(pointOnLine, lineDirection, firstIntersectionToPack, secondIntersectionToPack);
   }

   default int intersectionWithLine3D(FramePoint3DReadOnly pointOnLine, FrameVector3DReadOnly lineDirection, FixedFramePoint3DBasics firstIntersectionToPack,
                                      FixedFramePoint3DBasics secondIntersectionToPack)
   {
      checkReferenceFrameMatch(pointOnLine, lineDirection);
      if (firstIntersectionToPack != null)
         checkReferenceFrameMatch(firstIntersectionToPack);
      if (secondIntersectionToPack != null)
         checkReferenceFrameMatch(secondIntersectionToPack);
      return BoundingBox3DReadOnly.super.intersectionWithLine3D(pointOnLine, lineDirection, firstIntersectionToPack, secondIntersectionToPack);
   }

   default int intersectionWithLine3D(FramePoint3DReadOnly pointOnLine, FrameVector3DReadOnly lineDirection, FramePoint3DBasics firstIntersectionToPack,
                                      FramePoint3DBasics secondIntersectionToPack)
   {
      checkReferenceFrameMatch(pointOnLine, lineDirection);
      if (firstIntersectionToPack != null)
         firstIntersectionToPack.setReferenceFrame(getReferenceFrame());
      if (secondIntersectionToPack != null)
         secondIntersectionToPack.setReferenceFrame(getReferenceFrame());
      return BoundingBox3DReadOnly.super.intersectionWithLine3D(pointOnLine, lineDirection, firstIntersectionToPack, secondIntersectionToPack);
   }

   default int intersectionWithLineSegment3D(FrameLineSegment3DReadOnly lineSegment3D, Point3DBasics firstIntersectionToPack,
                                             Point3DBasics secondIntersectionToPack)
   {
      checkReferenceFrameMatch(lineSegment3D);
      return BoundingBox3DReadOnly.super.intersectionWithLineSegment3D(lineSegment3D, firstIntersectionToPack, secondIntersectionToPack);
   }

   default int intersectionWithLineSegment3D(LineSegment3DReadOnly lineSegment3D, FixedFramePoint3DBasics firstIntersectionToPack,
                                             FixedFramePoint3DBasics secondIntersectionToPack)
   {
      if (firstIntersectionToPack != null)
         checkReferenceFrameMatch(firstIntersectionToPack);
      if (secondIntersectionToPack != null)
         checkReferenceFrameMatch(secondIntersectionToPack);
      return BoundingBox3DReadOnly.super.intersectionWithLineSegment3D(lineSegment3D, firstIntersectionToPack, secondIntersectionToPack);
   }

   default int intersectionWithLineSegment3D(LineSegment3DReadOnly lineSegment3D, FramePoint3DBasics firstIntersectionToPack,
                                             FramePoint3DBasics secondIntersectionToPack)
   {
      if (firstIntersectionToPack != null)
         firstIntersectionToPack.setReferenceFrame(getReferenceFrame());
      if (secondIntersectionToPack != null)
         secondIntersectionToPack.setReferenceFrame(getReferenceFrame());
      return BoundingBox3DReadOnly.super.intersectionWithLineSegment3D(lineSegment3D, firstIntersectionToPack, secondIntersectionToPack);
   }

   default int intersectionWithLineSegment3D(FrameLineSegment3DReadOnly lineSegment3D, FixedFramePoint3DBasics firstIntersectionToPack,
                                             FixedFramePoint3DBasics secondIntersectionToPack)
   {
      checkReferenceFrameMatch(lineSegment3D);
      if (firstIntersectionToPack != null)
         checkReferenceFrameMatch(firstIntersectionToPack);
      if (secondIntersectionToPack != null)
         checkReferenceFrameMatch(secondIntersectionToPack);
      return BoundingBox3DReadOnly.super.intersectionWithLineSegment3D(lineSegment3D, firstIntersectionToPack, secondIntersectionToPack);
   }

   default int intersectionWithLineSegment3D(FrameLineSegment3DReadOnly lineSegment3D, FramePoint3DBasics firstIntersectionToPack,
                                             FramePoint3DBasics secondIntersectionToPack)
   {
      checkReferenceFrameMatch(lineSegment3D);
      if (firstIntersectionToPack != null)
         firstIntersectionToPack.setReferenceFrame(getReferenceFrame());
      if (secondIntersectionToPack != null)
         secondIntersectionToPack.setReferenceFrame(getReferenceFrame());
      return BoundingBox3DReadOnly.super.intersectionWithLineSegment3D(lineSegment3D, firstIntersectionToPack, secondIntersectionToPack);
   }

   default int intersectionWithLineSegment3D(FramePoint3DReadOnly lineSegmentStart, FramePoint3DReadOnly lineSegmentEnd, Point3DBasics firstIntersectionToPack,
                                             Point3DBasics secondIntersectionToPack)
   {
      checkReferenceFrameMatch(lineSegmentStart, lineSegmentEnd);
      return BoundingBox3DReadOnly.super.intersectionWithLineSegment3D(lineSegmentStart, lineSegmentEnd, firstIntersectionToPack, secondIntersectionToPack);
   }

   default int intersectionWithLineSegment3D(Point3DReadOnly lineSegmentStart, Point3DReadOnly lineSegmentEnd, FixedFramePoint3DBasics firstIntersectionToPack,
                                             FixedFramePoint3DBasics secondIntersectionToPack)
   {
      if (firstIntersectionToPack != null)
         checkReferenceFrameMatch(firstIntersectionToPack);
      if (secondIntersectionToPack != null)
         checkReferenceFrameMatch(secondIntersectionToPack);
      return BoundingBox3DReadOnly.super.intersectionWithLineSegment3D(lineSegmentStart, lineSegmentEnd, firstIntersectionToPack, secondIntersectionToPack);
   }

   default int intersectionWithLineSegment3D(Point3DReadOnly lineSegmentStart, Point3DReadOnly lineSegmentEnd, FramePoint3DBasics firstIntersectionToPack,
                                             FramePoint3DBasics secondIntersectionToPack)
   {
      if (firstIntersectionToPack != null)
         firstIntersectionToPack.setReferenceFrame(getReferenceFrame());
      if (secondIntersectionToPack != null)
         secondIntersectionToPack.setReferenceFrame(getReferenceFrame());
      return BoundingBox3DReadOnly.super.intersectionWithLineSegment3D(lineSegmentStart, lineSegmentEnd, firstIntersectionToPack, secondIntersectionToPack);
   }

   default int intersectionWithLineSegment3D(FramePoint3DReadOnly lineSegmentStart, FramePoint3DReadOnly lineSegmentEnd,
                                             FixedFramePoint3DBasics firstIntersectionToPack, FixedFramePoint3DBasics secondIntersectionToPack)
   {
      checkReferenceFrameMatch(lineSegmentStart, lineSegmentEnd);
      if (firstIntersectionToPack != null)
         checkReferenceFrameMatch(firstIntersectionToPack);
      if (secondIntersectionToPack != null)
         checkReferenceFrameMatch(secondIntersectionToPack);
      return BoundingBox3DReadOnly.super.intersectionWithLineSegment3D(lineSegmentStart, lineSegmentEnd, firstIntersectionToPack, secondIntersectionToPack);
   }

   default int intersectionWithLineSegment3D(FramePoint3DReadOnly lineSegmentStart, FramePoint3DReadOnly lineSegmentEnd,
                                             FramePoint3DBasics firstIntersectionToPack, FramePoint3DBasics secondIntersectionToPack)
   {
      checkReferenceFrameMatch(lineSegmentStart, lineSegmentEnd);
      if (firstIntersectionToPack != null)
         firstIntersectionToPack.setReferenceFrame(getReferenceFrame());
      if (secondIntersectionToPack != null)
         secondIntersectionToPack.setReferenceFrame(getReferenceFrame());
      return BoundingBox3DReadOnly.super.intersectionWithLineSegment3D(lineSegmentStart, lineSegmentEnd, firstIntersectionToPack, secondIntersectionToPack);
   }

   default int intersectionWithRay3D(FramePoint3DReadOnly rayOrigin, FrameVector3DReadOnly rayDirection, Point3DBasics firstIntersectionToPack,
                                     Point3DBasics secondIntersectionToPack)
   {
      checkReferenceFrameMatch(rayOrigin, rayDirection);
      return BoundingBox3DReadOnly.super.intersectionWithRay3D(rayOrigin, rayDirection, firstIntersectionToPack, secondIntersectionToPack);
   }

   default int intersectionWithRay3D(Point3DReadOnly rayOrigin, Vector3DReadOnly rayDirection, FixedFramePoint3DBasics firstIntersectionToPack,
                                     FixedFramePoint3DBasics secondIntersectionToPack)
   {
      if (firstIntersectionToPack != null)
         checkReferenceFrameMatch(firstIntersectionToPack);
      if (secondIntersectionToPack != null)
         checkReferenceFrameMatch(secondIntersectionToPack);
      return BoundingBox3DReadOnly.super.intersectionWithRay3D(rayOrigin, rayDirection, firstIntersectionToPack, secondIntersectionToPack);
   }

   default int intersectionWithRay3D(Point3DReadOnly rayOrigin, Vector3DReadOnly rayDirection, FramePoint3DBasics firstIntersectionToPack,
                                     FramePoint3DBasics secondIntersectionToPack)
   {
      if (firstIntersectionToPack != null)
         firstIntersectionToPack.setReferenceFrame(getReferenceFrame());
      if (secondIntersectionToPack != null)
         secondIntersectionToPack.setReferenceFrame(getReferenceFrame());
      return BoundingBox3DReadOnly.super.intersectionWithRay3D(rayOrigin, rayDirection, firstIntersectionToPack, secondIntersectionToPack);
   }

   default int intersectionWithRay3D(FramePoint3DReadOnly rayOrigin, FrameVector3DReadOnly rayDirection, FixedFramePoint3DBasics firstIntersectionToPack,
                                     FixedFramePoint3DBasics secondIntersectionToPack)
   {
      checkReferenceFrameMatch(rayOrigin, rayDirection);
      if (firstIntersectionToPack != null)
         checkReferenceFrameMatch(firstIntersectionToPack);
      if (secondIntersectionToPack != null)
         checkReferenceFrameMatch(secondIntersectionToPack);
      return BoundingBox3DReadOnly.super.intersectionWithRay3D(rayOrigin, rayDirection, firstIntersectionToPack, secondIntersectionToPack);
   }

   default int intersectionWithRay3D(FramePoint3DReadOnly rayOrigin, FrameVector3DReadOnly rayDirection, FramePoint3DBasics firstIntersectionToPack,
                                     FramePoint3DBasics secondIntersectionToPack)
   {
      checkReferenceFrameMatch(rayOrigin, rayDirection);
      if (firstIntersectionToPack != null)
         firstIntersectionToPack.setReferenceFrame(getReferenceFrame());
      if (secondIntersectionToPack != null)
         secondIntersectionToPack.setReferenceFrame(getReferenceFrame());
      return BoundingBox3DReadOnly.super.intersectionWithRay3D(rayOrigin, rayDirection, firstIntersectionToPack, secondIntersectionToPack);
   }

   default boolean equals(FrameBoundingBox3DReadOnly other)
   {
      if (other == this)
         return true;
      else if (other == null || getReferenceFrame() != other.getReferenceFrame())
         return false;
      else
         return getMinPoint().equals(other.getMinPoint()) && getMaxPoint().equals(other.getMaxPoint());
   }

   default boolean epsilonEquals(FrameBoundingBox3DReadOnly other, double epsilon)
   {
      if (getReferenceFrame() != other.getReferenceFrame())
         return false;
      else
         return BoundingBox3DReadOnly.super.epsilonEquals(other, epsilon);
   }

   default boolean geometricallyEquals(FrameBoundingBox3DReadOnly other, double epsilon)
   {
      checkReferenceFrameMatch(other);
      return BoundingBox3DReadOnly.super.geometricallyEquals(other, epsilon);
   }
}
