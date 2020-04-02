package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.geometry.interfaces.BoundingBox3DReadOnly;
import us.ihmc.euclid.geometry.interfaces.Line3DReadOnly;
import us.ihmc.euclid.geometry.interfaces.LineSegment3DReadOnly;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

/**
 * Read-only interface for a 3D axis-aligned bounding box expressed in a given reference frame.
 */
public interface FrameBoundingBox3DReadOnly extends BoundingBox3DReadOnly, ReferenceFrameHolder
{
   /** {@inheritDoc} */
   @Override
   FramePoint3DReadOnly getMinPoint();

   /** {@inheritDoc} */
   @Override
   FramePoint3DReadOnly getMaxPoint();

   /**
    * Calculates the coordinate of the center of this bounding box and stores it in the given
    * {@code centerToPack}.
    *
    * @param centerToPack point 3D in which the center of this bounding box is stored. Modified.
    * @throws RuntimeException                if this bounding box is improper according to
    *                                         {@link #checkBounds()}.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void getCenterPoint(FixedFramePoint3DBasics centerToPack)
   {
      checkReferenceFrameMatch(centerToPack);
      BoundingBox3DReadOnly.super.getCenterPoint(centerToPack);
   }

   /**
    * Calculates the coordinate of the center of this bounding box and stores it in the given
    * {@code centerToPack}.
    *
    * @param centerToPack point 3D in which the center of this bounding box is stored. Modified.
    * @throws RuntimeException if this bounding box is improper according to {@link #checkBounds()}.
    */
   default void getCenterPoint(FramePoint3DBasics centerToPack)
   {
      centerToPack.setReferenceFrame(getReferenceFrame());
      BoundingBox3DReadOnly.super.getCenterPoint(centerToPack);
   }

   /**
    * Performs a linear interpolation between this bounding box minimum and maximum coordinates using
    * independent parameters {@code xParameter}, {@code yParameter}, and {@code zParameter} for the
    * x-axis, y-axis, and z-axis respectively. The result is stored in {@code pointToPack}.
    * <p>
    * <ul>
    * <li>{@code (xParameter == 0)} results in: {@code (pointToPack.getX() == this.getMinX())}.
    * <li>{@code (xParameter == 1)} results in: {@code (pointToPack.getX() == this.getMaxX())}.
    * <li>{@code (yParameter == 0)} results in: {@code (pointToPack.getY() == this.getMinY())}.
    * <li>{@code (yParameter == 1)} results in: {@code (pointToPack.getY() == this.getMaxY())}.
    * <li>{@code (zParameter == 0)} results in: {@code (pointToPack.getZ() == this.getMinZ())}.
    * <li>{@code (zParameter == 1)} results in: {@code (pointToPack.getZ() == this.getMaxZ())}.
    * </ul>
    * </p>
    *
    * @param xParameter  the parameter to use for the interpolation along the x-axis.
    * @param yParameter  the parameter to use for the interpolation along the y-axis.
    * @param zParameter  the parameter to use for the interpolation along the z-axis.
    * @param pointToPack the point 3D in which the result is stored. Modified.
    * @throws RuntimeException                if this bounding box is improper according to
    *                                         {@link #checkBounds()}.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void getPointGivenParameters(double xParameter, double yParameter, double zParameter, FixedFramePoint3DBasics pointToPack)
   {
      checkReferenceFrameMatch(pointToPack);
      BoundingBox3DReadOnly.super.getPointGivenParameters(xParameter, yParameter, zParameter, pointToPack);
   }

   /**
    * Performs a linear interpolation between this bounding box minimum and maximum coordinates using
    * independent parameters {@code xParameter}, {@code yParameter}, and {@code zParameter} for the
    * x-axis, y-axis, and z-axis respectively. The result is stored in {@code pointToPack}.
    * <p>
    * <ul>
    * <li>{@code (xParameter == 0)} results in: {@code (pointToPack.getX() == this.getMinX())}.
    * <li>{@code (xParameter == 1)} results in: {@code (pointToPack.getX() == this.getMaxX())}.
    * <li>{@code (yParameter == 0)} results in: {@code (pointToPack.getY() == this.getMinY())}.
    * <li>{@code (yParameter == 1)} results in: {@code (pointToPack.getY() == this.getMaxY())}.
    * <li>{@code (zParameter == 0)} results in: {@code (pointToPack.getZ() == this.getMinZ())}.
    * <li>{@code (zParameter == 1)} results in: {@code (pointToPack.getZ() == this.getMaxZ())}.
    * </ul>
    * </p>
    *
    * @param xParameter  the parameter to use for the interpolation along the x-axis.
    * @param yParameter  the parameter to use for the interpolation along the y-axis.
    * @param zParameter  the parameter to use for the interpolation along the z-axis.
    * @param pointToPack the point 3D in which the result is stored. Modified.
    * @throws RuntimeException if this bounding box is improper according to {@link #checkBounds()}.
    */
   default void getPointGivenParameters(double xParameter, double yParameter, double zParameter, FramePoint3DBasics pointToPack)
   {
      pointToPack.setReferenceFrame(getReferenceFrame());
      BoundingBox3DReadOnly.super.getPointGivenParameters(xParameter, yParameter, zParameter, pointToPack);
   }

   /**
    * Tests if the {@code query} is located inside this bounding box.
    * <p>
    * The query is considered to be outside if located exactly on an edge of this bounding box.
    * </p>
    *
    * @param query the query to test if it is located inside this bounding box. Not modified.
    * @return {@code true} if the query is inside, {@code false} if outside or located on an edge of
    *         this bounding box.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default boolean isInsideExclusive(FramePoint3DReadOnly query)
   {
      checkReferenceFrameMatch(query);
      return BoundingBox3DReadOnly.super.isInsideExclusive(query);
   }

   /**
    * Tests if the {@code query} is located inside this bounding box.
    * <p>
    * The query is considered to be inside if located exactly on an edge of this bounding box.
    * </p>
    *
    * @param query the query to test if it is located inside this bounding box. Not modified.
    * @return {@code true} if the query is inside or located on an edge of this bounding box,
    *         {@code false} if outside.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default boolean isInsideInclusive(FramePoint3DReadOnly query)
   {
      checkReferenceFrameMatch(query);
      return BoundingBox3DReadOnly.super.isInsideInclusive(query);
   }

   /**
    * Tests if the {@code query} is located inside this bounding box given the tolerance
    * {@code epsilon}.
    * <p>
    * <ul>
    * <li>if {@code epsilon == 0}, this method is equivalent to
    * {@link #isInsideExclusive(FramePoint3DReadOnly)}.
    * <li>if {@code epsilon > 0}, the size of this bounding box is scaled up by shifting the edges of
    * {@code epsilon} toward the outside.
    * <li>if {@code epsilon > 0}, the size of this bounding box is scaled down by shifting the edges of
    * {@code epsilon} toward the inside.
    * </ul>
    * </p>
    *
    * @param query   the query to test if it is located inside this bounding box. Not modified.
    * @param epsilon the tolerance to use for this test.
    * @return {@code true} if the query is considered to be inside the bounding box, {@code false}
    *         otherwise.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default boolean isInsideEpsilon(FramePoint3DReadOnly query, double epsilon)
   {
      checkReferenceFrameMatch(query);
      return BoundingBox3DReadOnly.super.isInsideEpsilon(query, epsilon);
   }

   /**
    * Tests if the {@code query} is located inside the projection onto the XY-plane of this bounding
    * box.
    * <p>
    * The query is considered to be outside if located exactly on an edge of this bounding box.
    * </p>
    *
    * @param query the query to test if it is located inside this bounding box. Not modified.
    * @return {@code true} if the query is inside, {@code false} if outside or located on an edge of
    *         this bounding box.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default boolean isXYInsideExclusive(FramePoint2DReadOnly query)
   {
      checkReferenceFrameMatch(query);
      return BoundingBox3DReadOnly.super.isXYInsideExclusive(query);
   }

   /**
    * Tests if the {@code query} is located inside the projection onto the XY-plane of this bounding
    * box.
    * <p>
    * The query is considered to be inside if located exactly on an edge of this bounding box.
    * </p>
    *
    * @param query the query to test if it is located inside this bounding box. Not modified.
    * @return {@code true} if the query is inside or located on an edge of this bounding box,
    *         {@code false} if outside.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default boolean isXYInsideInclusive(FramePoint2DReadOnly query)
   {
      checkReferenceFrameMatch(query);
      return BoundingBox3DReadOnly.super.isXYInsideInclusive(query);
   }

   /**
    * * Tests if the {@code query} is located inside the projection onto the XY-plane of this bounding
    * box given the tolerance {@code epsilon}.
    * <p>
    * <ul>
    * <li>if {@code epsilon == 0}, this method is equivalent to
    * {@link #isXYInsideInclusive(FramePoint2DReadOnly)}.
    * <li>if {@code epsilon > 0}, the size of this bounding box is scaled up by shifting the edges of
    * {@code epsilon} toward the outside.
    * <li>if {@code epsilon > 0}, the size of this bounding box is scaled down by shifting the edges of
    * {@code epsilon} toward the inside.
    * </ul>
    * </p>
    *
    * @param query   the query to test if it is located inside this bounding box. Not modified.
    * @param epsilon the tolerance to use for this test.
    * @return {@code true} if the query is considered to be inside the bounding box, {@code false}
    *         otherwise.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default boolean isXYInsideEpsilon(FramePoint2DReadOnly query, double epsilon)
   {
      checkReferenceFrameMatch(query);
      return BoundingBox3DReadOnly.super.isXYInsideEpsilon(query, epsilon);
   }

   /**
    * Tests if this bounding box and {@code other} intersects.
    * <p>
    * The two bounding boxes are considered to not be intersecting if they share a corner, an edge, or
    * a face.
    * </p>
    *
    * @param other the other bounding box to test if it is intersecting with this bounding box. Not
    *              Modified.
    * @return {@code true} if the two bounding boxes intersect, {@code false} otherwise.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default boolean intersectsExclusive(FrameBoundingBox3DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      return BoundingBox3DReadOnly.super.intersectsExclusive(other);
   }

   /**
    * Tests if this bounding box and {@code other} intersects.
    * <p>
    * The two bounding boxes are considered to be intersecting if they share a corner, an edge, or a
    * face.
    * </p>
    *
    * @param other the other bounding box to test if it is intersecting with this bounding box. Not
    *              Modified.
    * @return {@code true} if the two bounding boxes intersect, {@code false} otherwise.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default boolean intersectsInclusive(FrameBoundingBox3DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      return BoundingBox3DReadOnly.super.intersectsInclusive(other);
   }

   /**
    * Tests if this bounding box and {@code other} intersects.
    * <p>
    * <ul>
    * <li>if {@code epsilon == 0}, this method is equivalent to
    * {@link #intersectsExclusive(FrameBoundingBox3DReadOnly)}.
    * <li>if {@code epsilon > 0}, the size of this bounding box is scaled up by shifting the faces of
    * {@code epsilon} toward the outside.
    * <li>if {@code epsilon < 0}, the size of this bounding box is scaled down by shifting the faces of
    * {@code epsilon} toward the inside.
    * </ul>
    * </p>
    *
    * @param other   the other bounding box to test if it is intersecting with this bounding box. Not
    *                Modified.
    * @param epsilon the tolerance to use in this test.
    * @return {@code true} if the two bounding boxes intersect, {@code false} otherwise.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default boolean intersectsEpsilon(FrameBoundingBox3DReadOnly other, double epsilon)
   {
      checkReferenceFrameMatch(other);
      return BoundingBox3DReadOnly.super.intersectsEpsilon(other, epsilon);
   }

   /**
    * Tests if this bounding box and {@code other} intersects.
    * <p>
    * The two bounding boxes are considered to not be intersecting if they share a corner or an edge.
    * </p>
    * <p>
    * This method is equivalent to projecting this bounding box onto the XY-plane before performing the
    * test.
    * </p>
    *
    * @param other the other bounding box to test if it is intersecting with this bounding box. Not
    *              Modified.
    * @return {@code true} if the two bounding boxes intersect, {@code false} otherwise.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default boolean intersectsExclusiveInXYPlane(FrameBoundingBox2DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      return BoundingBox3DReadOnly.super.intersectsExclusiveInXYPlane(other);
   }

   /**
    * Tests if this bounding box and {@code other} intersects.
    * <p>
    * The two bounding boxes are considered to be intersecting if they share a corner or an edge.
    * </p>
    * <p>
    * This method is equivalent to projecting this bounding box onto the XY-plane before performing the
    * test.
    * </p>
    *
    * @param other the other bounding box to test if it is intersecting with this bounding box. Not
    *              Modified.
    * @return {@code true} if the two bounding boxes intersect, {@code false} otherwise.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default boolean intersectsInclusiveInXYPlane(FrameBoundingBox2DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      return BoundingBox3DReadOnly.super.intersectsInclusiveInXYPlane(other);
   }

   /**
    * Tests if this bounding box and {@code other} intersects.
    * <p>
    * <ul>
    * <li>if {@code epsilon == 0}, this method is equivalent to
    * {@link #intersectsExclusiveInXYPlane(FrameBoundingBox2DReadOnly)}.
    * <li>if {@code epsilon > 0}, the size of this bounding box is scaled up by shifting the edges of
    * {@code epsilon} toward the outside.
    * <li>if {@code epsilon > 0}, the size of this bounding box is scaled down by shifting the edges of
    * {@code epsilon} toward the inside.
    * </ul>
    * </p>
    * <p>
    * This method is equivalent to projecting this bounding box onto the XY-plane before performing the
    * test.
    * </p>
    *
    * @param other   the other bounding box to test if it is intersecting with this bounding box. Not
    *                Modified.
    * @param epsilon the tolerance to use in this test.
    * @return {@code true} if the two bounding boxes intersect, {@code false} otherwise.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default boolean intersectsEpsilonInXYPlane(FrameBoundingBox2DReadOnly other, double epsilon)
   {
      checkReferenceFrameMatch(other);
      return BoundingBox3DReadOnly.super.intersectsEpsilonInXYPlane(other, epsilon);
   }

   /**
    * Tests if this the given line 3D intersects this bounding box.
    *
    * @param line3D the query. Not modified.
    * @return {@code true} if the line and this bounding box intersect, {@code false} otherwise.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default boolean doesIntersectWithLine3D(FrameLine3DReadOnly line3D)
   {
      checkReferenceFrameMatch(line3D);
      return BoundingBox3DReadOnly.super.doesIntersectWithLine3D(line3D);
   }

   /**
    * Tests if this the given line 3D intersects this bounding box.
    *
    * @param pointOnLine   a point located on the infinitely long line. Not modified.
    * @param lineDirection the line direction. Not modified.
    * @return {@code true} if the line and this bounding box intersect, {@code false} otherwise.
    * @throws ReferenceFrameMismatchException if any of the arguments is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default boolean doesIntersectWithLine3D(FramePoint3DReadOnly pointOnLine, FrameVector3DReadOnly lineDirection)
   {
      checkReferenceFrameMatch(pointOnLine, lineDirection);
      return BoundingBox3DReadOnly.super.doesIntersectWithLine3D(pointOnLine, lineDirection);
   }

   /**
    * Tests if this the given line segment 3D intersects this bounding box.
    *
    * @param lineSegment3D the query. Not modified.
    * @return {@code true} if the line segment and this bounding box intersect, {@code false}
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}. otherwise.
    */
   default boolean doesIntersectWithLineSegment3D(FrameLineSegment3DReadOnly lineSegment3D)
   {
      checkReferenceFrameMatch(lineSegment3D);
      return BoundingBox3DReadOnly.super.doesIntersectWithLineSegment3D(lineSegment3D);
   }

   /**
    * Tests if this the given line segment 3D intersects this bounding box.
    *
    * @param lineSegmentStart first endpoint of the line segment. Not modified.
    * @param lineSegmentEnd   second endpoint of the line segment. Not modified.
    * @return {@code true} if the line segment and this bounding box intersect, {@code false}
    *         otherwise.
    * @throws ReferenceFrameMismatchException if any of the arguments is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default boolean doesIntersectWithLineSegment3D(FramePoint3DReadOnly lineSegmentStart, FramePoint3DReadOnly lineSegmentEnd)
   {
      checkReferenceFrameMatch(lineSegmentStart, lineSegmentEnd);
      return BoundingBox3DReadOnly.super.doesIntersectWithLineSegment3D(lineSegmentStart, lineSegmentEnd);
   }

   /**
    * Tests if this the given ray 3D intersects this bounding box.
    *
    * @param rayOrigin    the origin of the ray. Not modified.
    * @param rayDirection the ray direction. Not modified.
    * @return {@code true} if the ray and this bounding box intersect, {@code false} otherwise.
    * @throws ReferenceFrameMismatchException if any of the arguments is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default boolean doesIntersectWithRay3D(FramePoint3DReadOnly rayOrigin, FrameVector3DReadOnly rayDirection)
   {
      checkReferenceFrameMatch(rayOrigin, rayDirection);
      return BoundingBox3DReadOnly.super.doesIntersectWithRay3D(rayOrigin, rayDirection);
   }

   /**
    * Computes the coordinates of the two intersections between a line and this bounding box.
    * <p>
    * In the case the line and the bounding box do not intersect, this method returns {@code 0} and
    * {@code firstIntersectionToPack} and {@code secondIntersectionToPack} remains unmodified.
    * </p>
    *
    * @param line3D                   the query. Not modified.
    * @param firstIntersectionToPack  the coordinate of the first intersection. Can be {@code null}.
    *                                 Modified.
    * @param secondIntersectionToPack the coordinate of the second intersection. Can be {@code null}.
    *                                 Modified.
    * @return the number of intersections between the line and this bounding box. It is either equal to
    *         0 or 2.
    * @throws ReferenceFrameMismatchException if the frame argument is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default int intersectionWithLine3D(FrameLine3DReadOnly line3D, Point3DBasics firstIntersectionToPack, Point3DBasics secondIntersectionToPack)
   {
      checkReferenceFrameMatch(line3D);
      return BoundingBox3DReadOnly.super.intersectionWithLine3D(line3D, firstIntersectionToPack, secondIntersectionToPack);
   }

   /**
    * Computes the coordinates of the two intersections between a line and this bounding box.
    * <p>
    * In the case the line and the bounding box do not intersect, this method returns {@code 0} and
    * {@code firstIntersectionToPack} and {@code secondIntersectionToPack} remains unmodified.
    * </p>
    *
    * @param line3D                   the query. Not modified.
    * @param firstIntersectionToPack  the coordinate of the first intersection. Can be {@code null}.
    *                                 Modified.
    * @param secondIntersectionToPack the coordinate of the second intersection. Can be {@code null}.
    *                                 Modified.
    * @return the number of intersections between the line and this bounding box. It is either equal to
    *         0 or 2.
    * @throws ReferenceFrameMismatchException if any of the frame arguments is not expressed in the
    *                                         same reference frame as {@code this}.
    */
   default int intersectionWithLine3D(Line3DReadOnly line3D, FixedFramePoint3DBasics firstIntersectionToPack, FixedFramePoint3DBasics secondIntersectionToPack)
   {
      if (firstIntersectionToPack != null)
         checkReferenceFrameMatch(firstIntersectionToPack);
      if (secondIntersectionToPack != null)
         checkReferenceFrameMatch(secondIntersectionToPack);
      return BoundingBox3DReadOnly.super.intersectionWithLine3D(line3D, firstIntersectionToPack, secondIntersectionToPack);
   }

   /**
    * Computes the coordinates of the two intersections between a line and this bounding box.
    * <p>
    * In the case the line and the bounding box do not intersect, this method returns {@code 0} and
    * {@code firstIntersectionToPack} and {@code secondIntersectionToPack} remains unmodified.
    * </p>
    *
    * @param line3D                   the query. Not modified.
    * @param firstIntersectionToPack  the coordinate of the first intersection. Can be {@code null}.
    *                                 Modified.
    * @param secondIntersectionToPack the coordinate of the second intersection. Can be {@code null}.
    *                                 Modified.
    * @return the number of intersections between the line and this bounding box. It is either equal to
    *         0 or 2.
    */
   default int intersectionWithLine3D(Line3DReadOnly line3D, FramePoint3DBasics firstIntersectionToPack, FramePoint3DBasics secondIntersectionToPack)
   {
      if (firstIntersectionToPack != null)
         firstIntersectionToPack.setReferenceFrame(getReferenceFrame());
      if (secondIntersectionToPack != null)
         secondIntersectionToPack.setReferenceFrame(getReferenceFrame());
      return BoundingBox3DReadOnly.super.intersectionWithLine3D(line3D, firstIntersectionToPack, secondIntersectionToPack);
   }

   /**
    * Computes the coordinates of the two intersections between a line and this bounding box.
    * <p>
    * In the case the line and the bounding box do not intersect, this method returns {@code 0} and
    * {@code firstIntersectionToPack} and {@code secondIntersectionToPack} remains unmodified.
    * </p>
    *
    * @param line3D                   the query. Not modified.
    * @param firstIntersectionToPack  the coordinate of the first intersection. Can be {@code null}.
    *                                 Modified.
    * @param secondIntersectionToPack the coordinate of the second intersection. Can be {@code null}.
    *                                 Modified.
    * @return the number of intersections between the line and this bounding box. It is either equal to
    *         0 or 2.
    * @throws ReferenceFrameMismatchException if any of the arguments is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default int intersectionWithLine3D(FrameLine3DReadOnly line3D, FixedFramePoint3DBasics firstIntersectionToPack,
                                      FixedFramePoint3DBasics secondIntersectionToPack)
   {
      checkReferenceFrameMatch(line3D);
      if (firstIntersectionToPack != null)
         checkReferenceFrameMatch(firstIntersectionToPack);
      if (secondIntersectionToPack != null)
         checkReferenceFrameMatch(secondIntersectionToPack);
      return BoundingBox3DReadOnly.super.intersectionWithLine3D(line3D, firstIntersectionToPack, secondIntersectionToPack);
   }

   /**
    * Computes the coordinates of the two intersections between a line and this bounding box.
    * <p>
    * In the case the line and the bounding box do not intersect, this method returns {@code 0} and
    * {@code firstIntersectionToPack} and {@code secondIntersectionToPack} remains unmodified.
    * </p>
    *
    * @param line3D                   the query. Not modified.
    * @param firstIntersectionToPack  the coordinate of the first intersection. Can be {@code null}.
    *                                 Modified.
    * @param secondIntersectionToPack the coordinate of the second intersection. Can be {@code null}.
    *                                 Modified.
    * @return the number of intersections between the line and this bounding box. It is either equal to
    *         0 or 2.
    * @throws ReferenceFrameMismatchException if {@code line3D} is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default int intersectionWithLine3D(FrameLine3DReadOnly line3D, FramePoint3DBasics firstIntersectionToPack, FramePoint3DBasics secondIntersectionToPack)
   {
      checkReferenceFrameMatch(line3D);
      if (firstIntersectionToPack != null)
         firstIntersectionToPack.setReferenceFrame(getReferenceFrame());
      if (secondIntersectionToPack != null)
         secondIntersectionToPack.setReferenceFrame(getReferenceFrame());
      return BoundingBox3DReadOnly.super.intersectionWithLine3D(line3D, firstIntersectionToPack, secondIntersectionToPack);
   }

   /**
    * Computes the coordinates of the two intersections between a line and this bounding box.
    * <p>
    * In the case the line and the bounding box do not intersect, this method returns {@code 0} and
    * {@code firstIntersectionToPack} and {@code secondIntersectionToPack} remains unmodified.
    * </p>
    *
    * @param pointOnLine              a point located on the infinitely long line. Not modified.
    * @param lineDirection            the line direction. Not modified.
    * @param firstIntersectionToPack  the coordinate of the first intersection. Can be {@code null}.
    *                                 Modified.
    * @param secondIntersectionToPack the coordinate of the second intersection. Can be {@code null}.
    *                                 Modified.
    * @return the number of intersections between the line and this bounding box. It is either equal to
    *         0 or 2.
    * @throws ReferenceFrameMismatchException if any of the frame arguments is not expressed in the
    *                                         same reference frame as {@code this}.
    */
   default int intersectionWithLine3D(FramePoint3DReadOnly pointOnLine, FrameVector3DReadOnly lineDirection, Point3DBasics firstIntersectionToPack,
                                      Point3DBasics secondIntersectionToPack)
   {
      checkReferenceFrameMatch(pointOnLine, lineDirection);
      return BoundingBox3DReadOnly.super.intersectionWithLine3D(pointOnLine, lineDirection, firstIntersectionToPack, secondIntersectionToPack);
   }

   /**
    * Computes the coordinates of the two intersections between a line and this bounding box.
    * <p>
    * In the case the line and the bounding box do not intersect, this method returns {@code 0} and
    * {@code firstIntersectionToPack} and {@code secondIntersectionToPack} remains unmodified.
    * </p>
    *
    * @param pointOnLine              a point located on the infinitely long line. Not modified.
    * @param lineDirection            the line direction. Not modified.
    * @param firstIntersectionToPack  the coordinate of the first intersection. Can be {@code null}.
    *                                 Modified.
    * @param secondIntersectionToPack the coordinate of the second intersection. Can be {@code null}.
    *                                 Modified.
    * @return the number of intersections between the line and this bounding box. It is either equal to
    *         0 or 2.
    * @throws ReferenceFrameMismatchException if any of the frame arguments is not expressed in the
    *                                         same reference frame as {@code this}.
    */
   default int intersectionWithLine3D(Point3DReadOnly pointOnLine, Vector3DReadOnly lineDirection, FixedFramePoint3DBasics firstIntersectionToPack,
                                      FixedFramePoint3DBasics secondIntersectionToPack)
   {
      if (firstIntersectionToPack != null)
         checkReferenceFrameMatch(firstIntersectionToPack);
      if (secondIntersectionToPack != null)
         checkReferenceFrameMatch(secondIntersectionToPack);
      return BoundingBox3DReadOnly.super.intersectionWithLine3D(pointOnLine, lineDirection, firstIntersectionToPack, secondIntersectionToPack);
   }

   /**
    * Computes the coordinates of the two intersections between a line and this bounding box.
    * <p>
    * In the case the line and the bounding box do not intersect, this method returns {@code 0} and
    * {@code firstIntersectionToPack} and {@code secondIntersectionToPack} remains unmodified.
    * </p>
    *
    * @param pointOnLine              a point located on the infinitely long line. Not modified.
    * @param lineDirection            the line direction. Not modified.
    * @param firstIntersectionToPack  the coordinate of the first intersection. Can be {@code null}.
    *                                 Modified.
    * @param secondIntersectionToPack the coordinate of the second intersection. Can be {@code null}.
    *                                 Modified.
    * @return the number of intersections between the line and this bounding box. It is either equal to
    *         0 or 2.
    */
   default int intersectionWithLine3D(Point3DReadOnly pointOnLine, Vector3DReadOnly lineDirection, FramePoint3DBasics firstIntersectionToPack,
                                      FramePoint3DBasics secondIntersectionToPack)
   {
      if (firstIntersectionToPack != null)
         firstIntersectionToPack.setReferenceFrame(getReferenceFrame());
      if (secondIntersectionToPack != null)
         secondIntersectionToPack.setReferenceFrame(getReferenceFrame());
      return BoundingBox3DReadOnly.super.intersectionWithLine3D(pointOnLine, lineDirection, firstIntersectionToPack, secondIntersectionToPack);
   }

   /**
    * Computes the coordinates of the two intersections between a line and this bounding box.
    * <p>
    * In the case the line and the bounding box do not intersect, this method returns {@code 0} and
    * {@code firstIntersectionToPack} and {@code secondIntersectionToPack} remains unmodified.
    * </p>
    *
    * @param pointOnLine              a point located on the infinitely long line. Not modified.
    * @param lineDirection            the line direction. Not modified.
    * @param firstIntersectionToPack  the coordinate of the first intersection. Can be {@code null}.
    *                                 Modified.
    * @param secondIntersectionToPack the coordinate of the second intersection. Can be {@code null}.
    *                                 Modified.
    * @return the number of intersections between the line and this bounding box. It is either equal to
    *         0 or 2.
    * @throws ReferenceFrameMismatchException if any of the arguments is not expressed in the same
    *                                         reference frame as {@code this}.
    */
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

   /**
    * Computes the coordinates of the two intersections between a line and this bounding box.
    * <p>
    * In the case the line and the bounding box do not intersect, this method returns {@code 0} and
    * {@code firstIntersectionToPack} and {@code secondIntersectionToPack} remains unmodified.
    * </p>
    *
    * @param pointOnLine              a point located on the infinitely long line. Not modified.
    * @param lineDirection            the line direction. Not modified.
    * @param firstIntersectionToPack  the coordinate of the first intersection. Can be {@code null}.
    *                                 Modified.
    * @param secondIntersectionToPack the coordinate of the second intersection. Can be {@code null}.
    *                                 Modified.
    * @return the number of intersections between the line and this bounding box. It is either equal to
    *         0 or 2.
    * @throws ReferenceFrameMismatchException if any of the frame arguments is not expressed in the
    *                                         same reference frame as {@code this}.
    * @throws ReferenceFrameMismatchException if either {@code pointOnLine} or {@code lineDirection} is
    *                                         not expressed in the same reference frame as
    *                                         {@code this}.
    */
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

   /**
    * Computes the coordinates of the two intersections between a line segment and this bounding box.
    * <p>
    * Intersection(s) between the line segment and this bounding box can only exist between the
    * endpoints of the line segment.
    * </p>
    * <p>
    * In the case the line segment and this bounding box do not intersect, this method returns
    * {@code 0} and {@code firstIntersectionToPack} and {@code secondIntersectionToPack} remains
    * unmodified.
    * </p>
    * <p>
    * In the case only one intersection exists between the line segment and the bounding box,
    * {@code firstIntersectionToPack} will contain the coordinate of the intersection and
    * {@code secondIntersectionToPack} will be set to contain only {@link Double#NaN}.
    * </p>
    *
    * @param lineSegment3D            the query. Not modified.
    * @param firstIntersectionToPack  the coordinate of the first intersection. Can be {@code null}.
    *                                 Modified.
    * @param secondIntersectionToPack the coordinate of the second intersection. Can be {@code null}.
    *                                 Modified.
    * @return the number of intersections between the line segment and this bounding box. It is either
    *         equal to 0, 1, or 2.
    * @throws ReferenceFrameMismatchException if the frame argument is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default int intersectionWithLineSegment3D(FrameLineSegment3DReadOnly lineSegment3D, Point3DBasics firstIntersectionToPack,
                                             Point3DBasics secondIntersectionToPack)
   {
      checkReferenceFrameMatch(lineSegment3D);
      return BoundingBox3DReadOnly.super.intersectionWithLineSegment3D(lineSegment3D, firstIntersectionToPack, secondIntersectionToPack);
   }

   /**
    * Computes the coordinates of the two intersections between a line segment and this bounding box.
    * <p>
    * Intersection(s) between the line segment and this bounding box can only exist between the
    * endpoints of the line segment.
    * </p>
    * <p>
    * In the case the line segment and this bounding box do not intersect, this method returns
    * {@code 0} and {@code firstIntersectionToPack} and {@code secondIntersectionToPack} remains
    * unmodified.
    * </p>
    * <p>
    * In the case only one intersection exists between the line segment and the bounding box,
    * {@code firstIntersectionToPack} will contain the coordinate of the intersection and
    * {@code secondIntersectionToPack} will be set to contain only {@link Double#NaN}.
    * </p>
    *
    * @param lineSegment3D            the query. Not modified.
    * @param firstIntersectionToPack  the coordinate of the first intersection. Can be {@code null}.
    *                                 Modified.
    * @param secondIntersectionToPack the coordinate of the second intersection. Can be {@code null}.
    *                                 Modified.
    * @return the number of intersections between the line segment and this bounding box. It is either
    *         equal to 0, 1, or 2.
    * @throws ReferenceFrameMismatchException if any of the frame arguments is not expressed in the
    *                                         same reference frame as {@code this}.
    */
   default int intersectionWithLineSegment3D(LineSegment3DReadOnly lineSegment3D, FixedFramePoint3DBasics firstIntersectionToPack,
                                             FixedFramePoint3DBasics secondIntersectionToPack)
   {
      if (firstIntersectionToPack != null)
         checkReferenceFrameMatch(firstIntersectionToPack);
      if (secondIntersectionToPack != null)
         checkReferenceFrameMatch(secondIntersectionToPack);
      return BoundingBox3DReadOnly.super.intersectionWithLineSegment3D(lineSegment3D, firstIntersectionToPack, secondIntersectionToPack);
   }

   /**
    * Computes the coordinates of the two intersections between a line segment and this bounding box.
    * <p>
    * Intersection(s) between the line segment and this bounding box can only exist between the
    * endpoints of the line segment.
    * </p>
    * <p>
    * In the case the line segment and this bounding box do not intersect, this method returns
    * {@code 0} and {@code firstIntersectionToPack} and {@code secondIntersectionToPack} remains
    * unmodified.
    * </p>
    * <p>
    * In the case only one intersection exists between the line segment and the bounding box,
    * {@code firstIntersectionToPack} will contain the coordinate of the intersection and
    * {@code secondIntersectionToPack} will be set to contain only {@link Double#NaN}.
    * </p>
    *
    * @param lineSegment3D            the query. Not modified.
    * @param firstIntersectionToPack  the coordinate of the first intersection. Can be {@code null}.
    *                                 Modified.
    * @param secondIntersectionToPack the coordinate of the second intersection. Can be {@code null}.
    *                                 Modified.
    * @return the number of intersections between the line segment and this bounding box. It is either
    *         equal to 0, 1, or 2.
    */
   default int intersectionWithLineSegment3D(LineSegment3DReadOnly lineSegment3D, FramePoint3DBasics firstIntersectionToPack,
                                             FramePoint3DBasics secondIntersectionToPack)
   {
      if (firstIntersectionToPack != null)
         firstIntersectionToPack.setReferenceFrame(getReferenceFrame());
      if (secondIntersectionToPack != null)
         secondIntersectionToPack.setReferenceFrame(getReferenceFrame());
      return BoundingBox3DReadOnly.super.intersectionWithLineSegment3D(lineSegment3D, firstIntersectionToPack, secondIntersectionToPack);
   }

   /**
    * Computes the coordinates of the two intersections between a line segment and this bounding box.
    * <p>
    * Intersection(s) between the line segment and this bounding box can only exist between the
    * endpoints of the line segment.
    * </p>
    * <p>
    * In the case the line segment and this bounding box do not intersect, this method returns
    * {@code 0} and {@code firstIntersectionToPack} and {@code secondIntersectionToPack} remains
    * unmodified.
    * </p>
    * <p>
    * In the case only one intersection exists between the line segment and the bounding box,
    * {@code firstIntersectionToPack} will contain the coordinate of the intersection and
    * {@code secondIntersectionToPack} will be set to contain only {@link Double#NaN}.
    * </p>
    *
    * @param lineSegment3D            the query. Not modified.
    * @param firstIntersectionToPack  the coordinate of the first intersection. Can be {@code null}.
    *                                 Modified.
    * @param secondIntersectionToPack the coordinate of the second intersection. Can be {@code null}.
    *                                 Modified.
    * @return the number of intersections between the line segment and this bounding box. It is either
    *         equal to 0, 1, or 2.
    * @throws ReferenceFrameMismatchException if any of the arguments is not expressed in the same
    *                                         reference frame as {@code this}.
    */
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

   /**
    * Computes the coordinates of the two intersections between a line segment and this bounding box.
    * <p>
    * Intersection(s) between the line segment and this bounding box can only exist between the
    * endpoints of the line segment.
    * </p>
    * <p>
    * In the case the line segment and this bounding box do not intersect, this method returns
    * {@code 0} and {@code firstIntersectionToPack} and {@code secondIntersectionToPack} remains
    * unmodified.
    * </p>
    * <p>
    * In the case only one intersection exists between the line segment and the bounding box,
    * {@code firstIntersectionToPack} will contain the coordinate of the intersection and
    * {@code secondIntersectionToPack} will be set to contain only {@link Double#NaN}.
    * </p>
    *
    * @param lineSegment3D            the query. Not modified.
    * @param firstIntersectionToPack  the coordinate of the first intersection. Can be {@code null}.
    *                                 Modified.
    * @param secondIntersectionToPack the coordinate of the second intersection. Can be {@code null}.
    *                                 Modified.
    * @return the number of intersections between the line segment and this bounding box. It is either
    *         equal to 0, 1, or 2.
    * @throws ReferenceFrameMismatchException if {@code lineSegment3D} is not expressed in the same
    *                                         reference frame as {@code this}.
    */
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

   /**
    * Computes the coordinates of the two intersections between a line segment and this bounding box.
    * <p>
    * Intersection(s) between the line segment and this bounding box can only exist between the
    * endpoints of the line segment.
    * </p>
    * <p>
    * In the case the line segment and this bounding box do not intersect, this method returns
    * {@code 0} and {@code firstIntersectionToPack} and {@code secondIntersectionToPack} are set to
    * {@link Double#NaN}.
    * </p>
    * <p>
    * In the case only one intersection exists between the line segment and the bounding box,
    * {@code firstIntersectionToPack} will contain the coordinate of the intersection and
    * {@code secondIntersectionToPack} will be set to contain only {@link Double#NaN}.
    * </p>
    *
    * @param lineSegmentStart         the first endpoint of the line segment. Not modified.
    * @param lineSegmentEnd           the second endpoint of the line segment. Not modified.
    * @param firstIntersectionToPack  the coordinate of the first intersection. Can be {@code null}.
    *                                 Modified.
    * @param secondIntersectionToPack the coordinate of the second intersection. Can be {@code null}.
    *                                 Modified.
    * @return the number of intersections between the line segment and this bounding box. It is either
    *         equal to 0, 1, or 2.
    * @throws ReferenceFrameMismatchException if any of the frame arguments is not expressed in the
    *                                         same reference frame as {@code this}.
    */
   default int intersectionWithLineSegment3D(FramePoint3DReadOnly lineSegmentStart, FramePoint3DReadOnly lineSegmentEnd, Point3DBasics firstIntersectionToPack,
                                             Point3DBasics secondIntersectionToPack)
   {
      checkReferenceFrameMatch(lineSegmentStart, lineSegmentEnd);
      return BoundingBox3DReadOnly.super.intersectionWithLineSegment3D(lineSegmentStart, lineSegmentEnd, firstIntersectionToPack, secondIntersectionToPack);
   }

   /**
    * Computes the coordinates of the two intersections between a line segment and this bounding box.
    * <p>
    * Intersection(s) between the line segment and this bounding box can only exist between the
    * endpoints of the line segment.
    * </p>
    * <p>
    * In the case the line segment and this bounding box do not intersect, this method returns
    * {@code 0} and {@code firstIntersectionToPack} and {@code secondIntersectionToPack} are set to
    * {@link Double#NaN}.
    * </p>
    * <p>
    * In the case only one intersection exists between the line segment and the bounding box,
    * {@code firstIntersectionToPack} will contain the coordinate of the intersection and
    * {@code secondIntersectionToPack} will be set to contain only {@link Double#NaN}.
    * </p>
    *
    * @param lineSegmentStart         the first endpoint of the line segment. Not modified.
    * @param lineSegmentEnd           the second endpoint of the line segment. Not modified.
    * @param firstIntersectionToPack  the coordinate of the first intersection. Can be {@code null}.
    *                                 Modified.
    * @param secondIntersectionToPack the coordinate of the second intersection. Can be {@code null}.
    *                                 Modified.
    * @return the number of intersections between the line segment and this bounding box. It is either
    *         equal to 0, 1, or 2.
    * @throws ReferenceFrameMismatchException if any of the frame arguments is not expressed in the
    *                                         same reference frame as {@code this}.
    */
   default int intersectionWithLineSegment3D(Point3DReadOnly lineSegmentStart, Point3DReadOnly lineSegmentEnd, FixedFramePoint3DBasics firstIntersectionToPack,
                                             FixedFramePoint3DBasics secondIntersectionToPack)
   {
      if (firstIntersectionToPack != null)
         checkReferenceFrameMatch(firstIntersectionToPack);
      if (secondIntersectionToPack != null)
         checkReferenceFrameMatch(secondIntersectionToPack);
      return BoundingBox3DReadOnly.super.intersectionWithLineSegment3D(lineSegmentStart, lineSegmentEnd, firstIntersectionToPack, secondIntersectionToPack);
   }

   /**
    * Computes the coordinates of the two intersections between a line segment and this bounding box.
    * <p>
    * Intersection(s) between the line segment and this bounding box can only exist between the
    * endpoints of the line segment.
    * </p>
    * <p>
    * In the case the line segment and this bounding box do not intersect, this method returns
    * {@code 0} and {@code firstIntersectionToPack} and {@code secondIntersectionToPack} are set to
    * {@link Double#NaN}.
    * </p>
    * <p>
    * In the case only one intersection exists between the line segment and the bounding box,
    * {@code firstIntersectionToPack} will contain the coordinate of the intersection and
    * {@code secondIntersectionToPack} will be set to contain only {@link Double#NaN}.
    * </p>
    *
    * @param lineSegmentStart         the first endpoint of the line segment. Not modified.
    * @param lineSegmentEnd           the second endpoint of the line segment. Not modified.
    * @param firstIntersectionToPack  the coordinate of the first intersection. Can be {@code null}.
    *                                 Modified.
    * @param secondIntersectionToPack the coordinate of the second intersection. Can be {@code null}.
    *                                 Modified.
    * @return the number of intersections between the line segment and this bounding box. It is either
    *         equal to 0, 1, or 2.
    */
   default int intersectionWithLineSegment3D(Point3DReadOnly lineSegmentStart, Point3DReadOnly lineSegmentEnd, FramePoint3DBasics firstIntersectionToPack,
                                             FramePoint3DBasics secondIntersectionToPack)
   {
      if (firstIntersectionToPack != null)
         firstIntersectionToPack.setReferenceFrame(getReferenceFrame());
      if (secondIntersectionToPack != null)
         secondIntersectionToPack.setReferenceFrame(getReferenceFrame());
      return BoundingBox3DReadOnly.super.intersectionWithLineSegment3D(lineSegmentStart, lineSegmentEnd, firstIntersectionToPack, secondIntersectionToPack);
   }

   /**
    * Computes the coordinates of the two intersections between a line segment and this bounding box.
    * <p>
    * Intersection(s) between the line segment and this bounding box can only exist between the
    * endpoints of the line segment.
    * </p>
    * <p>
    * In the case the line segment and this bounding box do not intersect, this method returns
    * {@code 0} and {@code firstIntersectionToPack} and {@code secondIntersectionToPack} are set to
    * {@link Double#NaN}.
    * </p>
    * <p>
    * In the case only one intersection exists between the line segment and the bounding box,
    * {@code firstIntersectionToPack} will contain the coordinate of the intersection and
    * {@code secondIntersectionToPack} will be set to contain only {@link Double#NaN}.
    * </p>
    *
    * @param lineSegmentStart         the first endpoint of the line segment. Not modified.
    * @param lineSegmentEnd           the second endpoint of the line segment. Not modified.
    * @param firstIntersectionToPack  the coordinate of the first intersection. Can be {@code null}.
    *                                 Modified.
    * @param secondIntersectionToPack the coordinate of the second intersection. Can be {@code null}.
    *                                 Modified.
    * @return the number of intersections between the line segment and this bounding box. It is either
    *         equal to 0, 1, or 2.
    * @throws ReferenceFrameMismatchException if any of the arguments is not expressed in the same
    *                                         reference frame as {@code this}.
    */
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

   /**
    * Computes the coordinates of the two intersections between a line segment and this bounding box.
    * <p>
    * Intersection(s) between the line segment and this bounding box can only exist between the
    * endpoints of the line segment.
    * </p>
    * <p>
    * In the case the line segment and this bounding box do not intersect, this method returns
    * {@code 0} and {@code firstIntersectionToPack} and {@code secondIntersectionToPack} are set to
    * {@link Double#NaN}.
    * </p>
    * <p>
    * In the case only one intersection exists between the line segment and the bounding box,
    * {@code firstIntersectionToPack} will contain the coordinate of the intersection and
    * {@code secondIntersectionToPack} will be set to contain only {@link Double#NaN}.
    * </p>
    *
    * @param lineSegmentStart         the first endpoint of the line segment. Not modified.
    * @param lineSegmentEnd           the second endpoint of the line segment. Not modified.
    * @param firstIntersectionToPack  the coordinate of the first intersection. Can be {@code null}.
    *                                 Modified.
    * @param secondIntersectionToPack the coordinate of the second intersection. Can be {@code null}.
    *                                 Modified.
    * @return the number of intersections between the line segment and this bounding box. It is either
    *         equal to 0, 1, or 2.
    * @throws ReferenceFrameMismatchException if either {@code lineSegmentStart} or
    *                                         {@code lineSegmentEnd} is not expressed in the same
    *                                         reference frame as {@code this}.
    */
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

   /**
    * Computes the coordinates of the two intersections between a ray and this bounding box.
    * <p>
    * Intersection(s) between the ray and the bounding box cannot exist before the origin of the ray.
    * </p>
    * </p>
    * In the case the ray and this bounding box do not intersect, this method returns {@code 0} and
    * {@code firstIntersectionToPack} and {@code secondIntersectionToPack} are set to
    * {@link Double#NaN}.
    * </p>
    * <p>
    * In the case only one intersection exists between the ray and this bounding box,
    * {@code firstIntersectionToPack} will contain the coordinate of the intersection and
    * {@code secondIntersectionToPack} will be set to contain only {@link Double#NaN}.
    * </p>
    *
    * @param rayOrigin                the coordinate of the ray origin. Not modified.
    * @param rayDirection             the direction of the ray. Not modified.
    * @param firstIntersectionToPack  the coordinate of the first intersection. Can be {@code null}.
    *                                 Modified.
    * @param secondIntersectionToPack the coordinate of the second intersection. Can be {@code null}.
    *                                 Modified.
    * @return the number of intersections between the ray and this bounding box. It is either equal to
    *         0, 1, or 2.
    * @throws ReferenceFrameMismatchException if any of the frame arguments is not expressed in the
    *                                         same reference frame as {@code this}.
    */
   default int intersectionWithRay3D(FramePoint3DReadOnly rayOrigin, FrameVector3DReadOnly rayDirection, Point3DBasics firstIntersectionToPack,
                                     Point3DBasics secondIntersectionToPack)
   {
      checkReferenceFrameMatch(rayOrigin, rayDirection);
      return BoundingBox3DReadOnly.super.intersectionWithRay3D(rayOrigin, rayDirection, firstIntersectionToPack, secondIntersectionToPack);
   }

   /**
    * Computes the coordinates of the two intersections between a ray and this bounding box.
    * <p>
    * Intersection(s) between the ray and the bounding box cannot exist before the origin of the ray.
    * </p>
    * </p>
    * In the case the ray and this bounding box do not intersect, this method returns {@code 0} and
    * {@code firstIntersectionToPack} and {@code secondIntersectionToPack} are set to
    * {@link Double#NaN}.
    * </p>
    * <p>
    * In the case only one intersection exists between the ray and this bounding box,
    * {@code firstIntersectionToPack} will contain the coordinate of the intersection and
    * {@code secondIntersectionToPack} will be set to contain only {@link Double#NaN}.
    * </p>
    *
    * @param rayOrigin                the coordinate of the ray origin. Not modified.
    * @param rayDirection             the direction of the ray. Not modified.
    * @param firstIntersectionToPack  the coordinate of the first intersection. Can be {@code null}.
    *                                 Modified.
    * @param secondIntersectionToPack the coordinate of the second intersection. Can be {@code null}.
    *                                 Modified.
    * @return the number of intersections between the ray and this bounding box. It is either equal to
    *         0, 1, or 2.
    * @throws ReferenceFrameMismatchException if any of the frame arguments is not expressed in the
    *                                         same reference frame as {@code this}.
    */
   default int intersectionWithRay3D(Point3DReadOnly rayOrigin, Vector3DReadOnly rayDirection, FixedFramePoint3DBasics firstIntersectionToPack,
                                     FixedFramePoint3DBasics secondIntersectionToPack)
   {
      if (firstIntersectionToPack != null)
         checkReferenceFrameMatch(firstIntersectionToPack);
      if (secondIntersectionToPack != null)
         checkReferenceFrameMatch(secondIntersectionToPack);
      return BoundingBox3DReadOnly.super.intersectionWithRay3D(rayOrigin, rayDirection, firstIntersectionToPack, secondIntersectionToPack);
   }

   /**
    * Computes the coordinates of the two intersections between a ray and this bounding box.
    * <p>
    * Intersection(s) between the ray and the bounding box cannot exist before the origin of the ray.
    * </p>
    * </p>
    * In the case the ray and this bounding box do not intersect, this method returns {@code 0} and
    * {@code firstIntersectionToPack} and {@code secondIntersectionToPack} are set to
    * {@link Double#NaN}.
    * </p>
    * <p>
    * In the case only one intersection exists between the ray and this bounding box,
    * {@code firstIntersectionToPack} will contain the coordinate of the intersection and
    * {@code secondIntersectionToPack} will be set to contain only {@link Double#NaN}.
    * </p>
    *
    * @param rayOrigin                the coordinate of the ray origin. Not modified.
    * @param rayDirection             the direction of the ray. Not modified.
    * @param firstIntersectionToPack  the coordinate of the first intersection. Can be {@code null}.
    *                                 Modified.
    * @param secondIntersectionToPack the coordinate of the second intersection. Can be {@code null}.
    *                                 Modified.
    * @return the number of intersections between the ray and this bounding box. It is either equal to
    *         0, 1, or 2.
    */
   default int intersectionWithRay3D(Point3DReadOnly rayOrigin, Vector3DReadOnly rayDirection, FramePoint3DBasics firstIntersectionToPack,
                                     FramePoint3DBasics secondIntersectionToPack)
   {
      if (firstIntersectionToPack != null)
         firstIntersectionToPack.setReferenceFrame(getReferenceFrame());
      if (secondIntersectionToPack != null)
         secondIntersectionToPack.setReferenceFrame(getReferenceFrame());
      return BoundingBox3DReadOnly.super.intersectionWithRay3D(rayOrigin, rayDirection, firstIntersectionToPack, secondIntersectionToPack);
   }

   /**
    * Computes the coordinates of the two intersections between a ray and this bounding box.
    * <p>
    * Intersection(s) between the ray and the bounding box cannot exist before the origin of the ray.
    * </p>
    * </p>
    * In the case the ray and this bounding box do not intersect, this method returns {@code 0} and
    * {@code firstIntersectionToPack} and {@code secondIntersectionToPack} are set to
    * {@link Double#NaN}.
    * </p>
    * <p>
    * In the case only one intersection exists between the ray and this bounding box,
    * {@code firstIntersectionToPack} will contain the coordinate of the intersection and
    * {@code secondIntersectionToPack} will be set to contain only {@link Double#NaN}.
    * </p>
    *
    * @param rayOrigin                the coordinate of the ray origin. Not modified.
    * @param rayDirection             the direction of the ray. Not modified.
    * @param firstIntersectionToPack  the coordinate of the first intersection. Can be {@code null}.
    *                                 Modified.
    * @param secondIntersectionToPack the coordinate of the second intersection. Can be {@code null}.
    *                                 Modified.
    * @return the number of intersections between the ray and this bounding box. It is either equal to
    *         0, 1, or 2.
    * @throws ReferenceFrameMismatchException if any of the arguments is not expressed in the same
    *                                         reference frame as {@code this}.
    */
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

   /**
    * Computes the coordinates of the two intersections between a ray and this bounding box.
    * <p>
    * Intersection(s) between the ray and the bounding box cannot exist before the origin of the ray.
    * </p>
    * </p>
    * In the case the ray and this bounding box do not intersect, this method returns {@code 0} and
    * {@code firstIntersectionToPack} and {@code secondIntersectionToPack} are set to
    * {@link Double#NaN}.
    * </p>
    * <p>
    * In the case only one intersection exists between the ray and this bounding box,
    * {@code firstIntersectionToPack} will contain the coordinate of the intersection and
    * {@code secondIntersectionToPack} will be set to contain only {@link Double#NaN}.
    * </p>
    *
    * @param rayOrigin                the coordinate of the ray origin. Not modified.
    * @param rayDirection             the direction of the ray. Not modified.
    * @param firstIntersectionToPack  the coordinate of the first intersection. Can be {@code null}.
    *                                 Modified.
    * @param secondIntersectionToPack the coordinate of the second intersection. Can be {@code null}.
    *                                 Modified.
    * @return the number of intersections between the ray and this bounding box. It is either equal to
    *         0, 1, or 2.
    * @throws ReferenceFrameMismatchException if either {@code rayOrigin} or {@code rayDirection} is
    *                                         not expressed in the same reference frame as
    *                                         {@code this}.
    */
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

   /**
    * Tests on a per-component basis on the minimum and maximum coordinates if this bounding box is
    * equal to {@code other} with the tolerance {@code epsilon}.
    * <p>
    * If the two bounding boxes have different frames, this method returns {@code false}.
    * </p>
    *
    * @param other   the query. Not modified.
    * @param epsilon the tolerance to use.
    * @return {@code true} if the two bounding boxes are equal and are expressed in the same reference
    *         frame, {@code false} otherwise.
    */
   default boolean epsilonEquals(FrameBoundingBox3DReadOnly other, double epsilon)
   {
      if (getReferenceFrame() != other.getReferenceFrame())
         return false;
      else
         return BoundingBox3DReadOnly.super.epsilonEquals(other, epsilon);
   }

   /**
    * Compares {@code this} to {@code other} to determine if the two bounding boxes are geometrically
    * similar, i.e. the distance between their min and max points is less than or equal to
    * {@code epsilon}.
    *
    * @param other   the bounding box to compare to. Not modified.
    * @param epsilon the tolerance of the comparison.
    * @return {@code true} if the two bounding boxes represent the same geometry, {@code false}
    *         otherwise.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code other} are not expressed in
    *                                         the same reference frame.
    */
   default boolean geometricallyEquals(FrameBoundingBox3DReadOnly other, double epsilon)
   {
      checkReferenceFrameMatch(other);
      return BoundingBox3DReadOnly.super.geometricallyEquals(other, epsilon);
   }

   /**
    * Tests on a per component basis, if this bounding box 3D is exactly equal to {@code other}.
    * <p>
    * If the two bounding boxes have different frames, this method returns {@code false}.
    * </p>
    *
    * @param other the other bounding box 3D to compare against this. Not modified.
    * @return {@code true} if the two bounding boxes are exactly equal component-wise and are expressed
    *         in the same reference frame, {@code false} otherwise.
    */
   default boolean equals(FrameBoundingBox3DReadOnly other)
   {
      if (other == this)
         return true;
      else if (other == null || getReferenceFrame() != other.getReferenceFrame())
         return false;
      else
         return getMinPoint().equals(other.getMinPoint()) && getMaxPoint().equals(other.getMaxPoint());
   }
}
