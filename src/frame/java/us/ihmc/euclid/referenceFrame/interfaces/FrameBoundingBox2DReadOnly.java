package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.geometry.interfaces.BoundingBox2DReadOnly;
import us.ihmc.euclid.geometry.interfaces.Line2DReadOnly;
import us.ihmc.euclid.geometry.interfaces.LineSegment2DReadOnly;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;

/**
 * Read-only interface for a 2D axis-aligned bounding box expressed in a given reference frame.
 */
public interface FrameBoundingBox2DReadOnly extends BoundingBox2DReadOnly, ReferenceFrameHolder
{
   /** {@inheritDoc} */
   @Override
   FramePoint2DReadOnly getMinPoint();

   /** {@inheritDoc} */
   @Override
   FramePoint2DReadOnly getMaxPoint();

   /**
    * Calculates the coordinate of the center of this bounding box and stores it in the given
    * {@code centerToPack}.
    *
    * @param centerToPack point 2D in which the center of this bounding box is stored. Modified.
    * @throws RuntimeException                if this bounding box is improper according to
    *                                         {@link #checkBounds()}.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void getCenterPoint(FixedFramePoint2DBasics centerToPack)
   {
      checkReferenceFrameMatch(centerToPack);
      BoundingBox2DReadOnly.super.getCenterPoint(centerToPack);
   }

   /**
    * Calculates the coordinate of the center of this bounding box and stores it in the given
    * {@code centerToPack}.
    *
    * @param centerToPack point 2D in which the center of this bounding box is stored. Modified.
    * @throws RuntimeException if this bounding box is improper according to {@link #checkBounds()}.
    */
   default void getCenterPoint(FramePoint2DBasics centerToPack)
   {
      centerToPack.setReferenceFrame(getReferenceFrame());
      BoundingBox2DReadOnly.super.getCenterPoint(centerToPack);
   }

   /**
    * Performs a linear interpolation between this bounding box minimum and maximum coordinates using
    * independent parameters {@code xParameter} and {@code yParameter} for the x-axis and y-axis
    * respectively. The result is stored in {@code pointToPack}.
    * <p>
    * <ul>
    * <li>{@code (xParameter == 0)} results in: {@code (pointToPack.getX() == this.getMinX())}.
    * <li>{@code (xParameter == 1)} results in: {@code (pointToPack.getX() == this.getMaxX())}.
    * <li>{@code (yParameter == 0)} results in: {@code (pointToPack.getY() == this.getMinY())}.
    * <li>{@code (yParameter == 1)} results in: {@code (pointToPack.getY() == this.getMaxY())}.
    * </ul>
    * </p>
    *
    * @param xParameter  the parameter to use for the interpolation along the x-axis.
    * @param yParameter  the parameter to use for the interpolation along the y-axis.
    * @param pointToPack the point 2D in which the result is stored. Modified.
    * @throws RuntimeException                if this bounding box is improper according to
    *                                         {@link #checkBounds()}.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void getPointGivenParameters(double xParameter, double yParameter, FixedFramePoint2DBasics pointToPack)
   {
      checkReferenceFrameMatch(pointToPack);
      BoundingBox2DReadOnly.super.getPointGivenParameters(xParameter, yParameter, pointToPack);
   }

   /**
    * Performs a linear interpolation between this bounding box minimum and maximum coordinates using
    * independent parameters {@code xParameter} and {@code yParameter} for the x-axis and y-axis
    * respectively. The result is stored in {@code pointToPack}.
    * <p>
    * <ul>
    * <li>{@code (xParameter == 0)} results in: {@code (pointToPack.getX() == this.getMinX())}.
    * <li>{@code (xParameter == 1)} results in: {@code (pointToPack.getX() == this.getMaxX())}.
    * <li>{@code (yParameter == 0)} results in: {@code (pointToPack.getY() == this.getMinY())}.
    * <li>{@code (yParameter == 1)} results in: {@code (pointToPack.getY() == this.getMaxY())}.
    * </ul>
    * </p>
    *
    * @param xParameter  the parameter to use for the interpolation along the x-axis.
    * @param yParameter  the parameter to use for the interpolation along the y-axis.
    * @param pointToPack the point 2D in which the result is stored. Modified.
    * @throws RuntimeException if this bounding box is improper according to {@link #checkBounds()}.
    */
   default void getPointGivenParameters(double xParameter, double yParameter, FramePoint2DBasics pointToPack)
   {
      pointToPack.setReferenceFrame(getReferenceFrame());
      BoundingBox2DReadOnly.super.getPointGivenParameters(xParameter, yParameter, pointToPack);
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
    * @throws RuntimeException                if this bounding box is improper according to
    *                                         {@link #checkBounds()}.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default boolean isInsideExclusive(FramePoint2DReadOnly query)
   {
      checkReferenceFrameMatch(query);
      return BoundingBox2DReadOnly.super.isInsideExclusive(query);
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
    * @throws RuntimeException                if this bounding box is improper according to
    *                                         {@link #checkBounds()}.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default boolean isInsideInclusive(FramePoint2DReadOnly query)
   {
      checkReferenceFrameMatch(query);
      return BoundingBox2DReadOnly.super.isInsideInclusive(query);
   }

   /**
    * Tests if the {@code query} is located inside this bounding box given the tolerance
    * {@code epsilon}.
    * <p>
    * <ul>
    * <li>if {@code epsilon == 0}, this method is equivalent to
    * {@link #isInsideExclusive(FramePoint2DReadOnly)}.
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
    * @throws RuntimeException                if this bounding box is improper according to
    *                                         {@link #checkBounds()}.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default boolean isInsideEpsilon(FramePoint2DReadOnly query, double epsilon)
   {
      checkReferenceFrameMatch(query);
      return BoundingBox2DReadOnly.super.isInsideEpsilon(query, epsilon);
   }

   /**
    * Tests if this bounding box and {@code other} intersects.
    * <p>
    * The two bounding boxes are considered to not be intersecting if they share a corner or an edge.
    * </p>
    *
    * @param other the other bounding box to test if it is intersecting with this bounding box. Not
    *              Modified.
    * @return {@code true} if the two bounding boxes intersect, {@code false} otherwise.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default boolean intersectsExclusive(FrameBoundingBox2DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      return BoundingBox2DReadOnly.super.intersectsExclusive(other);
   }

   /**
    * Tests if this bounding box and {@code other} intersects.
    * <p>
    * The two bounding boxes are considered to be intersecting if they share a corner or an edge.
    * </p>
    *
    * @param other the other bounding box to test if it is intersecting with this bounding box. Not
    *              Modified.
    * @return {@code true} if the two bounding boxes intersect, {@code false} otherwise.
    * @throws RuntimeException                if this bounding box is improper according to
    *                                         {@link #checkBounds()}.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default boolean intersectsInclusive(FrameBoundingBox2DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      return BoundingBox2DReadOnly.super.intersectsInclusive(other);
   }

   /**
    * Tests if this bounding box and {@code other} intersects.
    * <p>
    * <ul>
    * <li>if {@code epsilon == 0}, this method is equivalent to
    * {@link #intersectsExclusive(FrameBoundingBox2DReadOnly)}.
    * <li>if {@code epsilon > 0}, the size of this bounding box is scaled up by shifting the edges of
    * {@code epsilon} toward the outside.
    * <li>if {@code epsilon < 0}, the size of this bounding box is scaled down by shifting the edges of
    * {@code epsilon} toward the inside.
    * </ul>
    * </p>
    *
    * @param other   the other bounding box to test if it is intersecting with this bounding box. Not
    *                Modified.
    * @param epsilon the tolerance to use in this test.
    * @return {@code true} if the two bounding boxes intersect, {@code false} otherwise.
    * @throws RuntimeException                if this bounding box is improper according to
    *                                         {@link #checkBounds()}.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default boolean intersectsEpsilon(FrameBoundingBox2DReadOnly other, double epsilon)
   {
      checkReferenceFrameMatch(other);
      return BoundingBox2DReadOnly.super.intersectsEpsilon(other, epsilon);
   }

   /**
    * Tests if this the given line 2D intersects this bounding box.
    *
    * @param line2D the query. Not modified.
    * @return {@code true} if the line and this bounding box intersect, {@code false} otherwise.
    * @throws RuntimeException                if this bounding box is improper according to
    *                                         {@link #checkBounds()}.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default boolean doesIntersectWithLine2D(FrameLine2DReadOnly line2D)
   {
      checkReferenceFrameMatch(line2D);
      return BoundingBox2DReadOnly.super.doesIntersectWithLine2D(line2D);
   }

   /**
    * Tests if this the given line 2D intersects this bounding box.
    *
    * @param pointOnLine   a point located on the infinitely long line. Not modified.
    * @param lineDirection the line direction. Not modified.
    * @return {@code true} if the line and this bounding box intersect, {@code false} otherwise.
    * @throws RuntimeException                if this bounding box is improper according to
    *                                         {@link #checkBounds()}.
    * @throws ReferenceFrameMismatchException if any of the arguments is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default boolean doesIntersectWithLine2D(FramePoint2DReadOnly pointOnLine, FrameVector2DReadOnly lineDirection)
   {
      checkReferenceFrameMatch(pointOnLine, lineDirection);
      return BoundingBox2DReadOnly.super.doesIntersectWithLine2D(pointOnLine, lineDirection);
   }

   /**
    * Tests if this the given line segment 2D intersects this bounding box.
    *
    * @param lineSegment2D the query. Not modified.
    * @return {@code true} if the line segment and this bounding box intersect, {@code false}
    *         otherwise.
    * @throws RuntimeException                if this bounding box is improper according to
    *                                         {@link #checkBounds()}.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default boolean doesIntersectWithLineSegment2D(FrameLineSegment2DReadOnly lineSegment2D)
   {
      checkReferenceFrameMatch(lineSegment2D);
      return BoundingBox2DReadOnly.super.doesIntersectWithLineSegment2D(lineSegment2D);
   }

   /**
    * Tests if this the given line segment 2D intersects this bounding box.
    *
    * @param lineSegmentStart first endpoint of the line segment. Not modified.
    * @param lineSegmentEnd   second endpoint of the line segment. Not modified.
    * @return {@code true} if the line segment and this bounding box intersect, {@code false}
    *         otherwise.
    * @throws RuntimeException                if this bounding box is improper according to
    *                                         {@link #checkBounds()}.
    * @throws ReferenceFrameMismatchException if any of the arguments is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default boolean doesIntersectWithLineSegment2D(FramePoint2DReadOnly lineSegmentStart, FramePoint2DReadOnly lineSegmentEnd)
   {
      checkReferenceFrameMatch(lineSegmentStart, lineSegmentEnd);
      return BoundingBox2DReadOnly.super.doesIntersectWithLineSegment2D(lineSegmentStart, lineSegmentEnd);
   }

   /**
    * Tests if this the given ray 2D intersects this bounding box.
    *
    * @param rayOrigin    the origin of the ray. Not modified.
    * @param rayDirection the ray direction. Not modified.
    * @return {@code true} if the ray and this bounding box intersect, {@code false} otherwise.
    * @throws RuntimeException                if this bounding box is improper according to
    *                                         {@link #checkBounds()}.
    * @throws ReferenceFrameMismatchException if any of the arguments is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default boolean doesIntersectWithRay2D(FramePoint2DReadOnly rayOrigin, FrameVector2DReadOnly rayDirection)
   {
      checkReferenceFrameMatch(rayOrigin, rayDirection);
      return BoundingBox2DReadOnly.super.doesIntersectWithRay2D(rayOrigin, rayDirection);
   }

   /**
    * Computes the coordinates of the two intersections between a line and this bounding box.
    * <p>
    * In the case the line and the bounding box do not intersect, this method returns {@code 0} and
    * {@code firstIntersectionToPack} and {@code secondIntersectionToPack} are set to
    * {@link Double#NaN}.
    * </p>
    *
    * @param line2D                   the query. Not modified.
    * @param firstIntersectionToPack  the coordinate of the first intersection. Can be {@code null}.
    *                                 Modified.
    * @param secondIntersectionToPack the coordinate of the second intersection. Can be {@code null}.
    *                                 Modified.
    * @return the number of intersections between the line and this bounding box. It is either equal to
    *         0 or 2.
    * @throws RuntimeException                if this bounding box is improper according to
    *                                         {@link #checkBounds()}.
    * @throws ReferenceFrameMismatchException if the frame argument is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default int intersectionWithLine2D(FrameLine2DReadOnly line2D, Point2DBasics firstIntersectionToPack, Point2DBasics secondIntersectionToPack)
   {
      return intersectionWithLine2D(line2D.getPoint(), line2D.getDirection(), firstIntersectionToPack, secondIntersectionToPack);
   }

   /**
    * Computes the coordinates of the two intersections between a line and this bounding box.
    * <p>
    * In the case the line and the bounding box do not intersect, this method returns {@code 0} and
    * {@code firstIntersectionToPack} and {@code secondIntersectionToPack} are set to
    * {@link Double#NaN}.
    * </p>
    *
    * @param line2D                   the query. Not modified.
    * @param firstIntersectionToPack  the coordinate of the first intersection. Can be {@code null}.
    *                                 Modified.
    * @param secondIntersectionToPack the coordinate of the second intersection. Can be {@code null}.
    *                                 Modified.
    * @return the number of intersections between the line and this bounding box. It is either equal to
    *         0 or 2.
    * @throws RuntimeException if this bounding box is improper according to {@link #checkBounds()}.
    */
   default int intersectionWithLine2D(Line2DReadOnly line2D, FramePoint2DBasics firstIntersectionToPack, FramePoint2DBasics secondIntersectionToPack)
   {
      return intersectionWithLine2D(line2D.getPoint(), line2D.getDirection(), firstIntersectionToPack, secondIntersectionToPack);
   }

   /**
    * Computes the coordinates of the two intersections between a line and this bounding box.
    * <p>
    * In the case the line and the bounding box do not intersect, this method returns {@code 0} and
    * {@code firstIntersectionToPack} and {@code secondIntersectionToPack} are set to
    * {@link Double#NaN}.
    * </p>
    *
    * @param line2D                   the query. Not modified.
    * @param firstIntersectionToPack  the coordinate of the first intersection. Can be {@code null}.
    *                                 Modified.
    * @param secondIntersectionToPack the coordinate of the second intersection. Can be {@code null}.
    *                                 Modified.
    * @return the number of intersections between the line and this bounding box. It is either equal to
    *         0 or 2.
    * @throws RuntimeException                if this bounding box is improper according to
    *                                         {@link #checkBounds()}.
    * @throws ReferenceFrameMismatchException if any of the frame arguments is not expressed in the
    *                                         same reference frame as {@code this}.
    */
   default int intersectionWithLine2D(Line2DReadOnly line2D, FixedFramePoint2DBasics firstIntersectionToPack, FixedFramePoint2DBasics secondIntersectionToPack)
   {
      return intersectionWithLine2D(line2D.getPoint(), line2D.getDirection(), firstIntersectionToPack, secondIntersectionToPack);
   }

   /**
    * Computes the coordinates of the two intersections between a line and this bounding box.
    * <p>
    * In the case the line and the bounding box do not intersect, this method returns {@code 0} and
    * {@code firstIntersectionToPack} and {@code secondIntersectionToPack} are set to
    * {@link Double#NaN}.
    * </p>
    *
    * @param line2D                   the query. Not modified.
    * @param firstIntersectionToPack  the coordinate of the first intersection. Can be {@code null}.
    *                                 Modified.
    * @param secondIntersectionToPack the coordinate of the second intersection. Can be {@code null}.
    *                                 Modified.
    * @return the number of intersections between the line and this bounding box. It is either equal to
    *         0 or 2.
    * @throws RuntimeException                if this bounding box is improper according to
    *                                         {@link #checkBounds()}.
    * @throws ReferenceFrameMismatchException if {@code line2D} is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default int intersectionWithLine2D(FrameLine2DReadOnly line2D, FramePoint2DBasics firstIntersectionToPack, FramePoint2DBasics secondIntersectionToPack)
   {
      return intersectionWithLine2D(line2D.getPoint(), line2D.getDirection(), firstIntersectionToPack, secondIntersectionToPack);
   }

   /**
    * Computes the coordinates of the two intersections between a line and this bounding box.
    * <p>
    * In the case the line and the bounding box do not intersect, this method returns {@code 0} and
    * {@code firstIntersectionToPack} and {@code secondIntersectionToPack} are set to
    * {@link Double#NaN}.
    * </p>
    *
    * @param line2D                   the query. Not modified.
    * @param firstIntersectionToPack  the coordinate of the first intersection. Can be {@code null}.
    *                                 Modified.
    * @param secondIntersectionToPack the coordinate of the second intersection. Can be {@code null}.
    *                                 Modified.
    * @return the number of intersections between the line and this bounding box. It is either equal to
    *         0 or 2.
    * @throws RuntimeException                if this bounding box is improper according to
    *                                         {@link #checkBounds()}.
    * @throws ReferenceFrameMismatchException if any of the arguments is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default int intersectionWithLine2D(FrameLine2DReadOnly line2D,
                                      FixedFramePoint2DBasics firstIntersectionToPack,
                                      FixedFramePoint2DBasics secondIntersectionToPack)
   {
      return intersectionWithLine2D(line2D.getPoint(), line2D.getDirection(), firstIntersectionToPack, secondIntersectionToPack);
   }

   /**
    * Computes the coordinates of the two intersections between a line and this bounding box.
    * <p>
    * In the case the line and the bounding box do not intersect, this method returns {@code 0} and
    * {@code firstIntersectionToPack} and {@code secondIntersectionToPack} are set to
    * {@link Double#NaN}.
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
    * @throws RuntimeException                if this bounding box is improper according to
    *                                         {@link #checkBounds()}.
    * @throws ReferenceFrameMismatchException if any of the frame arguments is not expressed in the
    *                                         same reference frame as {@code this}.
    */
   default int intersectionWithLine2D(FramePoint2DReadOnly pointOnLine,
                                      FrameVector2DReadOnly lineDirection,
                                      Point2DBasics firstIntersectionToPack,
                                      Point2DBasics secondIntersectionToPack)
   {
      checkReferenceFrameMatch(pointOnLine, lineDirection);
      return BoundingBox2DReadOnly.super.intersectionWithLine2D(pointOnLine, lineDirection, firstIntersectionToPack, secondIntersectionToPack);
   }

   /**
    * Computes the coordinates of the two intersections between a line and this bounding box.
    * <p>
    * In the case the line and the bounding box do not intersect, this method returns {@code 0} and
    * {@code firstIntersectionToPack} and {@code secondIntersectionToPack} are set to
    * {@link Double#NaN}.
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
    * @throws RuntimeException                if this bounding box is improper according to
    *                                         {@link #checkBounds()}.
    * @throws ReferenceFrameMismatchException if any of the frame arguments is not expressed in the
    *                                         same reference frame as {@code this}.
    */
   default int intersectionWithLine2D(Point2DReadOnly pointOnLine,
                                      Vector2DReadOnly lineDirection,
                                      FixedFramePoint2DBasics firstIntersectionToPack,
                                      FixedFramePoint2DBasics secondIntersectionToPack)
   {
      if (firstIntersectionToPack != null)
         checkReferenceFrameMatch(firstIntersectionToPack);
      if (secondIntersectionToPack != null)
         checkReferenceFrameMatch(secondIntersectionToPack);
      return BoundingBox2DReadOnly.super.intersectionWithLine2D(pointOnLine, lineDirection, firstIntersectionToPack, secondIntersectionToPack);
   }

   /**
    * Computes the coordinates of the two intersections between a line and this bounding box.
    * <p>
    * In the case the line and the bounding box do not intersect, this method returns {@code 0} and
    * {@code firstIntersectionToPack} and {@code secondIntersectionToPack} are set to
    * {@link Double#NaN}.
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
    * @throws RuntimeException if this bounding box is improper according to {@link #checkBounds()}.
    */
   default int intersectionWithLine2D(Point2DReadOnly pointOnLine,
                                      Vector2DReadOnly lineDirection,
                                      FramePoint2DBasics firstIntersectionToPack,
                                      FramePoint2DBasics secondIntersectionToPack)
   {
      if (firstIntersectionToPack != null)
         firstIntersectionToPack.setReferenceFrame(getReferenceFrame());
      if (secondIntersectionToPack != null)
         secondIntersectionToPack.setReferenceFrame(getReferenceFrame());
      return BoundingBox2DReadOnly.super.intersectionWithLine2D(pointOnLine, lineDirection, firstIntersectionToPack, secondIntersectionToPack);
   }

   /**
    * Computes the coordinates of the two intersections between a line and this bounding box.
    * <p>
    * In the case the line and the bounding box do not intersect, this method returns {@code 0} and
    * {@code firstIntersectionToPack} and {@code secondIntersectionToPack} are set to
    * {@link Double#NaN}.
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
    * @throws RuntimeException                if this bounding box is improper according to
    *                                         {@link #checkBounds()}.
    * @throws ReferenceFrameMismatchException if any of the arguments is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default int intersectionWithLine2D(FramePoint2DReadOnly pointOnLine,
                                      FrameVector2DReadOnly lineDirection,
                                      FixedFramePoint2DBasics firstIntersectionToPack,
                                      FixedFramePoint2DBasics secondIntersectionToPack)
   {
      checkReferenceFrameMatch(pointOnLine, lineDirection);
      if (firstIntersectionToPack != null)
         checkReferenceFrameMatch(firstIntersectionToPack);
      if (secondIntersectionToPack != null)
         checkReferenceFrameMatch(secondIntersectionToPack);
      return BoundingBox2DReadOnly.super.intersectionWithLine2D(pointOnLine, lineDirection, firstIntersectionToPack, secondIntersectionToPack);
   }

   /**
    * Computes the coordinates of the two intersections between a line and this bounding box.
    * <p>
    * In the case the line and the bounding box do not intersect, this method returns {@code 0} and
    * {@code firstIntersectionToPack} and {@code secondIntersectionToPack} are set to
    * {@link Double#NaN}.
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
    * @throws RuntimeException                if this bounding box is improper according to
    *                                         {@link #checkBounds()}.
    * @throws ReferenceFrameMismatchException if either {@code pointOnLine} or {@code lineDirection} is
    *                                         not expressed in the same reference frame as
    *                                         {@code this}.
    */
   default int intersectionWithLine2D(FramePoint2DReadOnly pointOnLine,
                                      FrameVector2DReadOnly lineDirection,
                                      FramePoint2DBasics firstIntersectionToPack,
                                      FramePoint2DBasics secondIntersectionToPack)
   {
      checkReferenceFrameMatch(pointOnLine, lineDirection);
      if (firstIntersectionToPack != null)
         firstIntersectionToPack.setReferenceFrame(getReferenceFrame());
      if (secondIntersectionToPack != null)
         secondIntersectionToPack.setReferenceFrame(getReferenceFrame());
      return BoundingBox2DReadOnly.super.intersectionWithLine2D(pointOnLine, lineDirection, firstIntersectionToPack, secondIntersectionToPack);
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
    * @param lineSegment2D            the query. Not modified.
    * @param firstIntersectionToPack  the coordinate of the first intersection. Can be {@code null}.
    *                                 Modified.
    * @param secondIntersectionToPack the coordinate of the second intersection. Can be {@code null}.
    *                                 Modified.
    * @return the number of intersections between the line segment and this bounding box. It is either
    *         equal to 0, 1, or 2.
    * @throws RuntimeException                if this bounding box is improper according to
    *                                         {@link #checkBounds()}.
    * @throws ReferenceFrameMismatchException if any of the frame arguments is not expressed in the
    *                                         same reference frame as {@code this}.
    */
   default int intersectionWithLineSegment2D(FrameLineSegment2DReadOnly lineSegment2D,
                                             Point2DBasics firstIntersectionToPack,
                                             Point2DBasics secondIntersectionToPack)
   {
      return intersectionWithLineSegment2D(lineSegment2D.getFirstEndpoint(),
                                           lineSegment2D.getSecondEndpoint(),
                                           firstIntersectionToPack,
                                           secondIntersectionToPack);
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
    * @param lineSegment2D            the query. Not modified.
    * @param firstIntersectionToPack  the coordinate of the first intersection. Can be {@code null}.
    *                                 Modified.
    * @param secondIntersectionToPack the coordinate of the second intersection. Can be {@code null}.
    *                                 Modified.
    * @return the number of intersections between the line segment and this bounding box. It is either
    *         equal to 0, 1, or 2.
    * @throws RuntimeException if this bounding box is improper according to {@link #checkBounds()}.
    */
   default int intersectionWithLineSegment2D(LineSegment2DReadOnly lineSegment2D,
                                             FramePoint2DBasics firstIntersectionToPack,
                                             FramePoint2DBasics secondIntersectionToPack)
   {
      return intersectionWithLineSegment2D(lineSegment2D.getFirstEndpoint(),
                                           lineSegment2D.getSecondEndpoint(),
                                           firstIntersectionToPack,
                                           secondIntersectionToPack);
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
    * @param lineSegment2D            the query. Not modified.
    * @param firstIntersectionToPack  the coordinate of the first intersection. Can be {@code null}.
    *                                 Modified.
    * @param secondIntersectionToPack the coordinate of the second intersection. Can be {@code null}.
    *                                 Modified.
    * @return the number of intersections between the line segment and this bounding box. It is either
    *         equal to 0, 1, or 2.
    * @throws RuntimeException                if this bounding box is improper according to
    *                                         {@link #checkBounds()}.
    * @throws ReferenceFrameMismatchException if any of the frame arguments is not expressed in the
    *                                         same reference frame as {@code this}.
    */
   default int intersectionWithLineSegment2D(LineSegment2DReadOnly lineSegment2D,
                                             FixedFramePoint2DBasics firstIntersectionToPack,
                                             FixedFramePoint2DBasics secondIntersectionToPack)
   {
      return intersectionWithLineSegment2D(lineSegment2D.getFirstEndpoint(),
                                           lineSegment2D.getSecondEndpoint(),
                                           firstIntersectionToPack,
                                           secondIntersectionToPack);
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
    * @param lineSegment2D            the query. Not modified.
    * @param firstIntersectionToPack  the coordinate of the first intersection. Can be {@code null}.
    *                                 Modified.
    * @param secondIntersectionToPack the coordinate of the second intersection. Can be {@code null}.
    *                                 Modified.
    * @return the number of intersections between the line segment and this bounding box. It is either
    *         equal to 0, 1, or 2.
    * @throws RuntimeException                if this bounding box is improper according to
    *                                         {@link #checkBounds()}.
    * @throws ReferenceFrameMismatchException if {@code lineSegment2D} is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default int intersectionWithLineSegment2D(FrameLineSegment2DReadOnly lineSegment2D,
                                             FramePoint2DBasics firstIntersectionToPack,
                                             FramePoint2DBasics secondIntersectionToPack)
   {
      return intersectionWithLineSegment2D(lineSegment2D.getFirstEndpoint(),
                                           lineSegment2D.getSecondEndpoint(),
                                           firstIntersectionToPack,
                                           secondIntersectionToPack);
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
    * @param lineSegment2D            the query. Not modified.
    * @param firstIntersectionToPack  the coordinate of the first intersection. Can be {@code null}.
    *                                 Modified.
    * @param secondIntersectionToPack the coordinate of the second intersection. Can be {@code null}.
    *                                 Modified.
    * @return the number of intersections between the line segment and this bounding box. It is either
    *         equal to 0, 1, or 2.
    * @throws RuntimeException                if this bounding box is improper according to
    *                                         {@link #checkBounds()}.
    * @throws ReferenceFrameMismatchException if any of the arguments is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default int intersectionWithLineSegment2D(FrameLineSegment2DReadOnly lineSegment2D,
                                             FixedFramePoint2DBasics firstIntersectionToPack,
                                             FixedFramePoint2DBasics secondIntersectionToPack)
   {
      return intersectionWithLineSegment2D(lineSegment2D.getFirstEndpoint(),
                                           lineSegment2D.getSecondEndpoint(),
                                           firstIntersectionToPack,
                                           secondIntersectionToPack);
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
    * @throws RuntimeException                if this bounding box is improper according to
    *                                         {@link #checkBounds()}.
    * @throws ReferenceFrameMismatchException if any of the frame arguments is not expressed in the
    *                                         same reference frame as {@code this}.
    */
   default int intersectionWithLineSegment2D(FramePoint2DReadOnly lineSegmentStart,
                                             FramePoint2DReadOnly lineSegmentEnd,
                                             Point2DBasics firstIntersectionToPack,
                                             Point2DBasics secondIntersectionToPack)
   {
      checkReferenceFrameMatch(lineSegmentStart, lineSegmentEnd);
      return BoundingBox2DReadOnly.super.intersectionWithLineSegment2D(lineSegmentStart, lineSegmentEnd, firstIntersectionToPack, secondIntersectionToPack);
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
    * @throws RuntimeException                if this bounding box is improper according to
    *                                         {@link #checkBounds()}.
    * @throws ReferenceFrameMismatchException if any of the frame arguments is not expressed in the
    *                                         same reference frame as {@code this}.
    */
   default int intersectionWithLineSegment2D(Point2DReadOnly lineSegmentStart,
                                             Point2DReadOnly lineSegmentEnd,
                                             FixedFramePoint2DBasics firstIntersectionToPack,
                                             FixedFramePoint2DBasics secondIntersectionToPack)
   {
      if (firstIntersectionToPack != null)
         checkReferenceFrameMatch(firstIntersectionToPack);
      if (secondIntersectionToPack != null)
         checkReferenceFrameMatch(secondIntersectionToPack);
      return BoundingBox2DReadOnly.super.intersectionWithLineSegment2D(lineSegmentStart, lineSegmentEnd, firstIntersectionToPack, secondIntersectionToPack);
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
    * @throws RuntimeException if this bounding box is improper according to {@link #checkBounds()}.
    */
   default int intersectionWithLineSegment2D(Point2DReadOnly lineSegmentStart,
                                             Point2DReadOnly lineSegmentEnd,
                                             FramePoint2DBasics firstIntersectionToPack,
                                             FramePoint2DBasics secondIntersectionToPack)
   {
      if (firstIntersectionToPack != null)
         firstIntersectionToPack.setReferenceFrame(getReferenceFrame());
      if (secondIntersectionToPack != null)
         secondIntersectionToPack.setReferenceFrame(getReferenceFrame());
      return BoundingBox2DReadOnly.super.intersectionWithLineSegment2D(lineSegmentStart, lineSegmentEnd, firstIntersectionToPack, secondIntersectionToPack);
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
    * @throws RuntimeException                if this bounding box is improper according to
    *                                         {@link #checkBounds()}.
    * @throws ReferenceFrameMismatchException if any of the arguments is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default int intersectionWithLineSegment2D(FramePoint2DReadOnly lineSegmentStart,
                                             FramePoint2DReadOnly lineSegmentEnd,
                                             FixedFramePoint2DBasics firstIntersectionToPack,
                                             FixedFramePoint2DBasics secondIntersectionToPack)
   {
      checkReferenceFrameMatch(lineSegmentStart, lineSegmentEnd);
      if (firstIntersectionToPack != null)
         checkReferenceFrameMatch(firstIntersectionToPack);
      if (secondIntersectionToPack != null)
         checkReferenceFrameMatch(secondIntersectionToPack);
      return BoundingBox2DReadOnly.super.intersectionWithLineSegment2D(lineSegmentStart, lineSegmentEnd, firstIntersectionToPack, secondIntersectionToPack);
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
    * @throws RuntimeException                if this bounding box is improper according to
    *                                         {@link #checkBounds()}.
    * @throws ReferenceFrameMismatchException if either {@code lineSegmentStart} or
    *                                         {@code lineSegmentEnd} is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default int intersectionWithLineSegment2D(FramePoint2DReadOnly lineSegmentStart,
                                             FramePoint2DReadOnly lineSegmentEnd,
                                             FramePoint2DBasics firstIntersectionToPack,
                                             FramePoint2DBasics secondIntersectionToPack)
   {
      checkReferenceFrameMatch(lineSegmentStart, lineSegmentEnd);
      if (firstIntersectionToPack != null)
         firstIntersectionToPack.setReferenceFrame(getReferenceFrame());
      if (secondIntersectionToPack != null)
         secondIntersectionToPack.setReferenceFrame(getReferenceFrame());
      return BoundingBox2DReadOnly.super.intersectionWithLineSegment2D(lineSegmentStart, lineSegmentEnd, firstIntersectionToPack, secondIntersectionToPack);
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
    * @throws RuntimeException                if this bounding box is improper according to
    *                                         {@link #checkBounds()}.
    * @throws ReferenceFrameMismatchException if any of the frame arguments is not expressed in the
    *                                         same reference frame as {@code this}.
    */
   default int intersectionWithRay2D(FramePoint2DReadOnly rayOrigin,
                                     FrameVector2DReadOnly rayDirection,
                                     Point2DBasics firstIntersectionToPack,
                                     Point2DBasics secondIntersectionToPack)
   {
      checkReferenceFrameMatch(rayOrigin, rayDirection);
      return BoundingBox2DReadOnly.super.intersectionWithRay2D(rayOrigin, rayDirection, firstIntersectionToPack, secondIntersectionToPack);
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
    * @throws RuntimeException if this bounding box is improper according to {@link #checkBounds()}.
    */
   default int intersectionWithRay2D(Point2DReadOnly rayOrigin,
                                     Vector2DReadOnly rayDirection,
                                     FramePoint2DBasics firstIntersectionToPack,
                                     FramePoint2DBasics secondIntersectionToPack)
   {
      if (firstIntersectionToPack != null)
         firstIntersectionToPack.setReferenceFrame(getReferenceFrame());
      if (secondIntersectionToPack != null)
         secondIntersectionToPack.setReferenceFrame(getReferenceFrame());
      return BoundingBox2DReadOnly.super.intersectionWithRay2D(rayOrigin, rayDirection, firstIntersectionToPack, secondIntersectionToPack);
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
    * @throws RuntimeException                if this bounding box is improper according to
    *                                         {@link #checkBounds()}.
    * @throws ReferenceFrameMismatchException if any of the frame arguments is not expressed in the
    *                                         same reference frame as {@code this}.
    */
   default int intersectionWithRay2D(Point2DReadOnly rayOrigin,
                                     Vector2DReadOnly rayDirection,
                                     FixedFramePoint2DBasics firstIntersectionToPack,
                                     FixedFramePoint2DBasics secondIntersectionToPack)
   {
      if (firstIntersectionToPack != null)
         checkReferenceFrameMatch(firstIntersectionToPack);
      if (secondIntersectionToPack != null)
         checkReferenceFrameMatch(secondIntersectionToPack);
      return BoundingBox2DReadOnly.super.intersectionWithRay2D(rayOrigin, rayDirection, firstIntersectionToPack, secondIntersectionToPack);
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
    * @throws RuntimeException                if this bounding box is improper according to
    *                                         {@link #checkBounds()}.
    * @throws ReferenceFrameMismatchException if either {@code rayOrigin} or {@code rayDirection} is
    *                                         not expressed in the same reference frame as
    *                                         {@code this}.
    */
   default int intersectionWithRay2D(FramePoint2DReadOnly rayOrigin,
                                     FrameVector2DReadOnly rayDirection,
                                     FramePoint2DBasics firstIntersectionToPack,
                                     FramePoint2DBasics secondIntersectionToPack)
   {
      checkReferenceFrameMatch(rayOrigin, rayDirection);
      if (firstIntersectionToPack != null)
         firstIntersectionToPack.setReferenceFrame(getReferenceFrame());
      if (secondIntersectionToPack != null)
         secondIntersectionToPack.setReferenceFrame(getReferenceFrame());
      return BoundingBox2DReadOnly.super.intersectionWithRay2D(rayOrigin, rayDirection, firstIntersectionToPack, secondIntersectionToPack);
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
    * @throws RuntimeException                if this bounding box is improper according to
    *                                         {@link #checkBounds()}.
    * @throws ReferenceFrameMismatchException if any of the arguments is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default int intersectionWithRay2D(FramePoint2DReadOnly rayOrigin,
                                     FrameVector2DReadOnly rayDirection,
                                     FixedFramePoint2DBasics firstIntersectionToPack,
                                     FixedFramePoint2DBasics secondIntersectionToPack)
   {
      checkReferenceFrameMatch(rayOrigin, rayDirection);
      if (firstIntersectionToPack != null)
         checkReferenceFrameMatch(firstIntersectionToPack);
      if (secondIntersectionToPack != null)
         checkReferenceFrameMatch(secondIntersectionToPack);
      return BoundingBox2DReadOnly.super.intersectionWithRay2D(rayOrigin, rayDirection, firstIntersectionToPack, secondIntersectionToPack);
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
   default boolean epsilonEquals(FrameBoundingBox2DReadOnly other, double epsilon)
   {
      if (getReferenceFrame() != other.getReferenceFrame())
         return false;
      return BoundingBox2DReadOnly.super.epsilonEquals(other, epsilon);
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
   default boolean geometricallyEquals(FrameBoundingBox2DReadOnly other, double epsilon)
   {
      checkReferenceFrameMatch(other);
      return BoundingBox2DReadOnly.super.geometricallyEquals(other, epsilon);
   }

   /**
    * Tests on a per component basis, if this bounding box 2D is exactly equal to {@code other}.
    * <p>
    * If the two bounding boxes have different frames, this method returns {@code false}.
    * </p>
    *
    * @param other the other bounding box 2D to compare against this. Not modified.
    * @return {@code true} if the two bounding boxes are exactly equal component-wise and are expressed
    *         in the same reference frame, {@code false} otherwise.
    */
   default boolean equals(FrameBoundingBox2DReadOnly other)
   {
      if (other == this)
         return true;
      else if (other == null || getReferenceFrame() != other.getReferenceFrame())
         return false;
      else
         return getMinPoint().equals(other.getMinPoint()) && getMaxPoint().equals(other.getMaxPoint());
   }
}
