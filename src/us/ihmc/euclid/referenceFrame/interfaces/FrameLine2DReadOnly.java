package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Line2D;
import us.ihmc.euclid.geometry.exceptions.OutdatedPolygonException;
import us.ihmc.euclid.geometry.interfaces.Line2DBasics;
import us.ihmc.euclid.geometry.interfaces.Line2DReadOnly;
import us.ihmc.euclid.geometry.interfaces.LineSegment2DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FrameLine2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;

public interface FrameLine2DReadOnly extends Line2DReadOnly, ReferenceFrameHolder
{
   /** {@inheritDoc} */
   FramePoint2DReadOnly getPoint();

   /** {@inheritDoc} */
   FrameVector2DReadOnly getDirection();

   /**
    * Gets the direction defining this line by storing its components in the given argument
    * {@code directionToPack}.
    *
    * @param directionToPack vector in which the components of this line's direction are stored.
    *           Modified.
    */
   default void getDirection(FrameVector2D directionToPack)
   {
      directionToPack.setIncludingFrame(getDirection());
   }

   /**
    * Gets the point defining this line by storing its coordinates in the given argument
    * {@code pointToPack}.
    *
    * @param pointOnLineToPack point in which the coordinates of this line's point are stored.
    *           Modified.
    */
   default void getPoint(FramePoint2D pointOnLineToPack)
   {
      pointOnLineToPack.setIncludingFrame(getPoint());
   }

   /**
    * Gets the point and direction defining this line by storing their components in the given
    * arguments {@code pointToPack} and {@code directionToPack}.
    *
    * @param pointToPack point in which the coordinates of this line's point are stored. Modified.
    * @param directionToPack vector in which the components of this line's direction are stored.
    *           Modified.
    */
   default void getPointAndDirection(FramePoint2D pointToPack, Vector2DBasics directionToPack)
   {
      getPoint(pointToPack);
      getDirection(directionToPack);
   }

   /**
    * Gets the point and direction defining this line by storing their components in the given
    * arguments {@code pointToPack} and {@code directionToPack}.
    *
    * @param pointToPack point in which the coordinates of this line's point are stored. Modified.
    * @param directionToPack vector in which the components of this line's direction are stored.
    *           Modified.
    */
   default void getPointAndDirection(Point2DBasics pointToPack, FrameVector2D directionToPack)
   {
      getPoint(pointToPack);
      getDirection(directionToPack);
   }

   /**
    * Gets the point and direction defining this line by storing their components in the given
    * arguments {@code pointToPack} and {@code directionToPack}.
    *
    * @param pointToPack point in which the coordinates of this line's point are stored. Modified.
    * @param directionToPack vector in which the components of this line's direction are stored.
    *           Modified.
    */
   default void getPointAndDirection(FramePoint2D pointToPack, FrameVector2D directionToPack)
   {
      getPoint(pointToPack);
      getDirection(directionToPack);
   }

   /**
    * Returns a boolean value, stating whether a 2D point is on the left or right side of this line.
    * The idea of "side" is determined based on the direction of the line.
    * <p>
    * For instance, given the {@code this.direction} components x = 0, and y = 1, and the
    * {@code this.point} coordinates x = 0, and y = 0, a point located on:
    * <ul>
    * <li>the left side of this line has a negative y coordinate.
    * <li>the right side of this line has a positive y coordinate.
    * </ul>
    * </p>
    * This method will return {@code false} if the point is on this line.
    *
    * @param point the coordinates of the query point.
    * @return {@code true} if the point is on the left side of this line, {@code false} if the point
    *         is on the right side or exactly on this line.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code point} are not expressed in
    *            the same reference frame.
    */
   default boolean isPointOnLeftSideOfLine(FramePoint2DReadOnly point)
   {
      checkReferenceFrameMatch(point);
      return Line2DReadOnly.super.isPointOnLeftSideOfLine(point);
   }

   /**
    * Computes the minimum distance the given 3D point and this line.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if {@code direction.length() < }{@link EuclidGeometryTools#ONE_TRILLIONTH}, this method
    * returns the distance between {@code point} and the given {@code point}.
    * </ul>
    * </p>
    *
    * @param point 2D point to compute the distance from the line. Not modified.
    * @return the minimum distance between the 2D point and this 2D line.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code point} are not expressed in
    *            the same reference frame.
    */
   default double distance(FramePoint2DReadOnly point)
   {
      checkReferenceFrameMatch(point);
      return Line2DReadOnly.super.distance(point);
   }

   /**
    * Tests if the given is located on this line.
    * <p>
    * More precisely, the point is assumed to be on this line if it is located at a distance less
    * than {@code 1.0e-8} from it.
    * </p>
    *
    * @param point the coordinates of the query. Not modified.
    * @return {@code true} if the point is located on this line, {@code false} otherwise.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code point} are not expressed in
    *            the same reference frame.
    */
   default boolean isPointOnLine(FramePoint2DReadOnly point)
   {
      checkReferenceFrameMatch(point);
      return Line2DReadOnly.super.isPointOnLine(point);
   }

   /**
    * Tests if the given is located on this line.
    * <p>
    * More precisely, the point is assumed to be on this line if it is located at a distance less
    * than {@code epsilon} from it.
    * </p>
    *
    * @param point the coordinates of the query. Not modified.
    * @param epsilon the tolerance used for this test.
    * @return {@code true} if the point is located on this line, {@code false} otherwise.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code point} are not expressed in
    *            the same reference frame.
    */
   default boolean isPointOnLine(FramePoint2DReadOnly point, double epsilon)
   {
      checkReferenceFrameMatch(point);
      return Line2DReadOnly.super.isPointOnLine(point, epsilon);
   }

   /**
    * Returns a boolean value, stating whether a 2D point is on the left or right side of this line.
    * The idea of "side" is determined based on the direction of the line.
    * <p>
    * For instance, given the {@code this.direction} components x = 0, and y = 1, and the
    * {@code this.point} coordinates x = 0, and y = 0, a point located on:
    * <ul>
    * <li>the left side of this line has a negative y coordinate.
    * <li>the right side of this line has a positive y coordinate.
    * </ul>
    * </p>
    * This method will return {@code false} if the point is on this line.
    *
    * @param point the coordinates of the query point.
    * @return {@code true} if the point is on the right side of this line, {@code false} if the
    *         point is on the left side or exactly on this line.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code point} are not expressed in
    *            the same reference frame.
    */
   default boolean isPointOnRightSideOfLine(FramePoint2DReadOnly point)
   {
      checkReferenceFrameMatch(point);
      return Line2DReadOnly.super.isPointOnRightSideOfLine(point);
   }

   /**
    * Returns a boolean value, stating whether a 2D point is on the left or right side of this line.
    * The idea of "side" is determined based on the direction of the line.
    * <p>
    * For instance, given the {@code this.direction} components x = 0, and y = 1, and the
    * {@code this.point} coordinates x = 0, and y = 0, a point located on:
    * <ul>
    * <li>the left side of this line has a negative y coordinate.
    * <li>the right side of this line has a positive y coordinate.
    * </ul>
    * </p>
    * This method will return {@code false} if the point is on this line.
    *
    * @param point the coordinates of the query point.
    * @param testLeftSide the query of the side, when equal to {@code true} this will test for the
    *           left side, {@code false} this will test for the right side.
    * @return {@code true} if the point is on the query side of this line, {@code false} if the
    *         point is on the opposite side or exactly on this line.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code point} are not expressed in
    *            the same reference frame.
    */
   default boolean isPointOnSideOfLine(FramePoint2DReadOnly point, boolean testLeftSide)
   {
      checkReferenceFrameMatch(point);
      return Line2DReadOnly.super.isPointOnSideOfLine(point, testLeftSide);
   }

   /**
    * Calculates the coordinates of the intersection between this line and the given line and
    * returns the result.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the two lines are parallel but not collinear, the two lines do not intersect and this
    * method returns {@code null}.
    * <li>if the two lines are collinear, the two lines are assumed to be intersecting at
    * {@code this.point}.
    * </ul>
    * </p>
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param secondLine the other line that may intersect this line. Not modified.
    * @return the coordinates of the intersection if the two lines intersects, {@code null}
    *         otherwise.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code secondLine} are not
    *            expressed in the same reference frame.
    */
   default Point2D intersectionWith(FrameLine2DReadOnly secondLine)
   {
      checkReferenceFrameMatch(secondLine);
      return Line2DReadOnly.super.intersectionWith(secondLine);
   }

   /**
    * Calculates the coordinates of the intersection between this line and the given line and stores
    * the result in {@code intersectionToPack}.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the two lines are parallel but not collinear, the two lines do not intersect and this
    * method returns {@code null}.
    * <li>if the two lines are collinear, the two lines are assumed to be intersecting at
    * {@code this.point}.
    * </ul>
    * </p>
    *
    * @param secondLine the other line that may intersect this line. Not modified.
    * @param intersectionToPack the 2D point in which the result is stored. Modified.
    * @return {@code true} if the two lines intersects, {@code false} otherwise.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code secondLine} are not
    *            expressed in the same reference frame.
    */
   default boolean intersectionWith(FrameLine2DReadOnly secondLine, Point2DBasics intersectionToPack)
   {
      checkReferenceFrameMatch(secondLine);
      return Line2DReadOnly.super.intersectionWith(secondLine, intersectionToPack);
   }

   /**
    * Calculates the coordinates of the intersection between this line and the given line and stores
    * the result in {@code intersectionToPack}.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the two lines are parallel but not collinear, the two lines do not intersect and this
    * method returns {@code null}.
    * <li>if the two lines are collinear, the two lines are assumed to be intersecting at
    * {@code this.point}.
    * </ul>
    * </p>
    *
    * @param secondLine the other line that may intersect this line. Not modified.
    * @param intersectionToPack the 2D point in which the result is stored. Modified.
    * @return {@code true} if the two lines intersects, {@code false} otherwise.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code secondLine} are not
    *            expressed in the same reference frame.
    */
   default boolean intersectionWith(FrameLine2DReadOnly secondLine, FramePoint2D intersectionToPack)
   {
      checkReferenceFrameMatch(secondLine);
      intersectionToPack.setIncludingFrame(getReferenceFrame(), intersectionToPack);
      return Line2DReadOnly.super.intersectionWith(secondLine, intersectionToPack);
   }

   /**
    * Calculates the coordinates of the intersection between this line and the given line and stores
    * the result in {@code intersectionToPack}.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the two lines are parallel but not collinear, the two lines do not intersect and this
    * method returns {@code null}.
    * <li>if the two lines are collinear, the two lines are assumed to be intersecting at
    * {@code this.point}.
    * </ul>
    * </p>
    *
    * @param secondLine the other line that may intersect this line. Not modified.
    * @param intersectionToPack the 2D point in which the result is stored. Modified.
    * @return {@code true} if the two lines intersects, {@code false} otherwise.
    */
   default boolean intersectionWith(Line2DReadOnly secondLine, FramePoint2D intersectionToPack)
   {
      intersectionToPack.setIncludingFrame(getReferenceFrame(), intersectionToPack);
      return Line2DReadOnly.super.intersectionWith(secondLine, intersectionToPack);
   }

   /**
    * Calculates the coordinates of the intersection between this line and the given line segment
    * and stores the result in {@code intersectionToPack}.
    * <p>
    * Edge cases:
    * <ul>
    * <li>When this line and the line segment are parallel but not collinear, they do not intersect.
    * <li>When this line and the line segment are collinear, they are assumed to intersect at
    * {@code lineSegmentStart}.
    * <li>When this line intersects the line segment at one of its endpoints, this method returns
    * {@code true} and the endpoint is the intersection.
    * </ul>
    * </p>
    *
    * @param lineSegment the line segment that may intersect this line. Not modified.
    * @param intersectionToPack the 2D point in which the result is stored. Can be {@code null}.
    *           Modified.
    * @return {@code true} if the line intersects the line segment, {@code false} otherwise.
    */
   default boolean intersectionWith(LineSegment2DReadOnly lineSegment, FramePoint2D intersectionToPack)
   {
      intersectionToPack.setIncludingFrame(getReferenceFrame(), intersectionToPack);
      return Line2DReadOnly.super.intersectionWith(lineSegment, intersectionToPack);
   }

   /**
    * Calculates the coordinates of the intersection between this line and the given line segment
    * and stores the result in {@code intersectionToPack}.
    * <p>
    * Edge cases:
    * <ul>
    * <li>When this line and the line segment are parallel but not collinear, they do not intersect.
    * <li>When this line and the line segment are collinear, they are assumed to intersect at
    * {@code lineSegmentStart}.
    * <li>When this line intersects the line segment at one of its endpoints, this method returns
    * {@code true} and the endpoint is the intersection.
    * </ul>
    * </p>
    *
    * @param lineSegment the line segment that may intersect this line. Not modified.
    * @param intersectionToPack the 2D point in which the result is stored. Can be {@code null}.
    *           Modified.
    * @return {@code true} if the line intersects the line segment, {@code false} otherwise.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code lineSegment} are not
    *            expressed in the same reference frame.
    */
   default boolean intersectionWith(FrameLineSegment2DReadOnly lineSegment, FramePoint2D intersectionToPack)
   {
      checkReferenceFrameMatch(lineSegment);
      intersectionToPack.setIncludingFrame(getReferenceFrame(), intersectionToPack);
      return Line2DReadOnly.super.intersectionWith(lineSegment, intersectionToPack);
   }

   /**
    * Computes the coordinates of the possible intersection(s) between this line and the given
    * convex polygon 2D.
    * <p>
    * Edge cases:
    * <ul>
    * <li>If the polygon has no vertices, this method behaves as if there is no intersections.
    * <li>If no intersections exist, this method returns {@code 0} and the two intersection-to-pack
    * arguments remain unmodified.
    * <li>If there is only one intersection, this method returns {@code 1} and the coordinates of
    * the only intersection are stored in {@code firstIntersectionToPack}.
    * {@code secondIntersectionToPack} remains unmodified.
    * </ul>
    * </p>
    *
    * @param convexPolygon the convex polygon this line may intersect. Not modified.
    * @param firstIntersectionToPack point in which the coordinates of the first intersection. Can
    *           be {@code null}. Modified.
    * @param secondIntersectionToPack point in which the coordinates of the second intersection. Can
    *           be {@code null}. Modified.
    * @throws OutdatedPolygonException if the convex polygon is not up-to-date.
    */
   default int intersectionWith(ConvexPolygon2D convexPolygon, FramePoint2D firstIntersectionToPack, Point2DBasics secondIntersectionToPack)
   {
      firstIntersectionToPack.setIncludingFrame(getReferenceFrame(), firstIntersectionToPack);
      return Line2DReadOnly.super.intersectionWith(convexPolygon, firstIntersectionToPack, secondIntersectionToPack);
   }

   /**
    * Computes the coordinates of the possible intersection(s) between this line and the given
    * convex polygon 2D.
    * <p>
    * Edge cases:
    * <ul>
    * <li>If the polygon has no vertices, this method behaves as if there is no intersections.
    * <li>If no intersections exist, this method returns {@code 0} and the two intersection-to-pack
    * arguments remain unmodified.
    * <li>If there is only one intersection, this method returns {@code 1} and the coordinates of
    * the only intersection are stored in {@code firstIntersectionToPack}.
    * {@code secondIntersectionToPack} remains unmodified.
    * </ul>
    * </p>
    *
    * @param convexPolygon the convex polygon this line may intersect. Not modified.
    * @param firstIntersectionToPack point in which the coordinates of the first intersection. Can
    *           be {@code null}. Modified.
    * @param secondIntersectionToPack point in which the coordinates of the second intersection. Can
    *           be {@code null}. Modified.
    * @throws OutdatedPolygonException if the convex polygon is not up-to-date.
    */
   default int intersectionWith(ConvexPolygon2D convexPolygon, FramePoint2D firstIntersectionToPack, FramePoint2D secondIntersectionToPack)
   {
      firstIntersectionToPack.setIncludingFrame(getReferenceFrame(), firstIntersectionToPack);
      secondIntersectionToPack.setIncludingFrame(getReferenceFrame(), secondIntersectionToPack);
      return Line2DReadOnly.super.intersectionWith(convexPolygon, firstIntersectionToPack, secondIntersectionToPack);
   }

   /**
    * Computes the coordinates of the possible intersection(s) between this line and the given
    * convex polygon 2D.
    * <p>
    * Edge cases:
    * <ul>
    * <li>If the polygon has no vertices, this method behaves as if there is no intersections.
    * <li>If no intersections exist, this method returns {@code 0} and the two intersection-to-pack
    * arguments remain unmodified.
    * <li>If there is only one intersection, this method returns {@code 1} and the coordinates of
    * the only intersection are stored in {@code firstIntersectionToPack}.
    * {@code secondIntersectionToPack} remains unmodified.
    * </ul>
    * </p>
    *
    * @param convexPolygon the convex polygon this line may intersect. Not modified.
    * @param firstIntersectionToPack point in which the coordinates of the first intersection. Can
    *           be {@code null}. Modified.
    * @param secondIntersectionToPack point in which the coordinates of the second intersection. Can
    *           be {@code null}. Modified.
    * @throws OutdatedPolygonException if the convex polygon is not up-to-date.
    */
   default int intersectionWith(ConvexPolygon2D convexPolygon, Point2DBasics firstIntersectionToPack, FramePoint2D secondIntersectionToPack)
   {
      secondIntersectionToPack.setIncludingFrame(getReferenceFrame(), secondIntersectionToPack);
      return Line2DReadOnly.super.intersectionWith(convexPolygon, firstIntersectionToPack, secondIntersectionToPack);
   }

   /**
    * Calculates the coordinates of the intersection between this line and the given line segment
    * and stores the result in {@code intersectionToPack}.
    * <p>
    * Edge cases:
    * <ul>
    * <li>When this line and the line segment are parallel but not collinear, they do not intersect.
    * <li>When this line and the line segment are collinear, they are assumed to intersect at
    * {@code lineSegmentStart}.
    * <li>When this line intersects the line segment at one of its endpoints, this method returns
    * {@code true} and the endpoint is the intersection.
    * </ul>
    * </p>
    *
    * @param lineSegment the line segment that may intersect this line. Not modified.
    * @param intersectionToPack the 2D point in which the result is stored. Can be {@code null}.
    *           Modified.
    * @return {@code true} if the line intersects the line segment, {@code false} otherwise.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code lineSegment} are not
    *            expressed in the same reference frame.
    */
   default boolean intersectionWith(FrameLineSegment2DReadOnly lineSegment, Point2DBasics intersectionToPack)
   {
      checkReferenceFrameMatch(lineSegment);
      return Line2DReadOnly.super.intersectionWith(lineSegment, intersectionToPack);
   }

   /**
    * Calculates the coordinates of the intersection between this line and the given line segment
    * and returns the result.
    * <p>
    * Edge cases:
    * <ul>
    * <li>When this line and the line segment are parallel but not collinear, they do not intersect.
    * <li>When this line and the line segment are collinear, they are assumed to intersect at
    * {@code lineSegmentStart}.
    * <li>When this line intersects the line segment at one of its endpoints, this method returns a
    * copy of the endpoint where the intersection is happening.
    * </ul>
    * </p>
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param lineSegment the line segment that may intersect this line. Not modified.
    * @return the coordinates of the intersection if the line intersects the line segment,
    *         {@code null} otherwise.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code lineSegment} are not
    *            expressed in the same reference frame.
    */
   default Point2D intersectionWith(FrameLineSegment2DReadOnly lineSegment)
   {
      checkReferenceFrameMatch(lineSegment);
      return Line2DReadOnly.super.intersectionWith(lineSegment);
   }

   /**
    * Returns a boolean value, stating whether the query point is in behind of this line or not.
    * <p>
    * The idea of 'behind' refers to the side of the line the x-axis is pointing away.
    * </p>
    *
    * @param point the coordinates of the query. Not modified.
    * @return {@code true} if the point is in behind of this line, {@code false} if the point is
    *         front the line.
    * @throws RuntimeException if the given point is located exactly on this line.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code point} are not expressed in
    *            the same reference frame.
    */
   default boolean isPointBehindLine(FramePoint2DReadOnly point)
   {
      checkReferenceFrameMatch(point);
      return Line2DReadOnly.super.isPointBehindLine(point);
   }

   /**
    * Returns a boolean value, stating whether the query point is in front of this line or not.
    * <p>
    * The idea of 'front' refers to the side of the line toward which the given vector
    * {@code frontDirection} is pointing.
    * </p>
    *
    * @param frontDirection the vector used to define the side of the line which is to be considered
    *           as the front. Not modified.
    * @param point the coordinates of the query. Not modified.
    * @return {@code true} if the point is in front of this line, {@code false} if the point is
    *         behind the line.
    * @throws RuntimeException if the given point is located exactly on this line.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code frontDirection} are not
    *            expressed in the same reference frame.
    */
   default boolean isPointInFrontOfLine(FrameVector2DReadOnly frontDirection, Point2DReadOnly point)
   {
      checkReferenceFrameMatch(frontDirection);
      return Line2DReadOnly.super.isPointInFrontOfLine(frontDirection, point);
   }

   /**
    * Returns a boolean value, stating whether the query point is in front of this line or not.
    * <p>
    * The idea of 'front' refers to the side of the line toward which the given vector
    * {@code frontDirection} is pointing.
    * </p>
    *
    * @param frontDirection the vector used to define the side of the line which is to be considered
    *           as the front. Not modified.
    * @param point the coordinates of the query. Not modified.
    * @return {@code true} if the point is in front of this line, {@code false} if the point is
    *         behind the line.
    * @throws RuntimeException if the given point is located exactly on this line.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code point} are not expressed in
    *            the same reference frame.
    */
   default boolean isPointInFrontOfLine(Vector2DReadOnly frontDirection, FramePoint2DReadOnly point)
   {
      checkReferenceFrameMatch(point);
      return Line2DReadOnly.super.isPointInFrontOfLine(frontDirection, point);
   }

   /**
    * Returns a boolean value, stating whether the query point is in front of this line or not.
    * <p>
    * The idea of 'front' refers to the side of the line toward which the x-axis is pointing.
    * </p>
    *
    * @param point the coordinates of the query. Not modified.
    * @return {@code true} if the point is in front of this line, {@code false} if the point is
    *         behind the line.
    * @throws RuntimeException if the given point is located exactly on this line.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code point} are not expressed in
    *            the same reference frame.
    */
   default boolean isPointInFrontOfLine(FramePoint2DReadOnly point)
   {
      checkReferenceFrameMatch(point);
      return Line2DReadOnly.super.isPointInFrontOfLine(point);
   }

   /**
    * Returns a boolean value, stating whether the query point is in front of this line or not.
    * <p>
    * The idea of 'front' refers to the side of the line toward which the given vector
    * {@code frontDirection} is pointing.
    * </p>
    *
    * @param frontDirection the vector used to define the side of the line which is to be considered
    *           as the front. Not modified.
    * @param point the coordinates of the query. Not modified.
    * @return {@code true} if the point is in front of this line, {@code false} if the point is
    *         behind the line.
    * @throws RuntimeException if the given point is located exactly on this line.
    * @throws ReferenceFrameMismatchException if {@code this}, {@code point}, and
    *            {@code frontDirection} are not expressed in the same reference frame.
    */
   default boolean isPointInFrontOfLine(FrameVector2DReadOnly frontDirection, FramePoint2DReadOnly point)
   {
      checkReferenceFrameMatch(frontDirection);
      checkReferenceFrameMatch(point);
      return Line2DReadOnly.super.isPointInFrontOfLine(frontDirection, point);
   }

   /**
    * Computes the orthogonal projection of the given 2D point on this 2D line.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the given line direction is too small, i.e.
    * {@code lineDirection.lengthSquared() < }{@value EuclidGeometryTools#ONE_TRILLIONTH}, this
    * method fails and returns {@code false}.
    * </ul>
    * </p>
    *
    * @param pointToProject the point to project on this line. Modified.
    * @return whether the method succeeded or not.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code pointToProject} are not
    *            expressed in the same reference frame.
    */
   default boolean orthogonalProjection(FramePoint2D pointToProject)
   {
      checkReferenceFrameMatch(pointToProject);
      pointToProject.setIncludingFrame(getReferenceFrame(), pointToProject);
      return Line2DReadOnly.super.orthogonalProjection(pointToProject);
   }

   /**
    * Computes the orthogonal projection of the given 2D point on this 2D line.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the given line direction is too small, i.e.
    * {@code lineDirection.lengthSquared() < }{@value EuclidGeometryTools#ONE_TRILLIONTH}, this
    * method fails and returns {@code false}.
    * </ul>
    * </p>
    *
    * @param pointToProject the point to compute the projection of. Not modified.
    * @param projectionToPack point in which the projection of the point onto the line is stored.
    *           Modified.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code pointToProject} are not
    *            expressed in the same reference frame.
    */
   default boolean orthogonalProjection(FramePoint2DReadOnly pointToProject, Point2DBasics projectionToPack)
   {
      checkReferenceFrameMatch(pointToProject);
      return Line2DReadOnly.super.orthogonalProjection(pointToProject, projectionToPack);
   }

   /**
    * Computes the orthogonal projection of the given 2D point on this 2D line.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the given line direction is too small, i.e.
    * {@code lineDirection.lengthSquared() < }{@value EuclidGeometryTools#ONE_TRILLIONTH}, this
    * method fails and returns {@code false}.
    * </ul>
    * </p>
    *
    * @param pointToProject the point to compute the projection of. Not modified.
    * @param projectionToPack point in which the projection of the point onto the line is stored.
    *           Modified.
    */
   default boolean orthogonalProjection(Point2DReadOnly pointToProject, FramePoint2D projectionToPack)
   {
      projectionToPack.setToZero(getReferenceFrame());
      return Line2DReadOnly.super.orthogonalProjection(pointToProject, projectionToPack);
   }

   /**
    * Computes the orthogonal projection of the given 2D point on this 2D line.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the given line direction is too small, i.e.
    * {@code lineDirection.lengthSquared() < }{@value EuclidGeometryTools#ONE_TRILLIONTH}, this
    * method fails and returns {@code false}.
    * </ul>
    * </p>
    *
    * @param pointToProject the point to compute the projection of. Not modified.
    * @param projectionToPack point in which the projection of the point onto the line is stored.
    *           Modified.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code pointToProject} are not
    *            expressed in the same reference frame.
    */
   default boolean orthogonalProjection(FramePoint2DReadOnly pointToProject, FramePoint2D projectionToPack)
   {
      checkReferenceFrameMatch(pointToProject);
      projectionToPack.setToZero(getReferenceFrame());
      return Line2DReadOnly.super.orthogonalProjection(pointToProject, projectionToPack);
   }

   /**
    * Computes the orthogonal projection of the given 2D point on this 2D line.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the given line direction is too small, i.e.
    * {@code lineDirection.lengthSquared() < }{@value EuclidGeometryTools#ONE_TRILLIONTH}, this
    * method fails and returns {@code false}.
    * </ul>
    * </p>
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param pointToProject the point to compute the projection of. Not modified.
    * @return the projection of the point onto the line or {@code null} if the method failed.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code pointToProject} are not
    *            expressed in the same reference frame.
    */
   default Point2D orthogonalProjectionCopy(FramePoint2DReadOnly pointToProject)
   {
      checkReferenceFrameMatch(pointToProject);
      return Line2DReadOnly.super.orthogonalProjectionCopy(pointToProject);
   }

   /**
    * Calculates the interior bisector defined by this line and the given {@code secondLine}.
    * <p>
    * The interior bisector is defined as follows:
    * <ul>
    * <li>It goes through the intersection between this line and {@code secondLine}.
    * <li>Its direction point toward this line direction and the {@code secondLine}'s direction such
    * that: {@code interiorBisector.direction.dot(this.direction) > 0.0} and
    * {@code interiorBisector.direction.dot(secondLine.direction) > 0.0}.
    * <li>Finally the angle from {@code this} to the interior bisector is half the angle from
    * {@code this} to {@code secondLine}.
    * </ul>
    * </p>
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>If the two lines are parallel but not collinear, this method fails, returns {@code null}.
    * <li>If the two lines are collinear, this method returns a copy of {@code this}.
    * </ul>
    * </p>
    *
    * @param secondLine the second line needed to calculate the interior bisector. Not modified.
    * @return the interior bisector if this method succeeded, {@code null} otherwise.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code secondLine} are not
    *            expressed in the same reference frame.
    */
   default Line2D interiorBisector(FrameLine2DReadOnly secondLine)
   {
      checkReferenceFrameMatch(secondLine);
      return Line2DReadOnly.super.interiorBisector(secondLine);
   }

   /**
    * Compares {@code this} with {@code other} to determine if the two lines are collinear.
    *
    * @param other the line to compare to. Not modified.
    * @param epsilon the tolerance of the comparison.
    * @return {@code true} if the lines are collinear, {@code false} otherwise.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code other} are not expressed in
    *            the same reference frame.
    */
   default boolean isCollinear(FrameLine2DReadOnly other, double epsilon)
   {
      checkReferenceFrameMatch(other);
      return Line2DReadOnly.super.isCollinear(other, epsilon);
   }

   /**
    * Compares {@code this} with {@code other} to determine if the two lines are collinear.
    *
    * @param other the line to compare to. Not modified.
    * @param angleEpsilon the tolerance of the comparison for angle.
    * @param distanceEpsilon the tolerance of the comparison for distance.
    * @return {@code true} if the lines are collinear, {@code false} otherwise.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code other} are not expressed in
    *            the same reference frame.
    */
   default boolean isCollinear(FrameLine2DReadOnly other, double angleEpsilon, double distanceEpsilon)
   {
      checkReferenceFrameMatch(other);
      return Line2DReadOnly.super.isCollinear(other, angleEpsilon, distanceEpsilon);
   }

   /**
    * Calculates the parameter 't' corresponding to the coordinates of the given {@code pointOnLine}
    * 'p' by solving the line equation:<br>
    * p = t * n + p<sub>0</sub><br>
    * where n is the unit-vector defining the direction of this line and p<sub>0</sub> is the point
    * defining this line which also corresponds to the point for which t=0.
    * <p>
    * Note that the absolute value of 't' is equal to the distance between the point 'p' and the
    * point p<sub>0</sub> defining this line.
    * </p>
    *
    * @param pointOnLine the coordinates of the 'p' from which the parameter 't' is to be
    *           calculated. The point has to be on the line. Not modified.
    * @param epsilon the maximum distance allowed between the given point and this line. If the
    *           given point is at a distance less than {@code epsilon} from this line, it is
    *           considered as being located on this line.
    * @return the value of the parameter 't' corresponding to the given point.
    * @throws RuntimeException if the given point is located at a distance greater than
    *            {@code epsilon} from this line.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code pointOnLine} are not
    *            expressed in the same reference frame.
    */
   default double parameterGivenPointOnLine(FramePoint2DReadOnly pointOnLine, double epsilon)
   {
      checkReferenceFrameMatch(pointOnLine);
      return Line2DReadOnly.super.parameterGivenPointOnLine(pointOnLine, epsilon);
   }

   /**
    * Calculates and returns a line that is perpendicular to this line, with its direction pointing
    * to the left of this line, while going through the given point.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param point the point the line has to go through. Not modified.
    * @return the line perpendicular to {@code this} and going through {@code point}.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code point} are not expressed in
    *            the same reference frame.
    */
   default Line2D perpendicularLineThroughPoint(FramePoint2DReadOnly point)
   {
      checkReferenceFrameMatch(point);
      return Line2DReadOnly.super.perpendicularLineThroughPoint(point);
   }

   /**
    * Modifies {@code perpendicularLineToPack} such that it is perpendicular to this line, with its
    * direction pointing to the left of this line, while going through the given point.
    *
    * @param point the point the line has to go through. Not modified.
    * @param perpendicularLineToPack the line perpendicular to {@code this} and going through
    *           {@code point}. Modified.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code point} are not expressed in
    *            the same reference frame.
    */
   default void perpendicularLineThroughPoint(FramePoint2DReadOnly point, Line2DBasics perpendicularLineToPack)
   {
      checkReferenceFrameMatch(point);
      Line2DReadOnly.super.perpendicularLineThroughPoint(point, perpendicularLineToPack);
   }

   /**
    * Modifies {@code perpendicularLineToPack} such that it is perpendicular to this line, with its
    * direction pointing to the left of this line, while going through the given point.
    *
    * @param point the point the line has to go through. Not modified.
    * @param perpendicularLineToPack the line perpendicular to {@code this} and going through
    *           {@code point}. Modified.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code point} are not expressed in
    *            the same reference frame.
    */
   default void perpendicularLineThroughPoint(FramePoint2DReadOnly point, FrameLine2D perpendicularLineToPack)
   {
      checkReferenceFrameMatch(point);
      perpendicularLineToPack.setToZero(getReferenceFrame());
      Line2DReadOnly.super.perpendicularLineThroughPoint(point, perpendicularLineToPack);
   }

   /**
    * Modifies {@code perpendicularLineToPack} such that it is perpendicular to this line, with its
    * direction pointing to the left of this line, while going through the given point.
    *
    * @param point the point the line has to go through. Not modified.
    * @param perpendicularLineToPack the line perpendicular to {@code this} and going through
    *           {@code point}. Modified.
    */
   default void perpendicularLineThroughPoint(Point2DReadOnly point, FrameLine2D perpendicularLineToPack)
   {
      perpendicularLineToPack.setToZero(getReferenceFrame());
      Line2DReadOnly.super.perpendicularLineThroughPoint(point, perpendicularLineToPack);
   }

   /**
    * Gets the coordinates of two distinct points this line goes through.
    *
    * @param firstPointOnLineToPack the coordinates of a first point located on this line. Modified.
    * @param secondPointOnLineToPack the coordinates of a second point located on this line.
    *           Modified.
    */
   default void getTwoPointsOnLine(FramePoint2D firstPointOnLineToPack, Point2DBasics secondPointOnLineToPack)
   {
      firstPointOnLineToPack.setToZero(getReferenceFrame());
      Line2DReadOnly.super.getTwoPointsOnLine(firstPointOnLineToPack, secondPointOnLineToPack);
   }

   /**
    * Gets the coordinates of two distinct points this line goes through.
    *
    * @param firstPointOnLineToPack the coordinates of a first point located on this line. Modified.
    * @param secondPointOnLineToPack the coordinates of a second point located on this line.
    *           Modified.
    */
   default void getTwoPointsOnLine(Point2DBasics firstPointOnLineToPack, FramePoint2D secondPointOnLineToPack)
   {
      secondPointOnLineToPack.setToZero(getReferenceFrame());
      Line2DReadOnly.super.getTwoPointsOnLine(firstPointOnLineToPack, secondPointOnLineToPack);
   }

   /**
    * Gets the coordinates of two distinct points this line goes through.
    *
    * @param firstPointOnLineToPack the coordinates of a first point located on this line. Modified.
    * @param secondPointOnLineToPack the coordinates of a second point located on this line.
    *           Modified.
    */
   default void getTwoPointsOnLine(FramePoint2D firstPointOnLineToPack, FramePoint2D secondPointOnLineToPack)
   {
      firstPointOnLineToPack.setToZero(getReferenceFrame());
      secondPointOnLineToPack.setToZero(getReferenceFrame());
      Line2DReadOnly.super.getTwoPointsOnLine(firstPointOnLineToPack, secondPointOnLineToPack);
   }

   /**
    * Tests if this line and the other line are perpendicular.
    *
    * @param other the query. Not modified.
    * @return {@code true} if the two lines are perpendicular, {@code false} otherwise.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code other} are not expressed in
    *            the same reference frame.
    */
   default boolean areLinesPerpendicular(FrameLine2DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      return Line2DReadOnly.super.areLinesPerpendicular(other);
   }

   /**
    * Packs into {@code vectorToPack} the vector that is perpendicular to this line and pointing to
    * the left.
    *
    * @param vectorToPack the perpendicular vector to this line. Modified.
    */
   default void perpendicularVector(FrameVector2D vectorToPack)
   {
      vectorToPack.setToZero(getReferenceFrame());
      Line2DReadOnly.super.perpendicularVector(vectorToPack);
   }

   /**
    * Calculates the coordinates of the point 'p' given the parameter 't' as follows:<br>
    * p = t * n + p<sub>0</sub><br>
    * where n is the unit-vector defining the direction of this line and p<sub>0</sub> is the point
    * defining this line which also corresponds to the point for which t=0.
    * <p>
    * Note that the absolute value of 't' is equal to the distance between the point 'p' and the
    * point p<sub>0</sub> defining this line.
    * </p>
    *
    * @param t the parameter used to calculate the point coordinates.
    * @param pointToPack the point in which the coordinates of 'p' are stored. Modified.
    */
   default void pointOnLineGivenParameter(double t, FramePoint2D pointToPack)
   {
      pointToPack.setToZero(getReferenceFrame());
      Line2DReadOnly.super.pointOnLineGivenParameter(t, pointToPack);
   }

   /**
    * Calculates the interior bisector defined by this line and the given {@code secondLine}.
    * <p>
    * The interior bisector is defined as follows:
    * <ul>
    * <li>It goes through the intersection between this line and {@code secondLine}.
    * <li>Its direction point toward this line direction and the {@code secondLine}'s direction such
    * that: {@code interiorBisector.direction.dot(this.direction) > 0.0} and
    * {@code interiorBisector.direction.dot(secondLine.direction) > 0.0}.
    * <li>Finally the angle from {@code this} to the interior bisector is half the angle from
    * {@code this} to {@code secondLine}.
    * </ul>
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>If the two lines are parallel but not collinear, this method fails, returns {@code false},
    * and {@code interiorBisectorToPack} remains unchanged.
    * <li>If the two lines are collinear, {@code interiorBisectorToPack} is set to {@code this}.
    * </ul>
    * </p>
    *
    * @param secondLine the second line needed to calculate the interior bisector. Not modified.
    * @param interiorBisectorToPack the line in which the interior bisector point and direction are
    *           stored. Modified.
    * @return {@code true} if this method succeeded, {@code false} otherwise.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code secondLine} are not
    *            expressed in the same reference frame.
    */
   default boolean interiorBisector(FrameLine2DReadOnly secondLine, Line2DBasics interiorBisectorToPack)
   {
      checkReferenceFrameMatch(secondLine);
      return Line2DReadOnly.super.interiorBisector(secondLine, interiorBisectorToPack);
   }

   /**
    * Calculates the interior bisector defined by this line and the given {@code secondLine}.
    * <p>
    * The interior bisector is defined as follows:
    * <ul>
    * <li>It goes through the intersection between this line and {@code secondLine}.
    * <li>Its direction point toward this line direction and the {@code secondLine}'s direction such
    * that: {@code interiorBisector.direction.dot(this.direction) > 0.0} and
    * {@code interiorBisector.direction.dot(secondLine.direction) > 0.0}.
    * <li>Finally the angle from {@code this} to the interior bisector is half the angle from
    * {@code this} to {@code secondLine}.
    * </ul>
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>If the two lines are parallel but not collinear, this method fails, returns {@code false},
    * and {@code interiorBisectorToPack} remains unchanged.
    * <li>If the two lines are collinear, {@code interiorBisectorToPack} is set to {@code this}.
    * </ul>
    * </p>
    *
    * @param secondLine the second line needed to calculate the interior bisector. Not modified.
    * @param interiorBisectorToPack the line in which the interior bisector point and direction are
    *           stored. Modified.
    * @return {@code true} if this method succeeded, {@code false} otherwise.
    */
   default boolean interiorBisector(Line2DReadOnly secondLine, FrameLine2D interiorBisectorToPack)
   {
      interiorBisectorToPack.setIncludingFrame(getReferenceFrame(), interiorBisectorToPack.getGeometryObject());
      return Line2DReadOnly.super.interiorBisector(secondLine, interiorBisectorToPack);
   }

   /**
    * Calculates the interior bisector defined by this line and the given {@code secondLine}.
    * <p>
    * The interior bisector is defined as follows:
    * <ul>
    * <li>It goes through the intersection between this line and {@code secondLine}.
    * <li>Its direction point toward this line direction and the {@code secondLine}'s direction such
    * that: {@code interiorBisector.direction.dot(this.direction) > 0.0} and
    * {@code interiorBisector.direction.dot(secondLine.direction) > 0.0}.
    * <li>Finally the angle from {@code this} to the interior bisector is half the angle from
    * {@code this} to {@code secondLine}.
    * </ul>
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>If the two lines are parallel but not collinear, this method fails, returns {@code false},
    * and {@code interiorBisectorToPack} remains unchanged.
    * <li>If the two lines are collinear, {@code interiorBisectorToPack} is set to {@code this}.
    * </ul>
    * </p>
    *
    * @param secondLine the second line needed to calculate the interior bisector. Not modified.
    * @param interiorBisectorToPack the line in which the interior bisector point and direction are
    *           stored. Modified.
    * @return {@code true} if this method succeeded, {@code false} otherwise.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code secondLine} are not
    *            expressed in the same reference frame.
    */
   default boolean interiorBisector(FrameLine2DReadOnly secondLine, FrameLine2D interiorBisectorToPack)
   {
      checkReferenceFrameMatch(secondLine);
      interiorBisectorToPack.setIncludingFrame(getReferenceFrame(), interiorBisectorToPack.getGeometryObject());
      return Line2DReadOnly.super.interiorBisector(secondLine, interiorBisectorToPack);
   }
}
