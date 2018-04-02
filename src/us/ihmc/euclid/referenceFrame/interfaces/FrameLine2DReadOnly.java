package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.geometry.interfaces.Line2DBasics;
import us.ihmc.euclid.geometry.interfaces.Line2DReadOnly;
import us.ihmc.euclid.geometry.interfaces.LineSegment2DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FrameLine2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;

/**
 * Read-only interface for a line 2D expressed in a given reference frame.
 * <p>
 * A line 2D represents an infinitely long line in the XY-plane and defined by a point and a
 * direction.
 * </p>
 * <p>
 * In addition to representing a {@link Line2DReadOnly}, a {@link ReferenceFrame} is associated to a
 * {@code FrameLine2DReadOnly}. This allows, for instance, to enforce, at runtime, that operations
 * on lines occur in the same coordinate system.
 * </p>
 * <p>
 * Because a {@code FrameLine2DReadOnly} extends {@code Line2DReadOnly}, it is compatible with
 * methods only requiring {@code Line2DReadOnly}. However, these methods do NOT assert that the
 * operation occur in the proper coordinate system. Use this feature carefully and always prefer
 * using methods requiring {@code FrameLine2DReadOnly}.
 * </p>
 */
public interface FrameLine2DReadOnly extends Line2DReadOnly, ReferenceFrameHolder
{
   /** {@inheritDoc} */
   @Override
   FramePoint2DReadOnly getPoint();

   /** {@inheritDoc} */
   @Override
   FrameVector2DReadOnly getDirection();

   /**
    * Gets the point and direction defining this line by storing their components in the given
    * arguments {@code pointToPack} and {@code directionToPack}.
    *
    * @param pointToPack point in which the coordinates of this line's point are stored. Modified.
    * @param directionToPack vector in which the components of this line's direction are stored.
    *           Modified.
    * @throws ReferenceFrameMismatchException if either argument is not expressed in the same
    *            reference frame as this frame line 2D.
    */
   default void get(FixedFramePoint2DBasics pointToPack, FixedFrameVector2DBasics directionToPack)
   {
      checkReferenceFrameMatch(pointToPack);
      checkReferenceFrameMatch(directionToPack);
      Line2DReadOnly.super.get(pointToPack, directionToPack);
   }

   /**
    * Gets the point and direction defining this line by storing their components in the given
    * arguments {@code pointToPack} and {@code directionToPack}.
    *
    * @param pointToPack point in which the coordinates of this line's point are stored. Modified.
    * @param directionToPack vector in which the components of this line's direction are stored.
    *           Modified.
    */
   default void get(FramePoint2DBasics pointToPack, FrameVector2DBasics directionToPack)
   {
      pointToPack.setReferenceFrame(getReferenceFrame());
      directionToPack.setReferenceFrame(getReferenceFrame());
      Line2DReadOnly.super.get(pointToPack, directionToPack);
   }

   /**
    * Gets the point and direction defining this line by storing their components in the given
    * arguments {@code pointToPack} and {@code directionToPack}.
    *
    * @param pointToPack point in which the coordinates of this line's point are stored. Modified.
    * @param directionToPack vector in which the components of this line's direction are stored.
    *           Modified.
    * @throws ReferenceFrameMismatchException if {@code directionToPack} is not expressed in the
    *            same reference frame as this frame line 2D.
    */
   default void get(Point2DBasics pointToPack, FixedFrameVector2DBasics directionToPack)
   {
      checkReferenceFrameMatch(directionToPack);
      Line2DReadOnly.super.get(pointToPack, directionToPack);
   }

   /**
    * Gets the point and direction defining this line by storing their components in the given
    * arguments {@code pointToPack} and {@code directionToPack}.
    *
    * @param pointToPack point in which the coordinates of this line's point are stored. Modified.
    * @param directionToPack vector in which the components of this line's direction are stored.
    *           Modified.
    */
   default void get(Point2DBasics pointToPack, FrameVector2DBasics directionToPack)
   {
      directionToPack.setReferenceFrame(getReferenceFrame());
      Line2DReadOnly.super.get(pointToPack, directionToPack);
   }

   /**
    * Gets the point and direction defining this line by storing their components in the given
    * arguments {@code pointToPack} and {@code directionToPack}.
    *
    * @param pointToPack point in which the coordinates of this line's point are stored. Modified.
    * @param directionToPack vector in which the components of this line's direction are stored.
    *           Modified.
    * @throws ReferenceFrameMismatchException if {@code pointToPack} is not expressed in the same
    *            reference frame as this frame line 2D.
    */
   default void get(FixedFramePoint2DBasics pointToPack, Vector2DBasics directionToPack)
   {
      checkReferenceFrameMatch(pointToPack);
      Line2DReadOnly.super.get(pointToPack, directionToPack);
   }

   /**
    * Gets the point and direction defining this line by storing their components in the given
    * arguments {@code pointToPack} and {@code directionToPack}.
    *
    * @param pointToPack point in which the coordinates of this line's point are stored. Modified.
    * @param directionToPack vector in which the components of this line's direction are stored.
    *           Modified.
    */
   default void get(FramePoint2DBasics pointToPack, Vector2DBasics directionToPack)
   {
      pointToPack.setReferenceFrame(getReferenceFrame());
      Line2DReadOnly.super.get(pointToPack, directionToPack);
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

   /** {@inheritDoc} */
   @Override
   default FramePoint2DBasics intersectionWith(Line2DReadOnly secondLine)
   {
      Point2DBasics intersection = Line2DReadOnly.super.intersectionWith(secondLine);
      if (intersection == null)
         return null;
      else
         return new FramePoint2D(getReferenceFrame(), intersection);
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
   default FramePoint2DBasics intersectionWith(FrameLine2DReadOnly secondLine)
   {
      checkReferenceFrameMatch(secondLine);
      Point2DBasics intersection = Line2DReadOnly.super.intersectionWith(secondLine);
      if (intersection == null)
         return null;
      else
         return new FramePoint2D(getReferenceFrame(), intersection);
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
    * @throws ReferenceFrameMismatchException if {@code this}, {@code secondLine}, and
    *            {@code intersectionToPack} are not expressed in the same reference frame.
    */
   default boolean intersectionWith(FrameLine2DReadOnly secondLine, FixedFramePoint2DBasics intersectionToPack)
   {
      checkReferenceFrameMatch(secondLine);
      checkReferenceFrameMatch(intersectionToPack);
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
   default boolean intersectionWith(FrameLine2DReadOnly secondLine, FramePoint2DBasics intersectionToPack)
   {
      checkReferenceFrameMatch(secondLine);
      intersectionToPack.setReferenceFrame(getReferenceFrame());
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
    * @throws ReferenceFrameMismatchException if {@code this} and {@code intersectionToPack} are not
    *            expressed in the same reference frame.
    */
   default boolean intersectionWith(Line2DReadOnly secondLine, FixedFramePoint2DBasics intersectionToPack)
   {
      checkReferenceFrameMatch(intersectionToPack);
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
   default boolean intersectionWith(Line2DReadOnly secondLine, FramePoint2DBasics intersectionToPack)
   {
      intersectionToPack.setReferenceFrame(getReferenceFrame());
      return Line2DReadOnly.super.intersectionWith(secondLine, intersectionToPack);
   }

   /** {@inheritDoc} */
   @Override
   default FramePoint2DBasics intersectionWith(LineSegment2DReadOnly lineSegment)
   {
      Point2DBasics intersection = Line2DReadOnly.super.intersectionWith(lineSegment);
      if (intersection == null)
         return null;
      else
         return new FramePoint2D(getReferenceFrame(), intersection);
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
   default FramePoint2DBasics intersectionWith(FrameLineSegment2DReadOnly lineSegment)
   {
      checkReferenceFrameMatch(lineSegment);
      Point2DBasics intersection = Line2DReadOnly.super.intersectionWith(lineSegment);
      if (intersection == null)
         return null;
      else
         return new FramePoint2D(getReferenceFrame(), intersection);
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
    * @throws ReferenceFrameMismatchException if {@code this} and {@code intersectionToPack} are not
    *            expressed in the same reference frame.
    */
   default boolean intersectionWith(LineSegment2DReadOnly lineSegment, FixedFramePoint2DBasics intersectionToPack)
   {
      checkReferenceFrameMatch(intersectionToPack);
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
    */
   default boolean intersectionWith(LineSegment2DReadOnly lineSegment, FramePoint2DBasics intersectionToPack)
   {
      intersectionToPack.setReferenceFrame(getReferenceFrame());
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
    * @throws ReferenceFrameMismatchException if {@code this}, {@code lineSegment}, and
    *            {@code intersectionToPack} are not expressed in the same reference frame.
    */
   default boolean intersectionWith(FrameLineSegment2DReadOnly lineSegment, FixedFramePoint2DBasics intersectionToPack)
   {
      checkReferenceFrameMatch(lineSegment);
      checkReferenceFrameMatch(intersectionToPack);
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
   default boolean intersectionWith(FrameLineSegment2DReadOnly lineSegment, FramePoint2DBasics intersectionToPack)
   {
      checkReferenceFrameMatch(lineSegment);
      intersectionToPack.setReferenceFrame(getReferenceFrame());
      return Line2DReadOnly.super.intersectionWith(lineSegment, intersectionToPack);
   }

   /** {@inheritDoc} */
   @Override
   default FramePoint2DBasics[] intersectionWith(ConvexPolygon2DReadOnly convexPolygon)
   {
      Point2DBasics[] intersections = convexPolygon.intersectionWith(this);
      if (intersections == null)
      {
         return null;
      }
      else
      {
         FramePoint2D[] frameIntersections = new FramePoint2D[intersections.length];
         for (int i = 0; i < intersections.length; i++)
            frameIntersections[i] = new FramePoint2D(getReferenceFrame(), intersections[i]);
         return frameIntersections;
      }
   }

   /**
    * Computes the coordinates of the possible intersection(s) between this line and the given
    * convex polygon 2D.
    * 
    * @param convexPolygon the convex polygon this line may intersect. Not modified.
    * @return the intersections between between the line and the polygon or {@code null} if the
    *         method failed or if there is no intersections.
    * @see #intersectionWith(ConvexPolygon2DReadOnly)
    */
   default FramePoint2DBasics[] intersectionWith(FrameConvexPolygon2DReadOnly convexPolygon)
   {
      checkReferenceFrameMatch(convexPolygon);
      return intersectionWith((ConvexPolygon2DReadOnly) convexPolygon);
   }

   /**
    * Computes the coordinates of the possible intersection(s) between this line and the given
    * convex polygon 2D.
    *
    * @param convexPolygon the convex polygon this line may intersect. Not modified.
    * @param firstIntersectionToPack point in which the coordinates of the first intersection. Can
    *           be {@code null}. Modified.
    * @param secondIntersectionToPack point in which the coordinates of the second intersection. Can
    *           be {@code null}. Modified.
    * @return the number of intersections between this line and the polygon.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code firstIntersectionToPack}
    *            are not expressed in the same reference frame.
    * @see #intersectionWith(ConvexPolygon2DReadOnly, Point2DBasics, Point2DBasics)
    */
   default int intersectionWith(ConvexPolygon2DReadOnly convexPolygon, FixedFramePoint2DBasics firstIntersectionToPack, Point2DBasics secondIntersectionToPack)
   {
      checkReferenceFrameMatch(firstIntersectionToPack);
      return Line2DReadOnly.super.intersectionWith(convexPolygon, firstIntersectionToPack, secondIntersectionToPack);
   }

   /**
    * Computes the coordinates of the possible intersection(s) between this line and the given
    * convex polygon 2D.
    *
    * @param convexPolygon the convex polygon this line may intersect. Not modified.
    * @param firstIntersectionToPack point in which the coordinates of the first intersection. Can
    *           be {@code null}. Modified.
    * @param secondIntersectionToPack point in which the coordinates of the second intersection. Can
    *           be {@code null}. Modified.
    * @return the number of intersections between this line and the polygon.
    * @throws ReferenceFrameMismatchException if {@code this}, {@code convexPolygon}, and
    *            {@code firstIntersectionToPack} are not expressed in the same reference frame.
    * @see #intersectionWith(ConvexPolygon2DReadOnly, Point2DBasics, Point2DBasics)
    */
   default int intersectionWith(FrameConvexPolygon2DReadOnly convexPolygon, FixedFramePoint2DBasics firstIntersectionToPack,
                                Point2DBasics secondIntersectionToPack)
   {
      checkReferenceFrameMatch(convexPolygon);
      checkReferenceFrameMatch(firstIntersectionToPack);
      return Line2DReadOnly.super.intersectionWith(convexPolygon, firstIntersectionToPack, secondIntersectionToPack);
   }

   /**
    * Computes the coordinates of the possible intersection(s) between this line and the given
    * convex polygon 2D.
    *
    * @param convexPolygon the convex polygon this line may intersect. Not modified.
    * @param firstIntersectionToPack point in which the coordinates of the first intersection. Can
    *           be {@code null}. Modified.
    * @param secondIntersectionToPack point in which the coordinates of the second intersection. Can
    *           be {@code null}. Modified.
    * @return the number of intersections between this line and the polygon.
    * @see #intersectionWith(ConvexPolygon2DReadOnly, Point2DBasics, Point2DBasics)
    */
   default int intersectionWith(ConvexPolygon2DReadOnly convexPolygon, FramePoint2DBasics firstIntersectionToPack, Point2DBasics secondIntersectionToPack)
   {
      firstIntersectionToPack.setReferenceFrame(getReferenceFrame());
      return Line2DReadOnly.super.intersectionWith(convexPolygon, firstIntersectionToPack, secondIntersectionToPack);
   }

   /**
    * Computes the coordinates of the possible intersection(s) between this line and the given
    * convex polygon 2D.
    *
    * @param convexPolygon the convex polygon this line may intersect. Not modified.
    * @param firstIntersectionToPack point in which the coordinates of the first intersection. Can
    *           be {@code null}. Modified.
    * @param secondIntersectionToPack point in which the coordinates of the second intersection. Can
    *           be {@code null}. Modified.
    * @return the number of intersections between this line and the polygon.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code convexPolygon} are not
    *            expressed in the same reference frame.
    * @see #intersectionWith(ConvexPolygon2DReadOnly, Point2DBasics, Point2DBasics)
    */
   default int intersectionWith(FrameConvexPolygon2DReadOnly convexPolygon, FramePoint2DBasics firstIntersectionToPack, Point2DBasics secondIntersectionToPack)
   {
      checkReferenceFrameMatch(convexPolygon);
      firstIntersectionToPack.setReferenceFrame(getReferenceFrame());
      return Line2DReadOnly.super.intersectionWith(convexPolygon, firstIntersectionToPack, secondIntersectionToPack);
   }

   /**
    * Computes the coordinates of the possible intersection(s) between this line and the given
    * convex polygon 2D.
    *
    * @param convexPolygon the convex polygon this line may intersect. Not modified.
    * @param firstIntersectionToPack point in which the coordinates of the first intersection. Can
    *           be {@code null}. Modified.
    * @param secondIntersectionToPack point in which the coordinates of the second intersection. Can
    *           be {@code null}. Modified.
    * @return the number of intersections between this line and the polygon.
    * @throws ReferenceFrameMismatchException if {@code this}, {@code firstIntersectionToPack}, and
    *            {@code secondIntersectionToPack} are not expressed in the same reference frame.
    * @see #intersectionWith(ConvexPolygon2DReadOnly, Point2DBasics, Point2DBasics)
    */
   default int intersectionWith(ConvexPolygon2DReadOnly convexPolygon, FixedFramePoint2DBasics firstIntersectionToPack,
                                FixedFramePoint2DBasics secondIntersectionToPack)
   {
      checkReferenceFrameMatch(firstIntersectionToPack);
      checkReferenceFrameMatch(secondIntersectionToPack);
      return Line2DReadOnly.super.intersectionWith(convexPolygon, firstIntersectionToPack, secondIntersectionToPack);
   }

   /**
    * Computes the coordinates of the possible intersection(s) between this line and the given
    * convex polygon 2D.
    *
    * @param convexPolygon the convex polygon this line may intersect. Not modified.
    * @param firstIntersectionToPack point in which the coordinates of the first intersection. Can
    *           be {@code null}. Modified.
    * @param secondIntersectionToPack point in which the coordinates of the second intersection. Can
    *           be {@code null}. Modified.
    * @return the number of intersections between this line and the polygon.
    * @throws ReferenceFrameMismatchException if {@code this}, {@code convexPolygon},
    *            {@code firstIntersectionToPack}, and {@code secondIntersectionToPack} are not
    *            expressed in the same reference frame.
    * @see #intersectionWith(ConvexPolygon2DReadOnly, Point2DBasics, Point2DBasics)
    */
   default int intersectionWith(FrameConvexPolygon2DReadOnly convexPolygon, FixedFramePoint2DBasics firstIntersectionToPack,
                                FixedFramePoint2DBasics secondIntersectionToPack)
   {
      checkReferenceFrameMatch(convexPolygon);
      checkReferenceFrameMatch(firstIntersectionToPack);
      checkReferenceFrameMatch(secondIntersectionToPack);
      return Line2DReadOnly.super.intersectionWith(convexPolygon, firstIntersectionToPack, secondIntersectionToPack);
   }

   /**
    * Computes the coordinates of the possible intersection(s) between this line and the given
    * convex polygon 2D.
    *
    * @param convexPolygon the convex polygon this line may intersect. Not modified.
    * @param firstIntersectionToPack point in which the coordinates of the first intersection. Can
    *           be {@code null}. Modified.
    * @param secondIntersectionToPack point in which the coordinates of the second intersection. Can
    *           be {@code null}. Modified.
    * @return the number of intersections between this line and the polygon.
    * @see #intersectionWith(ConvexPolygon2DReadOnly, Point2DBasics, Point2DBasics)
    */
   default int intersectionWith(ConvexPolygon2DReadOnly convexPolygon, FramePoint2DBasics firstIntersectionToPack, FramePoint2DBasics secondIntersectionToPack)
   {
      firstIntersectionToPack.setReferenceFrame(getReferenceFrame());
      secondIntersectionToPack.setReferenceFrame(getReferenceFrame());
      return Line2DReadOnly.super.intersectionWith(convexPolygon, firstIntersectionToPack, secondIntersectionToPack);
   }

   /**
    * Computes the coordinates of the possible intersection(s) between this line and the given
    * convex polygon 2D.
    *
    * @param convexPolygon the convex polygon this line may intersect. Not modified.
    * @param firstIntersectionToPack point in which the coordinates of the first intersection. Can
    *           be {@code null}. Modified.
    * @param secondIntersectionToPack point in which the coordinates of the second intersection. Can
    *           be {@code null}. Modified.
    * @return the number of intersections between this line and the polygon.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code convexPolygon} are not
    *            expressed in the same reference frame.
    * @see #intersectionWith(ConvexPolygon2DReadOnly, Point2DBasics, Point2DBasics)
    */
   default int intersectionWith(FrameConvexPolygon2DReadOnly convexPolygon, FramePoint2DBasics firstIntersectionToPack,
                                FramePoint2DBasics secondIntersectionToPack)
   {
      checkReferenceFrameMatch(convexPolygon);
      firstIntersectionToPack.setReferenceFrame(getReferenceFrame());
      secondIntersectionToPack.setReferenceFrame(getReferenceFrame());
      return Line2DReadOnly.super.intersectionWith(convexPolygon, firstIntersectionToPack, secondIntersectionToPack);
   }

   /**
    * Computes the coordinates of the possible intersection(s) between this line and the given
    * convex polygon 2D.
    *
    * @param convexPolygon the convex polygon this line may intersect. Not modified.
    * @param firstIntersectionToPack point in which the coordinates of the first intersection. Can
    *           be {@code null}. Modified.
    * @param secondIntersectionToPack point in which the coordinates of the second intersection. Can
    *           be {@code null}. Modified.
    * @return the number of intersections between this line and the polygon.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code secondIntersectionToPack}
    *            are not expressed in the same reference frame.
    * @see #intersectionWith(ConvexPolygon2DReadOnly, Point2DBasics, Point2DBasics)
    */
   default int intersectionWith(ConvexPolygon2DReadOnly convexPolygon, Point2DBasics firstIntersectionToPack, FixedFramePoint2DBasics secondIntersectionToPack)
   {
      checkReferenceFrameMatch(secondIntersectionToPack);
      return Line2DReadOnly.super.intersectionWith(convexPolygon, firstIntersectionToPack, secondIntersectionToPack);
   }

   /**
    * Computes the coordinates of the possible intersection(s) between this line and the given
    * convex polygon 2D.
    *
    * @param convexPolygon the convex polygon this line may intersect. Not modified.
    * @param firstIntersectionToPack point in which the coordinates of the first intersection. Can
    *           be {@code null}. Modified.
    * @param secondIntersectionToPack point in which the coordinates of the second intersection. Can
    *           be {@code null}. Modified.
    * @return the number of intersections between this line and the polygon.
    * @throws ReferenceFrameMismatchException if {@code this}, {@code convexPolygon}, and
    *            {@code secondIntersectionToPack} are not expressed in the same reference frame.
    * @see #intersectionWith(ConvexPolygon2DReadOnly, Point2DBasics, Point2DBasics)
    */
   default int intersectionWith(FrameConvexPolygon2DReadOnly convexPolygon, Point2DBasics firstIntersectionToPack,
                                FixedFramePoint2DBasics secondIntersectionToPack)
   {
      checkReferenceFrameMatch(convexPolygon);
      checkReferenceFrameMatch(secondIntersectionToPack);
      return Line2DReadOnly.super.intersectionWith(convexPolygon, firstIntersectionToPack, secondIntersectionToPack);
   }

   /**
    * Computes the coordinates of the possible intersection(s) between this line and the given
    * convex polygon 2D.
    *
    * @param convexPolygon the convex polygon this line may intersect. Not modified.
    * @param firstIntersectionToPack point in which the coordinates of the first intersection. Can
    *           be {@code null}. Modified.
    * @param secondIntersectionToPack point in which the coordinates of the second intersection. Can
    *           be {@code null}. Modified.
    * @return the number of intersections between this line and the polygon.
    * @see #intersectionWith(ConvexPolygon2DReadOnly, Point2DBasics, Point2DBasics)
    */
   default int intersectionWith(ConvexPolygon2DReadOnly convexPolygon, Point2DBasics firstIntersectionToPack, FramePoint2DBasics secondIntersectionToPack)
   {
      secondIntersectionToPack.setReferenceFrame(getReferenceFrame());
      return Line2DReadOnly.super.intersectionWith(convexPolygon, firstIntersectionToPack, secondIntersectionToPack);
   }

   /**
    * Computes the coordinates of the possible intersection(s) between this line and the given
    * convex polygon 2D.
    *
    * @param convexPolygon the convex polygon this line may intersect. Not modified.
    * @param firstIntersectionToPack point in which the coordinates of the first intersection. Can
    *           be {@code null}. Modified.
    * @param secondIntersectionToPack point in which the coordinates of the second intersection. Can
    *           be {@code null}. Modified.
    * @return the number of intersections between this line and the polygon.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code other} are not expressed in
    *            the same reference frame.
    * @see #intersectionWith(ConvexPolygon2DReadOnly, Point2DBasics, Point2DBasics)
    */
   default int intersectionWith(FrameConvexPolygon2DReadOnly convexPolygon, Point2DBasics firstIntersectionToPack, FramePoint2DBasics secondIntersectionToPack)
   {
      checkReferenceFrameMatch(convexPolygon);
      secondIntersectionToPack.setReferenceFrame(getReferenceFrame());
      return Line2DReadOnly.super.intersectionWith(convexPolygon, firstIntersectionToPack, secondIntersectionToPack);
   }

   /**
    * Computes the coordinates of the possible intersection(s) between this line and the given
    * convex polygon 2D.
    *
    * @param convexPolygon the convex polygon this line may intersect. Not modified.
    * @param firstIntersectionToPack point in which the coordinates of the first intersection. Can
    *           be {@code null}. Modified.
    * @param secondIntersectionToPack point in which the coordinates of the second intersection. Can
    *           be {@code null}. Modified.
    * @return the number of intersections between this line and the polygon.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code other} are not expressed in
    *            the same reference frame.
    * @see #intersectionWith(ConvexPolygon2DReadOnly, Point2DBasics, Point2DBasics)
    */
   default int intersectionWith(FrameConvexPolygon2DReadOnly convexPolygon, Point2DBasics firstIntersectionToPack, Point2DBasics secondIntersectionToPack)
   {
      checkReferenceFrameMatch(convexPolygon);
      return Line2DReadOnly.super.intersectionWith(convexPolygon, firstIntersectionToPack, secondIntersectionToPack);
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
   default boolean orthogonalProjection(FixedFramePoint2DBasics pointToProject)
   {
      checkReferenceFrameMatch(pointToProject);
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
    * @return whether the method succeeded or not.
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
    * @return whether the method succeeded or not.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code projectionToPack} are not
    *            expressed in the same reference frame.
    */
   default boolean orthogonalProjection(Point2DReadOnly pointToProject, FixedFramePoint2DBasics projectionToPack)
   {
      checkReferenceFrameMatch(projectionToPack);
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
    * @return whether the method succeeded or not.
    */
   default boolean orthogonalProjection(Point2DReadOnly pointToProject, FramePoint2DBasics projectionToPack)
   {
      projectionToPack.setReferenceFrame(getReferenceFrame());
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
    * @return whether the method succeeded or not.
    * @throws ReferenceFrameMismatchException if {@code this}, {@code pointToProject}, and
    *            {@code projectionToPack} are not expressed in the same reference frame.
    */
   default boolean orthogonalProjection(FramePoint2DReadOnly pointToProject, FixedFramePoint2DBasics projectionToPack)
   {
      checkReferenceFrameMatch(pointToProject);
      checkReferenceFrameMatch(projectionToPack);
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
    * @return whether the method succeeded or not.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code pointToProject} are not
    *            expressed in the same reference frame.
    */
   default boolean orthogonalProjection(FramePoint2DReadOnly pointToProject, FramePoint2DBasics projectionToPack)
   {
      checkReferenceFrameMatch(pointToProject);
      projectionToPack.setReferenceFrame(getReferenceFrame());
      return Line2DReadOnly.super.orthogonalProjection(pointToProject, projectionToPack);
   }

   /** {@inheritDoc} */
   @Override
   default FramePoint2DBasics orthogonalProjectionCopy(Point2DReadOnly pointToProject)
   {
      Point2DBasics projection = Line2DReadOnly.super.orthogonalProjectionCopy(pointToProject);
      if (projection == null)
         return null;
      else
         return new FramePoint2D(getReferenceFrame(), projection);
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
   default FramePoint2DBasics orthogonalProjectionCopy(FramePoint2DReadOnly pointToProject)
   {
      checkReferenceFrameMatch(pointToProject);
      return new FramePoint2D(getReferenceFrame(), Line2DReadOnly.super.orthogonalProjectionCopy(pointToProject));
   }

   /** {@inheritDoc} */
   @Override
   default FrameLine2DBasics interiorBisector(Line2DReadOnly secondLine)
   {
      return new FrameLine2D(getReferenceFrame(), Line2DReadOnly.super.interiorBisector(secondLine));
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
   default FrameLine2DBasics interiorBisector(FrameLine2DReadOnly secondLine)
   {
      checkReferenceFrameMatch(secondLine);
      return new FrameLine2D(getReferenceFrame(), Line2DReadOnly.super.interiorBisector(secondLine));
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

   /** {@inheritDoc} */
   @Override
   default FrameLine2DBasics perpendicularLineThroughPoint(Point2DReadOnly point)
   {
      return new FrameLine2D(getReferenceFrame(), Line2DReadOnly.super.perpendicularLineThroughPoint(point));
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
   default FrameLine2DBasics perpendicularLineThroughPoint(FramePoint2DReadOnly point)
   {
      checkReferenceFrameMatch(point);
      return new FrameLine2D(getReferenceFrame(), Line2DReadOnly.super.perpendicularLineThroughPoint(point));
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
    * @throws ReferenceFrameMismatchException if {@code this}, {@code point}, and
    *            {@code perpendicularLineToPack} are not expressed in the same reference frame.
    */
   default void perpendicularLineThroughPoint(FramePoint2DReadOnly point, FixedFrameLine2DBasics perpendicularLineToPack)
   {
      checkReferenceFrameMatch(point);
      checkReferenceFrameMatch(perpendicularLineToPack);
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
   default void perpendicularLineThroughPoint(FramePoint2DReadOnly point, FrameLine2DBasics perpendicularLineToPack)
   {
      checkReferenceFrameMatch(point);
      perpendicularLineToPack.setReferenceFrame(getReferenceFrame());
      Line2DReadOnly.super.perpendicularLineThroughPoint(point, perpendicularLineToPack);
   }

   /**
    * Modifies {@code perpendicularLineToPack} such that it is perpendicular to this line, with its
    * direction pointing to the left of this line, while going through the given point.
    *
    * @param point the point the line has to go through. Not modified.
    * @param perpendicularLineToPack the line perpendicular to {@code this} and going through
    *           {@code point}. Modified.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code perpendicularLineToPack}
    *            are not expressed in the same reference frame.
    */
   default void perpendicularLineThroughPoint(Point2DReadOnly point, FixedFrameLine2DBasics perpendicularLineToPack)
   {
      checkReferenceFrameMatch(perpendicularLineToPack);
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
   default void perpendicularLineThroughPoint(Point2DReadOnly point, FrameLine2DBasics perpendicularLineToPack)
   {
      perpendicularLineToPack.setReferenceFrame(getReferenceFrame());
      Line2DReadOnly.super.perpendicularLineThroughPoint(point, perpendicularLineToPack);
   }

   /**
    * Gets the coordinates of two distinct points this line goes through.
    *
    * @param firstPointOnLineToPack the coordinates of a first point located on this line. Modified.
    * @param secondPointOnLineToPack the coordinates of a second point located on this line.
    *           Modified.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code firstPointOnLineToPack} are
    *            not expressed in the same reference frame.
    */
   default void getTwoPointsOnLine(FixedFramePoint2DBasics firstPointOnLineToPack, Point2DBasics secondPointOnLineToPack)
   {
      checkReferenceFrameMatch(firstPointOnLineToPack);
      Line2DReadOnly.super.getTwoPointsOnLine(firstPointOnLineToPack, secondPointOnLineToPack);
   }

   /**
    * Gets the coordinates of two distinct points this line goes through.
    *
    * @param firstPointOnLineToPack the coordinates of a first point located on this line. Modified.
    * @param secondPointOnLineToPack the coordinates of a second point located on this line.
    *           Modified.
    */
   default void getTwoPointsOnLine(FramePoint2DBasics firstPointOnLineToPack, Point2DBasics secondPointOnLineToPack)
   {
      firstPointOnLineToPack.setReferenceFrame(getReferenceFrame());
      Line2DReadOnly.super.getTwoPointsOnLine(firstPointOnLineToPack, secondPointOnLineToPack);
   }

   /**
    * Gets the coordinates of two distinct points this line goes through.
    *
    * @param firstPointOnLineToPack the coordinates of a first point located on this line. Modified.
    * @param secondPointOnLineToPack the coordinates of a second point located on this line.
    *           Modified.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code secondPointOnLineToPack}
    *            are not expressed in the same reference frame.
    */
   default void getTwoPointsOnLine(Point2DBasics firstPointOnLineToPack, FixedFramePoint2DBasics secondPointOnLineToPack)
   {
      checkReferenceFrameMatch(secondPointOnLineToPack);
      Line2DReadOnly.super.getTwoPointsOnLine(firstPointOnLineToPack, secondPointOnLineToPack);
   }

   /**
    * Gets the coordinates of two distinct points this line goes through.
    *
    * @param firstPointOnLineToPack the coordinates of a first point located on this line. Modified.
    * @param secondPointOnLineToPack the coordinates of a second point located on this line.
    *           Modified.
    */
   default void getTwoPointsOnLine(Point2DBasics firstPointOnLineToPack, FramePoint2DBasics secondPointOnLineToPack)
   {
      secondPointOnLineToPack.setReferenceFrame(getReferenceFrame());
      Line2DReadOnly.super.getTwoPointsOnLine(firstPointOnLineToPack, secondPointOnLineToPack);
   }

   /**
    * Gets the coordinates of two distinct points this line goes through.
    *
    * @param firstPointOnLineToPack the coordinates of a first point located on this line. Modified.
    * @param secondPointOnLineToPack the coordinates of a second point located on this line.
    *           Modified.
    * @throws ReferenceFrameMismatchException if either of the given arguments is not expressed in
    *            the same reference frame as this frame line.
    */
   default void getTwoPointsOnLine(FixedFramePoint2DBasics firstPointOnLineToPack, FixedFramePoint2DBasics secondPointOnLineToPack)
   {
      checkReferenceFrameMatch(firstPointOnLineToPack);
      checkReferenceFrameMatch(secondPointOnLineToPack);
      Line2DReadOnly.super.getTwoPointsOnLine(firstPointOnLineToPack, secondPointOnLineToPack);
   }

   /**
    * Gets the coordinates of two distinct points this line goes through.
    *
    * @param firstPointOnLineToPack the coordinates of a first point located on this line. Modified.
    * @param secondPointOnLineToPack the coordinates of a second point located on this line.
    *           Modified.
    */
   default void getTwoPointsOnLine(FramePoint2DBasics firstPointOnLineToPack, FramePoint2DBasics secondPointOnLineToPack)
   {
      firstPointOnLineToPack.setReferenceFrame(getReferenceFrame());
      secondPointOnLineToPack.setReferenceFrame(getReferenceFrame());
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

   /** {@inheritDoc} */
   @Override
   default FrameVector2DBasics perpendicularVector()
   {
      return new FrameVector2D(getReferenceFrame(), Line2DReadOnly.super.perpendicularVector());
   }

   /**
    * Packs into {@code vectorToPack} the vector that is perpendicular to this line and pointing to
    * the left.
    *
    * @param vectorToPack the perpendicular vector to this line. Modified.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code vectorToPack} are not
    *            expressed in the same reference frame.
    */
   default void perpendicularVector(FixedFrameVector2DBasics vectorToPack)
   {
      checkReferenceFrameMatch(vectorToPack);
      Line2DReadOnly.super.perpendicularVector(vectorToPack);
   }

   /**
    * Packs into {@code vectorToPack} the vector that is perpendicular to this line and pointing to
    * the left.
    *
    * @param vectorToPack the perpendicular vector to this line. Modified.
    */
   default void perpendicularVector(FrameVector2DBasics vectorToPack)
   {
      vectorToPack.setReferenceFrame(getReferenceFrame());
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
    * @throws ReferenceFrameMismatchException if {@code this} and {@code pointToPack} are not
    *            expressed in the same reference frame.
    */
   default void pointOnLineGivenParameter(double t, FixedFramePoint2DBasics pointToPack)
   {
      checkReferenceFrameMatch(pointToPack);
      Line2DReadOnly.super.pointOnLineGivenParameter(t, pointToPack);
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
   default void pointOnLineGivenParameter(double t, FramePoint2DBasics pointToPack)
   {
      pointToPack.setReferenceFrame(getReferenceFrame());
      Line2DReadOnly.super.pointOnLineGivenParameter(t, pointToPack);
   }

   /** {@inheritDoc} */
   @Override
   default FramePoint2DBasics pointOnLineGivenParameter(double t)
   {
      return new FramePoint2D(getReferenceFrame(), Line2DReadOnly.super.pointOnLineGivenParameter(t));
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
    * @throws ReferenceFrameMismatchException if {@code this} and {@code interiorBisectorToPack} are
    *            not expressed in the same reference frame.
    */
   default boolean interiorBisector(Line2DReadOnly secondLine, FixedFrameLine2DBasics interiorBisectorToPack)
   {
      checkReferenceFrameMatch(interiorBisectorToPack);
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
   default boolean interiorBisector(Line2DReadOnly secondLine, FrameLine2DBasics interiorBisectorToPack)
   {
      interiorBisectorToPack.setReferenceFrame(getReferenceFrame());
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
    * @throws ReferenceFrameMismatchException if {@code this}, {@code secondLine}, and
    *            {@code interiorBisectorToPack} are not expressed in the same reference frame.
    */
   default boolean interiorBisector(FrameLine2DReadOnly secondLine, FixedFrameLine2DBasics interiorBisectorToPack)
   {
      checkReferenceFrameMatch(secondLine);
      checkReferenceFrameMatch(interiorBisectorToPack);
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
   default boolean interiorBisector(FrameLine2DReadOnly secondLine, FrameLine2DBasics interiorBisectorToPack)
   {
      checkReferenceFrameMatch(secondLine);
      interiorBisectorToPack.setReferenceFrame(getReferenceFrame());
      return Line2DReadOnly.super.interiorBisector(secondLine, interiorBisectorToPack);
   }

   /**
    * Tests on a per-component basis on the point and vector if this line is equal to {@code other}
    * with the tolerance {@code epsilon}. This method will return {@code false} if the two lines are
    * physically the same but either the point or vector of each line is different. For instance, if
    * {@code this.point == other.point} and {@code this.direction == - other.direction}, the two
    * lines are physically the same but this method returns {@code false}.
    * <p>
    * If the two lines have different frames, this method returns {@code false}.
    * </p>
    *
    * @param other the query. Not modified.
    * @param epsilon the tolerance to use.
    * @return {@code true} if the two lines are equal and are expressed in the same reference frame,
    *         {@code false} otherwise.
    */
   default boolean epsilonEquals(FrameLine2DReadOnly other, double epsilon)
   {
      if (getReferenceFrame() != other.getReferenceFrame())
         return false;
      if (!getPoint().epsilonEquals(other.getPoint(), epsilon))
         return false;
      if (!getDirection().epsilonEquals(other.getDirection(), epsilon))
         return false;

      return true;
   }

   /**
    * Compares {@code this} to {@code other} to determine if the two lines are geometrically
    * similar.
    * <p>
    * Two lines are considered geometrically equal is they are collinear, pointing toward the same
    * or opposite direction.
    * </p>
    *
    * @param other the line to compare to. Not modified.
    * @param epsilon the tolerance of the comparison.
    * @return {@code true} if the two lines represent the same geometry, {@code false} otherwise.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code other} are not expressed in
    *            the same reference frame.
    */
   default boolean geometricallyEquals(FrameLine2DReadOnly other, double epsilon)
   {
      checkReferenceFrameMatch(other);
      return Line2DReadOnly.super.geometricallyEquals(other, epsilon);
   }

   /**
    * Tests on a per component basis, if this line 2D is exactly equal to {@code other}.
    * <p>
    * If the two lines have different frames, this method returns {@code false}.
    * </p>
    *
    * @param other the other line 2D to compare against this. Not modified.
    * @return {@code true} if the two lines are exactly equal component-wise and are expressed in
    *         the same reference frame, {@code false} otherwise.
    */
   default boolean equals(FrameLine2DReadOnly other)
   {
      if (other == null || getReferenceFrame() != other.getReferenceFrame())
         return false;
      else
         return getPoint().equals(other.getPoint()) && getDirection().equals(other.getDirection());
   }
}
