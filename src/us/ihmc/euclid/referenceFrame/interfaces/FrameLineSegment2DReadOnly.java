package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.geometry.interfaces.Line2DReadOnly;
import us.ihmc.euclid.geometry.interfaces.LineSegment2DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;

/**
 * Read-only interface for a line segment 2D expressed in a given reference frame, i.e. this line
 * segment is always expressed in the same reference frame.
 * <p>
 * A line segment 2D is a finite-length line defined in the XY-plane by its two 2D endpoints.
 * </p>
 * <p>
 * In addition to representing a {@link LineSegment2DReadOnly}, a {@link ReferenceFrame} is
 * associated to a {@code FrameLineSegment2DReadOnly}. This allows, for instance, to enforce, at
 * runtime, that operations on lines occur in the same coordinate system.
 * </p>
 * <p>
 * Because a {@code FrameLineSegment2DReadOnly} extends {@code LineSegment2DReadOnly}, it is
 * compatible with methods only requiring {@code LineSegment2DReadOnly}. However, these methods do
 * NOT assert that the operation occur in the proper coordinate system. Use this feature carefully
 * and always prefer using methods requiring {@code FrameLineSegment2DReadOnly}.
 * </p>
 */
public interface FrameLineSegment2DReadOnly extends LineSegment2DReadOnly, ReferenceFrameHolder
{
   /**
    * Gets the read-only reference to the first endpoint of this line segment.
    *
    * @return the reference to the first endpoint of this line segment.
    */
   @Override
   FramePoint2DReadOnly getFirstEndpoint();

   /**
    * Gets the read-only reference to the second endpoint of this line segment.
    *
    * @return the reference to the second endpoint of this line segment.
    */
   @Override
   FramePoint2DReadOnly getSecondEndpoint();

   /**
    * Gets the endpoints defining this line segment by storing their coordinates in the given
    * arguments.
    *
    * @param firstEndpointToPack point in which the coordinates of this line segment's first
    *           endpoint are stored. Modified.
    * @param secondEndpointToPack point in which the coordinates of this line segment's second
    *           endpoint are stored. Modified.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code firstEndpointToPack} are
    *            not expressed in the same reference frame.
    */
   default void get(FixedFramePoint2DBasics firstEndpointToPack, Point2DBasics secondEndpointToPack)
   {
      checkReferenceFrameMatch(firstEndpointToPack);
      LineSegment2DReadOnly.super.get(firstEndpointToPack, secondEndpointToPack);
   }

   /**
    * Gets the endpoints defining this line segment by storing their coordinates in the given
    * arguments.
    *
    * @param firstEndpointToPack point in which the coordinates of this line segment's first
    *           endpoint are stored. Modified.
    * @param secondEndpointToPack point in which the coordinates of this line segment's second
    *           endpoint are stored. Modified.
    */
   default void get(FramePoint2DBasics firstEndpointToPack, Point2DBasics secondEndpointToPack)
   {
      firstEndpointToPack.setReferenceFrame(getReferenceFrame());
      LineSegment2DReadOnly.super.get(firstEndpointToPack, secondEndpointToPack);
   }

   /**
    * Gets the endpoints defining this line segment by storing their coordinates in the given
    * arguments.
    *
    * @param firstEndpointToPack point in which the coordinates of this line segment's first
    *           endpoint are stored. Modified.
    * @param secondEndpointToPack point in which the coordinates of this line segment's second
    *           endpoint are stored. Modified.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code secondEndpointToPack} are
    *            not expressed in the same reference frame.
    */
   default void get(Point2DBasics firstEndpointToPack, FixedFramePoint2DBasics secondEndpointToPack)
   {
      checkReferenceFrameMatch(secondEndpointToPack);
      LineSegment2DReadOnly.super.get(firstEndpointToPack, secondEndpointToPack);
   }

   /**
    * Gets the endpoints defining this line segment by storing their coordinates in the given
    * arguments.
    *
    * @param firstEndpointToPack point in which the coordinates of this line segment's first
    *           endpoint are stored. Modified.
    * @param secondEndpointToPack point in which the coordinates of this line segment's second
    *           endpoint are stored. Modified.
    */
   default void get(Point2DBasics firstEndpointToPack, FramePoint2DBasics secondEndpointToPack)
   {
      secondEndpointToPack.setReferenceFrame(getReferenceFrame());
      LineSegment2DReadOnly.super.get(firstEndpointToPack, secondEndpointToPack);
   }

   /**
    * Gets the endpoints defining this line segment by storing their coordinates in the given
    * arguments.
    *
    * @param firstEndpointToPack point in which the coordinates of this line segment's first
    *           endpoint are stored. Modified.
    * @param secondEndpointToPack point in which the coordinates of this line segment's second
    *           endpoint are stored. Modified.
    * @throws ReferenceFrameMismatchException if {@code this}, {@code firstEndpointToPack} and
    *            {@code secondEndpointToPack} are not expressed in the same reference frame.
    */
   default void get(FixedFramePoint2DBasics firstEndpointToPack, FixedFramePoint2DBasics secondEndpointToPack)
   {
      checkReferenceFrameMatch(firstEndpointToPack);
      checkReferenceFrameMatch(secondEndpointToPack);
      LineSegment2DReadOnly.super.get(firstEndpointToPack, secondEndpointToPack);
   }

   /**
    * Gets the endpoints defining this line segment by storing their coordinates in the given
    * arguments.
    *
    * @param firstEndpointToPack point in which the coordinates of this line segment's first
    *           endpoint are stored. Modified.
    * @param secondEndpointToPack point in which the coordinates of this line segment's second
    *           endpoint are stored. Modified.
    */
   default void get(FramePoint2DBasics firstEndpointToPack, FramePoint2DBasics secondEndpointToPack)
   {
      firstEndpointToPack.setReferenceFrame(getReferenceFrame());
      secondEndpointToPack.setReferenceFrame(getReferenceFrame());
      LineSegment2DReadOnly.super.get(firstEndpointToPack, secondEndpointToPack);
   }

   /** {@inheritDoc} */
   @Override
   default FramePoint2DBasics midpoint()
   {
      return new FramePoint2D(getReferenceFrame(), LineSegment2DReadOnly.super.midpoint());
   }

   /**
    * Computes the coordinates of the point located exactly at the middle of this line segment.
    *
    * @param midpointToPack point in which the mid-point of this line segment is stored. Modified.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code midpointToPack} are not
    *            expressed in the same reference frame.
    */
   default void midpoint(FixedFramePoint2DBasics midpointToPack)
   {
      checkReferenceFrameMatch(midpointToPack);
      LineSegment2DReadOnly.super.midpoint(midpointToPack);
   }

   /**
    * Computes the coordinates of the point located exactly at the middle of this line segment.
    *
    * @param midpointToPack point in which the mid-point of this line segment is stored. Modified.
    */
   default void midpoint(FramePoint2DBasics midpointToPack)
   {
      midpointToPack.setReferenceFrame(getReferenceFrame());
      LineSegment2DReadOnly.super.midpoint(midpointToPack);
   }

   /**
    * Tests if the given is located on this line segment.
    * <p>
    * More precisely, the point is assumed to be on this line segment if it is located at a distance
    * less than {@code 1.0e-8} from it.
    * </p>
    *
    * @param point the coordinates of the query. Not modified.
    * @return {@code true} if the point is located on this line segment, {@code false} otherwise.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code point} are not expressed in
    *            the same reference frame.
    */
   default boolean isPointOnLineSegment(FramePoint2DReadOnly point)
   {
      checkReferenceFrameMatch(point);
      return LineSegment2DReadOnly.super.isPointOnLineSegment(point);
   }

   /**
    * Tests if the given is located on this line segment.
    * <p>
    * More precisely, the point is assumed to be on this line segment if it is located at a distance
    * less than {@code epsilon} from it.
    * </p>
    *
    * @param point the coordinates of the query. Not modified.
    * @param epsilon the tolerance used for this test.
    * @return {@code true} if the point is located on this line segment, {@code false} otherwise.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code point} are not expressed in
    *            the same reference frame.
    */
   default boolean isPointOnLineSegment(FramePoint2DReadOnly point, double epsilon)
   {
      checkReferenceFrameMatch(point);
      return LineSegment2DReadOnly.super.isPointOnLineSegment(point, epsilon);
   }

   /**
    * Tests whether the projection of the given point onto this line segment is located between the
    * two endpoints with a given conservative tolerance {@code epsilon}:
    * <ul>
    * <li>if {@code epsilon > 0}, the point has to be between the endpoints and at a minimum
    * distance of {@code epsilon * this.length()} from the closest endpoint.
    * <li>if {@code epsilon < 0}, the point has to be between the endpoints or at a maximum distance
    * of {@code -epsilon * this.length()} from the closest endpoint.
    * <li>if {@code epsilon = 0}, the point has to be between the endpoints or equal to one of the
    * endpoints.
    * </ul>
    *
    * @param point the query. Not modified.
    * @param epsilon the tolerance to use.
    * @return {@code true} if the projection of the point is between the endpoints of this line
    *         segment, {@code false} otherwise.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code point} are not expressed in
    *            the same reference frame.
    */
   default boolean isBetweenEndpoints(FramePoint2DReadOnly point, double epsilon)
   {
      checkReferenceFrameMatch(point);
      return LineSegment2DReadOnly.super.isBetweenEndpoints(point, epsilon);
   }

   /**
    * Tests whether the projection of the given point onto this line segment is located between the
    * two endpoints or exactly on an endpoint.
    *
    * @param point the query. Not modified.
    * @return {@code true} if the projection of the point is between the endpoints of this line
    *         segment, {@code false} otherwise.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code point} are not expressed in
    *            the same reference frame.
    */
   default boolean isBetweenEndpoints(FramePoint2DReadOnly point)
   {
      return isBetweenEndpoints(point, 0);
   }

   /**
    * Returns a boolean value, stating whether a 2D point is on the left or right side of this line
    * segment. The idea of "side" is determined based on the direction of the line segment.
    * <p>
    * For instance, given the {@code this.firstEndpoint = (0, 0)} and
    * {@code this.secondEndpoint = (0, 1)}:
    * <ul>
    * <li>the left side of this line segment has a negative y coordinate.
    * <li>the right side of this line segment has a positive y coordinate.
    * </ul>
    * </p>
    * This method will return {@code false} if the point is on this line.
    *
    * @param point the coordinates of the query point.
    * @return {@code true} if the point is on the left side of this line segment, {@code false} if
    *         the point is on the right side or exactly on this line segment.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code point} are not expressed in
    *            the same reference frame.
    */
   default boolean isPointOnLeftSideOfLineSegment(FramePoint2DReadOnly point)
   {
      checkReferenceFrameMatch(point);
      return LineSegment2DReadOnly.super.isPointOnLeftSideOfLineSegment(point);
   }

   /** {@inheritDoc} */
   @Override
   default FrameVector2DBasics direction(boolean normalize)
   {
      return new FrameVector2D(getReferenceFrame(), LineSegment2DReadOnly.super.direction(normalize));
   }

   /**
    * Computes the vector going from the first to the second endpoint of this line segment.
    *
    * @param normalize whether the direction vector is to be normalized.
    * @param directionToPack vector in which the direction is stored. Modified.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code secondEndpointToPack} are
    *            not expressed in the same reference frame.
    */
   default void direction(boolean normalize, FixedFrameVector2DBasics directionToPack)
   {
      checkReferenceFrameMatch(directionToPack);
      LineSegment2DReadOnly.super.direction(normalize, directionToPack);
   }

   /**
    * Computes the vector going from the first to the second endpoint of this line segment.
    *
    * @param normalize whether the direction vector is to be normalized.
    * @param directionToPack vector in which the direction is stored. Modified.
    */
   default void direction(boolean normalize, FrameVector2DBasics directionToPack)
   {
      directionToPack.setReferenceFrame(getReferenceFrame());
      LineSegment2DReadOnly.super.direction(normalize, directionToPack);
   }

   /**
    * Returns the minimum distance between a point and this given line segment.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if {@code this.lengthSquared() < }{@link EuclidGeometryTools#ONE_TRILLIONTH}, this method
    * returns the distance between {@code firstEndpoint} and the given {@code point}.
    * </ul>
    * </p>
    *
    * @param point 3D point to compute the distance from this line segment. Not modified.
    * @return the minimum distance between the 2D point and this 2D line segment.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code point} are not expressed in
    *            the same reference frame.
    */
   default double distance(FramePoint2DReadOnly point)
   {
      checkReferenceFrameMatch(point);
      return LineSegment2DReadOnly.super.distance(point);
   }

   /**
    * Returns the square of the minimum distance between a point and this given line segment.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if {@code this.lengthSquared() < }{@link EuclidGeometryTools#ONE_TRILLIONTH}, this method
    * returns the distance between {@code firstEndpoint} and the given {@code point}.
    * </ul>
    * </p>
    *
    * @param point 3D point to compute the distance from this line segment. Not modified.
    * @return the minimum distance between the 2D point and this 2D line segment.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code point} are not expressed in
    *            the same reference frame.
    */
   default double distanceSquared(FramePoint2DReadOnly point)
   {
      checkReferenceFrameMatch(point);
      return LineSegment2DReadOnly.super.distanceSquared(point);
   }

   /**
    * Returns a boolean value, stating whether a 2D point is on the left or right side of this line
    * segment. The idea of "side" is determined based on the direction of the line segment.
    * <p>
    * For instance, given the {@code this.firstEndpoint = (0, 0)} and
    * {@code this.secondEndpoint = (0, 1)}:
    * <ul>
    * <li>the left side of this line segment has a negative y coordinate.
    * <li>the right side of this line segment has a positive y coordinate.
    * </ul>
    * </p>
    * This method will return {@code false} if the point is on this line.
    *
    * @param point the coordinates of the query point.
    * @return {@code true} if the point is on the right side of this line segment, {@code false} if
    *         the point is on the left side or exactly on this line segment.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code point} are not expressed in
    *            the same reference frame.
    */
   default boolean isPointOnRightSideOfLineSegment(FramePoint2DReadOnly point)
   {
      checkReferenceFrameMatch(point);
      return LineSegment2DReadOnly.super.isPointOnRightSideOfLineSegment(point);
   }

   /**
    * Computes a percentage along the line segment representing the location of the given point once
    * projected onto this line segment. The returned percentage is in ] -&infin;; &infin; [,
    * {@code 0.0} representing {@code firstEndpoint}, and {@code 1.0} representing
    * {@code secondEndpoint}.
    * <p>
    * For example, if the returned percentage is {@code 0.5}, it means that the projection of the
    * given point is located at the middle of this line segment. The coordinates of the projection
    * of the point can be computed from the {@code percentage} as follows: <code>
    * Point3D projection = new Point3D(); </br>
    * projection.interpolate(lineSegmentStart, lineSegmentEnd, percentage); </br>
    * </code>
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the length of the given line segment is too small, i.e.
    * {@code this.lengthSquared() < }{@link EuclidGeometryTools#ONE_TRILLIONTH}, this method fails
    * and returns {@code 0.0}.
    * </ul>
    * </p>
    *
    * @param point the query point. Not modified.
    * @return the computed percentage along the line segment representing where the point projection
    *         is located.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code point} are not expressed in
    *            the same reference frame.
    */
   default double percentageAlongLineSegment(FramePoint2DReadOnly point)
   {
      checkReferenceFrameMatch(point);
      return LineSegment2DReadOnly.super.percentageAlongLineSegment(point);
   }

   /**
    * Computes the dot product of this line segment with the other line segment such that:<br>
    * {@code this }&middot;
    * {@code other = Math.cos(}&alpha;{@code ) * this.length() * other.length()}<br>
    * where &alpha; is the angle from this to the other line segment.
    *
    * @param other the other line segment used to compute the dot product. Not modified.
    * @return the value of the dot product.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code other} are not expressed in
    *            the same reference frame.
    */
   default double dotProduct(FrameLineSegment2DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      return LineSegment2DReadOnly.super.dotProduct(other);
   }

   /**
    * Computes the orthogonal projection of a 2D point on this 2D line segment.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the length of this line segment is too small, i.e.
    * {@code this.lengthSquared() < }{@link EuclidGeometryTools#ONE_TRILLIONTH}, this method returns
    * {@code firstEndpoint}.
    * <li>the projection can not be outside the line segment. When the projection on the
    * corresponding line is outside the line segment, the result is the closest of the two
    * endpoints.
    * </ul>
    * </p>
    *
    * @param pointToProject the point to project on this line segment. Modified.
    * @return whether the method succeeded or not.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code pointToProject} are not
    *            expressed in the same reference frame.
    */
   default boolean orthogonalProjection(FixedFramePoint2DBasics pointToProject)
   {
      return orthogonalProjection(pointToProject, pointToProject);
   }

   /**
    * Computes the orthogonal projection of a 2D point on this 2D line segment.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the length of this line segment is too small, i.e.
    * {@code this.lengthSquared() < }{@link EuclidGeometryTools#ONE_TRILLIONTH}, this method returns
    * {@code firstEndpoint}.
    * <li>the projection can not be outside the line segment. When the projection on the
    * corresponding line is outside the line segment, the result is the closest of the two
    * endpoints.
    * </ul>
    * </p>
    *
    * @param pointToProject the point to compute the projection of. Not modified.
    * @param projectionToPack point in which the projection of the point onto this line segment is
    *           stored. Modified.
    * @return whether the method succeeded or not.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code pointToProject} are not
    *            expressed in the same reference frame.
    */
   default boolean orthogonalProjection(FramePoint2DReadOnly pointToProject, Point2DBasics projectionToPack)
   {
      checkReferenceFrameMatch(pointToProject);
      return LineSegment2DReadOnly.super.orthogonalProjection(pointToProject, projectionToPack);
   }

   /**
    * Computes the orthogonal projection of a 2D point on this 2D line segment.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the length of this line segment is too small, i.e.
    * {@code this.lengthSquared() < }{@link EuclidGeometryTools#ONE_TRILLIONTH}, this method returns
    * {@code firstEndpoint}.
    * <li>the projection can not be outside the line segment. When the projection on the
    * corresponding line is outside the line segment, the result is the closest of the two
    * endpoints.
    * </ul>
    * </p>
    *
    * @param pointToProject the point to compute the projection of. Not modified.
    * @param projectionToPack point in which the projection of the point onto this line segment is
    *           stored. Modified.
    * @return whether the method succeeded or not.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code projectionToPack} are not
    *            expressed in the same reference frame.
    */
   default boolean orthogonalProjection(Point2DReadOnly pointToProject, FixedFramePoint2DBasics projectionToPack)
   {
      checkReferenceFrameMatch(projectionToPack);
      return LineSegment2DReadOnly.super.orthogonalProjection(pointToProject, projectionToPack);
   }

   /**
    * Computes the orthogonal projection of a 2D point on this 2D line segment.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the length of this line segment is too small, i.e.
    * {@code this.lengthSquared() < }{@link EuclidGeometryTools#ONE_TRILLIONTH}, this method returns
    * {@code firstEndpoint}.
    * <li>the projection can not be outside the line segment. When the projection on the
    * corresponding line is outside the line segment, the result is the closest of the two
    * endpoints.
    * </ul>
    * </p>
    *
    * @param pointToProject the point to compute the projection of. Not modified.
    * @param projectionToPack point in which the projection of the point onto this line segment is
    *           stored. Modified.
    * @return whether the method succeeded or not.
    */
   default boolean orthogonalProjection(Point2DReadOnly pointToProject, FramePoint2DBasics projectionToPack)
   {
      projectionToPack.setReferenceFrame(getReferenceFrame());
      return LineSegment2DReadOnly.super.orthogonalProjection(pointToProject, projectionToPack);
   }

   /**
    * Computes the orthogonal projection of a 2D point on this 2D line segment.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the length of this line segment is too small, i.e.
    * {@code this.lengthSquared() < }{@link EuclidGeometryTools#ONE_TRILLIONTH}, this method returns
    * {@code firstEndpoint}.
    * <li>the projection can not be outside the line segment. When the projection on the
    * corresponding line is outside the line segment, the result is the closest of the two
    * endpoints.
    * </ul>
    * </p>
    *
    * @param pointToProject the point to compute the projection of. Not modified.
    * @param projectionToPack point in which the projection of the point onto this line segment is
    *           stored. Modified.
    * @return whether the method succeeded or not.
    * @throws ReferenceFrameMismatchException if {@code this}, {@code pointToProject}, and
    *            {@code projectionToPack} are not expressed in the same reference frame.
    */
   default boolean orthogonalProjection(FramePoint2DReadOnly pointToProject, FixedFramePoint2DBasics projectionToPack)
   {
      checkReferenceFrameMatch(pointToProject);
      checkReferenceFrameMatch(projectionToPack);
      return LineSegment2DReadOnly.super.orthogonalProjection(pointToProject, projectionToPack);
   }

   /**
    * Computes the orthogonal projection of a 2D point on this 2D line segment.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the length of this line segment is too small, i.e.
    * {@code this.lengthSquared() < }{@link EuclidGeometryTools#ONE_TRILLIONTH}, this method returns
    * {@code firstEndpoint}.
    * <li>the projection can not be outside the line segment. When the projection on the
    * corresponding line is outside the line segment, the result is the closest of the two
    * endpoints.
    * </ul>
    * </p>
    *
    * @param pointToProject the point to compute the projection of. Not modified.
    * @param projectionToPack point in which the projection of the point onto this line segment is
    *           stored. Modified.
    * @return whether the method succeeded or not.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code pointToProject} are not
    *            expressed in the same reference frame.
    */
   default boolean orthogonalProjection(FramePoint2DReadOnly pointToProject, FramePoint2DBasics projectionToPack)
   {
      checkReferenceFrameMatch(pointToProject);
      projectionToPack.setReferenceFrame(getReferenceFrame());
      return LineSegment2DReadOnly.super.orthogonalProjection(pointToProject, projectionToPack);
   }

   /** {@inheritDoc} */
   @Override
   default FramePoint2DBasics orthogonalProjectionCopy(Point2DReadOnly pointToProject)
   {
      return new FramePoint2D(getReferenceFrame(), LineSegment2DReadOnly.super.orthogonalProjectionCopy(pointToProject));
   }

   /**
    * Computes the orthogonal projection of a 2D point on this 2D line segment.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the length of this line segment is too small, i.e.
    * {@code this.lengthSquared() < }{@link EuclidGeometryTools#ONE_TRILLIONTH}, this method returns
    * {@code firstEndpoint}.
    * <li>the projection can not be outside the line segment. When the projection on the
    * corresponding line is outside the line segment, the result is the closest of the two
    * endpoints.
    * </ul>
    * </p>
    *
    * @param pointToProject the point to compute the projection of. Not modified.
    * @return the projection of the point onto this line segment or {@code null} if the method
    *         failed.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code pointToProject} are not
    *            expressed in the same reference frame.
    */
   default FramePoint2D orthogonalProjectionCopy(FramePoint2DReadOnly pointToProject)
   {
      checkReferenceFrameMatch(pointToProject);
      return new FramePoint2D(getReferenceFrame(), LineSegment2DReadOnly.super.orthogonalProjectionCopy(pointToProject));
   }

   /** {@inheritDoc} */
   @Override
   default FramePoint2DBasics intersectionWith(Line2DReadOnly line)
   {
      Point2DBasics intersection = LineSegment2DReadOnly.super.intersectionWith(line);
      if (intersection == null)
         return null;
      else
         return new FramePoint2D(getReferenceFrame(), intersection);
   }

   /**
    * Calculates the coordinates of the intersection between this line segment and the given line
    * and stores the result in {@code intersectionToPack}.
    * <p>
    * Edge cases:
    * <ul>
    * <li>When this line segment and the line are parallel but not collinear, they do not intersect.
    * <li>When this line segment and the line are collinear, they are assumed to intersect at
    * {@code lineSegmentStart}.
    * <li>When the line intersects this line segment at one of its endpoints, this method returns
    * {@code true} and the endpoint is the intersection.
    * </ul>
    * </p>
    *
    * @param line the line that may intersect this line segment. Not modified.
    * @param intersectionToPack the 2D point in which the result is stored. Can be {@code null}.
    *           Modified.
    * @return {@code true} if the line intersects this line segment, {@code false} otherwise.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code intersectionToPack} are not
    *            expressed in the same reference frame.
    */
   default boolean intersectionWith(Line2DReadOnly line, FixedFramePoint2DBasics intersectionToPack)
   {
      checkReferenceFrameMatch(intersectionToPack);
      return LineSegment2DReadOnly.super.intersectionWith(line, intersectionToPack);
   }

   /**
    * Calculates the coordinates of the intersection between this line segment and the given line
    * and stores the result in {@code intersectionToPack}.
    * <p>
    * Edge cases:
    * <ul>
    * <li>When this line segment and the line are parallel but not collinear, they do not intersect.
    * <li>When this line segment and the line are collinear, they are assumed to intersect at
    * {@code lineSegmentStart}.
    * <li>When the line intersects this line segment at one of its endpoints, this method returns
    * {@code true} and the endpoint is the intersection.
    * </ul>
    * </p>
    *
    * @param line the line that may intersect this line segment. Not modified.
    * @param intersectionToPack the 2D point in which the result is stored. Can be {@code null}.
    *           Modified.
    * @return {@code true} if the line intersects this line segment, {@code false} otherwise.
    */
   default boolean intersectionWith(Line2DReadOnly line, FramePoint2DBasics intersectionToPack)
   {
      intersectionToPack.setReferenceFrame(getReferenceFrame());
      return LineSegment2DReadOnly.super.intersectionWith(line, intersectionToPack);
   }

   /**
    * Calculates the coordinates of the intersection between this line segment and the given line
    * and stores the result in {@code intersectionToPack}.
    * <p>
    * Edge cases:
    * <ul>
    * <li>When this line segment and the line are parallel but not collinear, they do not intersect.
    * <li>When this line segment and the line are collinear, they are assumed to intersect at
    * {@code lineSegmentStart}.
    * <li>When the line intersects this line segment at one of its endpoints, this method returns
    * {@code true} and the endpoint is the intersection.
    * </ul>
    * </p>
    *
    * @param line the line that may intersect this line segment. Not modified.
    * @param intersectionToPack the 2D point in which the result is stored. Can be {@code null}.
    *           Modified.
    * @return {@code true} if the line intersects this line segment, {@code false} otherwise.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code intersectionToPack} are not
    *            expressed in the same reference frame.
    */
   default boolean intersectionWith(FrameLine2DReadOnly line, Point2DBasics intersectionToPack)
   {
      checkReferenceFrameMatch(line);
      return LineSegment2DReadOnly.super.intersectionWith(line, intersectionToPack);
   }

   /**
    * Calculates the coordinates of the intersection between this line segment and the given line
    * and stores the result in {@code intersectionToPack}.
    * <p>
    * Edge cases:
    * <ul>
    * <li>When this line segment and the line are parallel but not collinear, they do not intersect.
    * <li>When this line segment and the line are collinear, they are assumed to intersect at
    * {@code lineSegmentStart}.
    * <li>When the line intersects this line segment at one of its endpoints, this method returns
    * {@code true} and the endpoint is the intersection.
    * </ul>
    * </p>
    *
    * @param line the line that may intersect this line segment. Not modified.
    * @param intersectionToPack the 2D point in which the result is stored. Can be {@code null}.
    *           Modified.
    * @return {@code true} if the line intersects this line segment, {@code false} otherwise.
    * @throws ReferenceFrameMismatchException if {@code this}, {@code line}, and
    *            {@code intersectionToPack} are not expressed in the same reference frame.
    */
   default boolean intersectionWith(FrameLine2DReadOnly line, FixedFramePoint2DBasics intersectionToPack)
   {
      checkReferenceFrameMatch(line);
      checkReferenceFrameMatch(intersectionToPack);
      return LineSegment2DReadOnly.super.intersectionWith(line, intersectionToPack);
   }

   /**
    * Calculates the coordinates of the intersection between this line segment and the given line
    * and stores the result in {@code intersectionToPack}.
    * <p>
    * Edge cases:
    * <ul>
    * <li>When this line segment and the line are parallel but not collinear, they do not intersect.
    * <li>When this line segment and the line are collinear, they are assumed to intersect at
    * {@code lineSegmentStart}.
    * <li>When the line intersects this line segment at one of its endpoints, this method returns
    * {@code true} and the endpoint is the intersection.
    * </ul>
    * </p>
    *
    * @param line the line that may intersect this line segment. Not modified.
    * @param intersectionToPack the 2D point in which the result is stored. Can be {@code null}.
    *           Modified.
    * @return {@code true} if the line intersects this line segment, {@code false} otherwise.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code line} are not expressed in
    *            the same reference frame.
    */
   default boolean intersectionWith(FrameLine2DReadOnly line, FramePoint2DBasics intersectionToPack)
   {
      checkReferenceFrameMatch(line);
      intersectionToPack.setReferenceFrame(getReferenceFrame());
      return LineSegment2DReadOnly.super.intersectionWith(line, intersectionToPack);
   }

   /**
    * Calculates the coordinates of the intersection between this line segment and the given line
    * and returns the result.
    * <p>
    * Edge cases:
    * <ul>
    * <li>When this line segment and the line are parallel but not collinear, they do not intersect.
    * <li>When this line segment and the line are collinear, they are assumed to intersect at
    * {@code lineSegmentStart}.
    * <li>When the line intersects this line segment at one of its endpoints, this method returns
    * {@code true} and the endpoint is the intersection.
    * </ul>
    * </p>
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param line the line that may intersect this line segment. Not modified.
    * @return the coordinates of the intersection if the line intersects this line segment,
    *         {@code null} otherwise.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code line} are not expressed in
    *            the same reference frame.
    */
   default FramePoint2D intersectionWith(FrameLine2DReadOnly line)
   {
      checkReferenceFrameMatch(line);
      Point2DBasics intersection = LineSegment2DReadOnly.super.intersectionWith(line);
      if (intersection == null)
         return null;
      else
         return new FramePoint2D(getReferenceFrame(), intersection);
   }

   /**
    * Computes the vector perpendicular to the direction of this line segment.
    *
    * @param normalize whether the perpendicular vector is to be normalized.
    * @param perpendicularVectorToPack vector in which the perpendicular vector components are
    *           stored. Modified.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code perpendicularVectorToPack}
    *            are not expressed in the same reference frame.
    */
   default void perpendicular(boolean normalize, FixedFrameVector2DBasics perpendicularVectorToPack)
   {
      checkReferenceFrameMatch(perpendicularVectorToPack);
      LineSegment2DReadOnly.super.perpendicular(normalize, perpendicularVectorToPack);
   }

   /**
    * Computes the vector perpendicular to the direction of this line segment.
    *
    * @param normalize whether the perpendicular vector is to be normalized.
    * @param perpendicularVectorToPack vector in which the perpendicular vector components are
    *           stored. Modified.
    */
   default void perpendicular(boolean normalize, FrameVector2DBasics perpendicularVectorToPack)
   {
      perpendicularVectorToPack.setReferenceFrame(getReferenceFrame());
      LineSegment2DReadOnly.super.perpendicular(normalize, perpendicularVectorToPack);
   }

   /** {@inheritDoc} */
   @Override
   default FramePoint2DBasics intersectionWith(LineSegment2DReadOnly other)
   {
      Point2DBasics intersection = LineSegment2DReadOnly.super.intersectionWith(other);
      if (intersection == null)
         return null;
      else
         return new FramePoint2D(getReferenceFrame(), intersection);
   }

   /**
    * Computes the intersection between this line segment and the given line segment and returns the
    * result.
    * <p>
    * Edge cases:
    * <ul>
    * <li>When the two line segments are parallel but not collinear, the two line segments do not
    * intersect, this method returns {@code null}.
    * <li>When the two line segments are collinear, if the two line segments do not overlap do not
    * have at least one common endpoint, this method returns {@code null}.
    * <li>When the two line segments have a common endpoint, this method returns the common endpoint
    * as the intersection.
    * </ul>
    * </p>
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param other the other line segment that may intersect this line segment. Not modified.
    * @return the intersection point if it exists, {@code null} otherwise.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code other} are not expressed in
    *            the same reference frame.
    */
   default FramePoint2D intersectionWith(FrameLineSegment2DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      Point2DBasics intersection = LineSegment2DReadOnly.super.intersectionWith(other);
      if (intersection == null)
         return null;
      else
         return new FramePoint2D(getReferenceFrame(), intersection);
   }

   /**
    * Computes the intersection between this line segment and the given line segment and stores the
    * result in {@code intersectionToPack}.
    * <p>
    * Edge cases:
    * <ul>
    * <li>When the two line segments are parallel but not collinear, the two line segments do not
    * intersect.
    * <li>When the two line segments are collinear, this methods returns {@code true} only if the
    * two line segments overlap or have at least one common endpoint.
    * <li>When the two line segments have a common endpoint, this method returns true.
    * </ul>
    * </p>
    *
    * @param other the other line segment that may intersect this line segment. Not modified.
    * @param intersectionToPack the 2D point in which the result is stored. Modified.
    * @return {@code true} if the two lines intersects, {@code false} otherwise.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code other} are not expressed in
    *            the same reference frame.
    */
   default boolean intersectionWith(FrameLineSegment2DReadOnly other, Point2DBasics intersectionToPack)
   {
      checkReferenceFrameMatch(other);
      return LineSegment2DReadOnly.super.intersectionWith(other, intersectionToPack);
   }

   /**
    * Computes the intersection between this line segment and the given line segment and stores the
    * result in {@code intersectionToPack}.
    * <p>
    * Edge cases:
    * <ul>
    * <li>When the two line segments are parallel but not collinear, the two line segments do not
    * intersect.
    * <li>When the two line segments are collinear, this methods returns {@code true} only if the
    * two line segments overlap or have at least one common endpoint.
    * <li>When the two line segments have a common endpoint, this method returns true.
    * </ul>
    * </p>
    *
    * @param other the other line segment that may intersect this line segment. Not modified.
    * @param intersectionToPack the 2D point in which the result is stored. Modified.
    * @return {@code true} if the two lines intersects, {@code false} otherwise.
    * @throws ReferenceFrameMismatchException if {@code this}, {@code other}, and
    *            {@code intersectionToPack} are not expressed in the same reference frame.
    */
   default boolean intersectionWith(LineSegment2DReadOnly other, FixedFramePoint2DBasics intersectionToPack)
   {
      checkReferenceFrameMatch(intersectionToPack);
      return LineSegment2DReadOnly.super.intersectionWith(other, intersectionToPack);
   }

   /**
    * Computes the intersection between this line segment and the given line segment and stores the
    * result in {@code intersectionToPack}.
    * <p>
    * Edge cases:
    * <ul>
    * <li>When the two line segments are parallel but not collinear, the two line segments do not
    * intersect.
    * <li>When the two line segments are collinear, this methods returns {@code true} only if the
    * two line segments overlap or have at least one common endpoint.
    * <li>When the two line segments have a common endpoint, this method returns true.
    * </ul>
    * </p>
    *
    * @param other the other line segment that may intersect this line segment. Not modified.
    * @param intersectionToPack the 2D point in which the result is stored. Modified.
    * @return {@code true} if the two lines intersects, {@code false} otherwise.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code other} are not expressed in
    *            the same reference frame.
    */
   default boolean intersectionWith(LineSegment2DReadOnly other, FramePoint2DBasics intersectionToPack)
   {
      intersectionToPack.setReferenceFrame(getReferenceFrame());
      return LineSegment2DReadOnly.super.intersectionWith(other, intersectionToPack);
   }

   /**
    * Computes the intersection between this line segment and the given line segment and stores the
    * result in {@code intersectionToPack}.
    * <p>
    * Edge cases:
    * <ul>
    * <li>When the two line segments are parallel but not collinear, the two line segments do not
    * intersect.
    * <li>When the two line segments are collinear, this methods returns {@code true} only if the
    * two line segments overlap or have at least one common endpoint.
    * <li>When the two line segments have a common endpoint, this method returns true.
    * </ul>
    * </p>
    *
    * @param other the other line segment that may intersect this line segment. Not modified.
    * @param intersectionToPack the 2D point in which the result is stored. Modified.
    * @return {@code true} if the two lines intersects, {@code false} otherwise.
    * @throws ReferenceFrameMismatchException if {@code this}, {@code other}, and
    *            {@code intersectionToPack} are not expressed in the same reference frame.
    */
   default boolean intersectionWith(FrameLineSegment2DReadOnly other, FixedFramePoint2DBasics intersectionToPack)
   {
      checkReferenceFrameMatch(other);
      checkReferenceFrameMatch(intersectionToPack);
      return LineSegment2DReadOnly.super.intersectionWith(other, intersectionToPack);
   }

   /**
    * Computes the intersection between this line segment and the given line segment and stores the
    * result in {@code intersectionToPack}.
    * <p>
    * Edge cases:
    * <ul>
    * <li>When the two line segments are parallel but not collinear, the two line segments do not
    * intersect.
    * <li>When the two line segments are collinear, this methods returns {@code true} only if the
    * two line segments overlap or have at least one common endpoint.
    * <li>When the two line segments have a common endpoint, this method returns true.
    * </ul>
    * </p>
    *
    * @param other the other line segment that may intersect this line segment. Not modified.
    * @param intersectionToPack the 2D point in which the result is stored. Modified.
    * @return {@code true} if the two lines intersects, {@code false} otherwise.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code other} are not expressed in
    *            the same reference frame.
    */
   default boolean intersectionWith(FrameLineSegment2DReadOnly other, FramePoint2DBasics intersectionToPack)
   {
      checkReferenceFrameMatch(other);
      intersectionToPack.setReferenceFrame(getReferenceFrame());
      return LineSegment2DReadOnly.super.intersectionWith(other, intersectionToPack);
   }

   /** {@inheritDoc} */
   @Override
   default FramePoint2DBasics pointBetweenEndpointsGivenPercentage(double percentage)
   {
      return new FramePoint2D(getReferenceFrame(), LineSegment2DReadOnly.super.pointBetweenEndpointsGivenPercentage(percentage));
   }

   /**
    * Computes the coordinates of the point located at a given percentage on this line segment: <br>
    * {@code pointToPack.interpolate(firstEndpoint, secondEndpoint, percentage)} </br>
    *
    * @param percentage the percentage along this line segment of the point. Must be in [0, 1].
    * @param pointToPack where the result is stored. Modified.
    * @throws {@link RuntimeException} if {@code percentage} &notin; [0, 1].
    * @throws ReferenceFrameMismatchException if {@code this} and {@code pointToPack} are not
    *            expressed in the same reference frame.
    */
   default void pointBetweenEndpointsGivenPercentage(double percentage, FixedFramePoint2DBasics pointToPack)
   {
      checkReferenceFrameMatch(pointToPack);
      LineSegment2DReadOnly.super.pointBetweenEndpointsGivenPercentage(percentage, pointToPack);
   }

   /**
    * Computes the coordinates of the point located at a given percentage on this line segment: <br>
    * {@code pointToPack.interpolate(firstEndpoint, secondEndpoint, percentage)} </br>
    *
    * @param percentage the percentage along this line segment of the point. Must be in [0, 1].
    * @param pointToPack where the result is stored. Modified.
    * @throws {@link RuntimeException} if {@code percentage} &notin; [0, 1].
    */
   default void pointBetweenEndpointsGivenPercentage(double percentage, FramePoint2DBasics pointToPack)
   {
      pointToPack.setReferenceFrame(getReferenceFrame());
      LineSegment2DReadOnly.super.pointBetweenEndpointsGivenPercentage(percentage, pointToPack);
   }

   /** {@inheritDoc} */
   @Override
   default FramePoint2DBasics[] intersectionWith(ConvexPolygon2DReadOnly convexPolygon)
   {
      Point2DBasics[] intersections = LineSegment2DReadOnly.super.intersectionWith(convexPolygon);
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
    * Computes the coordinates of the possible intersection(s) between a given line segment 2D and
    * this convex polygon 2D.
    * 
    * @param convexPolygon the polygon this line segment may intersect. Not modified.
    * @return the intersections between the line segment and the polygon.
    * @throws ReferenceFrameMismatchException if {@code convexPolygon} and {@code this} are not
    *            expressed in the same reference frame.
    * @see #intersectionWith(ConvexPolygon2DReadOnly)
    */
   default FramePoint2DBasics[] intersectionWith(FrameConvexPolygon2DReadOnly convexPolygon)
   {
      checkReferenceFrameMatch(convexPolygon);
      return intersectionWith((ConvexPolygon2DReadOnly) convexPolygon);
   }

   /**
    * Computes the coordinates of the possible intersection(s) between this line segment and the
    * given convex polygon 2D.
    *
    * @param convexPolygon the convex polygon this line segment may intersect. Not modified.
    * @param firstIntersectionToPack point in which the coordinates of the first intersection. Can
    *           be {@code null}. Modified.
    * @param secondIntersectionToPack point in which the coordinates of the second intersection. Can
    *           be {@code null}. Modified.
    * @return the number of intersections between this line segment and the polygon.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code firstIntersectionToPack}
    *            are not expressed in the same reference frame.
    * @see #intersectionWith(ConvexPolygon2DReadOnly, Point2DBasics, Point2DBasics)
    */
   default int intersectionWith(ConvexPolygon2DReadOnly convexPolygon, FixedFramePoint2DBasics firstIntersectionToPack, Point2DBasics secondIntersectionToPack)
   {
      checkReferenceFrameMatch(firstIntersectionToPack);
      return LineSegment2DReadOnly.super.intersectionWith(convexPolygon, firstIntersectionToPack, secondIntersectionToPack);
   }

   /**
    * Computes the coordinates of the possible intersection(s) between this line segment and the
    * given convex polygon 2D.
    *
    * @param convexPolygon the convex polygon this line segment may intersect. Not modified.
    * @param firstIntersectionToPack point in which the coordinates of the first intersection. Can
    *           be {@code null}. Modified.
    * @param secondIntersectionToPack point in which the coordinates of the second intersection. Can
    *           be {@code null}. Modified.
    * @return the number of intersections between this line segment and the polygon.
    * @throws ReferenceFrameMismatchException if {@code this}, {@code convexPolygon}, and
    *            {@code firstIntersectionToPack} are not expressed in the same reference frame.
    * @see #intersectionWith(ConvexPolygon2DReadOnly, Point2DBasics, Point2DBasics)
    */
   default int intersectionWith(FrameConvexPolygon2DReadOnly convexPolygon, FixedFramePoint2DBasics firstIntersectionToPack,
                                Point2DBasics secondIntersectionToPack)
   {
      checkReferenceFrameMatch(convexPolygon);
      checkReferenceFrameMatch(firstIntersectionToPack);
      return LineSegment2DReadOnly.super.intersectionWith(convexPolygon, firstIntersectionToPack, secondIntersectionToPack);
   }

   /**
    * Computes the coordinates of the possible intersection(s) between this line segment and the
    * given convex polygon 2D.
    *
    * @param convexPolygon the convex polygon this line segment may intersect. Not modified.
    * @param firstIntersectionToPack point in which the coordinates of the first intersection. Can
    *           be {@code null}. Modified.
    * @param secondIntersectionToPack point in which the coordinates of the second intersection. Can
    *           be {@code null}. Modified.
    * @return the number of intersections between this line segment and the polygon.
    * @see #intersectionWith(ConvexPolygon2DReadOnly, Point2DBasics, Point2DBasics)
    */
   default int intersectionWith(ConvexPolygon2DReadOnly convexPolygon, FramePoint2DBasics firstIntersectionToPack, Point2DBasics secondIntersectionToPack)
   {
      firstIntersectionToPack.setReferenceFrame(getReferenceFrame());
      return LineSegment2DReadOnly.super.intersectionWith(convexPolygon, firstIntersectionToPack, secondIntersectionToPack);
   }

   /**
    * Computes the coordinates of the possible intersection(s) between this line segment and the
    * given convex polygon 2D.
    *
    * @param convexPolygon the convex polygon this line segment may intersect. Not modified.
    * @param firstIntersectionToPack point in which the coordinates of the first intersection. Can
    *           be {@code null}. Modified.
    * @param secondIntersectionToPack point in which the coordinates of the second intersection. Can
    *           be {@code null}. Modified.
    * @return the number of intersections between this line segment and the polygon.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code convexPolygon} are not
    *            expressed in the same reference frame.
    * @see #intersectionWith(ConvexPolygon2DReadOnly, Point2DBasics, Point2DBasics)
    */
   default int intersectionWith(FrameConvexPolygon2DReadOnly convexPolygon, FramePoint2DBasics firstIntersectionToPack, Point2DBasics secondIntersectionToPack)
   {
      checkReferenceFrameMatch(convexPolygon);
      firstIntersectionToPack.setReferenceFrame(getReferenceFrame());
      return LineSegment2DReadOnly.super.intersectionWith(convexPolygon, firstIntersectionToPack, secondIntersectionToPack);
   }

   /**
    * Computes the coordinates of the possible intersection(s) between this line segment and the
    * given convex polygon 2D.
    *
    * @param convexPolygon the convex polygon this line segment may intersect. Not modified.
    * @param firstIntersectionToPack point in which the coordinates of the first intersection. Can
    *           be {@code null}. Modified.
    * @param secondIntersectionToPack point in which the coordinates of the second intersection. Can
    *           be {@code null}. Modified.
    * @return the number of intersections between this line segment and the polygon.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code secondIntersectionToPack}
    *            are not expressed in the same reference frame.
    * @see #intersectionWith(ConvexPolygon2DReadOnly, Point2DBasics, Point2DBasics)
    */
   default int intersectionWith(ConvexPolygon2DReadOnly convexPolygon, Point2DBasics firstIntersectionToPack, FixedFramePoint2DBasics secondIntersectionToPack)
   {
      checkReferenceFrameMatch(secondIntersectionToPack);
      return LineSegment2DReadOnly.super.intersectionWith(convexPolygon, firstIntersectionToPack, secondIntersectionToPack);
   }

   /**
    * Computes the coordinates of the possible intersection(s) between this line segment and the
    * given convex polygon 2D.
    *
    * @param convexPolygon the convex polygon this line segment may intersect. Not modified.
    * @param firstIntersectionToPack point in which the coordinates of the first intersection. Can
    *           be {@code null}. Modified.
    * @param secondIntersectionToPack point in which the coordinates of the second intersection. Can
    *           be {@code null}. Modified.
    * @return the number of intersections between this line segment and the polygon.
    * @throws ReferenceFrameMismatchException if {@code this}, {@code convexPolygon}, and
    *            {@code secondIntersectionToPack} are not expressed in the same reference frame.
    * @see #intersectionWith(ConvexPolygon2DReadOnly, Point2DBasics, Point2DBasics)
    */
   default int intersectionWith(FrameConvexPolygon2DReadOnly convexPolygon, Point2DBasics firstIntersectionToPack,
                                FixedFramePoint2DBasics secondIntersectionToPack)
   {
      checkReferenceFrameMatch(convexPolygon);
      checkReferenceFrameMatch(secondIntersectionToPack);
      return LineSegment2DReadOnly.super.intersectionWith(convexPolygon, firstIntersectionToPack, secondIntersectionToPack);
   }

   /**
    * Computes the coordinates of the possible intersection(s) between this line segment and the
    * given convex polygon 2D.
    *
    * @param convexPolygon the convex polygon this line segment may intersect. Not modified.
    * @param firstIntersectionToPack point in which the coordinates of the first intersection. Can
    *           be {@code null}. Modified.
    * @param secondIntersectionToPack point in which the coordinates of the second intersection. Can
    *           be {@code null}. Modified.
    * @return the number of intersections between this line segment and the polygon.
    * @see #intersectionWith(ConvexPolygon2DReadOnly, Point2DBasics, Point2DBasics)
    */
   default int intersectionWith(ConvexPolygon2DReadOnly convexPolygon, Point2DBasics firstIntersectionToPack, FramePoint2DBasics secondIntersectionToPack)
   {
      secondIntersectionToPack.setReferenceFrame(getReferenceFrame());
      return LineSegment2DReadOnly.super.intersectionWith(convexPolygon, firstIntersectionToPack, secondIntersectionToPack);
   }

   /**
    * Computes the coordinates of the possible intersection(s) between this line segment and the
    * given convex polygon 2D.
    *
    * @param convexPolygon the convex polygon this line segment may intersect. Not modified.
    * @param firstIntersectionToPack point in which the coordinates of the first intersection. Can
    *           be {@code null}. Modified.
    * @param secondIntersectionToPack point in which the coordinates of the second intersection. Can
    *           be {@code null}. Modified.
    * @return the number of intersections between this line segment and the polygon.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code convexPolygon} are not
    *            expressed in the same reference frame.
    * @see #intersectionWith(ConvexPolygon2DReadOnly, Point2DBasics, Point2DBasics)
    */
   default int intersectionWith(FrameConvexPolygon2DReadOnly convexPolygon, Point2DBasics firstIntersectionToPack, FramePoint2DBasics secondIntersectionToPack)
   {
      checkReferenceFrameMatch(convexPolygon);
      secondIntersectionToPack.setReferenceFrame(getReferenceFrame());
      return LineSegment2DReadOnly.super.intersectionWith(convexPolygon, firstIntersectionToPack, secondIntersectionToPack);
   }

   /**
    * Computes the coordinates of the possible intersection(s) between this line segment and the
    * given convex polygon 2D.
    *
    * @param convexPolygon the convex polygon this line segment may intersect. Not modified.
    * @param firstIntersectionToPack point in which the coordinates of the first intersection. Can
    *           be {@code null}. Modified.
    * @param secondIntersectionToPack point in which the coordinates of the second intersection. Can
    *           be {@code null}. Modified.
    * @return the number of intersections between this line segment and the polygon.
    * @throws ReferenceFrameMismatchException if {@code this}, {@code firstIntersectionToPack}, and
    *            {@code secondIntersectionToPack} are not expressed in the same reference frame.
    * @see #intersectionWith(ConvexPolygon2DReadOnly, Point2DBasics, Point2DBasics)
    */
   default int intersectionWith(ConvexPolygon2DReadOnly convexPolygon, FixedFramePoint2DBasics firstIntersectionToPack,
                                FixedFramePoint2DBasics secondIntersectionToPack)
   {
      checkReferenceFrameMatch(firstIntersectionToPack);
      checkReferenceFrameMatch(secondIntersectionToPack);
      return LineSegment2DReadOnly.super.intersectionWith(convexPolygon, firstIntersectionToPack, secondIntersectionToPack);
   }

   /**
    * Computes the coordinates of the possible intersection(s) between this line segment and the
    * given convex polygon 2D.
    *
    * @param convexPolygon the convex polygon this line segment may intersect. Not modified.
    * @param firstIntersectionToPack point in which the coordinates of the first intersection. Can
    *           be {@code null}. Modified.
    * @param secondIntersectionToPack point in which the coordinates of the second intersection. Can
    *           be {@code null}. Modified.
    * @return the number of intersections between this line segment and the polygon.
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
      return LineSegment2DReadOnly.super.intersectionWith(convexPolygon, firstIntersectionToPack, secondIntersectionToPack);
   }

   /**
    * Computes the coordinates of the possible intersection(s) between this line segment and the
    * given convex polygon 2D.
    *
    * @param convexPolygon the convex polygon this line segment may intersect. Not modified.
    * @param firstIntersectionToPack point in which the coordinates of the first intersection. Can
    *           be {@code null}. Modified.
    * @param secondIntersectionToPack point in which the coordinates of the second intersection. Can
    *           be {@code null}. Modified.
    * @return the number of intersections between this line segment and the polygon.
    * @see #intersectionWith(ConvexPolygon2DReadOnly, Point2DBasics, Point2DBasics)
    */
   default int intersectionWith(ConvexPolygon2DReadOnly convexPolygon, FramePoint2DBasics firstIntersectionToPack, FramePoint2DBasics secondIntersectionToPack)
   {
      firstIntersectionToPack.setReferenceFrame(getReferenceFrame());
      secondIntersectionToPack.setReferenceFrame(getReferenceFrame());
      return LineSegment2DReadOnly.super.intersectionWith(convexPolygon, firstIntersectionToPack, secondIntersectionToPack);
   }

   /**
    * Computes the coordinates of the possible intersection(s) between this line segment and the
    * given convex polygon 2D.
    *
    * @param convexPolygon the convex polygon this line segment may intersect. Not modified.
    * @param firstIntersectionToPack point in which the coordinates of the first intersection. Can
    *           be {@code null}. Modified.
    * @param secondIntersectionToPack point in which the coordinates of the second intersection. Can
    *           be {@code null}. Modified.
    * @return the number of intersections between this line segment and the polygon.
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
      return LineSegment2DReadOnly.super.intersectionWith(convexPolygon, firstIntersectionToPack, secondIntersectionToPack);
   }

   /**
    * Computes the coordinates of the possible intersection(s) between this line segment and the
    * given convex polygon 2D.
    *
    * @param convexPolygon the convex polygon this line segment may intersect. Not modified.
    * @param firstIntersectionToPack point in which the coordinates of the first intersection. Can
    *           be {@code null}. Modified.
    * @param secondIntersectionToPack point in which the coordinates of the second intersection. Can
    *           be {@code null}. Modified.
    * @return the number of intersections between this line segment and the polygon.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code convexPolygon} are not
    *            expressed in the same reference frame.
    * @see #intersectionWith(ConvexPolygon2DReadOnly, Point2DBasics, Point2DBasics)
    */
   default int intersectionWith(FrameConvexPolygon2DReadOnly convexPolygon, Point2DBasics firstIntersectionToPack, Point2DBasics secondIntersectionToPack)
   {
      checkReferenceFrameMatch(convexPolygon);
      return LineSegment2DReadOnly.super.intersectionWith(convexPolygon, firstIntersectionToPack, secondIntersectionToPack);
   }

   /** {@inheritDoc} */
   @Override
   default FramePoint2DBasics pointOnLineGivenPercentage(double percentage)
   {
      return new FramePoint2D(getReferenceFrame(), LineSegment2DReadOnly.super.pointOnLineGivenPercentage(percentage));
   }

   /**
    * Computes the coordinates of the point located on the line this line segment is lying on: <br>
    * {@code pointToPack.interpolate(firstEndpoint, secondEndpoint, percentage)} </br>
    *
    * @param percentage the percentage along this line segment of the point.
    * @param pointToPack where the result is stored. Modified.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code pointToPack} are not
    *            expressed in the same reference frame.
    */
   default void pointOnLineGivenPercentage(double percentage, FixedFramePoint2DBasics pointToPack)
   {
      checkReferenceFrameMatch(pointToPack);
      LineSegment2DReadOnly.super.pointOnLineGivenPercentage(percentage, pointToPack);
   }

   /**
    * Computes the coordinates of the point located on the line this line segment is lying on: <br>
    * {@code pointToPack.interpolate(firstEndpoint, secondEndpoint, percentage)} </br>
    *
    * @param percentage the percentage along this line segment of the point.
    * @param pointToPack where the result is stored. Modified.
    */
   default void pointOnLineGivenPercentage(double percentage, FramePoint2DBasics pointToPack)
   {
      pointToPack.setReferenceFrame(getReferenceFrame());
      LineSegment2DReadOnly.super.pointOnLineGivenPercentage(percentage, pointToPack);
   }

   /**
    * Tests on a per-component basis on both endpoints if this line segment is equal to
    * {@code other} with the tolerance {@code epsilon}.
    * <p>
    * If the two line segments have different frames, this method returns {@code false}.
    * </p>
    *
    * @param other the query. Not modified.
    * @param epsilon the tolerance to use.
    * @return {@code true} if the two line segments are equal and are expressed in the same
    *         reference frame, {@code false} otherwise.
    */
   default boolean epsilonEquals(FrameLineSegment2DReadOnly other, double epsilon)
   {
      if (getReferenceFrame() != other.getReferenceFrame())
         return false;
      return LineSegment2DReadOnly.super.epsilonEquals(other, epsilon);
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
   default boolean geometricallyEquals(FrameLineSegment2DReadOnly other, double epsilon)
   {
      checkReferenceFrameMatch(other);
      return LineSegment2DReadOnly.super.geometricallyEquals(other, epsilon);
   }

   /**
    * Tests on a per component basis, if this line segment 2D is exactly equal to {@code other}.
    * <p>
    * If the two line segments have different frames, this method returns {@code false}.
    * </p>
    *
    * @param other the other line segment 2D to compare against this. Not modified.
    * @return {@code true} if the two line segments are exactly equal component-wise and are
    *         expressed in the same reference frame, {@code false} otherwise.
    */
   default boolean equals(FrameLineSegment2DReadOnly other)
   {
      if (other == null || getReferenceFrame() != other.getReferenceFrame())
         return false;
      else
         return LineSegment2DReadOnly.super.equals(other);
   }
}
