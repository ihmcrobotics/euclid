package us.ihmc.euclid.geometry.interfaces;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.LineSegment2D;
import us.ihmc.euclid.geometry.exceptions.OutdatedPolygonException;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DBasics;

/**
 * Read-only interface for a line segment 2D.
 * <p>
 * A line segment 2D is a finite-length line defined in the XY-plane by its two 2D endpoints.
 * </p>
 */
public interface LineSegment2DReadOnly
{
   /**
    * Gets the read-only reference to the first endpoint of this line segment.
    *
    * @return the reference to the first endpoint of this line segment.
    */
   Point2DReadOnly getFirstEndpoint();

   /**
    * Gets the read-only reference to the second endpoint of this line segment.
    *
    * @return the reference to the second endpoint of this line segment.
    */
   Point2DReadOnly getSecondEndpoint();

   /**
    * Gets the endpoints defining this line segment by storing their coordinates in the given
    * arguments.
    *
    * @param firstEndpointToPack point in which the coordinates of this line segment's first endpoint
    *           are stored. Modified.
    * @param secondEndpointToPack point in which the coordinates of this line segment's second endpoint
    *           are stored. Modified.
    */
   default void get(Point2DBasics firstEndpointToPack, Point2DBasics secondEndpointToPack)
   {
      firstEndpointToPack.set(getFirstEndpoint());
      secondEndpointToPack.set(getSecondEndpoint());
   }

   /**
    * Gets the x-coordinate of the first endpoint defining this line segment.
    *
    * @return the first endpoint x-coordinate.
    */
   default double getFirstEndpointX()
   {
      return getFirstEndpoint().getX();
   }

   /**
    * Gets the y-coordinate of the first endpoint defining this line segment.
    *
    * @return the first endpoint y-coordinate.
    */
   default double getFirstEndpointY()
   {
      return getFirstEndpoint().getY();
   }

   /**
    * Gets the x-coordinate of the second endpoint defining this line segment.
    *
    * @return the first endpoint x-coordinate.
    */
   default double getSecondEndpointX()
   {
      return getSecondEndpoint().getX();
   }

   /**
    * Gets the y-coordinate of the second endpoint defining this line segment.
    *
    * @return the first endpoint y-coordinate.
    */
   default double getSecondEndpointY()
   {
      return getSecondEndpoint().getY();
   }

   /**
    * Test if the first endpoint of this line segment contains {@link Double#NaN}.
    *
    * @return {@code true} if {@link #getFirstEndpoint()} contains {@link Double#NaN}, {@code false}
    *         otherwise.
    */
   default boolean firstEndpointContainsNaN()
   {
      return getFirstEndpoint().containsNaN();
   }

   /**
    * Test if the second endpoint of this line segment contains {@link Double#NaN}.
    *
    * @return {@code true} if {@link #getSecondEndpoint()} contains {@link Double#NaN}, {@code false}
    *         otherwise.
    */
   default boolean secondEndpointContainsNaN()
   {
      return getSecondEndpoint().containsNaN();
   }

   /**
    * Tests if this line segment contains {@link Double#NaN}.
    *
    * @return {@code true} if {@link #getFirstEndpoint()} and/or {@link #getSecondEndpoint()} contains
    *         {@link Double#NaN}, {@code false} otherwise.
    */
   default boolean containsNaN()
   {
      return getFirstEndpoint().containsNaN() || getSecondEndpoint().containsNaN();
   }

   /**
    * Computes the coordinates of the point located exactly at the middle of this line segment.
    *
    * @param midpointToPack point in which the mid-point of this line segment is stored. Modified.
    */
   default void midpoint(Point2DBasics midpointToPack)
   {
      midpointToPack.interpolate(getFirstEndpoint(), getSecondEndpoint(), 0.5);
   }

   /**
    * Computes and returns the coordinates of the point located exactly at the middle of this line
    * segment.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @return the mid-point of this line segment.
    */
   default Point2DBasics midpoint()
   {
      Point2D midpoint = new Point2D();
      midpoint(midpoint);
      return midpoint;
   }

   /**
    * Computes the length of this line segment.
    *
    * @return the length of this line segment.
    */
   default double length()
   {
      return getFirstEndpoint().distance(getSecondEndpoint());
   }

   /**
    * Computes the squared value of the length of this line segment.
    *
    * @return the length squared of this line segment.
    */
   default double lengthSquared()
   {
      return getFirstEndpoint().distanceSquared(getSecondEndpoint());
   }

   /**
    * Tests whether the projection of the given point onto this line segment is located between the two
    * endpoints with a given conservative tolerance {@code epsilon}:
    * <ul>
    * <li>if {@code epsilon > 0}, the point has to be between the endpoints and at a minimum distance
    * of {@code epsilon * this.length()} from the closest endpoint.
    * <li>if {@code epsilon < 0}, the point has to be between the endpoints or at a maximum distance of
    * {@code -epsilon * this.length()} from the closest endpoint.
    * <li>if {@code epsilon = 0}, the point has to be between the endpoints or equal to one of the
    * endpoints.
    * </ul>
    *
    * @param x the x-coordinate of the query point.
    * @param y the y-coordinate of the query point.
    * @param z the z-coordinate of the query point.
    * @param epsilon the tolerance to use.
    * @return {@code true} if the projection of the point is between the endpoints of this line
    *         segment, {@code false} otherwise.
    */
   default boolean isBetweenEndpoints(double x, double y, double epsilon)
   {
      double alpha = percentageAlongLineSegment(x, y);

      if (alpha < epsilon)
         return false;
      if (alpha > 1.0 - epsilon)
         return false;

      return true;
   }

   /**
    * Tests whether the projection of the given point onto this line segment is located between the two
    * endpoints or exactly on an endpoint.
    *
    * @param point the query. Not modified.
    * @return {@code true} if the projection of the point is between the endpoints of this line
    *         segment, {@code false} otherwise.
    */
   default boolean isBetweenEndpoints(Point2DReadOnly point)
   {
      return isBetweenEndpoints(point, 0);
   }

   /**
    * Tests whether the projection of the given point onto this line segment is located between the two
    * endpoints with a given conservative tolerance {@code epsilon}:
    * <ul>
    * <li>if {@code epsilon > 0}, the point has to be between the endpoints and at a minimum distance
    * of {@code epsilon * this.length()} from the closest endpoint.
    * <li>if {@code epsilon < 0}, the point has to be between the endpoints or at a maximum distance of
    * {@code -epsilon * this.length()} from the closest endpoint.
    * <li>if {@code epsilon = 0}, the point has to be between the endpoints or equal to one of the
    * endpoints.
    * </ul>
    *
    * @param point the query. Not modified.
    * @param epsilon the tolerance to use.
    * @return {@code true} if the projection of the point is between the endpoints of this line
    *         segment, {@code false} otherwise.
    */
   default boolean isBetweenEndpoints(Point2DReadOnly point, double epsilon)
   {
      return isBetweenEndpoints(point.getX(), point.getY(), epsilon);
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
    * @return {@code true} if the point is on the left side of this line segment, {@code false} if the
    *         point is on the right side or exactly on this line segment.
    */
   default boolean isPointOnLeftSideOfLineSegment(Point2DReadOnly point)
   {
      return EuclidGeometryTools.isPoint2DOnLeftSideOfLine2D(point, getFirstEndpoint(), getSecondEndpoint());
   }

   /**
    * Computes the vector going from the first to the second endpoint of this line segment.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param normalize whether the direction vector is to be normalized.
    * @return the direction of this line segment.
    */
   default Vector2DBasics direction(boolean normalize)
   {
      Vector2D direction = new Vector2D();
      direction(normalize, direction);
      return direction;
   }

   /**
    * Computes the vector going from the first to the second endpoint of this line segment.
    *
    * @param normalize whether the direction vector is to be normalized.
    * @param directionToPack vector in which the direction is stored. Modified.
    */
   default void direction(boolean normalize, Vector2DBasics directionToPack)
   {
      directionToPack.sub(getSecondEndpoint(), getFirstEndpoint());
      if (normalize)
         directionToPack.normalize();
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
    */
   default double distance(Point2DReadOnly point)
   {
      return EuclidGeometryTools.distanceFromPoint2DToLineSegment2D(point, getFirstEndpoint(), getSecondEndpoint());
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
    */
   default double distanceSquared(Point2DReadOnly point)
   {
      return EuclidGeometryTools.distanceSquaredFromPoint2DToLineSegment2D(point.getX(), point.getY(), getFirstEndpoint(), getSecondEndpoint());
   }

   /**
    * Returns a copy of this line segment with the endpoints swapped.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    * @return copy of this line segment flipped.
    */
   default LineSegment2D flipDirectionCopy()
   {
      return new LineSegment2D(getSecondEndpoint(), getFirstEndpoint());
   }

   /**
    * Computes the coordinates of the possible intersection(s) between a given line segment 2D and this
    * convex polygon 2D.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>If the polygon has no vertices, this method behaves as if there is no intersections and this
    * method returns {@code null}.
    * <li>If no intersections exist, this method returns {@code null}.
    * <li>If the line segment is collinear to an edge:
    * <ul>
    * <li>The edge entirely contains the line segment: this method finds two intersections which are
    * the endpoints of the line segment.
    * <li>The line segment entirely contains the edge: this method finds two intersections which are
    * the vertices of the edge.
    * <li>The edge and the line segment partially overlap: this method finds two intersections which
    * the polygon's vertex that on the line segment and the line segment's endpoint that is on the
    * polygon's edge.
    * </ul>
    * </ul>
    * </p>
    *
    * @param convexPolygon the polygon this line segment may intersect. Not modified.
    * @return the intersections between the line segment and the polygon.
    * @throws OutdatedPolygonException if {@link ConvexPolygon2D#update()} has not been called since last time this
    *            polygon's vertices were edited.
    */
   default Point2DBasics[] intersectionWith(ConvexPolygon2DReadOnly convexPolygon)
   {
      return convexPolygon.intersectionWith(this);
   }

   /**
    * Computes the coordinates of the possible intersection(s) between this line segment and the given
    * convex polygon 2D.
    * <p>
    * Edge cases:
    * <ul>
    * <li>If the polygon has no vertices, this method behaves as if there is no intersections.
    * <li>If no intersections exist, this method returns {@code 0} and the two intersection-to-pack
    * arguments remain unmodified.
    * <li>If there is only one intersection, this method returns {@code 1} and the coordinates of the
    * only intersection are stored in {@code firstIntersectionToPack}. {@code secondIntersectionToPack}
    * remains unmodified.
    * <li>If this line segment is collinear to an edge:
    * <ul>
    * <li>The edge entirely contains this line segment: this method finds two intersections which are
    * the endpoints of this line segment.
    * <li>This line segment entirely contains the edge: this method finds two intersections which are
    * the vertices of the edge.
    * <li>The edge and this line segment partially overlap: this method finds two intersections which
    * the polygon's vertex that on this line segment and this line segment's endpoint that is on the
    * polygon's edge.
    * </ul>
    * </ul>
    * </p>
    *
    * @param convexPolygon the convex polygon this line segment may intersect. Not modified.
    * @param firstIntersectionToPack point in which the coordinates of the first intersection. Can be
    *           {@code null}. Modified.
    * @param secondIntersectionToPack point in which the coordinates of the second intersection. Can be
    *           {@code null}. Modified.
    * @return the number of intersections between this line segment and the polygon.
    * @throws OutdatedPolygonException if the convex polygon is not up-to-date.
    */
   default int intersectionWith(ConvexPolygon2DReadOnly convexPolygon, Point2DBasics firstIntersectionToPack, Point2DBasics secondIntersectionToPack)
   {
      return convexPolygon.intersectionWith(this, firstIntersectionToPack, secondIntersectionToPack);
   }

   /**
    * Calculates the coordinates of the intersection between this line segment and the given line and
    * returns the result.
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
    */
   default Point2DBasics intersectionWith(Line2DReadOnly line)
   {
      return EuclidGeometryTools.intersectionBetweenLine2DAndLineSegment2D(line.getPoint(), line.getDirection(), getFirstEndpoint(), getSecondEndpoint());
   }

   /**
    * Calculates the coordinates of the intersection between this line segment and the given line and
    * stores the result in {@code intersectionToPack}.
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
   default boolean intersectionWith(Line2DReadOnly line, Point2DBasics intersectionToPack)
   {
      return EuclidGeometryTools.intersectionBetweenLine2DAndLineSegment2D(line.getPoint(), line.getDirection(), getFirstEndpoint(), getSecondEndpoint(),
                                                                           intersectionToPack);
   }

   /**
    * Computes the intersection between this line segment and the given line segment and returns the
    * result.
    * <p>
    * Edge cases:
    * <ul>
    * <li>When the two line segments are parallel but not collinear, the two line segments do not
    * intersect, this method returns {@code null}.
    * <li>When the two line segments are collinear, if the two line segments do not overlap do not have
    * at least one common endpoint, this method returns {@code null}.
    * <li>When the two line segments have a common endpoint, this method returns the common endpoint as
    * the intersection.
    * </ul>
    * </p>
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param other the other line segment that may intersect this line segment. Not modified.
    * @return the intersection point if it exists, {@code null} otherwise.
    */
   default Point2DBasics intersectionWith(LineSegment2DReadOnly other)
   {
      return EuclidGeometryTools.intersectionBetweenTwoLineSegment2Ds(getFirstEndpoint(), getSecondEndpoint(), other.getFirstEndpoint(),
                                                                      other.getSecondEndpoint());
   }

   /**
    * Computes the intersection between this line segment and the given line segment and stores the
    * result in {@code intersectionToPack}.
    * <p>
    * Edge cases:
    * <ul>
    * <li>When the two line segments are parallel but not collinear, the two line segments do not
    * intersect.
    * <li>When the two line segments are collinear, this methods returns {@code true} only if the two
    * line segments overlap or have at least one common endpoint.
    * <li>When the two line segments have a common endpoint, this method returns true.
    * </ul>
    * </p>
    *
    * @param other the other line segment that may intersect this line segment. Not modified.
    * @param intersectionToPack the 2D point in which the result is stored. Modified.
    * @return {@code true} if the two lines intersects, {@code false} otherwise.
    */
   default boolean intersectionWith(LineSegment2DReadOnly other, Point2DBasics intersectionToPack)
   {
      return EuclidGeometryTools.intersectionBetweenTwoLineSegment2Ds(getFirstEndpoint(), getSecondEndpoint(), other.getFirstEndpoint(),
                                                                      other.getSecondEndpoint(), intersectionToPack);
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
    */
   default boolean isPointOnLineSegment(Point2DReadOnly point)
   {
      return isPointOnLineSegment(point, 1.0e-8);
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
    */
   default boolean isPointOnLineSegment(Point2DReadOnly point, double epsilon)
   {
      return EuclidGeometryTools.distanceFromPoint2DToLineSegment2D(point, getFirstEndpoint(), getSecondEndpoint()) < epsilon;
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
    * @return {@code true} if the point is on the right side of this line segment, {@code false} if the
    *         point is on the left side or exactly on this line segment.
    */
   default boolean isPointOnRightSideOfLineSegment(Point2DReadOnly point)
   {
      return EuclidGeometryTools.isPoint2DOnRightSideOfLine2D(point, getFirstEndpoint(), getSecondEndpoint());
   }

   /**
    * Computes a percentage along the line segment representing the location of the given point once
    * projected onto this line segment. The returned percentage is in ] -&infin;; &infin; [,
    * {@code 0.0} representing {@code firstEndpoint}, and {@code 1.0} representing
    * {@code secondEndpoint}.
    * <p>
    * For example, if the returned percentage is {@code 0.5}, it means that the projection of the given
    * point is located at the middle of this line segment. The coordinates of the projection of the
    * point can be computed from the {@code percentage} as follows:<br>
    * <code>
    * Point3D projection = new Point3D(); </br>
    * projection.interpolate(lineSegmentStart, lineSegmentEnd, percentage); </br>
    * </code>
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the length of the given line segment is too small, i.e.
    * {@code this.lengthSquared() < }{@link EuclidGeometryTools#ONE_TRILLIONTH}, this method fails and
    * returns {@code 0.0}.
    * </ul>
    * </p>
    *
    * @param x the x-coordinate of the query point.
    * @param y the y-coordinate of the query point.
    * @param z the z-coordinate of the query point.
    * @return the computed percentage along the line segment representing where the point projection is
    *         located.
    */
   default double percentageAlongLineSegment(double x, double y)
   {
      return EuclidGeometryTools.percentageAlongLineSegment2D(x, y, getFirstEndpoint(), getSecondEndpoint());
   }

   /**
    * Computes a percentage along the line segment representing the location of the given point once
    * projected onto this line segment. The returned percentage is in ] -&infin;; &infin; [,
    * {@code 0.0} representing {@code firstEndpoint}, and {@code 1.0} representing
    * {@code secondEndpoint}.
    * <p>
    * For example, if the returned percentage is {@code 0.5}, it means that the projection of the given
    * point is located at the middle of this line segment. The coordinates of the projection of the
    * point can be computed from the {@code percentage} as follows: <code>
    * Point3D projection = new Point3D(); </br>
    * projection.interpolate(lineSegmentStart, lineSegmentEnd, percentage); </br>
    * </code>
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the length of the given line segment is too small, i.e.
    * {@code this.lengthSquared() < }{@link EuclidGeometryTools#ONE_TRILLIONTH}, this method fails and
    * returns {@code 0.0}.
    * </ul>
    * </p>
    *
    * @param point the query point. Not modified.
    * @return the computed percentage along the line segment representing where the point projection is
    *         located.
    */
   default double percentageAlongLineSegment(Point2DReadOnly point)
   {
      return percentageAlongLineSegment(point.getX(), point.getY());
   }

   /**
    * Computes the dot product of this line segment with the other line segment such that:<br>
    * {@code this }&middot;
    * {@code other = Math.cos(}&alpha;{@code ) * this.length() * other.length()}<br>
    * where &alpha; is the angle from this to the other line segment.
    *
    * @param other the other line segment used to compute the dot product. Not modified.
    * @return the value of the dot product.
    */
   default double dotProduct(LineSegment2DReadOnly other)
   {
      return EuclidGeometryTools.dotProduct(getFirstEndpoint(), getSecondEndpoint(), other.getFirstEndpoint(), other.getSecondEndpoint());
   }

   /**
    * Computes the orthogonal projection of a 2D point on this 2D line segment.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the length of this line segment is too small, i.e.
    * {@code this.lengthSquared() < }{@link EuclidGeometryTools#ONE_TRILLIONTH}, this method returns
    * {@code firstEndpoint}.
    * <li>the projection can not be outside the line segment. When the projection on the corresponding
    * line is outside the line segment, the result is the closest of the two endpoints.
    * </ul>
    * </p>
    *
    * @param pointToProject the point to project on this line segment. Modified.
    * @return whether the method succeeded or not.
    */
   default boolean orthogonalProjection(Point2DBasics pointToProject)
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
    * <li>the projection can not be outside the line segment. When the projection on the corresponding
    * line is outside the line segment, the result is the closest of the two endpoints.
    * </ul>
    * </p>
    *
    * @param pointToProject the point to compute the projection of. Not modified.
    * @param projectionToPack point in which the projection of the point onto this line segment is
    *           stored. Modified.
    * @return whether the method succeeded or not.
    */
   default boolean orthogonalProjection(Point2DReadOnly pointToProject, Point2DBasics projectionToPack)
   {
      return EuclidGeometryTools.orthogonalProjectionOnLineSegment2D(pointToProject, getFirstEndpoint(), getSecondEndpoint(), projectionToPack);
   }

   /**
    * Computes the orthogonal projection of a 2D point on this 2D line segment.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the length of this line segment is too small, i.e.
    * {@code this.lengthSquared() < }{@link EuclidGeometryTools#ONE_TRILLIONTH}, this method returns
    * {@code firstEndpoint}.
    * <li>the projection can not be outside the line segment. When the projection on the corresponding
    * line is outside the line segment, the result is the closest of the two endpoints.
    * </ul>
    * </p>
    *
    * @param pointToProject the point to compute the projection of. Not modified.
    * @return the projection of the point onto this line segment or {@code null} if the method failed.
    */
   default Point2DBasics orthogonalProjectionCopy(Point2DReadOnly pointToProject)
   {
      return EuclidGeometryTools.orthogonalProjectionOnLineSegment2D(pointToProject, getFirstEndpoint(), getSecondEndpoint());
   }

   /**
    * Computes the vector perpendicular to the direction of this line segment.
    *
    * @param normalize whether the perpendicular vector is to be normalized.
    * @param perpendicularVectorToPack vector in which the perpendicular vector components are stored.
    *           Modified.
    */
   default void perpendicular(boolean normalize, Vector2DBasics perpendicularVectorToPack)
   {
      direction(normalize, perpendicularVectorToPack);
      EuclidGeometryTools.perpendicularVector2D(perpendicularVectorToPack, perpendicularVectorToPack);
   }

   /**
    * Computes the coordinates of the point located at a given percentage on this line segment: <br>
    * {@code pointToPack.interpolate(firstEndpoint, secondEndpoint, percentage)} </br>
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param percentage the percentage along this line segment of the point. Must be in [0, 1].
    * @return the computed point.
    * @throws {@link RuntimeException} if {@code percentage} &notin; [0, 1].
    */
   default Point2DBasics pointBetweenEndpointsGivenPercentage(double percentage)
   {
      Point2D point = new Point2D();
      pointBetweenEndpointsGivenPercentage(percentage, point);
      return point;
   }

   /**
    * Computes the coordinates of the point located at a given percentage on this line segment: <br>
    * {@code pointToPack.interpolate(firstEndpoint, secondEndpoint, percentage)} </br>
    *
    * @param percentage the percentage along this line segment of the point. Must be in [0, 1].
    * @param pointToPack where the result is stored. Modified.
    * @throws {@link RuntimeException} if {@code percentage} &notin; [0, 1].
    */
   default void pointBetweenEndpointsGivenPercentage(double percentage, Point2DBasics pointToPack)
   {
      if (percentage < 0.0 || percentage > 1.0)
         throw new RuntimeException("Percentage must be between 0.0 and 1.0. Was: " + percentage);

      pointToPack.interpolate(getFirstEndpoint(), getSecondEndpoint(), percentage);
   }

   /**
    * Computes the coordinates of the point located on the line this line segment is lying on: <br>
    * {@code pointToPack.interpolate(firstEndpoint, secondEndpoint, percentage)} </br>
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param percentage the percentage along this line segment of the point.
    * @return the computed point.
    */
   default Point2DBasics pointOnLineGivenPercentage(double percentage)
   {
      Point2D point = new Point2D();
      pointOnLineGivenPercentage(percentage, point);
      return point;
   }

   /**
    * Computes the coordinates of the point located on the line this line segment is lying on: <br>
    * {@code pointToPack.interpolate(firstEndpoint, secondEndpoint, percentage)} </br>
    *
    * @param percentage the percentage along this line segment of the point.
    * @param pointToPack where the result is stored. Modified.
    */
   default void pointOnLineGivenPercentage(double percentage, Point2DBasics pointToPack)
   {
      pointToPack.interpolate(getFirstEndpoint(), getSecondEndpoint(), percentage);
   }

   /**
    * Tests on a per-component basis on both endpoints if this line segment is equal to {@code other}
    * with the tolerance {@code epsilon}.
    *
    * @param other the query. Not modified.
    * @param epsilon the tolerance to use.
    * @return {@code true} if the two line segments are equal, {@code false} otherwise.
    */
   default boolean epsilonEquals(LineSegment2DReadOnly other, double epsilon)
   {
      return getFirstEndpoint().epsilonEquals(other.getFirstEndpoint(), epsilon) && getSecondEndpoint().epsilonEquals(other.getSecondEndpoint(), epsilon);
   }

   /**
    * Tests on a per component basis, if this line segment 2D is exactly equal to {@code other}.
    *
    * @param other the other line segment 2D to compare against this. Not modified.
    * @return {@code true} if the two line segments are exactly equal component-wise, {@code false}
    *         otherwise.
    */
   default boolean equals(LineSegment2DReadOnly other)
   {
      if (other == null)
         return false;
      else
         return getFirstEndpoint().equals(other.getFirstEndpoint()) && getSecondEndpoint().equals(other.getSecondEndpoint());
   }

   /**
    * Compares {@code this} to {@code other} to determine if the two line segments are geometrically
    * similar.
    * <p>
    * The comparison is based on comparing the line segments' endpoints. Two line segments are
    * considered geometrically equal even if they are defined with opposite direction.
    * </p>
    *
    * @param other the line segment to compare to. Not modified.
    * @param epsilon the tolerance of the comparison.
    * @return {@code true} if the two line segments represent the same geometry, {@code false}
    *         otherwise.
    */
   default boolean geometricallyEquals(LineSegment2DReadOnly other, double epsilon)
   {
      if (getFirstEndpoint().geometricallyEquals(other.getFirstEndpoint(), epsilon)
            && getSecondEndpoint().geometricallyEquals(other.getSecondEndpoint(), epsilon))
         return true;
      if (getFirstEndpoint().geometricallyEquals(other.getSecondEndpoint(), epsilon)
            && getSecondEndpoint().geometricallyEquals(other.getFirstEndpoint(), epsilon))
         return true;
      return false;
   }
}
