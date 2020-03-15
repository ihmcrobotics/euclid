package us.ihmc.euclid.geometry.interfaces;

import static us.ihmc.euclid.geometry.tools.EuclidGeometryTools.intersectionBetweenLine2DAndBoundingBox2D;
import static us.ihmc.euclid.geometry.tools.EuclidGeometryTools.intersectionBetweenLineSegment2DAndBoundingBox2D;
import static us.ihmc.euclid.geometry.tools.EuclidGeometryTools.intersectionBetweenRay2DAndBoundingBox2D;

import us.ihmc.euclid.geometry.exceptions.BoundingBoxException;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;

/**
 * Read-only interface for a 2D axis-aligned bounding box defined from a set of minimum and maximum
 * coordinates.
 */
public interface BoundingBox2DReadOnly
{
   /**
    * Gets the read-only reference to the minimum coordinate of this bounding box.
    *
    * @return the read-only reference to the minimum coordinate.
    */
   Point2DReadOnly getMinPoint();

   /**
    * Gets the read-only reference to the maximum coordinate of this bounding box.
    *
    * @return the read-only reference to the maximum coordinate.
    */
   Point2DReadOnly getMaxPoint();

   /**
    * Gets the minimum x-coordinate of this bounding box.
    *
    * @return the minimum x-coordinate.
    */
   default double getMinX()
   {
      return getMinPoint().getX();
   }

   /**
    * Gets the minimum y-coordinate of this bounding box.
    *
    * @return the minimum y-coordinate.
    */
   default double getMinY()
   {
      return getMinPoint().getY();
   }

   /**
    * Gets the maximum x-coordinate of this bounding box.
    *
    * @return the maximum x-coordinate.
    */
   default double getMaxX()
   {
      return getMaxPoint().getX();
   }

   /**
    * Gets the maximum y-coordinate of this bounding box.
    *
    * @return the maximum y-coordinate.
    */
   default double getMaxY()
   {
      return getMaxPoint().getY();
   }

   /**
    * Asserts that the minimum coordinates are less or equal to the maximum coordinates.
    *
    * @throws RuntimeException if any of the minimum coordinates is strictly greater than the maximum
    *                          coordinate on the same axis.
    */
   default void checkBounds()
   {
      if (getMinX() > getMaxX())
         throw new BoundingBoxException("minPoint.getX() > maxPoint.getX(): " + getMinX() + ">" + getMaxX());
      if (getMinY() > getMaxY())
         throw new BoundingBoxException("minPoint.getY() > maxPoint.getY(): " + getMinY() + ">" + getMaxY());
   }

   /**
    * Tests if the min and max points defining this bounding box contain at least one
    * {@link Double#NaN}.
    *
    * @return {@code true} if this bounding box contains at least one {@link Double#NaN}, {@code false}
    *         otherwise.
    */
   default boolean containsNaN()
   {
      return getMinPoint().containsNaN() || getMaxPoint().containsNaN();
   }

   /**
    * Calculates the coordinate of the center of this bounding box and stores it in the given
    * {@code centerToPack}.
    *
    * @param centerToPack point 2D in which the center of this bounding box is stored. Modified.
    * @throws RuntimeException if this bounding box is improper according to {@link #checkBounds()}.
    */
   default void getCenterPoint(Point2DBasics centerToPack)
   {
      checkBounds();
      centerToPack.interpolate(getMinPoint(), getMaxPoint(), 0.5);
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
   default void getPointGivenParameters(double xParameter, double yParameter, Point2DBasics pointToPack)
   {
      checkBounds();
      pointToPack.setX(getMinX() + xParameter * (getMaxX() - getMinX()));
      pointToPack.setY(getMinY() + yParameter * (getMaxY() - getMinY()));
   }

   /**
    * Calculates the squared value of the distance between the minimum and maximum coordinates of this
    * bounding box.
    *
    * @return the squared value of this bounding box diagonal.
    * @throws RuntimeException if this bounding box is improper according to {@link #checkBounds()}.
    */
   default double getDiagonalLengthSquared()
   {
      checkBounds();
      return getMinPoint().distanceSquared(getMaxPoint());
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
    * @throws RuntimeException if this bounding box is improper according to {@link #checkBounds()}.
    */
   default boolean isInsideExclusive(Point2DReadOnly query)
   {
      return isInsideExclusive(query.getX(), query.getY());
   }

   /**
    * Tests if the {@code query} is located inside this bounding box.
    * <p>
    * The query is considered to be outside if located exactly on an edge of this bounding box.
    * </p>
    *
    * @param x the x-coordinate of the query to test if it is located inside this bounding box. Not
    *          modified.
    * @param y the y-coordinate of the query to test if it is located inside this bounding box. Not
    *          modified.
    * @return {@code true} if the query is inside, {@code false} if outside or located on an edge of
    *         this bounding box.
    * @throws RuntimeException if this bounding box is improper according to {@link #checkBounds()}.
    */
   default boolean isInsideExclusive(double x, double y)
   {
      checkBounds();
      if (x <= getMinX() || x >= getMaxX())
         return false;

      if (y <= getMinY() || y >= getMaxY())
         return false;

      return true;
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
    * @throws RuntimeException if this bounding box is improper according to {@link #checkBounds()}.
    */
   default boolean isInsideInclusive(Point2DReadOnly query)
   {
      return isInsideInclusive(query.getX(), query.getY());
   }

   /**
    * Tests if the {@code query} is located inside this bounding box.
    * <p>
    * The query is considered to be inside if located exactly on an edge of this bounding box.
    * </p>
    *
    * @param x the x-coordinate of the query to test if it is located inside this bounding box. Not
    *          modified.
    * @param y the y-coordinate of the query to test if it is located inside this bounding box. Not
    *          modified.
    * @return {@code true} if the query is inside or located on an edge of this bounding box,
    *         {@code false} if outside.
    * @throws RuntimeException if this bounding box is improper according to {@link #checkBounds()}.
    */
   default boolean isInsideInclusive(double x, double y)
   {
      checkBounds();
      if (x < getMinX() || x > getMaxX())
         return false;

      if (y < getMinY() || y > getMaxY())
         return false;

      return true;
   }

   /**
    * Tests if the {@code query} is located inside this bounding box given the tolerance
    * {@code epsilon}.
    * <p>
    * <ul>
    * <li>if {@code epsilon == 0}, this method is equivalent to
    * {@link #isInsideExclusive(Point2DReadOnly)}.
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
    * @throws RuntimeException if this bounding box is improper according to {@link #checkBounds()}.
    */
   default boolean isInsideEpsilon(Point2DReadOnly query, double epsilon)
   {
      return isInsideEpsilon(query.getX(), query.getY(), epsilon);
   }

   /**
    * Tests if the {@code query} is located inside this bounding box given the tolerance
    * {@code epsilon}.
    * <p>
    * <ul>
    * <li>if {@code epsilon == 0}, this method is equivalent to
    * {@link #isInsideExclusive(Point2DReadOnly)}.
    * <li>if {@code epsilon > 0}, the size of this bounding box is scaled up by shifting the edges of
    * {@code epsilon} toward the outside.
    * <li>if {@code epsilon > 0}, the size of this bounding box is scaled down by shifting the edges of
    * {@code epsilon} toward the inside.
    * </ul>
    * </p>
    *
    * @param x       the x-coordinate of the query to test if it is located inside this bounding box.
    *                Not modified.
    * @param y       the y-coordinate of the query to test if it is located inside this bounding box.
    *                Not modified.
    * @param epsilon the tolerance to use for this test.
    * @return {@code true} if the query is considered to be inside the bounding box, {@code false}
    *         otherwise.
    * @throws RuntimeException if this bounding box is improper according to {@link #checkBounds()}.
    */
   default boolean isInsideEpsilon(double x, double y, double epsilon)
   {
      checkBounds();
      if (x <= getMinX() - epsilon || x >= getMaxX() + epsilon)
         return false;

      if (y <= getMinY() - epsilon || y >= getMaxY() + epsilon)
         return false;

      return true;
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
    */
   default boolean intersectsExclusive(BoundingBox2DReadOnly other)
   {
      checkBounds();
      if (other.getMinX() >= getMaxX() || other.getMaxX() <= getMinX())
         return false;

      if (other.getMinY() >= getMaxY() || other.getMaxY() <= getMinY())
         return false;

      return true;
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
    * @throws RuntimeException if this bounding box is improper according to {@link #checkBounds()}.
    */
   default boolean intersectsInclusive(BoundingBox2DReadOnly other)
   {
      checkBounds();
      if (other.getMinX() > getMaxX() || other.getMaxX() < getMinX())
         return false;

      if (other.getMinY() > getMaxY() || other.getMaxY() < getMinY())
         return false;

      return true;
   }

   /**
    * Tests if this bounding box and {@code other} intersects.
    * <p>
    * <ul>
    * <li>if {@code epsilon == 0}, this method is equivalent to
    * {@link #intersectsExclusive(BoundingBox2DReadOnly)}.
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
    * @throws RuntimeException if this bounding box is improper according to {@link #checkBounds()}.
    */
   default boolean intersectsEpsilon(BoundingBox2DReadOnly other, double epsilon)
   {
      checkBounds();
      if (other.getMinX() >= getMaxX() + epsilon || other.getMaxX() <= getMinX() - epsilon)
         return false;

      if (other.getMinY() >= getMaxY() + epsilon || other.getMaxY() <= getMinY() - epsilon)
         return false;

      return true;
   }

   /**
    * Tests if this the given line 2D intersects this bounding box.
    *
    * @param pointOnLine   a point located on the infinitely long line. Not modified.
    * @param lineDirection the line direction. Not modified.
    * @return {@code true} if the line and this bounding box intersect, {@code false} otherwise.
    * @throws RuntimeException if this bounding box is improper according to {@link #checkBounds()}.
    */
   default boolean doesIntersectWithLine2D(Point2DReadOnly pointOnLine, Vector2DReadOnly lineDirection)
   {
      return intersectionWithLine2D(pointOnLine, lineDirection, null, null) > 0;
   }

   /**
    * Tests if this the given line segment 2D intersects this bounding box.
    *
    * @param lineSegmentStart first endpoint of the line segment. Not modified.
    * @param lineSegmentEnd   second endpoint of the line segment. Not modified.
    * @return {@code true} if the line segment and this bounding box intersect, {@code false}
    *         otherwise.
    * @throws RuntimeException if this bounding box is improper according to {@link #checkBounds()}.
    */
   default boolean doesIntersectWithLineSegment2D(Point2DReadOnly lineSegmentStart, Point2DReadOnly lineSegmentEnd)
   {
      return intersectionWithLineSegment2D(lineSegmentStart, lineSegmentEnd, null, null) > 0;
   }

   /**
    * Tests if this the given ray 2D intersects this bounding box.
    *
    * @param rayOrigin    the origin of the ray. Not modified.
    * @param rayDirection the ray direction. Not modified.
    * @return {@code true} if the ray and this bounding box intersect, {@code false} otherwise.
    * @throws RuntimeException if this bounding box is improper according to {@link #checkBounds()}.
    */
   default boolean doesIntersectWithRay2D(Point2DReadOnly rayOrigin, Vector2DReadOnly rayDirection)
   {
      return intersectionWithRay2D(rayOrigin, rayDirection, null, null) > 0;
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
   default int intersectionWithLine2D(Point2DReadOnly pointOnLine, Vector2DReadOnly lineDirection, Point2DBasics firstIntersectionToPack,
                                      Point2DBasics secondIntersectionToPack)
   {
      checkBounds();
      return intersectionBetweenLine2DAndBoundingBox2D(getMinPoint(),
                                                       getMaxPoint(),
                                                       pointOnLine,
                                                       lineDirection,
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
    * @throws RuntimeException if this bounding box is improper according to {@link #checkBounds()}.
    */
   default int intersectionWithLineSegment2D(Point2DReadOnly lineSegmentStart, Point2DReadOnly lineSegmentEnd, Point2DBasics firstIntersectionToPack,
                                             Point2DBasics secondIntersectionToPack)
   {
      checkBounds();
      return intersectionBetweenLineSegment2DAndBoundingBox2D(getMinPoint(),
                                                              getMaxPoint(),
                                                              lineSegmentStart,
                                                              lineSegmentEnd,
                                                              firstIntersectionToPack,
                                                              secondIntersectionToPack);
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
   default int intersectionWithRay2D(Point2DReadOnly rayOrigin, Vector2DReadOnly rayDirection, Point2DBasics firstIntersectionToPack,
                                     Point2DBasics secondIntersectionToPack)
   {
      checkBounds();
      return intersectionBetweenRay2DAndBoundingBox2D(getMinPoint(), getMaxPoint(), rayOrigin, rayDirection, firstIntersectionToPack, secondIntersectionToPack);
   }

   /**
    * Tests on a per component basis, if this bounding box 2D is exactly equal to {@code other}.
    *
    * @param other the other bounding box 2D to compare against this. Not modified.
    * @return {@code true} if the two bounding boxes are exactly equal component-wise, {@code false}
    *         otherwise.
    */
   default boolean equals(BoundingBox2DReadOnly other)
   {
      if (other == this)
         return true;
      else if (other == null)
         return false;
      else
         return getMinPoint().equals(other.getMinPoint()) && getMaxPoint().equals(other.getMaxPoint());
   }

   /**
    * Tests on a per-component basis on the minimum and maximum coordinates if this bounding box is
    * equal to {@code other} with the tolerance {@code epsilon}.
    *
    * @param other   the query. Not modified.
    * @param epsilon the tolerance to use.
    * @return {@code true} if the two bounding boxes are equal, {@code false} otherwise.
    */
   default boolean epsilonEquals(BoundingBox2DReadOnly other, double epsilon)
   {
      return getMinPoint().epsilonEquals(other.getMinPoint(), epsilon) && getMaxPoint().epsilonEquals(other.getMaxPoint(), epsilon);
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
    */
   default boolean geometricallyEquals(BoundingBox2DReadOnly other, double epsilon)
   {
      return getMinPoint().geometricallyEquals(other.getMinPoint(), epsilon) && getMaxPoint().geometricallyEquals(other.getMaxPoint(), epsilon);
   }
}
