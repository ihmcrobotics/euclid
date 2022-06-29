package us.ihmc.euclid.geometry.interfaces;

import static us.ihmc.euclid.geometry.tools.EuclidGeometryTools.intersectionBetweenLine3DAndBoundingBox3D;
import static us.ihmc.euclid.geometry.tools.EuclidGeometryTools.intersectionBetweenLineSegment3DAndBoundingBox3D;
import static us.ihmc.euclid.geometry.tools.EuclidGeometryTools.intersectionBetweenRay3DAndBoundingBox3D;

import us.ihmc.euclid.geometry.exceptions.BoundingBoxException;
import us.ihmc.euclid.geometry.tools.EuclidGeometryIOTools;
import us.ihmc.euclid.interfaces.EuclidGeometry;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

/**
 * Read-only interface for a 3D axis-aligned bounding box defined from a set of minimum and maximum
 * coordinates.
 */
public interface BoundingBox3DReadOnly extends EuclidGeometry
{
   /**
    * Gets the read-only reference to the minimum coordinate of this bounding box.
    *
    * @return the read-only reference to the minimum coordinate.
    */
   Point3DReadOnly getMinPoint();

   /**
    * Gets the read-only reference to the maximum coordinate of this bounding box.
    *
    * @return the read-only reference to the maximum coordinate.
    */
   Point3DReadOnly getMaxPoint();

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
    * Gets the minimum z-coordinate of this bounding box.
    *
    * @return the minimum z-coordinate.
    */
   default double getMinZ()
   {
      return getMinPoint().getZ();
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
    * Gets the maximum z-coordinate of this bounding box.
    *
    * @return the maximum z-coordinate.
    */
   default double getMaxZ()
   {
      return getMaxPoint().getZ();
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
      if (getMinZ() > getMaxZ())
         throw new BoundingBoxException("minPoint.getZ() > maxPoint.getZ(): " + getMinZ() + ">" + getMaxZ());
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
    * @param centerToPack point 3D in which the center of this bounding box is stored. Modified.
    */
   default void getCenterPoint(Point3DBasics centerToPack)
   {
      centerToPack.interpolate(getMinPoint(), getMaxPoint(), 0.5);
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
    */
   default void getPointGivenParameters(double xParameter, double yParameter, double zParameter, Point3DBasics pointToPack)
   {
      pointToPack.set(getMinX() + xParameter * (getMaxX() - getMinX()),
                      getMinY() + yParameter * (getMaxY() - getMinY()),
                      getMinZ() + zParameter * (getMaxZ() - getMinZ()));
   }

   /**
    * Calculates the squared value of the distance between the minimum and maximum coordinates of this
    * bounding box.
    *
    * @return the squared value of this bounding box diagonal.
    */
   default double getDiagonalLengthSquared()
   {
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
    */
   default boolean isInsideExclusive(Point3DReadOnly query)
   {
      return isInsideExclusive(query.getX(), query.getY(), query.getZ());
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
    * @param z the z-coordinate of the query to test if it is located inside this bounding box. Not
    *          modified.
    * @return {@code true} if the query is inside, {@code false} if outside or located on an edge of
    *         this bounding box.
    */
   default boolean isInsideExclusive(double x, double y, double z)
   {
      if (x <= getMinX())
         return false;
      if (x >= getMaxX())
         return false;
      if (y <= getMinY())
         return false;
      if (y >= getMaxY())
         return false;
      if (z <= getMinZ())
         return false;
      if (z >= getMaxZ())
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
    */
   default boolean isInsideInclusive(Point3DReadOnly query)
   {
      return isInsideInclusive(query.getX(), query.getY(), query.getZ());
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
    * @param z the z-coordinate of the query to test if it is located inside this bounding box. Not
    *          modified.
    * @return {@code true} if the query is inside or located on an edge of this bounding box,
    *         {@code false} if outside.
    */
   default boolean isInsideInclusive(double x, double y, double z)
   {
      if (x < getMinX())
         return false;
      if (x > getMaxX())
         return false;
      if (y < getMinY())
         return false;
      if (y > getMaxY())
         return false;
      if (z < getMinZ())
         return false;
      if (z > getMaxZ())
         return false;

      return true;
   }

   /**
    * Tests if the {@code query} is located inside this bounding box given the tolerance
    * {@code epsilon}.
    * <p>
    * <ul>
    * <li>if {@code epsilon == 0}, this method is equivalent to
    * {@link #isInsideExclusive(Point3DReadOnly)}.
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
    */
   default boolean isInsideEpsilon(Point3DReadOnly query, double epsilon)
   {
      return isInsideEpsilon(query.getX(), query.getY(), query.getZ(), epsilon);
   }

   /**
    * Tests if the {@code query} is located inside this bounding box given the tolerance
    * {@code epsilon}.
    * <p>
    * <ul>
    * <li>if {@code epsilon == 0}, this method is equivalent to
    * {@link #isInsideExclusive(Point3DReadOnly)}.
    * <li>if {@code epsilon > 0}, the size of this bounding box is scaled up by shifting the edges of
    * {@code epsilon} toward the outside.
    * <li>if {@code epsilon > 0}, the size of this bounding box is scaled down by shifting the edges of
    * {@code epsilon} toward the inside.
    * </ul>
    * </p>
    *
    * @param x       the x-coordinate the query to test if it is located inside this bounding box. Not
    *                modified.
    * @param y       the y-coordinate the query to test if it is located inside this bounding box. Not
    *                modified.
    * @param z       the z-coordinate the query to test if it is located inside this bounding box. Not
    *                modified.
    * @param epsilon the tolerance to use for this test.
    * @return {@code true} if the query is considered to be inside the bounding box, {@code false}
    *         otherwise.
    */
   default boolean isInsideEpsilon(double x, double y, double z, double epsilon)
   {
      if (x <= getMinX() - epsilon || x >= getMaxX() + epsilon || y <= getMinY() - epsilon || y >= getMaxY() + epsilon)
         return false;

      if (z <= getMinZ() - epsilon || z >= getMaxZ() + epsilon)
         return false;

      return true;
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
    */
   default boolean isXYInsideExclusive(Point2DReadOnly query)
   {
      return isXYInsideExclusive(query.getX(), query.getY());
   }

   /**
    * Tests if the {@code query} is located inside the projection onto the XY-plane of this bounding
    * box.
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
    */
   default boolean isXYInsideExclusive(double x, double y)
   {
      if (x <= getMinX())
         return false;
      if (x >= getMaxX())
         return false;
      if (y <= getMinY())
         return false;
      if (y >= getMaxY())
         return false;

      return true;
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
    */
   default boolean isXYInsideInclusive(Point2DReadOnly query)
   {
      return isXYInsideInclusive(query.getX(), query.getY());
   }

   /**
    * Tests if the {@code query} is located inside the projection onto the XY-plane of this bounding
    * box.
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
    */
   default boolean isXYInsideInclusive(double x, double y)
   {
      if (x < getMinX())
         return false;
      if (x > getMaxX())
         return false;
      if (y < getMinY())
         return false;
      if (y > getMaxY())
         return false;

      return true;
   }

   /**
    * * Tests if the {@code query} is located inside the projection onto the XY-plane of this bounding
    * box given the tolerance {@code epsilon}.
    * <p>
    * <ul>
    * <li>if {@code epsilon == 0}, this method is equivalent to
    * {@link #isInsideExclusive(Point3DReadOnly)}.
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
    */
   default boolean isXYInsideEpsilon(Point2DReadOnly query, double epsilon)
   {
      return isXYInsideEpsilon(query.getX(), query.getY(), epsilon);
   }

   /**
    * * Tests if the {@code query} is located inside the projection onto the XY-plane of this bounding
    * box given the tolerance {@code epsilon}.
    * <p>
    * <ul>
    * <li>if {@code epsilon == 0}, this method is equivalent to
    * {@link #isInsideExclusive(Point3DReadOnly)}.
    * <li>if {@code epsilon > 0}, the size of this bounding box is scaled up by shifting the edges of
    * {@code epsilon} toward the outside.
    * <li>if {@code epsilon > 0}, the size of this bounding box is scaled down by shifting the edges of
    * {@code epsilon} toward the inside.
    * </ul>
    * </p>
    *
    * @param x       the x-coordinate the query to test if it is located inside this bounding box. Not
    *                modified.
    * @param y       the y-coordinate the query to test if it is located inside this bounding box. Not
    *                modified.
    * @param epsilon the tolerance to use for this test.
    * @return {@code true} if the query is considered to be inside the bounding box, {@code false}
    *         otherwise.
    */
   default boolean isXYInsideEpsilon(double x, double y, double epsilon)
   {
      if (x <= getMinX() - epsilon)
         return false;
      if (x >= getMaxX() + epsilon)
         return false;
      if (y <= getMinY() - epsilon)
         return false;
      if (y >= getMaxY() + epsilon)
         return false;

      return true;
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
    */
   default boolean intersectsExclusive(BoundingBox3DReadOnly other)
   {
      if (other.getMinX() >= getMaxX())
         return false;
      if (other.getMaxX() <= getMinX())
         return false;
      if (other.getMinY() >= getMaxY())
         return false;
      if (other.getMaxY() <= getMinY())
         return false;
      if (other.getMinZ() >= getMaxZ())
         return false;
      if (other.getMaxZ() <= getMinZ())
         return false;

      return true;
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
    */
   default boolean intersectsInclusive(BoundingBox3DReadOnly other)
   {
      if (other.getMinX() > getMaxX())
         return false;
      if (other.getMaxX() < getMinX())
         return false;
      if (other.getMinY() > getMaxY())
         return false;
      if (other.getMaxY() < getMinY())
         return false;
      if (other.getMinZ() > getMaxZ())
         return false;
      if (other.getMaxZ() < getMinZ())
         return false;

      return true;
   }

   /**
    * Tests if this bounding box and {@code other} intersects.
    * <p>
    * <ul>
    * <li>if {@code epsilon == 0}, this method is equivalent to
    * {@link #intersectsExclusive(BoundingBox3DReadOnly)}.
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
    */
   default boolean intersectsEpsilon(BoundingBox3DReadOnly other, double epsilon)
   {
      if (other.getMinX() >= getMaxX() + epsilon)
         return false;
      if (other.getMaxX() <= getMinX() - epsilon)
         return false;
      if (other.getMinY() >= getMaxY() + epsilon)
         return false;
      if (other.getMaxY() <= getMinY() - epsilon)
         return false;
      if (other.getMinZ() >= getMaxZ() + epsilon)
         return false;
      if (other.getMaxZ() <= getMinZ() - epsilon)
         return false;

      return true;
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
    */
   default boolean intersectsExclusiveInXYPlane(BoundingBox2DReadOnly other)
   {
      if (other.getMinX() >= getMaxX())
         return false;
      if (other.getMaxX() <= getMinX())
         return false;
      if (other.getMinY() >= getMaxY())
         return false;
      if (other.getMaxY() <= getMinY())
         return false;

      return true;
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
    */
   default boolean intersectsInclusiveInXYPlane(BoundingBox2DReadOnly other)
   {
      if (other.getMinX() > getMaxX())
         return false;
      if (other.getMaxX() < getMinX())
         return false;
      if (other.getMinY() > getMaxY())
         return false;
      if (other.getMaxY() < getMinY())
         return false;

      return true;
   }

   /**
    * Tests if this bounding box and {@code other} intersects.
    * <p>
    * <ul>
    * <li>if {@code epsilon == 0}, this method is equivalent to
    * {@link #intersectsExclusiveInXYPlane(BoundingBox2DReadOnly)}.
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
    */
   default boolean intersectsEpsilonInXYPlane(BoundingBox2DReadOnly other, double epsilon)
   {
      if (other.getMinX() >= getMaxX() + epsilon)
         return false;
      if (other.getMaxX() <= getMinX() - epsilon)
         return false;
      if (other.getMinY() >= getMaxY() + epsilon)
         return false;
      if (other.getMaxY() <= getMinY() - epsilon)
         return false;

      return true;
   }

   /**
    * Tests if this the given line 3D intersects this bounding box.
    *
    * @param line3D the query. Not modified.
    * @return {@code true} if the line and this bounding box intersect, {@code false} otherwise.
    */
   default boolean doesIntersectWithLine3D(Line3DReadOnly line3D)
   {
      return doesIntersectWithLine3D(line3D.getPoint(), line3D.getDirection());
   }

   /**
    * Tests if this the given line 3D intersects this bounding box.
    *
    * @param pointOnLine   a point located on the infinitely long line. Not modified.
    * @param lineDirection the line direction. Not modified.
    * @return {@code true} if the line and this bounding box intersect, {@code false} otherwise.
    */
   default boolean doesIntersectWithLine3D(Point3DReadOnly pointOnLine, Vector3DReadOnly lineDirection)
   {
      return intersectionWithLine3D(pointOnLine, lineDirection, null, null) > 0;
   }

   /**
    * Tests if this the given line segment 3D intersects this bounding box.
    *
    * @param lineSegment3D the query. Not modified.
    * @return {@code true} if the line segment and this bounding box intersect, {@code false}
    *         otherwise.
    */
   default boolean doesIntersectWithLineSegment3D(LineSegment3DReadOnly lineSegment3D)
   {
      return doesIntersectWithLineSegment3D(lineSegment3D.getFirstEndpoint(), lineSegment3D.getSecondEndpoint());
   }

   /**
    * Tests if this the given line segment 3D intersects this bounding box.
    *
    * @param lineSegmentStart first endpoint of the line segment. Not modified.
    * @param lineSegmentEnd   second endpoint of the line segment. Not modified.
    * @return {@code true} if the line segment and this bounding box intersect, {@code false}
    *         otherwise.
    */
   default boolean doesIntersectWithLineSegment3D(Point3DReadOnly lineSegmentStart, Point3DReadOnly lineSegmentEnd)
   {
      return intersectionWithLineSegment3D(lineSegmentStart, lineSegmentEnd, null, null) > 0;
   }

   /**
    * Tests if this the given ray 3D intersects this bounding box.
    *
    * @param rayOrigin    the origin of the ray. Not modified.
    * @param rayDirection the ray direction. Not modified.
    * @return {@code true} if the ray and this bounding box intersect, {@code false} otherwise.
    */
   default boolean doesIntersectWithRay3D(Point3DReadOnly rayOrigin, Vector3DReadOnly rayDirection)
   {
      return intersectionWithRay3D(rayOrigin, rayDirection, null, null) > 0;
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
   default int intersectionWithLine3D(Line3DReadOnly line3D, Point3DBasics firstIntersectionToPack, Point3DBasics secondIntersectionToPack)
   {
      return intersectionWithLine3D(line3D.getPoint(), line3D.getDirection(), firstIntersectionToPack, secondIntersectionToPack);
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
   default int intersectionWithLine3D(Point3DReadOnly pointOnLine,
                                      Vector3DReadOnly lineDirection,
                                      Point3DBasics firstIntersectionToPack,
                                      Point3DBasics secondIntersectionToPack)
   {
      return intersectionBetweenLine3DAndBoundingBox3D(getMinPoint(),
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
   default int intersectionWithLineSegment3D(LineSegment3DReadOnly lineSegment3D, Point3DBasics firstIntersectionToPack, Point3DBasics secondIntersectionToPack)
   {
      return intersectionWithLineSegment3D(lineSegment3D.getFirstEndpoint(),
                                           lineSegment3D.getSecondEndpoint(),
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
    */
   default int intersectionWithLineSegment3D(Point3DReadOnly lineSegmentStart,
                                             Point3DReadOnly lineSegmentEnd,
                                             Point3DBasics firstIntersectionToPack,
                                             Point3DBasics secondIntersectionToPack)
   {
      return intersectionBetweenLineSegment3DAndBoundingBox3D(getMinPoint(),
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
    */
   default int intersectionWithRay3D(Point3DReadOnly rayOrigin,
                                     Vector3DReadOnly rayDirection,
                                     Point3DBasics firstIntersectionToPack,
                                     Point3DBasics secondIntersectionToPack)
   {
      return intersectionBetweenRay3DAndBoundingBox3D(getMinPoint(), getMaxPoint(), rayOrigin, rayDirection, firstIntersectionToPack, secondIntersectionToPack);
   }

   /** {@inheritDoc} */
   @Override
   default boolean epsilonEquals(EuclidGeometry geometry, double epsilon)
   {
      if (geometry == this)
         return true;
      if (geometry == null)
         return false;
      if (!(geometry instanceof BoundingBox3DReadOnly))
         return false;
      BoundingBox3DReadOnly other = (BoundingBox3DReadOnly) geometry;
      return getMinPoint().epsilonEquals(other.getMinPoint(), epsilon) && getMaxPoint().epsilonEquals(other.getMaxPoint(), epsilon);
   }

   /** {@inheritDoc} */
   @Override
   default boolean geometricallyEquals(EuclidGeometry geometry, double epsilon)
   {
      if (geometry == this)
         return true;
      if (geometry == null)
         return false;
      if (!(geometry instanceof BoundingBox3DReadOnly))
         return false;
      BoundingBox3DReadOnly other = (BoundingBox3DReadOnly) geometry;
      return getMinPoint().geometricallyEquals(other.getMinPoint(), epsilon) && getMaxPoint().geometricallyEquals(other.getMaxPoint(), epsilon);
   }

   /** {@inheritDoc} */
   @Override
   default boolean equals(EuclidGeometry geometry)
   {
      if (geometry == this)
         return true;
      if ((geometry == null) || !(geometry instanceof BoundingBox3DReadOnly))
         return false;
      BoundingBox3DReadOnly other = (BoundingBox3DReadOnly) geometry;
      return getMinPoint().equals(other.getMinPoint()) && getMaxPoint().equals(other.getMaxPoint());
   }

   /**
    * Gets a representative {@code String} of this bounding box 3D given a specific format to use.
    * <p>
    * Using the default format {@link EuclidCoreIOTools#DEFAULT_FORMAT}, this provides a {@code String}
    * as follows:
    *
    * <pre>
    * Bounding Box 3D: min = ( 0.174,  0.732, -0.222 ), max = (-0.558, -0.380,  0.130 )
    * </pre>
    * </p>
    */
   @Override
   default String toString(String format)
   {
      return EuclidGeometryIOTools.getBoundingBox3DString(format, this);
   }
}
