package us.ihmc.euclid.geometry.interfaces;

import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;

/**
 * Read-only interface for a line segment 3D.
 * <p>
 * A line segment 3D is a finite-length line defined in the XY-plane by its two 3D endpoints.
 * </p>
 */
public interface LineSegment3DReadOnly
{
   /**
    * Gets the read-only reference to the first endpoint of this line segment.
    *
    * @return the reference to the first endpoint of this line segment.
    */
   Point3DReadOnly getFirstEndpoint();

   /**
    * Gets the read-only reference to the second endpoint of this line segment.
    *
    * @return the reference to the second endpoint of this line segment.
    */
   Point3DReadOnly getSecondEndpoint();

   /**
    * Gets the endpoints defining this line segment by storing their coordinates in the given
    * arguments.
    *
    * @param firstEndpointToPack point in which the coordinates of this line segment's first endpoint
    *           are stored. Modified.
    * @param secondEndpointToPack point in which the coordinates of this line segment's second endpoint
    *           are stored. Modified.
    */
   default void get(Point3DBasics firstEndpointToPack, Point3DBasics secondEndpointToPack)
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
    * Gets the z-coordinate of the first endpoint defining this line segment.
    *
    * @return the first endpoint z-coordinate.
    */
   default double getFirstEndpointZ()
   {
      return getFirstEndpoint().getZ();
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
    * Gets the z-coordinate of the second endpoint defining this line segment.
    *
    * @return the first endpoint z-coordinate.
    */
   default double getSecondEndpointZ()
   {
      return getSecondEndpoint().getZ();
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
    * @return the minimum distance between the 3D point and this 3D line segment.
    */
   default double distanceSquared(Point3DReadOnly point)
   {
      return EuclidGeometryTools.distanceSquaredFromPoint3DToLineSegment3D(point, getFirstEndpoint(), getSecondEndpoint());
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
    * @return the minimum distance between the 3D point and this 3D line segment.
    */
   default double distance(Point3DReadOnly point)
   {
      return EuclidGeometryTools.distanceFromPoint3DToLineSegment3D(point, getFirstEndpoint(), getSecondEndpoint());
   }

   /**
    * This methods computes the minimum distance between this line segment and the given one.
    * <a href="http://geomalgorithms.com/a07-_distance.html"> Useful link</a>.
    *
    * @param otherLineSegment the other line segment to compute the distance from. Not modified.
    * @return the minimum distance between the two line segments.
    */
   default double distance(LineSegment3DReadOnly otherLineSegment)
   {
      return EuclidGeometryTools.distanceBetweenTwoLineSegment3Ds(getFirstEndpoint(), getSecondEndpoint(), otherLineSegment.getFirstEndpoint(),
                                                                  otherLineSegment.getSecondEndpoint());
   }

   /**
    * Computes the orthogonal projection of a 3D point on this 3D line segment.
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
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param pointToProject the point to compute the projection of. Not modified.
    * @return the projection of the point onto the line segment or {@code null} if the method failed.
    */
   default Point3DBasics orthogonalProjectionCopy(Point3DReadOnly pointToProject)
   {
      return EuclidGeometryTools.orthogonalProjectionOnLineSegment3D(pointToProject, getFirstEndpoint(), getSecondEndpoint());
   }

   /**
    * Computes the orthogonal projection of a 3D point on this 3D line segment.
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
   default boolean orthogonalProjection(Point3DBasics pointToProject)
   {
      return orthogonalProjection(pointToProject, pointToProject);
   }

   /**
    * Computes the orthogonal projection of a 3D point on this 3D line segment.
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
   default boolean orthogonalProjection(Point3DReadOnly pointToProject, Point3DBasics projectionToPack)
   {
      return EuclidGeometryTools.orthogonalProjectionOnLineSegment3D(pointToProject, getFirstEndpoint(), getSecondEndpoint(), projectionToPack);
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
   default Point3DBasics pointBetweenEndpointsGivenPercentage(double percentage)
   {
      Point3D point = new Point3D();
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
   default void pointBetweenEndpointsGivenPercentage(double percentage, Point3DBasics pointToPack)
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
   default Point3DBasics pointOnLineGivenPercentage(double percentage)
   {
      Point3D point = new Point3D();
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
   default void pointOnLineGivenPercentage(double percentage, Point3DBasics pointToPack)
   {
      pointToPack.interpolate(getFirstEndpoint(), getSecondEndpoint(), percentage);
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
   default Point3DBasics midpoint()
   {
      Point3D midpoint = new Point3D();
      midpoint(midpoint);
      return midpoint;
   }

   /**
    * Computes the coordinates of the point located exactly at the middle of this line segment.
    *
    * @param midpointToPack point in which the mid-point of this line segment is stored. Modified.
    */
   default void midpoint(Point3DBasics midpointToPack)
   {
      midpointToPack.interpolate(getFirstEndpoint(), getSecondEndpoint(), 0.5);
   }

   /**
    * Computes the vector going from the first to the second endpoint of this line segment.
    *
    * @param normalize whether the direction vector is to be normalized.
    * @param directionToPack vector in which the direction is stored. Modified.
    */
   default void getDirection(boolean normalize, Vector3DBasics directionToPack)
   {
      directionToPack.sub(getSecondEndpoint(), getFirstEndpoint());
      if (normalize)
         directionToPack.normalize();
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
   default Vector3DBasics getDirection(boolean normalize)
   {
      Vector3D direction = new Vector3D();
      getDirection(normalize, direction);
      return direction;
   }

   /**
    * Tests whether the projection of the given point onto this line segment is located between the two
    * endpoints or exactly on an endpoint.
    *
    * @param point the query. Not modified.
    * @return {@code true} if the projection of the point is between the endpoints of this line
    *         segment, {@code false} otherwise.
    */
   default boolean isBetweenEndpoints(Point3DReadOnly point)
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
   default boolean isBetweenEndpoints(Point3DReadOnly point, double epsilon)
   {
      return isBetweenEndpoints(point.getX(), point.getY(), point.getZ(), epsilon);
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
   default boolean isBetweenEndpoints(double x, double y, double z, double epsilon)
   {
      double alpha = percentageAlongLineSegment(x, y, z);

      if (alpha < epsilon)
         return false;
      if (alpha > 1.0 - epsilon)
         return false;

      return true;
   }

   /**
    * Computes a percentage along the line segment representing the location of the given point once
    * projected onto this line segment. The returned percentage is in ] -&infin;; &infin; [,
    * {@code 0.0} representing {@code firstEndpoint}, and {@code 1.0} representing
    * {@code secondEndpoint}.
    * <p>
    * For example, if the returned percentage is {@code 0.5}, it means that the projection of the given
    * point is located at the middle of this line segment. The coordinates of the projection of the
    * point can be computed from the {@code percentage} as follows: </br>
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
    * @param point the query point. Not modified.
    * @return the computed percentage along the line segment representing where the point projection is
    *         located.
    */
   default double percentageAlongLineSegment(Point3DReadOnly point)
   {
      return percentageAlongLineSegment(point.getX(), point.getY(), point.getZ());
   }

   /**
    * Computes a percentage along the line segment representing the location of the given point once
    * projected onto this line segment. The returned percentage is in ] -&infin;; &infin; [,
    * {@code 0.0} representing {@code firstEndpoint}, and {@code 1.0} representing
    * {@code secondEndpoint}.
    * <p>
    * For example, if the returned percentage is {@code 0.5}, it means that the projection of the given
    * point is located at the middle of this line segment. The coordinates of the projection of the
    * point can be computed from the {@code percentage} as follows: </br>
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
   default double percentageAlongLineSegment(double x, double y, double z)
   {
      return EuclidGeometryTools.percentageAlongLineSegment3D(x, y, z, getFirstEndpoint(), getSecondEndpoint());
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
   default double dotProduct(LineSegment3DReadOnly other)
   {
      return EuclidGeometryTools.dotProduct(getFirstEndpoint(), getSecondEndpoint(), other.getFirstEndpoint(), other.getSecondEndpoint());
   }

   /**
    * Tests on a per-component basis on both endpoints if this line segment is equal to {@code other}
    * with the tolerance {@code epsilon}.
    *
    * @param other the query. Not modified.
    * @param epsilon the tolerance to use.
    * @return {@code true} if the two line segments are equal, {@code false} otherwise.
    */
   default boolean epsilonEquals(LineSegment3DReadOnly other, double epsilon)
   {
      return getFirstEndpoint().epsilonEquals(other.getFirstEndpoint(), epsilon) && getSecondEndpoint().epsilonEquals(other.getSecondEndpoint(), epsilon);
   }

   /**
    * Tests on a per component basis, if this line segment 3D is exactly equal to {@code other}.
    *
    * @param other the other line segment 3D to compare against this. Not modified.
    * @return {@code true} if the two line segments are exactly equal component-wise, {@code false}
    *         otherwise.
    */
   default boolean equals(LineSegment3DReadOnly other)
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
   default boolean geometricallyEquals(LineSegment3DReadOnly other, double epsilon)
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
