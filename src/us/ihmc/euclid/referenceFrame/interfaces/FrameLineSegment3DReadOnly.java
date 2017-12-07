package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.geometry.interfaces.LineSegment3DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

public interface FrameLineSegment3DReadOnly extends LineSegment3DReadOnly, ReferenceFrameHolder
{
   /**
    * Gets the first endpoint defining this line segment by storing its coordinates in the given
    * argument {@code firstEndpointToPack}.
    *
    * @param firstEndpointToPack point in which the coordinates of this line segment's first
    *           endpoint are stored. Modified.
    */
   default void getFirstEndpoint(FramePoint3D firstEndpointToPack)
   {
      firstEndpointToPack.setToZero(getReferenceFrame());
      LineSegment3DReadOnly.super.getFirstEndpoint(firstEndpointToPack);
   }

   /**
    * Gets the second endpoint defining this line segment by storing its coordinates in the given
    * argument {@code secondEndpointToPack}.
    *
    * @param secondEndpointToPack point in which the coordinates of this line segment's second
    *           endpoint are stored. Modified.
    */
   default void getSecondEndpoint(FramePoint3D secondEndpointToPack)
   {
      secondEndpointToPack.setToZero(getReferenceFrame());
      LineSegment3DReadOnly.super.getSecondEndpoint(secondEndpointToPack);
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
   default void getEndpoints(FramePoint3D firstEndpointToPack, Point3DBasics secondEndpointToPack)
   {
      firstEndpointToPack.setToZero(getReferenceFrame());
      LineSegment3DReadOnly.super.getEndpoints(firstEndpointToPack, secondEndpointToPack);
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
   default void getEndpoints(Point3DBasics firstEndpointToPack, FramePoint3D secondEndpointToPack)
   {
      secondEndpointToPack.setToZero(getReferenceFrame());
      LineSegment3DReadOnly.super.getEndpoints(firstEndpointToPack, secondEndpointToPack);
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
   default void getEndpoints(FramePoint3D firstEndpointToPack, FramePoint3D secondEndpointToPack)
   {
      firstEndpointToPack.setToZero(getReferenceFrame());
      secondEndpointToPack.setToZero(getReferenceFrame());
      LineSegment3DReadOnly.super.getEndpoints(firstEndpointToPack, secondEndpointToPack);
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
   default double distanceSquared(FramePoint3DReadOnly point)
   {
      checkReferenceFrameMatch(point);
      return LineSegment3DReadOnly.super.distanceSquared(point);
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
   default double distance(FramePoint3DReadOnly point)
   {
      checkReferenceFrameMatch(point);
      return LineSegment3DReadOnly.super.distance(point);
   }

   /**
    * This methods computes the minimum distance between this line segment and the given one.
    * <a href="http://geomalgorithms.com/a07-_distance.html"> Useful link</a>.
    *
    * @param otherLineSegment the other line segment to compute the distance from. Not modified.
    * @return the minimum distance between the two line segments.
    */
   default double distance(FrameLineSegment3DReadOnly otherLineSegment)
   {
      checkReferenceFrameMatch(otherLineSegment);
      return LineSegment3DReadOnly.super.distance(otherLineSegment);
   }

   /**
    * Computes the orthogonal projection of a 3D point on this 3D line segment.
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
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param pointToProject the point to compute the projection of. Not modified.
    * @return the projection of the point onto the line segment or {@code null} if the method
    *         failed.
    */
   default Point3D orthogonalProjectionCopy(FramePoint3DReadOnly pointToProject)
   {
      checkReferenceFrameMatch(pointToProject);
      return LineSegment3DReadOnly.super.orthogonalProjectionCopy(pointToProject);
   }

   /**
    * Computes the orthogonal projection of a 3D point on this 3D line segment.
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
    */
   default boolean orthogonalProjection(FramePoint3D pointToProject)
   {
      checkReferenceFrameMatch(pointToProject);
      return LineSegment3DReadOnly.super.orthogonalProjection(pointToProject);
   }

   /**
    * Computes the orthogonal projection of a 3D point on this 3D line segment.
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
   default boolean orthogonalProjection(FramePoint3DReadOnly pointToProject, Point3DBasics projectionToPack)
   {
      checkReferenceFrameMatch(pointToProject);
      return LineSegment3DReadOnly.super.orthogonalProjection(pointToProject, projectionToPack);
   }

   /**
    * Computes the orthogonal projection of a 3D point on this 3D line segment.
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
   default boolean orthogonalProjection(Point3DReadOnly pointToProject, FramePoint3D projectionToPack)
   {
      projectionToPack.setIncludingFrame(getReferenceFrame(), projectionToPack);
      return LineSegment3DReadOnly.super.orthogonalProjection(pointToProject, projectionToPack);
   }

   /**
    * Computes the orthogonal projection of a 3D point on this 3D line segment.
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
   default boolean orthogonalProjection(FramePoint3DReadOnly pointToProject, FramePoint3D projectionToPack)
   {
      checkReferenceFrameMatch(pointToProject);
      projectionToPack.setToZero(getReferenceFrame());
      return LineSegment3DReadOnly.super.orthogonalProjection(pointToProject, projectionToPack);
   }

   /**
    * Tests whether the projection of the given point onto this line segment is located between the
    * two endpoints or exactly on an endpoint.
    *
    * @param point the query. Not modified.
    * @return {@code true} if the projection of the point is between the endpoints of this line
    *         segment, {@code false} otherwise.
    */
   default boolean isBetweenEndpoints(FramePoint3DReadOnly point)
   {
      checkReferenceFrameMatch(point);
      return LineSegment3DReadOnly.super.isBetweenEndpoints(point);
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
    */
   default boolean isBetweenEndpoints(FramePoint3DReadOnly point, double epsilon)
   {
      checkReferenceFrameMatch(point);
      return LineSegment3DReadOnly.super.isBetweenEndpoints(point, epsilon);
   }

   /**
    * Computes a percentage along the line segment representing the location of the given point once
    * projected onto this line segment. The returned percentage is in ] -&infin;; &infin; [,
    * {@code 0.0} representing {@code firstEndpoint}, and {@code 1.0} representing
    * {@code secondEndpoint}.
    * <p>
    * For example, if the returned percentage is {@code 0.5}, it means that the projection of the
    * given point is located at the middle of this line segment. The coordinates of the projection
    * of the point can be computed from the {@code percentage} as follows: </br><code>
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
    */
   default double percentageAlongLineSegment(FramePoint3DReadOnly point)
   {
      checkReferenceFrameMatch(point);
      return LineSegment3DReadOnly.super.percentageAlongLineSegment(point);
   }

   /**
    * Computes the coordinates of the point located at a given percentage on this line segment: <br>
    * {@code pointToPack.interpolate(firstEndpoint, secondEndpoint, percentage)} </br>
    *
    * @param percentage the percentage along this line segment of the point. Must be in [0, 1].
    * @param pointToPack where the result is stored. Modified.
    * @throws {@link RuntimeException} if {@code percentage} &notin; [0, 1].
    */
   default void pointBetweenEndpointsGivenPercentage(double percentage, FramePoint3D pointToPack)
   {
      checkReferenceFrameMatch(pointToPack);
      LineSegment3DReadOnly.super.pointBetweenEndpointsGivenPercentage(percentage, pointToPack);
   }

   /**
    * Computes the coordinates of the point located on the line this line segment is lying on: <br>
    * {@code pointToPack.interpolate(firstEndpoint, secondEndpoint, percentage)} </br>
    *
    * @param percentage the percentage along this line segment of the point.
    * @param pointToPack where the result is stored. Modified.
    */
   default void pointOnLineGivenPercentage(double percentage, FramePoint3D pointToPack)
   {
      pointToPack.setToZero(getReferenceFrame());
      LineSegment3DReadOnly.super.pointOnLineGivenPercentage(percentage, pointToPack);
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
   default double dotProduct(FrameLineSegment3DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      return LineSegment3DReadOnly.super.dotProduct(other);
   }

   /**
    * Computes the coordinates of the point located exactly at the middle of this line segment.
    *
    * @param midpointToPack point in which the mid-point of this line segment is stored. Modified.
    */
   default void midpoint(FramePoint3D midpointToPack)
   {
      midpointToPack.setToZero(getReferenceFrame());
      LineSegment3DReadOnly.super.midpoint(midpointToPack);
   }

   /**
    * Computes the vector going from the first to the second endpoint of this line segment.
    *
    * @param normalize whether the direction vector is to be normalized.
    * @param directionToPack vector in which the direction is stored. Modified.
    */
   default void getDirection(boolean normalize, FrameVector3D directionToPack)
   {
      directionToPack.setToZero(getReferenceFrame());
      LineSegment3DReadOnly.super.getDirection(normalize, directionToPack);
   }
}
