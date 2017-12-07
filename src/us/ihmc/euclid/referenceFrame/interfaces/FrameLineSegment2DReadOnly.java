package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Line2D;
import us.ihmc.euclid.geometry.exceptions.OutdatedPolygonException;
import us.ihmc.euclid.geometry.interfaces.LineSegment2DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;

public interface FrameLineSegment2DReadOnly extends LineSegment2DReadOnly, ReferenceFrameHolder
{
   /**
    * Gets the read-only reference to the first endpoint of this line segment.
    *
    * @return the reference to the first endpoint of this line segment.
    */
   FramePoint2DReadOnly getFirstEndpoint();

   /**
    * Gets the read-only reference to the second endpoint of this line segment.
    *
    * @return the reference to the second endpoint of this line segment.
    */
   FramePoint2DReadOnly getSecondEndpoint();

   /**
    * Gets the first endpoint defining this line segment by storing its coordinates in the given
    * argument {@code firstEndpointToPack}.
    *
    * @param firstEndpointToPack point in which the coordinates of this line segment's first
    *           endpoint are stored. Modified.
    */
   default void getFirstEndpoint(FramePoint2D firstEndpointToPack)
   {
      firstEndpointToPack.setToZero(getReferenceFrame());
      LineSegment2DReadOnly.super.getFirstEndpoint(firstEndpointToPack);
   }

   /**
    * Gets the second endpoint defining this line segment by storing its coordinates in the given
    * argument {@code secondEndpointToPack}.
    *
    * @param secondEndpointToPack point in which the coordinates of this line segment's second
    *           endpoint are stored. Modified.
    */
   default void getSecondEndpoint(FramePoint2D secondEndpointToPack)
   {
      secondEndpointToPack.setToZero(getReferenceFrame());
      LineSegment2DReadOnly.super.getSecondEndpoint(secondEndpointToPack);
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
   default void getEndpoints(FramePoint2D firstEndpointToPack, Point2DBasics secondEndpointToPack)
   {
      firstEndpointToPack.setToZero(getReferenceFrame());
      LineSegment2DReadOnly.super.getEndpoints(firstEndpointToPack, secondEndpointToPack);
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
   default void getEndpoints(Point2DBasics firstEndpointToPack, FramePoint2D secondEndpointToPack)
   {
      secondEndpointToPack.setToZero(getReferenceFrame());
      LineSegment2DReadOnly.super.getEndpoints(firstEndpointToPack, secondEndpointToPack);
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
   default void getEndpoints(FramePoint2D firstEndpointToPack, FramePoint2D secondEndpointToPack)
   {
      firstEndpointToPack.setToZero(getReferenceFrame());
      secondEndpointToPack.setToZero(getReferenceFrame());
      LineSegment2DReadOnly.super.getEndpoints(firstEndpointToPack, secondEndpointToPack);
   }
   
   /**
    * Computes the coordinates of the point located exactly at the middle of this line segment.
    *
    * @param midpointToPack point in which the mid-point of this line segment is stored. Modified.
    */
   default void midpoint(FramePoint2D midpointToPack)
   {
      midpointToPack.setToZero(getReferenceFrame());
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
    * @throws ReferenceFrameMismatchException if {@code this} and {@code point} are not expressed
    *          in the same reference frame.
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
    * @throws ReferenceFrameMismatchException if {@code this} and {@code point} are not expressed
    *          in the same reference frame.
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
    * @throws ReferenceFrameMismatchException if {@code this} and {@code point} are not expressed
    *          in the same reference frame.
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
    * @throws ReferenceFrameMismatchException if {@code this} and {@code point} are not expressed
    *          in the same reference frame.
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
    * @throws ReferenceFrameMismatchException if {@code this} and {@code point} are not expressed
    *          in the same reference frame.
    */
   default boolean isPointOnLeftSideOfLineSegment(FramePoint2DReadOnly point)
   {
      checkReferenceFrameMatch(point);
      return LineSegment2DReadOnly.super.isPointOnLeftSideOfLineSegment(point);
   }

   /**
    * Computes the vector going from the first to the second endpoint of this line segment.
    *
    * @param normalize whether the direction vector is to be normalized.
    * @param directionToPack vector in which the direction is stored. Modified.
    */
   default void direction(boolean normalize, FrameVector2D directionToPack)
   {
      directionToPack.setToZero(getReferenceFrame());
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
    * @throws ReferenceFrameMismatchException if {@code this} and {@code point} are not expressed
    *          in the same reference frame.
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
    * @throws ReferenceFrameMismatchException if {@code this} and {@code point} are not expressed
    *          in the same reference frame.
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
    * @throws ReferenceFrameMismatchException if {@code this} and {@code point} are not expressed
    *          in the same reference frame.
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
    * @throws ReferenceFrameMismatchException if {@code this} and {@code point} are not expressed
    *          in the same reference frame.
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
    * @throws ReferenceFrameMismatchException if {@code this} and {@code other} are not expressed
    *          in the same reference frame.
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
    */
   default boolean orthogonalProjection(FramePoint2D pointToProject)
   {
      return orthogonalProjection(new FramePoint2D(pointToProject), pointToProject);
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
    * @throws ReferenceFrameMismatchException if {@code this} and {@code pointToProject} are not expressed
    *          in the same reference frame.
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
    * @throws ReferenceFrameMismatchException if {@code this} and {@code pointToProject} are not expressed
    *          in the same reference frame.
    */
   default boolean orthogonalProjection(Point2DReadOnly pointToProject, FramePoint2D projectionToPack)
   {
      projectionToPack.setToZero(getReferenceFrame());
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
    * @throws ReferenceFrameMismatchException if {@code this} and {@code pointToProject} are not expressed
    *          in the same reference frame.
    */
   default boolean orthogonalProjection(FramePoint2DReadOnly pointToProject, FramePoint2D projectionToPack)
   {
      checkReferenceFrameMatch(pointToProject);
      projectionToPack.setToZero(getReferenceFrame());
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
    * @return the projection of the point onto this line segment or {@code null} if the method
    *         failed.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code pointToProject} are not expressed
    *          in the same reference frame.
    */
   default FramePoint2D orthogonalProjectionCopy(FramePoint2DReadOnly pointToProject)
   {
      checkReferenceFrameMatch(pointToProject);
      return new FramePoint2D(getReferenceFrame(), LineSegment2DReadOnly.super.orthogonalProjectionCopy(pointToProject));
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
   default boolean intersectionWith(Line2D line, FramePoint2D intersectionToPack)
   {
      intersectionToPack.setIncludingFrame(getReferenceFrame(), intersectionToPack);
      return LineSegment2DReadOnly.super.intersectionWith(line, intersectionToPack);
   }

   /**
    * Computes the vector perpendicular to the direction of this line segment.
    *
    * @param normalize whether the perpendicular vector is to be normalized.
    * @param perpendicularVectorToPack vector in which the perpendicular vector components are
    *           stored. Modified.
    */
   default void perpendicular(boolean normalize, FrameVector2D perpendicularVectorToPack)
   {
      perpendicularVectorToPack.setIncludingFrame(getReferenceFrame(), perpendicularVectorToPack);
      LineSegment2DReadOnly.super.perpendicular(normalize, perpendicularVectorToPack);
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
    * @throws ReferenceFrameMismatchException if {@code this} and {@code other} are not expressed
    *          in the same reference frame.
    */
   default Point2D intersectionWith(FrameLineSegment2DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      return LineSegment2DReadOnly.super.intersectionWith(other);
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
    * @throws ReferenceFrameMismatchException if {@code this} and {@code other} are not expressed
    *          in the same reference frame.
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
    * @throws ReferenceFrameMismatchException if {@code this} and {@code other} are not expressed
    *          in the same reference frame.
    */
   default boolean intersectionWith(LineSegment2DReadOnly other, FramePoint2D intersectionToPack)
   {
      intersectionToPack.setIncludingFrame(getReferenceFrame(), intersectionToPack);
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
    * @throws ReferenceFrameMismatchException if {@code this} and {@code other} are not expressed
    *          in the same reference frame.
    */
   default boolean intersectionWith(FrameLineSegment2DReadOnly other, FramePoint2D intersectionToPack)
   {
      checkReferenceFrameMatch(other);
      intersectionToPack.setIncludingFrame(getReferenceFrame(), intersectionToPack);
      return LineSegment2DReadOnly.super.intersectionWith(other, intersectionToPack);
   }

   /**
    * Computes the coordinates of the point located at a given percentage on this line segment: <br>
    * {@code pointToPack.interpolate(firstEndpoint, secondEndpoint, percentage)} </br>
    *
    * @param percentage the percentage along this line segment of the point. Must be in [0, 1].
    * @param pointToPack where the result is stored. Modified.
    * @throws {@link RuntimeException} if {@code percentage} &notin; [0, 1].
    */
   default void pointBetweenEndpointsGivenPercentage(double percentage, FramePoint2D pointToPack)
   {
      pointToPack.setToZero(getReferenceFrame());
      LineSegment2DReadOnly.super.pointBetweenEndpointsGivenPercentage(percentage, pointToPack);
   }

   /**
    * Computes the coordinates of the possible intersection(s) between this line segment and the
    * given convex polygon 2D.
    * <p>
    * Edge cases:
    * <ul>
    * <li>If the polygon has no vertices, this method behaves as if there is no intersections.
    * <li>If no intersections exist, this method returns {@code 0} and the two intersection-to-pack
    * arguments remain unmodified.
    * <li>If there is only one intersection, this method returns {@code 1} and the coordinates of
    * the only intersection are stored in {@code firstIntersectionToPack}.
    * {@code secondIntersectionToPack} remains unmodified.
    * <li>If this line segment is collinear to an edge:
    * <ul>
    * <li>The edge entirely contains this line segment: this method finds two intersections which
    * are the endpoints of this line segment.
    * <li>This line segment entirely contains the edge: this method finds two intersections which
    * are the vertices of the edge.
    * <li>The edge and this line segment partially overlap: this method finds two intersections
    * which the polygon's vertex that on this line segment and this line segment's endpoint that is
    * on the polygon's edge.
    * </ul>
    * </ul>
    * </p>
    *
    * @param convexPolygon the convex polygon this line segment may intersect. Not modified.
    * @param firstIntersectionToPack point in which the coordinates of the first intersection. Can
    *           be {@code null}. Modified.
    * @param secondIntersectionToPack point in which the coordinates of the second intersection. Can
    *           be {@code null}. Modified.
    * @return the number of intersections between this line segment and the polygon.
    * @throws OutdatedPolygonException if the convex polygon is not up-to-date.
    */
   default int intersectionWith(ConvexPolygon2D convexPolygon, FramePoint2D firstIntersectionToPack, Point2DBasics secondIntersectionToPack)
   {
      firstIntersectionToPack.setIncludingFrame(getReferenceFrame(), firstIntersectionToPack);
      return LineSegment2DReadOnly.super.intersectionWith(convexPolygon, firstIntersectionToPack, secondIntersectionToPack);
   }

   /**
    * Computes the coordinates of the possible intersection(s) between this line segment and the
    * given convex polygon 2D.
    * <p>
    * Edge cases:
    * <ul>
    * <li>If the polygon has no vertices, this method behaves as if there is no intersections.
    * <li>If no intersections exist, this method returns {@code 0} and the two intersection-to-pack
    * arguments remain unmodified.
    * <li>If there is only one intersection, this method returns {@code 1} and the coordinates of
    * the only intersection are stored in {@code firstIntersectionToPack}.
    * {@code secondIntersectionToPack} remains unmodified.
    * <li>If this line segment is collinear to an edge:
    * <ul>
    * <li>The edge entirely contains this line segment: this method finds two intersections which
    * are the endpoints of this line segment.
    * <li>This line segment entirely contains the edge: this method finds two intersections which
    * are the vertices of the edge.
    * <li>The edge and this line segment partially overlap: this method finds two intersections
    * which the polygon's vertex that on this line segment and this line segment's endpoint that is
    * on the polygon's edge.
    * </ul>
    * </ul>
    * </p>
    *
    * @param convexPolygon the convex polygon this line segment may intersect. Not modified.
    * @param firstIntersectionToPack point in which the coordinates of the first intersection. Can
    *           be {@code null}. Modified.
    * @param secondIntersectionToPack point in which the coordinates of the second intersection. Can
    *           be {@code null}. Modified.
    * @return the number of intersections between this line segment and the polygon.
    * @throws OutdatedPolygonException if the convex polygon is not up-to-date.
    */
   default int intersectionWith(ConvexPolygon2D convexPolygon, Point2DBasics firstIntersectionToPack, FramePoint2D secondIntersectionToPack)
   {
      secondIntersectionToPack.setIncludingFrame(getReferenceFrame(), secondIntersectionToPack);
      return LineSegment2DReadOnly.super.intersectionWith(convexPolygon, firstIntersectionToPack, secondIntersectionToPack);
   }

   /**
    * Computes the coordinates of the possible intersection(s) between this line segment and the
    * given convex polygon 2D.
    * <p>
    * Edge cases:
    * <ul>
    * <li>If the polygon has no vertices, this method behaves as if there is no intersections.
    * <li>If no intersections exist, this method returns {@code 0} and the two intersection-to-pack
    * arguments remain unmodified.
    * <li>If there is only one intersection, this method returns {@code 1} and the coordinates of
    * the only intersection are stored in {@code firstIntersectionToPack}.
    * {@code secondIntersectionToPack} remains unmodified.
    * <li>If this line segment is collinear to an edge:
    * <ul>
    * <li>The edge entirely contains this line segment: this method finds two intersections which
    * are the endpoints of this line segment.
    * <li>This line segment entirely contains the edge: this method finds two intersections which
    * are the vertices of the edge.
    * <li>The edge and this line segment partially overlap: this method finds two intersections
    * which the polygon's vertex that on this line segment and this line segment's endpoint that is
    * on the polygon's edge.
    * </ul>
    * </ul>
    * </p>
    *
    * @param convexPolygon the convex polygon this line segment may intersect. Not modified.
    * @param firstIntersectionToPack point in which the coordinates of the first intersection. Can
    *           be {@code null}. Modified.
    * @param secondIntersectionToPack point in which the coordinates of the second intersection. Can
    *           be {@code null}. Modified.
    * @return the number of intersections between this line segment and the polygon.
    * @throws OutdatedPolygonException if the convex polygon is not up-to-date.
    */
   default int intersectionWith(ConvexPolygon2D convexPolygon, FramePoint2D firstIntersectionToPack, FramePoint2D secondIntersectionToPack)
   {
      firstIntersectionToPack.setIncludingFrame(getReferenceFrame(), firstIntersectionToPack);
      secondIntersectionToPack.setIncludingFrame(getReferenceFrame(), secondIntersectionToPack);
      return LineSegment2DReadOnly.super.intersectionWith(convexPolygon, firstIntersectionToPack, secondIntersectionToPack);
   }

   /**
    * Computes the coordinates of the point located on the line this line segment is lying on: <br>
    * {@code pointToPack.interpolate(firstEndpoint, secondEndpoint, percentage)} </br>
    *
    * @param percentage the percentage along this line segment of the point.
    * @param pointToPack where the result is stored. Modified.
    */
   default void pointOnLineGivenPercentage(double percentage, FramePoint2D pointToPack)
   {
      pointToPack.setToZero(getReferenceFrame());
      LineSegment2DReadOnly.super.pointOnLineGivenPercentage(percentage, pointToPack);
   }
}
