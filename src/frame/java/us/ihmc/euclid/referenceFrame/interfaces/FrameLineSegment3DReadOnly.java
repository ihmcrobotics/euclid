package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.geometry.interfaces.LineSegment3DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

/**
 * Read-only interface for a line segment 3D expressed in a given reference frame, i.e. this line
 * segment is always expressed in the same reference frame.
 * <p>
 * A line segment 3D is a finite-length line defined in the XY-plane by its two 3D endpoints.
 * </p>
 * <p>
 * In addition to representing a {@link LineSegment3DReadOnly}, a {@link ReferenceFrame} is
 * associated to a {@code FrameLineSegment3DReadOnly}. This allows, for instance, to enforce, at
 * runtime, that operations on lines occur in the same coordinate system.
 * </p>
 * <p>
 * Because a {@code FrameLineSegment3DReadOnly} extends {@code LineSegment3DReadOnly}, it is
 * compatible with methods only requiring {@code LineSegment3DReadOnly}. However, these methods do
 * NOT assert that the operation occur in the proper coordinate system. Use this feature carefully
 * and always prefer using methods requiring {@code FrameLineSegment3DReadOnly}.
 * </p>
 */
public interface FrameLineSegment3DReadOnly extends LineSegment3DReadOnly, ReferenceFrameHolder
{
   /**
    * Gets the read-only reference to the first endpoint of this line segment.
    *
    * @return the reference to the first endpoint of this line segment.
    */
   @Override
   FramePoint3DReadOnly getFirstEndpoint();

   /**
    * Gets the read-only reference to the second endpoint of this line segment.
    *
    * @return the reference to the second endpoint of this line segment.
    */
   @Override
   FramePoint3DReadOnly getSecondEndpoint();

   /**
    * Gets the endpoints defining this line segment by storing their coordinates in the given
    * arguments.
    *
    * @param firstEndpointToPack point in which the coordinates of this line segment's first endpoint
    *           are stored. Modified.
    * @param secondEndpointToPack point in which the coordinates of this line segment's second endpoint
    *           are stored. Modified.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code firstEndpointToPack} are not
    *            expressed in the same reference frame.
    */
   default void get(FixedFramePoint3DBasics firstEndpointToPack, Point3DBasics secondEndpointToPack)
   {
      checkReferenceFrameMatch(firstEndpointToPack);
      LineSegment3DReadOnly.super.get(firstEndpointToPack, secondEndpointToPack);
   }

   /**
    * Gets the endpoints defining this line segment by storing their coordinates in the given
    * arguments.
    *
    * @param firstEndpointToPack point in which the coordinates of this line segment's first endpoint
    *           are stored. Modified.
    * @param secondEndpointToPack point in which the coordinates of this line segment's second endpoint
    *           are stored. Modified.
    */
   default void get(FramePoint3DBasics firstEndpointToPack, Point3DBasics secondEndpointToPack)
   {
      firstEndpointToPack.setReferenceFrame(getReferenceFrame());
      LineSegment3DReadOnly.super.get(firstEndpointToPack, secondEndpointToPack);
   }

   /**
    * Gets the endpoints defining this line segment by storing their coordinates in the given
    * arguments.
    *
    * @param firstEndpointToPack point in which the coordinates of this line segment's first endpoint
    *           are stored. Modified.
    * @param secondEndpointToPack point in which the coordinates of this line segment's second endpoint
    *           are stored. Modified.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code secondEndpointToPack} are not
    *            expressed in the same reference frame.
    */
   default void get(Point3DBasics firstEndpointToPack, FixedFramePoint3DBasics secondEndpointToPack)
   {
      checkReferenceFrameMatch(secondEndpointToPack);
      LineSegment3DReadOnly.super.get(firstEndpointToPack, secondEndpointToPack);
   }

   /**
    * Gets the endpoints defining this line segment by storing their coordinates in the given
    * arguments.
    *
    * @param firstEndpointToPack point in which the coordinates of this line segment's first endpoint
    *           are stored. Modified.
    * @param secondEndpointToPack point in which the coordinates of this line segment's second endpoint
    *           are stored. Modified.
    */
   default void get(Point3DBasics firstEndpointToPack, FramePoint3DBasics secondEndpointToPack)
   {
      secondEndpointToPack.setReferenceFrame(getReferenceFrame());
      LineSegment3DReadOnly.super.get(firstEndpointToPack, secondEndpointToPack);
   }

   /**
    * Gets the endpoints defining this line segment by storing their coordinates in the given
    * arguments.
    *
    * @param firstEndpointToPack point in which the coordinates of this line segment's first endpoint
    *           are stored. Modified.
    * @param secondEndpointToPack point in which the coordinates of this line segment's second endpoint
    *           are stored. Modified.
    * @throws ReferenceFrameMismatchException if {@code this}, {@code firstEndpointToPack}, and
    *            {@code secondEndpointToPack} are not expressed in the same reference frame.
    */
   default void get(FixedFramePoint3DBasics firstEndpointToPack, FixedFramePoint3DBasics secondEndpointToPack)
   {
      checkReferenceFrameMatch(firstEndpointToPack);
      checkReferenceFrameMatch(secondEndpointToPack);
      LineSegment3DReadOnly.super.get(firstEndpointToPack, secondEndpointToPack);
   }

   /**
    * Gets the endpoints defining this line segment by storing their coordinates in the given
    * arguments.
    *
    * @param firstEndpointToPack point in which the coordinates of this line segment's first endpoint
    *           are stored. Modified.
    * @param secondEndpointToPack point in which the coordinates of this line segment's second endpoint
    *           are stored. Modified.
    */
   default void get(FramePoint3DBasics firstEndpointToPack, FramePoint3DBasics secondEndpointToPack)
   {
      firstEndpointToPack.setReferenceFrame(getReferenceFrame());
      secondEndpointToPack.setReferenceFrame(getReferenceFrame());
      LineSegment3DReadOnly.super.get(firstEndpointToPack, secondEndpointToPack);
   }

   /**
    * Computes the vector going from the first to the second endpoint of this line segment.
    *
    * @param normalize whether the direction vector is to be normalized.
    * @param directionToPack vector in which the direction is stored. Modified.
    */
   @Override
   default FrameVector3DBasics getDirection(boolean normalize)
   {
      return new FrameVector3D(getReferenceFrame(), LineSegment3DReadOnly.super.getDirection(normalize));
   }

   /**
    * Computes the vector going from the first to the second endpoint of this line segment.
    *
    * @param normalize whether the direction vector is to be normalized.
    * @param directionToPack vector in which the direction is stored. Modified.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code directionToPack} are not
    *            expressed in the same reference frame.
    */
   default void getDirection(boolean normalize, FixedFrameVector3DBasics directionToPack)
   {
      checkReferenceFrameMatch(directionToPack);
      LineSegment3DReadOnly.super.getDirection(normalize, directionToPack);
   }

   /**
    * Computes the vector going from the first to the second endpoint of this line segment.
    *
    * @param normalize whether the direction vector is to be normalized.
    * @param directionToPack vector in which the direction is stored. Modified.
    */
   default void getDirection(boolean normalize, FrameVector3DBasics directionToPack)
   {
      directionToPack.setReferenceFrame(getReferenceFrame());
      LineSegment3DReadOnly.super.getDirection(normalize, directionToPack);
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
    * @throws ReferenceFrameMismatchException if {@code this} and {@code point} are not expressed in
    *            the same reference frame.
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
    * @throws ReferenceFrameMismatchException if {@code this} and {@code point} are not expressed in
    *            the same reference frame.
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
    * @param frameLineSegment3DReadOnly the other line segment to compute the distance from. Not
    *           modified.
    * @return the minimum distance between the two line segments.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code frameLineSegment3DReadOnly}
    *            are not expressed in the same reference frame.
    */
   default double distance(FrameLineSegment3DReadOnly frameLineSegment3DReadOnly)
   {
      checkReferenceFrameMatch(frameLineSegment3DReadOnly);
      return LineSegment3DReadOnly.super.distance(frameLineSegment3DReadOnly);
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
   @Override
   default FramePoint3DBasics orthogonalProjectionCopy(Point3DReadOnly pointToProject)
   {
      Point3DBasics projection = LineSegment3DReadOnly.super.orthogonalProjectionCopy(pointToProject);
      if (projection == null)
         return null;
      else
         return new FramePoint3D(getReferenceFrame(), projection);
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
    * @throws ReferenceFrameMismatchException if {@code this} and {@code pointToProject} are not
    *            expressed in the same reference frame.
    */
   default FramePoint3DBasics orthogonalProjectionCopy(FramePoint3DReadOnly pointToProject)
   {
      checkReferenceFrameMatch(pointToProject);

      Point3DBasics projection = LineSegment3DReadOnly.super.orthogonalProjectionCopy(pointToProject);
      if (projection == null)
         return null;
      else
         return new FramePoint3D(getReferenceFrame(), projection);
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
    * @throws ReferenceFrameMismatchException if {@code this} and {@code pointToProject} are not
    *            expressed in the same reference frame.
    */
   default boolean orthogonalProjection(FixedFramePoint3DBasics pointToProject)
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
    * <li>the projection can not be outside the line segment. When the projection on the corresponding
    * line is outside the line segment, the result is the closest of the two endpoints.
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
    * <li>the projection can not be outside the line segment. When the projection on the corresponding
    * line is outside the line segment, the result is the closest of the two endpoints.
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
   default boolean orthogonalProjection(Point3DReadOnly pointToProject, FixedFramePoint3DBasics projectionToPack)
   {
      checkReferenceFrameMatch(projectionToPack);
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
   default boolean orthogonalProjection(Point3DReadOnly pointToProject, FramePoint3DBasics projectionToPack)
   {
      projectionToPack.setReferenceFrame(getReferenceFrame());
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
    * <li>the projection can not be outside the line segment. When the projection on the corresponding
    * line is outside the line segment, the result is the closest of the two endpoints.
    * </ul>
    * </p>
    *
    * @param pointToProject the point to compute the projection of. Not modified.
    * @param projectionToPack point in which the projection of the point onto this line segment is
    *           stored. Modified.
    * @return whether the method succeeded or not.
    * @throws ReferenceFrameMismatchException if {@code this}, {@code pointToProject} and
    *            {@code projectionToPack} are not expressed in the same reference frame.
    */
   default boolean orthogonalProjection(FramePoint3DReadOnly pointToProject, FixedFramePoint3DBasics projectionToPack)
   {
      checkReferenceFrameMatch(pointToProject);
      checkReferenceFrameMatch(projectionToPack);
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
    * <li>the projection can not be outside the line segment. When the projection on the corresponding
    * line is outside the line segment, the result is the closest of the two endpoints.
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
   default boolean orthogonalProjection(FramePoint3DReadOnly pointToProject, FramePoint3DBasics projectionToPack)
   {
      checkReferenceFrameMatch(pointToProject);
      projectionToPack.setReferenceFrame(getReferenceFrame());
      return LineSegment3DReadOnly.super.orthogonalProjection(pointToProject, projectionToPack);
   }

   /**
    * Tests whether the projection of the given point onto this line segment is located between the two
    * endpoints or exactly on an endpoint.
    *
    * @param point the query. Not modified.
    * @return {@code true} if the projection of the point is between the endpoints of this line
    *         segment, {@code false} otherwise.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code point} are not expressed in
    *            the same reference frame.
    */
   default boolean isBetweenEndpoints(FramePoint3DReadOnly point)
   {
      checkReferenceFrameMatch(point);
      return LineSegment3DReadOnly.super.isBetweenEndpoints(point);
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
    * @throws ReferenceFrameMismatchException if {@code this} and {@code point} are not expressed in
    *            the same reference frame.
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
    * @throws ReferenceFrameMismatchException if {@code this} and {@code point} are not expressed in
    *            the same reference frame.
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
    * @throws ReferenceFrameMismatchException if {@code this} and {@code pointToPack} are not expressed
    *            in the same reference frame.
    */
   @Override
   default FramePoint3DBasics pointBetweenEndpointsGivenPercentage(double percentage)
   {
      return new FramePoint3D(getReferenceFrame(), LineSegment3DReadOnly.super.pointBetweenEndpointsGivenPercentage(percentage));
   }

   /**
    * Computes the coordinates of the point located at a given percentage on this line segment: <br>
    * {@code pointToPack.interpolate(firstEndpoint, secondEndpoint, percentage)} </br>
    *
    * @param percentage the percentage along this line segment of the point. Must be in [0, 1].
    * @param pointToPack where the result is stored. Modified.
    * @throws {@link RuntimeException} if {@code percentage} &notin; [0, 1].
    * @throws ReferenceFrameMismatchException if {@code this} and {@code pointToPack} are not expressed
    *            in the same reference frame.
    */
   default void pointBetweenEndpointsGivenPercentage(double percentage, FixedFramePoint3DBasics pointToPack)
   {
      checkReferenceFrameMatch(pointToPack);
      LineSegment3DReadOnly.super.pointBetweenEndpointsGivenPercentage(percentage, pointToPack);
   }

   /**
    * Computes the coordinates of the point located at a given percentage on this line segment: <br>
    * {@code pointToPack.interpolate(firstEndpoint, secondEndpoint, percentage)} </br>
    *
    * @param percentage the percentage along this line segment of the point. Must be in [0, 1].
    * @param pointToPack where the result is stored. Modified.
    * @throws {@link RuntimeException} if {@code percentage} &notin; [0, 1].
    */
   default void pointBetweenEndpointsGivenPercentage(double percentage, FramePoint3DBasics pointToPack)
   {
      pointToPack.setReferenceFrame(getReferenceFrame());
      LineSegment3DReadOnly.super.pointBetweenEndpointsGivenPercentage(percentage, pointToPack);
   }

   /**
    * Computes the coordinates of the point located on the line this line segment is lying on: <br>
    * {@code pointToPack.interpolate(firstEndpoint, secondEndpoint, percentage)} </br>
    *
    * @param percentage the percentage along this line segment of the point.
    * @param pointToPack where the result is stored. Modified.
    */
   @Override
   default FramePoint3DBasics pointOnLineGivenPercentage(double percentage)
   {
      return new FramePoint3D(getReferenceFrame(), LineSegment3DReadOnly.super.pointOnLineGivenPercentage(percentage));
   }

   /**
    * Computes the coordinates of the point located on the line this line segment is lying on: <br>
    * {@code pointToPack.interpolate(firstEndpoint, secondEndpoint, percentage)} </br>
    *
    * @param percentage the percentage along this line segment of the point.
    * @param pointToPack where the result is stored. Modified.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code pointToPack} are not expressed
    *            in the same reference frame.
    */
   default void pointOnLineGivenPercentage(double percentage, FixedFramePoint3DBasics pointToPack)
   {
      checkReferenceFrameMatch(pointToPack);
      LineSegment3DReadOnly.super.pointOnLineGivenPercentage(percentage, pointToPack);
   }

   /**
    * Computes the coordinates of the point located on the line this line segment is lying on: <br>
    * {@code pointToPack.interpolate(firstEndpoint, secondEndpoint, percentage)} </br>
    *
    * @param percentage the percentage along this line segment of the point.
    * @param pointToPack where the result is stored. Modified.
    */
   default void pointOnLineGivenPercentage(double percentage, FramePoint3DBasics pointToPack)
   {
      pointToPack.setReferenceFrame(getReferenceFrame());
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
    * @throws ReferenceFrameMismatchException if {@code this} and {@code other} are not expressed in
    *            the same reference frame.
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
   @Override
   default FramePoint3DBasics midpoint()
   {
      return new FramePoint3D(getReferenceFrame(), LineSegment3DReadOnly.super.midpoint());
   }

   /**
    * Computes the coordinates of the point located exactly at the middle of this line segment.
    *
    * @param midpointToPack point in which the mid-point of this line segment is stored. Modified.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code midpointToPack} are not
    *            expressed in the same reference frame.
    */
   default void midpoint(FixedFramePoint3DBasics midpointToPack)
   {
      checkReferenceFrameMatch(midpointToPack);
      LineSegment3DReadOnly.super.midpoint(midpointToPack);
   }

   /**
    * Computes the coordinates of the point located exactly at the middle of this line segment.
    *
    * @param midpointToPack point in which the mid-point of this line segment is stored. Modified.
    */
   default void midpoint(FramePoint3DBasics midpointToPack)
   {
      midpointToPack.setReferenceFrame(getReferenceFrame());
      LineSegment3DReadOnly.super.midpoint(midpointToPack);
   }

   /**
    * Tests on a per-component basis on both endpoints if this line segment is equal to {@code other}
    * with the tolerance {@code epsilon}.
    * <p>
    * If the two line segments have different frames, this method returns {@code false}.
    * </p>
    *
    * @param other the query. Not modified.
    * @param epsilon the tolerance to use.
    * @return {@code true} if the two line segments are equal and are expressed in the same reference
    *         frame, {@code false} otherwise.
    */
   default boolean epsilonEquals(FrameLineSegment3DReadOnly other, double epsilon)
   {
      if (getReferenceFrame() != other.getReferenceFrame())
         return false;
      return LineSegment3DReadOnly.super.epsilonEquals(other, epsilon);
   }

   /**
    * Compares {@code this} to {@code other} to determine if the two lines are geometrically similar.
    * <p>
    * Two lines are considered geometrically equal is they are collinear, pointing toward the same or
    * opposite direction.
    * </p>
    *
    * @param other the line to compare to. Not modified.
    * @param epsilon the tolerance of the comparison.
    * @return {@code true} if the two lines represent the same geometry, {@code false} otherwise.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code other} are not expressed in
    *            the same reference frame.
    */
   default boolean geometricallyEquals(FrameLineSegment3DReadOnly other, double epsilon)
   {
      checkReferenceFrameMatch(other);
      return LineSegment3DReadOnly.super.geometricallyEquals(other, epsilon);
   }

   /**
    * Tests on a per component basis, if this line segment 3D is exactly equal to {@code other}.
    * <p>
    * If the two line segments have different frames, this method returns {@code false}.
    * </p>
    *
    * @param other the other line segment 3D to compare against this. Not modified.
    * @return {@code true} if the two line segments are exactly equal component-wise and are expressed
    *         in the same reference frame, {@code false} otherwise.
    */
   default boolean equals(FrameLineSegment3DReadOnly other)
   {
      if (other == null || getReferenceFrame() != other.getReferenceFrame())
         return false;
      else
         return LineSegment3DReadOnly.super.equals(other);
   }
}
