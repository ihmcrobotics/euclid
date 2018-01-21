package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.geometry.interfaces.Line3DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;

public interface FrameLine3DReadOnly extends Line3DReadOnly, ReferenceFrameHolder
{
   /** {@inheritDoc} */
   FramePoint3DReadOnly getPoint();

   /** {@inheritDoc} */
   FrameVector3DReadOnly getDirection();

   /**
    * Gets the direction defining this line by storing its components in the given argument
    * {@code directionToPack}.
    *
    * @param directionToPack vector in which the components of this line's direction are stored.
    *           Modified.
    */
   default void getDirection(FrameVector3D directionToPack)
   {
      directionToPack.setIncludingFrame(getDirection());
   }

   /**
    * Gets the point defining this line by storing its coordinates in the given argument
    * {@code pointToPack}.
    *
    * @param pointToPack point in which the coordinates of this line's point are stored. Modified.
    */
   default void getPoint(FramePoint3D pointOnLineToPack)
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
   default void getPointAndDirection(FramePoint3D pointToPack, Vector3DBasics directionToPack)
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
   default void getPointAndDirection(Point3DBasics pointToPack, FrameVector3D directionToPack)
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
   default void getPointAndDirection(FramePoint3D pointToPack, FrameVector3D directionToPack)
   {
      getPoint(pointToPack);
      getDirection(directionToPack);
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
   default boolean isCollinear(FrameLine3DReadOnly other, double epsilon)
   {
      checkReferenceFrameMatch(other);
      return Line3DReadOnly.super.isCollinear(other, epsilon);
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
   default boolean isCollinear(FrameLine3DReadOnly other, double angleEpsilon, double distanceEpsilon)
   {
      checkReferenceFrameMatch(other);
      return Line3DReadOnly.super.isCollinear(other, angleEpsilon, distanceEpsilon);
   }

   /**
    * This methods computes two points P &in; this line and Q &in; {@code otherLine} such that the
    * distance || P - Q || is the minimum distance between the two 3D lines.
    * <a href="http://geomalgorithms.com/a07-_distance.html"> Useful link</a>.
    *
    * @param otherLine the second line. Not modified.
    * @param closestPointOnThisLineToPack the 3D coordinates of the point P are packed in this 3D
    *           point. Modified. Can be {@code null}.
    * @param closestPointOnOtherLineToPack the 3D coordinates of the point Q are packed in this 3D
    *           point. Modified. Can be {@code null}.
    * @return the minimum distance between the two lines.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code otherLine} are not
    *            expressed in the same reference frame.
    */
   default double closestPointsWith(FrameLine3DReadOnly otherLine, Point3DBasics closestPointOnThisLineToPack, Point3DBasics closestPointOnOtherLineToPack)
   {
      checkReferenceFrameMatch(otherLine);
      return Line3DReadOnly.super.closestPointsWith(otherLine, closestPointOnThisLineToPack, closestPointOnOtherLineToPack);
   }

   /**
    * This methods computes two points P &in; this line and Q &in; {@code otherLine} such that the
    * distance || P - Q || is the minimum distance between the two 3D lines.
    * <a href="http://geomalgorithms.com/a07-_distance.html"> Useful link</a>.
    *
    * @param otherLine the second line. Not modified.
    * @param closestPointOnThisLineToPack the 3D coordinates of the point P are packed in this 3D
    *           point. Modified. Can be {@code null}.
    * @param closestPointOnOtherLineToPack the 3D coordinates of the point Q are packed in this 3D
    *           point. Modified. Can be {@code null}.
    * @return the minimum distance between the two lines.
    */
   default double closestPointsWith(Line3DReadOnly otherLine, FramePoint3D closestPointOnThisLineToPack, Point3DBasics closestPointOnOtherLineToPack)
   {
      closestPointOnThisLineToPack.setIncludingFrame(getReferenceFrame(), closestPointOnThisLineToPack);
      return Line3DReadOnly.super.closestPointsWith(otherLine, closestPointOnThisLineToPack, closestPointOnOtherLineToPack);
   }

   /**
    * This methods computes two points P &in; this line and Q &in; {@code otherLine} such that the
    * distance || P - Q || is the minimum distance between the two 3D lines.
    * <a href="http://geomalgorithms.com/a07-_distance.html"> Useful link</a>.
    *
    * @param otherLine the second line. Not modified.
    * @param closestPointOnThisLineToPack the 3D coordinates of the point P are packed in this 3D
    *           point. Modified. Can be {@code null}.
    * @param closestPointOnOtherLineToPack the 3D coordinates of the point Q are packed in this 3D
    *           point. Modified. Can be {@code null}.
    * @return the minimum distance between the two lines.
    */
   default double closestPointsWith(Line3DReadOnly otherLine, Point3DBasics closestPointOnThisLineToPack, FramePoint3D closestPointOnOtherLineToPack)
   {
      closestPointOnOtherLineToPack.setIncludingFrame(getReferenceFrame(), closestPointOnOtherLineToPack);
      return Line3DReadOnly.super.closestPointsWith(otherLine, closestPointOnThisLineToPack, closestPointOnOtherLineToPack);
   }

   /**
    * This methods computes two points P &in; this line and Q &in; {@code otherLine} such that the
    * distance || P - Q || is the minimum distance between the two 3D lines.
    * <a href="http://geomalgorithms.com/a07-_distance.html"> Useful link</a>.
    *
    * @param otherLine the second line. Not modified.
    * @param closestPointOnThisLineToPack the 3D coordinates of the point P are packed in this 3D
    *           point. Modified. Can be {@code null}.
    * @param closestPointOnOtherLineToPack the 3D coordinates of the point Q are packed in this 3D
    *           point. Modified. Can be {@code null}.
    * @return the minimum distance between the two lines.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code otherLine} are not
    *            expressed in the same reference frame.
    */
   default double closestPointsWith(FrameLine3DReadOnly otherLine, FramePoint3D closestPointOnThisLineToPack, Point3DBasics closestPointOnOtherLineToPack)
   {
      checkReferenceFrameMatch(otherLine);
      closestPointOnThisLineToPack.setIncludingFrame(getReferenceFrame(), closestPointOnThisLineToPack);
      return Line3DReadOnly.super.closestPointsWith(otherLine, closestPointOnThisLineToPack, closestPointOnOtherLineToPack);
   }

   /**
    * This methods computes two points P &in; this line and Q &in; {@code otherLine} such that the
    * distance || P - Q || is the minimum distance between the two 3D lines.
    * <a href="http://geomalgorithms.com/a07-_distance.html"> Useful link</a>.
    *
    * @param otherLine the second line. Not modified.
    * @param closestPointOnThisLineToPack the 3D coordinates of the point P are packed in this 3D
    *           point. Modified. Can be {@code null}.
    * @param closestPointOnOtherLineToPack the 3D coordinates of the point Q are packed in this 3D
    *           point. Modified. Can be {@code null}.
    * @return the minimum distance between the two lines.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code otherLine} are not
    *            expressed in the same reference frame.
    */
   default double closestPointsWith(FrameLine3DReadOnly otherLine, Point3DBasics closestPointOnThisLineToPack, FramePoint3D closestPointOnOtherLineToPack)
   {
      checkReferenceFrameMatch(otherLine);
      closestPointOnOtherLineToPack.setIncludingFrame(getReferenceFrame(), closestPointOnOtherLineToPack);
      return Line3DReadOnly.super.closestPointsWith(otherLine, closestPointOnThisLineToPack, closestPointOnOtherLineToPack);
   }

   /**
    * This methods computes two points P &in; this line and Q &in; {@code otherLine} such that the
    * distance || P - Q || is the minimum distance between the two 3D lines.
    * <a href="http://geomalgorithms.com/a07-_distance.html"> Useful link</a>.
    *
    * @param otherLine the second line. Not modified.
    * @param closestPointOnThisLineToPack the 3D coordinates of the point P are packed in this 3D
    *           point. Modified. Can be {@code null}.
    * @param closestPointOnOtherLineToPack the 3D coordinates of the point Q are packed in this 3D
    *           point. Modified. Can be {@code null}.
    * @return the minimum distance between the two lines.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code otherLine} are not
    *            expressed in the same reference frame.
    */
   default double closestPointsWith(FrameLine3DReadOnly otherLine, FramePoint3D closestPointOnThisLineToPack, FramePoint3D closestPointOnOtherLineToPack)
   {
      checkReferenceFrameMatch(otherLine);
      closestPointOnThisLineToPack.setIncludingFrame(getReferenceFrame(), closestPointOnThisLineToPack);
      closestPointOnOtherLineToPack.setIncludingFrame(getReferenceFrame(), closestPointOnOtherLineToPack);
      return Line3DReadOnly.super.closestPointsWith(otherLine, closestPointOnThisLineToPack, closestPointOnOtherLineToPack);
   }

   /**
    * This methods computes two points P &in; this line and Q &in; {@code otherLine} such that the
    * distance || P - Q || is the minimum distance between the two 3D lines.
    * <a href="http://geomalgorithms.com/a07-_distance.html"> Useful link</a>.
    *
    * @param otherLine the second line. Not modified.
    * @param closestPointOnThisLineToPack the 3D coordinates of the point P are packed in this 3D
    *           point. Modified. Can be {@code null}.
    * @param closestPointOnOtherLineToPack the 3D coordinates of the point Q are packed in this 3D
    *           point. Modified. Can be {@code null}.
    * @return the minimum distance between the two lines.
    */
   default double closestPointsWith(Line3DReadOnly otherLine, FramePoint3D closestPointOnThisLineToPack, FramePoint3D closestPointOnOtherLineToPack)
   {
      closestPointOnThisLineToPack.setIncludingFrame(getReferenceFrame(), closestPointOnThisLineToPack);
      closestPointOnOtherLineToPack.setIncludingFrame(getReferenceFrame(), closestPointOnOtherLineToPack);
      return Line3DReadOnly.super.closestPointsWith(otherLine, closestPointOnThisLineToPack, closestPointOnOtherLineToPack);
   }

   /**
    * This methods computes the minimum distance between this line and {@code otherLine}.
    * <a href="http://geomalgorithms.com/a07-_distance.html"> Useful link</a>.
    *
    * @param otherLine the other line to compute the distance from. Not modified.
    * @return the minimum distance between the two lines.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code otherLine} are not
    *            expressed in the same reference frame.
    */
   default double distance(FrameLine3DReadOnly otherLine)
   {
      checkReferenceFrameMatch(otherLine);
      return Line3DReadOnly.super.distance(otherLine);
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
    * @param point 3D point to compute the distance from the line. Not modified.
    * @return the minimum distance between the 3D point and this 3D line.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code point} are not expressed in
    *            the same reference frame.
    */
   default double distance(FramePoint3DReadOnly point)
   {
      checkReferenceFrameMatch(point);
      return Line3DReadOnly.super.distance(point);
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
   default void pointOnLineGivenParameter(double t, FramePoint3D pointToPack)
   {
      pointToPack.setToZero(getReferenceFrame());
      Line3DReadOnly.super.pointOnLineGivenParameter(t, pointToPack);
   }

   /**
    * Computes the orthogonal projection of the given 3D point on this 3D line.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the given line direction is too small, i.e.
    * {@code lineDirection.lengthSquared() < }{@value #ONE_TRILLIONTH}, this method fails and
    * returns {@code false}.
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
   default Point3D orthogonalProjectionCopy(FramePoint3DReadOnly pointToProject)
   {
      checkReferenceFrameMatch(pointToProject);
      return Line3DReadOnly.super.orthogonalProjectionCopy(pointToProject);
   }

   /**
    * Computes the orthogonal projection of the given 3D point on this 3D line.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the given line direction is too small, i.e.
    * {@code lineDirection.lengthSquared() < }{@value #ONE_TRILLIONTH}, this method fails and
    * returns {@code false}.
    * </ul>
    * </p>
    *
    * @param pointToProject the point to compute the projection of. Not modified.
    * @param projectionToPack point in which the projection of the point onto the line is stored.
    *           Modified.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code pointToProject} are not
    *            expressed in the same reference frame.
    */
   default boolean orthogonalProjection(FramePoint3DReadOnly pointToProject, Point3DBasics projectionToPack)
   {
      checkReferenceFrameMatch(pointToProject);
      return Line3DReadOnly.super.orthogonalProjection(pointToProject, projectionToPack);
   }

   /**
    * Computes the orthogonal projection of the given 3D point on this 3D line.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the given line direction is too small, i.e.
    * {@code lineDirection.lengthSquared() < }{@value #ONE_TRILLIONTH}, this method fails and
    * returns {@code false}.
    * </ul>
    * </p>
    *
    * @param pointToProject the point to compute the projection of. Not modified.
    * @param projectionToPack point in which the projection of the point onto the line is stored.
    *           Modified.
    */
   default boolean orthogonalProjection(Point3DReadOnly pointToProject, FramePoint3D projectionToPack)
   {
      projectionToPack.setToZero(getReferenceFrame());
      return Line3DReadOnly.super.orthogonalProjection(pointToProject, projectionToPack);
   }

   /**
    * Computes the orthogonal projection of the given 3D point on this 3D line.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the given line direction is too small, i.e.
    * {@code lineDirection.lengthSquared() < }{@value #ONE_TRILLIONTH}, this method fails and
    * returns {@code false}.
    * </ul>
    * </p>
    *
    * @param pointToProject the point to compute the projection of. Not modified.
    * @param projectionToPack point in which the projection of the point onto the line is stored.
    *           Modified.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code pointToProject} are not
    *            expressed in the same reference frame.
    */
   default boolean orthogonalProjection(FramePoint3DReadOnly pointToProject, FramePoint3D projectionToPack)
   {
      checkReferenceFrameMatch(pointToProject);
      projectionToPack.setToZero(getReferenceFrame());
      return Line3DReadOnly.super.orthogonalProjection(pointToProject, projectionToPack);
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
    * @throws ReferenceFrameMismatchException if {@code this} and {@code point} are not expressed in
    *            the same reference frame.
    */
   default double parameterGivenPointOnLine(FramePoint3DReadOnly pointOnLine, double epsilon)
   {
      checkReferenceFrameMatch(pointOnLine);
      return Line3DReadOnly.super.parameterGivenPointOnLine(pointOnLine, epsilon);
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
   default boolean isPointOnLine(FramePoint3DReadOnly point, double epsilon)
   {
      checkReferenceFrameMatch(point);
      return Line3DReadOnly.super.isPointOnLine(point, epsilon);
   }

   /**
    * Gets the coordinates of two distinct points this line goes through.
    *
    * @param firstPointOnLineToPack the coordinates of a first point located on this line. Modified.
    * @param secondPointOnLineToPack the coordinates of a second point located on this line.
    *           Modified.
    */
   default void getTwoPointsOnLine(FramePoint3D firstPointOnLineToPack, Point3DBasics secondPointOnLineToPack)
   {
      firstPointOnLineToPack.setToZero(getReferenceFrame());
      Line3DReadOnly.super.getTwoPointsOnLine(firstPointOnLineToPack, secondPointOnLineToPack);
   }

   /**
    * Gets the coordinates of two distinct points this line goes through.
    *
    * @param firstPointOnLineToPack the coordinates of a first point located on this line. Modified.
    * @param secondPointOnLineToPack the coordinates of a second point located on this line.
    *           Modified.
    */
   default void getTwoPointsOnLine(Point3DBasics firstPointOnLineToPack, FramePoint3D secondPointOnLineToPack)
   {
      secondPointOnLineToPack.setToZero(getReferenceFrame());
      Line3DReadOnly.super.getTwoPointsOnLine(firstPointOnLineToPack, secondPointOnLineToPack);
   }

   /**
    * Gets the coordinates of two distinct points this line goes through.
    *
    * @param firstPointOnLineToPack the coordinates of a first point located on this line. Modified.
    * @param secondPointOnLineToPack the coordinates of a second point located on this line.
    *           Modified.
    */
   default void getTwoPointsOnLine(FramePoint3D firstPointOnLineToPack, FramePoint3D secondPointOnLineToPack)
   {
      firstPointOnLineToPack.setToZero(getReferenceFrame());
      secondPointOnLineToPack.setToZero(getReferenceFrame());
      Line3DReadOnly.super.getTwoPointsOnLine(firstPointOnLineToPack, secondPointOnLineToPack);
   }

   /**
    * Tests on a per component basis, if this line 3D is exactly equal to {@code other}.
    *
    * @param other the other line 3D to compare against this. Not modified.
    * @return {@code true} if the two lines are exactly equal component-wise, {@code false}
    *         otherwise.
    */
   default boolean equals(FrameLine3DReadOnly other)
   {
      if (getReferenceFrame() != other.getReferenceFrame())
         return false;

      return Line3DReadOnly.super.equals(other);
   }
}