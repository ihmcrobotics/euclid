package us.ihmc.euclid.geometry.interfaces;

import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

/**
 * Read-only interface for a line 3D.
 * <p>
 * A line 3D represents an infinitely long line in 3 dimensions and defined by a point and a
 * direction.
 * </p>
 */
public interface Line3DReadOnly
{
   /**
    * Gets the read-only reference to the point through which this line is going.
    *
    * @return the reference to the point.
    */
   Point3DReadOnly getPoint();

   /**
    * Gets the read-only reference to the direction of this line.
    *
    * @return the reference to the direction.
    */
   Vector3DReadOnly getDirection();

   /**
    * Gets the x-component of this line's direction.
    *
    * @return the x-component of this line's direction.
    */
   default double getDirectionX()
   {
      return getDirection().getX();
   }

   /**
    * Gets the y-component of this line's direction.
    *
    * @return the y-component of this line's direction.
    */
   default double getDirectionY()
   {
      return getDirection().getY();
   }

   /**
    * Gets the z-component of this line's direction.
    *
    * @return the z-component of this line's direction.
    */
   default double getDirectionZ()
   {
      return getDirection().getZ();
   }

   /**
    * Gets the point and direction defining this line by storing their components in the given
    * arguments {@code pointToPack} and {@code directionToPack}.
    *
    * @param pointToPack point in which the coordinates of this line's point are stored. Modified.
    * @param directionToPack vector in which the components of this line's direction are stored.
    *           Modified.
    */
   default void get(Point3DBasics pointToPack, Vector3DBasics directionToPack)
   {
      pointToPack.set(getPoint());
      directionToPack.set(getDirection());
   }

   /**
    * Gets the x-coordinate of a point this line goes through.
    *
    * @return the x-coordinate of this line's point.
    */
   default double getPointX()
   {
      return getPoint().getX();
   }

   /**
    * Gets the y-coordinate of a point this line goes through.
    *
    * @return the y-coordinate of this line's point.
    */
   default double getPointY()
   {
      return getPoint().getY();
   }

   /**
    * Gets the z-coordinate of a point this line goes through.
    *
    * @return the z-coordinate of this line's point.
    */
   default double getPointZ()
   {
      return getPoint().getZ();
   }

   /**
    * This methods computes the minimum distance between this line and {@code otherLine}.
    * <a href="http://geomalgorithms.com/a07-_distance.html"> Useful link</a>.
    *
    * @param otherLine the other line to compute the distance from. Not modified.
    * @return the minimum distance between the two lines.
    */
   default double distance(Line3DReadOnly otherLine)
   {
      return EuclidGeometryTools.distanceBetweenTwoLine3Ds(getPoint(), getDirection(), otherLine.getPoint(), otherLine.getDirection());
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
    */
   default double distance(Point3DReadOnly point)
   {
      return EuclidGeometryTools.distanceFromPoint3DToLine3D(point, getPoint(), getDirection());
   }

   /**
    * Computes the orthogonal projection of the given 3D point on this 3D line.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the given line direction is too small, i.e.
    * {@code lineDirection.lengthSquared() < }{@value EuclidGeometryTools#ONE_TRILLIONTH}, this method
    * fails and returns {@code false}.
    * </ul>
    * </p>
    *
    * @param pointToProject the point to compute the projection of. Not modified.
    * @param projectionToPack point in which the projection of the point onto the line is stored.
    *           Modified.
    * @return whether the method succeeded or not.
    */
   default boolean orthogonalProjection(Point3DReadOnly pointToProject, Point3DBasics projectionToPack)
   {
      return EuclidGeometryTools.orthogonalProjectionOnLine3D(pointToProject, getPoint(), getDirection(), projectionToPack);
   }

   /**
    * Computes the orthogonal projection of the given 3D point on this 3D line.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the given line direction is too small, i.e.
    * {@code lineDirection.lengthSquared() < }{@value EuclidGeometryTools#ONE_TRILLIONTH}, this method
    * fails and returns {@code false}.
    * </ul>
    * </p>
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param pointToProject the point to compute the projection of. Not modified.
    * @return the projection of the point onto the line or {@code null} if the method failed.
    */
   default Point3D orthogonalProjectionCopy(Point3DReadOnly pointToProject)
   {
      return EuclidGeometryTools.orthogonalProjectionOnLine3D(pointToProject, getPoint(), getDirection());
   }

   /**
    * Tests if the given is located on this line.
    * <p>
    * More precisely, the point is assumed to be on this line if it is located at a distance less than
    * {@code epsilon} from it.
    * </p>
    *
    * @param point the coordinates of the query. Not modified.
    * @param epsilon the tolerance used for this test.
    * @return {@code true} if the point is located on this line, {@code false} otherwise.
    */
   default boolean isPointOnLine(Point3DReadOnly point, double epsilon)
   {
      return EuclidGeometryTools.distanceFromPoint3DToLine3D(point, getPoint(), getDirection()) < epsilon;
   }

   /**
    * Calculates the parameter 't' corresponding to the coordinates of the given {@code pointOnLine}
    * 'p' by solving the line equation:<br>
    * p = t * n + p<sub>0</sub><br>
    * where n is the unit-vector defining the direction of this line and p<sub>0</sub> is the point
    * defining this line which also corresponds to the point for which t=0.
    * <p>
    * Note that the absolute value of 't' is equal to the distance between the point 'p' and the point
    * p<sub>0</sub> defining this line.
    * </p>
    *
    * @param pointOnLine the coordinates of the 'p' from which the parameter 't' is to be calculated.
    *           The point has to be on the line. Not modified.
    * @param epsilon the maximum distance allowed between the given point and this line. If the given
    *           point is at a distance less than {@code epsilon} from this line, it is considered as
    *           being located on this line.
    * @return the value of the parameter 't' corresponding to the given point.
    * @throws RuntimeException if the given point is located at a distance greater than {@code epsilon}
    *            from this line.
    */
   default double parameterGivenPointOnLine(Point3DReadOnly pointOnLine, double epsilon)
   {
      if (!isPointOnLine(pointOnLine, epsilon))
      {
         throw new RuntimeException("The given point is not on this line, distance from line: " + distance(pointOnLine));
      }
      else
      {
         double x0 = getPointX();
         double y0 = getPointY();
         double z0 = getPointZ();
         double x1 = x0 + getDirectionX();
         double y1 = y0 + getDirectionY();
         double z1 = z0 + getDirectionZ();
         return EuclidGeometryTools.percentageAlongLineSegment3D(pointOnLine.getX(), pointOnLine.getY(), pointOnLine.getZ(), x0, y0, z0, x1, y1, z1);
      }
   }

   /**
    * Calculates the coordinates of the point 'p' given the parameter 't' as follows:<br>
    * p = t * n + p<sub>0</sub><br>
    * where n is the unit-vector defining the direction of this line and p<sub>0</sub> is the point
    * defining this line which also corresponds to the point for which t=0.
    * <p>
    * Note that the absolute value of 't' is equal to the distance between the point 'p' and the point
    * p<sub>0</sub> defining this line.
    * </p>
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param t the parameter used to calculate the point coordinates.
    * @return the coordinates of the point 'p'.
    */
   default Point3D pointOnLineGivenParameter(double t)
   {
      Point3D pointToReturn = new Point3D();
      pointOnLineGivenParameter(t, pointToReturn);
      return pointToReturn;
   }

   /**
    * Calculates the coordinates of the point 'p' given the parameter 't' as follows:<br>
    * p = t * n + p<sub>0</sub><br>
    * where n is the unit-vector defining the direction of this line and p<sub>0</sub> is the point
    * defining this line which also corresponds to the point for which t=0.
    * <p>
    * Note that the absolute value of 't' is equal to the distance between the point 'p' and the point
    * p<sub>0</sub> defining this line.
    * </p>
    *
    * @param t the parameter used to calculate the point coordinates.
    * @param pointToPack the point in which the coordinates of 'p' are stored. Modified.
    */
   default void pointOnLineGivenParameter(double t, Point3DBasics pointToPack)
   {
      pointToPack.scaleAdd(t, getDirection(), getPoint());
   }

   /**
    * Gets the coordinates of two distinct points this line goes through.
    *
    * @param firstPointOnLineToPack the coordinates of a first point located on this line. Modified.
    * @param secondPointOnLineToPack the coordinates of a second point located on this line. Modified.
    */
   default void getTwoPointsOnLine(Point3DBasics firstPointOnLineToPack, Point3DBasics secondPointOnLineToPack)
   {
      firstPointOnLineToPack.set(getPoint());
      secondPointOnLineToPack.add(getPoint(), getDirection());
   }

   /**
    * Compares {@code this} with {@code other} to determine if the two lines are collinear.
    *
    * @param other the line to compare to. Not modified.
    * @param epsilon the tolerance of the comparison.
    * @return {@code true} if the lines are collinear, {@code false} otherwise.
    */
   default boolean isCollinear(Line3DReadOnly other, double epsilon)
   {
      return isCollinear(other, epsilon, epsilon);
   }

   /**
    * Compares {@code this} with {@code other} to determine if the two lines are collinear.
    *
    * @param other the line to compare to. Not modified.
    * @param angleEpsilon the tolerance of the comparison for angle.
    * @param distanceEpsilon the tolerance of the comparison for distance.
    * @return {@code true} if the lines are collinear, {@code false} otherwise.
    */
   default boolean isCollinear(Line3DReadOnly other, double angleEpsilon, double distanceEpsilon)
   {
      return EuclidGeometryTools.areLine3DsCollinear(getPoint(), getDirection(), other.getPoint(), other.getDirection(), angleEpsilon, distanceEpsilon);
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
   default double closestPointsWith(Line3DReadOnly otherLine, Point3DBasics closestPointOnThisLineToPack, Point3DBasics closestPointOnOtherLineToPack)
   {
      return EuclidGeometryTools.closestPoint3DsBetweenTwoLine3Ds(getPoint(), getDirection(), otherLine.getPoint(), otherLine.getDirection(),
                                                                  closestPointOnThisLineToPack, closestPointOnOtherLineToPack);
   }

   /**
    * Tests on a per-component basis on the point and vector if this line is equal to {@code other}
    * with the tolerance {@code epsilon}. This method will return {@code false} if the two lines are
    * physically the same but either the point or vector of each line is different. For instance, if
    * {@code this.point == other.point} and {@code this.direction == - other.direction}, the two lines
    * are physically the same but this method returns {@code false}.
    *
    * @param other the query. Not modified.
    * @param epsilon the tolerance to use.
    * @return {@code true} if the two lines are equal, {@code false} otherwise.
    */
   default boolean epsilonEquals(Line3DReadOnly other, double epsilon)
   {
      if (!getPoint().epsilonEquals(other.getPoint(), epsilon))
         return false;
      if (!getDirection().epsilonEquals(other.getDirection(), epsilon))
         return false;

      return true;
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
    */
   default boolean geometricallyEquals(Line3DReadOnly other, double epsilon)
   {
      return isCollinear(other, epsilon);
   }

   /**
    * Tests on a per component basis, if this line 3D is exactly equal to {@code other}.
    *
    * @param other the other line 3D to compare against this. Not modified.
    * @return {@code true} if the two lines are exactly equal component-wise, {@code false} otherwise.
    */
   default boolean equals(Line3DReadOnly other)
   {
      if (other == null)
         return false;
      else
         return getPoint().equals(other.getPoint()) && getDirection().equals(other.getDirection());
   }
}
