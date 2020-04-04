package us.ihmc.euclid.geometry.interfaces;

import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

/**
 * Read-only interface for an infinitely wide and long 3D plane defined by a 3D point and a 3D
 * unit-vector.
 */
public interface Plane3DReadOnly
{
   /**
    * Gets the read-only reference to the point through which this plane is going.
    *
    * @return the reference to the point.
    */
   Point3DReadOnly getPoint();

   /**
    * Gets the read-only reference to the normal of this plane.
    *
    * @return the reference to the normal.
    */
   Vector3DReadOnly getNormal();

   /**
    * Gets the x-coordinate of a point this plane goes through.
    *
    * @return the x-coordinate of this plane's point.
    */
   default double getPointX()
   {
      return getPoint().getX();
   }

   /**
    * Gets the y-coordinate of a point this plane goes through.
    *
    * @return the y-coordinate of this plane's point.
    */
   default double getPointY()
   {
      return getPoint().getY();
   }

   /**
    * Gets the z-coordinate of a point this plane goes through.
    *
    * @return the z-coordinate of this plane's point.
    */
   default double getPointZ()
   {
      return getPoint().getZ();
   }

   /**
    * Gets the x-component of this plane's normal.
    *
    * @return the x-component of this plane's normal.
    */
   default double getNormalX()
   {
      return getNormal().getX();
   }

   /**
    * Gets the y-component of this plane's normal.
    *
    * @return the y-component of this plane's normal.
    */
   default double getNormalY()
   {
      return getNormal().getY();
   }

   /**
    * Gets the z-component of this plane's normal.
    *
    * @return the z-component of this plane's normal.
    */
   default double getNormalZ()
   {
      return getNormal().getZ();
   }

   /**
    * Tests if this plane contains {@link Double#NaN}.
    *
    * @return {@code true} if {@code point} and/or {@code normal} contains {@link Double#NaN},
    *         {@code false} otherwise.
    */
   default boolean containsNaN()
   {
      return getPoint().containsNaN() || getNormal().containsNaN();
   }

   /**
    * Computes the minimum distance the given 3D point and this plane.
    *
    * @param pointX the x-coordinate of the point to compute the distance from the plane. Not modified.
    * @param pointY the y-coordinate of the point to compute the distance from the plane. Not modified.
    * @param pointZ the z-coordinate of the point to compute the distance from the plane. Not modified.
    * @return the minimum distance between the 3D point and this 3D plane.
    */
   default double distance(double pointX, double pointY, double pointZ)
   {
      return EuclidGeometryTools.distanceFromPoint3DToPlane3D(pointX, pointY, pointZ, getPoint(), getNormal());
   }

   /**
    * Computes the minimum distance the given 3D point and this plane.
    *
    * @param point 3D point to compute the distance from the plane. Not modified.
    * @return the minimum distance between the 3D point and this 3D plane.
    */
   default double distance(Point3DReadOnly point)
   {
      return EuclidGeometryTools.distanceFromPoint3DToPlane3D(point, getPoint(), getNormal());
   }

   /**
    * Computes the minimum signed distance the given 3D point and this plane.
    * <p>
    * The returned value is negative when the query is located below the plane, positive otherwise.
    * </p>
    *
    * @param point 3D point to compute the distance from the plane. Not modified.
    * @return the signed distance between the point and this plane.
    */
   default double signedDistance(Point3DReadOnly point)
   {
      return EuclidGeometryTools.signedDistanceFromPoint3DToPlane3D(point, getPoint(), getNormal());
   }

   /**
    * Computes the z-coordinate such that the point at (x, y, z) is located on this plane.
    *
    * @param pointX the x-coordinate of the query.
    * @param pointY the y-coordinate of the query.
    * @return the z-coordinate of the plane.
    */
   default double getZOnPlane(double pointX, double pointY)
   {
      // The three components of the plane origin
      double x0 = getPointX();
      double y0 = getPointY();
      double z0 = getPointZ();
      // The three components of the plane normal
      double a = getNormalX();
      double b = getNormalY();
      double c = getNormalZ();

      // Given the plane equation: a*x + b*y + c*z + d = 0, with d = -(a*x0 + b*y0 + c*z0), we find z:
      double z = a / c * (x0 - pointX) + b / c * (y0 - pointY) + z0;
      return z;
   }

   /**
    * Computes the coordinates of the intersection between this plane and an infinitely long 3D line.
    * <p>
    * Edge cases:
    * <ul>
    * <li>If the line is parallel to the plane, this methods fails and returns {@code false}.
    * </ul>
    * </p>
    *
    * @param line               the line that may intersect this plane. Not modified.
    * @param intersectionToPack point in which the coordinates of the intersection are stored.
    * @return {@code true} if the method succeeds, {@code false} otherwise.
    */
   default boolean intersectionWith(Line3DReadOnly line, Point3DBasics intersectionToPack)
   {
      return intersectionWith(intersectionToPack, line.getPoint(), line.getDirection());
   }

   /**
    * Computes the coordinates of the intersection between this plane and an infinitely long 3D line.
    * <p>
    * Edge cases:
    * <ul>
    * <li>If the line is parallel to the plane, this methods fails and returns {@code false}.
    * </ul>
    * </p>
    *
    * @param pointOnLine        a point located on the line. Not modified.
    * @param lineDirection      the direction of the line. Not modified.
    * @param intersectionToPack point in which the coordinates of the intersection are stored.
    * @return {@code true} if the method succeeds, {@code false} otherwise.
    */
   default boolean intersectionWith(Point3DBasics intersectionToPack, Point3DReadOnly pointOnLine, Vector3DReadOnly lineDirection)
   {
      return EuclidGeometryTools.intersectionBetweenLine3DAndPlane3D(getPoint(), getNormal(), pointOnLine, lineDirection, intersectionToPack);
   }

   /**
    * Tests if this plane and the given plane are coincident:
    * <ul>
    * <li>{@code this.normal} and {@code otherPlane.normal} are collinear given the tolerance
    * {@code angleEpsilon}.
    * <li>the distance of {@code otherPlane.point} from the this plane is less than
    * {@code distanceEpsilon}.
    * </ul>
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the length of either normal is below {@code 1.0E-7}, this method fails and returns
    * {@code false}.
    * </ul>
    * </p>
    *
    * @param otherPlane      the other plane to do the test with. Not modified.
    * @param angleEpsilon    tolerance on the angle in radians to determine if the plane normals are
    *                        collinear.
    * @param distanceEpsilon tolerance on the distance to determine if {@code otherPlane.point} belongs
    *                        to this plane.
    * @return {@code true} if the two planes are coincident, {@code false} otherwise.
    */
   default boolean isCoincident(Plane3DReadOnly otherPlane, double angleEpsilon, double distanceEpsilon)
   {
      return EuclidGeometryTools.arePlane3DsCoincident(getPoint(), getNormal(), otherPlane.getPoint(), otherPlane.getNormal(), angleEpsilon, distanceEpsilon);
   }

   /**
    * Tests if the query point is located strictly on or above this plane.
    * <p>
    * Above is defined as the side of the plane toward which the normal is pointing.
    * </p>
    *
    * @param x the x-coordinate of the query.
    * @param y the y-coordinate of the query.
    * @param z the z-coordinate of the query.
    * @return {@code true} if the query is strictly on or above this plane, {@code false} otherwise.
    */
   default boolean isOnOrAbove(double x, double y, double z)
   {
      return isOnOrAbove(x, y, z, 0.0);
   }

   /**
    * Tests if the query point is located on or above this plane given the tolerance {@code epsilon}.
    * <p>
    * Above is defined as the side of the plane toward which the normal is pointing.
    * </p>
    * <p>
    * <ol>
    * <li>if {@code epsilon == 0}, the query has to be either exactly on the plane or strictly above
    * for this method to return {@code true}.
    * <li>if {@code epsilon > 0}, this method returns {@code true} if the query meets the requirements
    * of 1., in addition, this method returns also {@code true} if the query is below the plane at a
    * distance less or equal than {@code epsilon}.
    * <li>if {@code epsilon < 0}, this method returns {@code true} only if the query is above the plane
    * and at a distance of at least {@code Math.abs(epsilon)}.
    * </ol>
    * </p>
    *
    * @param pointX  the x-coordinate of the query.
    * @param pointY  the y-coordinate of the query.
    * @param pointZ  the z-coordinate of the query.
    * @param epsilon the tolerance to use for the test.
    * @return {@code true} if the query is considered to be on or above this plane, {@code false}
    *         otherwise.
    */
   default boolean isOnOrAbove(double pointX, double pointY, double pointZ, double epsilon)
   {
      double dx = (pointX - getPointX()) * getNormalX();
      double dy = (pointY - getPointY()) * getNormalY();
      double dz = (pointZ - getPointZ()) * getNormalZ();

      return dx + dy + dz >= -epsilon;
   }

   /**
    * Tests if the query point is located strictly on or above this plane.
    * <p>
    * Above is defined as the side of the plane toward which the normal is pointing.
    * </p>
    *
    * @param pointToTest the coordinates of the query. Not modified.
    * @return {@code true} if the query is strictly on or above this plane, {@code false} otherwise.
    */
   default boolean isOnOrAbove(Point3DReadOnly pointToTest)
   {
      return isOnOrAbove(pointToTest, 0.0);
   }

   /**
    * Tests if the query point is located on or above this plane given the tolerance {@code epsilon}.
    * <p>
    * Above is defined as the side of the plane toward which the normal is pointing.
    * </p>
    * <p>
    * <ol>
    * <li>if {@code epsilon == 0}, the query has to be either exactly on the plane or strictly above
    * for this method to return {@code true}.
    * <li>if {@code epsilon > 0}, this method returns {@code true} if the query meets the requirements
    * of 1., in addition, this method returns also {@code true} if the query is below the plane at a
    * distance less or equal than {@code epsilon}.
    * <li>if {@code epsilon < 0}, this method returns {@code true} only if the query is above the plane
    * and at a distance of at least {@code Math.abs(epsilon)}.
    * </ol>
    * </p>
    *
    * @param pointToTest the coordinates of the query. Not modified.
    * @param epsilon     the tolerance to use for the test.
    * @return {@code true} if the query is considered to be on or above this plane, {@code false}
    *         otherwise.
    */
   default boolean isOnOrAbove(Point3DReadOnly pointToTest, double epsilon)
   {
      return isOnOrAbove(pointToTest.getX(), pointToTest.getY(), pointToTest.getZ(), epsilon);
   }

   /**
    * Tests if the query point is located strictly on or below this plane.
    * <p>
    * Below is defined as the side of the plane which the normal is pointing away from.
    * </p>
    *
    * @param pointX the x-coordinate of the query.
    * @param pointY the y-coordinate of the query.
    * @param pointZ the z-coordinate of the query.
    * @return {@code true} if the query is strictly on or below this plane, {@code false} otherwise.
    */
   default boolean isOnOrBelow(double pointX, double pointY, double pointZ)
   {
      return isOnOrBelow(pointX, pointY, pointZ, 0.0);
   }

   /**
    * Tests if the query point is located on or below this plane given the tolerance {@code epsilon}.
    * <p>
    * Below is defined as the side of the plane which the normal is pointing away from.
    * </p>
    * <p>
    * <ol>
    * <li>if {@code epsilon == 0}, the query has to be either exactly on the plane or strictly below
    * for this method to return {@code true}.
    * <li>if {@code epsilon > 0}, this method returns {@code true} if the query meets the requirements
    * of 1., in addition, this method returns also {@code true} if the query is above the plane at a
    * distance less or equal than {@code epsilon}.
    * <li>if {@code epsilon < 0}, this method returns {@code true} only if the query is below the plane
    * and at a distance of at least {@code Math.abs(epsilon)}.
    * </ol>
    * </p>
    *
    * @param pointX  the x-coordinate of the query.
    * @param pointY  the y-coordinate of the query.
    * @param pointZ  the z-coordinate of the query.
    * @param epsilon the tolerance to use for the test.
    * @return {@code true} if the query is considered to be on or below this plane, {@code false}
    *         otherwise.
    */
   default boolean isOnOrBelow(double pointX, double pointY, double pointZ, double epsilon)
   {
      double dx = (pointX - getPointX()) * getNormalX();
      double dy = (pointY - getPointY()) * getNormalY();
      double dz = (pointZ - getPointZ()) * getNormalZ();

      return dx + dy + dz <= epsilon;
   }

   /**
    * Tests if the query point is located strictly on or below this plane.
    * <p>
    * Below is defined as the side of the plane which the normal is pointing away from.
    * </p>
    *
    * @param pointToTest the coordinates of the query. Not modified.
    * @return {@code true} if the query is strictly on or below this plane, {@code false} otherwise.
    */
   default boolean isOnOrBelow(Point3DReadOnly pointToTest)
   {
      return isOnOrBelow(pointToTest, 0.0);
   }

   /**
    * Tests if the query point is located on or below this plane given the tolerance {@code epsilon}.
    * <p>
    * Below is defined as the side of the plane which the normal is pointing away from.
    * </p>
    * <p>
    * <ol>
    * <li>if {@code epsilon == 0}, the query has to be either exactly on the plane or strictly below
    * for this method to return {@code true}.
    * <li>if {@code epsilon > 0}, this method returns {@code true} if the query meets the requirements
    * of 1., in addition, this method returns also {@code true} if the query is above the plane at a
    * distance less or equal than {@code epsilon}.
    * <li>if {@code epsilon < 0}, this method returns {@code true} only if the query is below the plane
    * and at a distance of at least {@code Math.abs(epsilon)}.
    * </ol>
    * </p>
    *
    * @param pointToTest the coordinates of the query. Not modified.
    * @param epsilon     the tolerance to use for the test.
    * @return {@code true} if the query is considered to be on or below this plane, {@code false}
    *         otherwise.
    */
   default boolean isOnOrBelow(Point3DReadOnly pointToTest, double epsilon)
   {
      return isOnOrBelow(pointToTest.getX(), pointToTest.getY(), pointToTest.getZ(), epsilon);
   }

   /**
    * Tests if the two planes are parallel by testing if their normals are collinear. The latter is
    * done given a tolerance on the angle between the two normal axes in the range ]0; <i>pi</i>/2[.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if the length of either normal is below {@code 1.0E-7}, this method fails and returns
    * {@code false}.
    * </ul>
    * </p>
    *
    * @param otherPlane   the other plane to do the test with. Not modified.
    * @param angleEpsilon tolerance on the angle in radians.
    * @return {@code true} if the two planes are parallel, {@code false} otherwise.
    */
   default boolean isParallel(Plane3DReadOnly otherPlane, double angleEpsilon)
   {
      return EuclidGeometryTools.areVector3DsParallel(getNormal(), otherPlane.getNormal(), angleEpsilon);
   }

   /**
    * Computes the orthogonal projection of the given 3D point on this 3D plane.
    *
    * @param pointToProject the point to project on this plane. Modified.
    * @return whether the method succeeded or not.
    */
   default boolean orthogonalProjection(Point3DBasics pointToProject)
   {
      return orthogonalProjection(pointToProject, pointToProject);
   }

   /**
    * Computes the orthogonal projection of the given 3D point on this 3D plane.
    *
    * @param pointX           the x-coordinate of the point to compute the projection of.
    * @param pointY           the y-coordinate of the point to compute the projection of.
    * @param pointZ           the z-coordinate of the point to compute the projection of.
    * @param projectionToPack point in which the projection of the point onto the plane is stored.
    *                         Modified.
    * @return whether the method succeeded or not.
    */
   default boolean orthogonalProjection(double pointX, double pointY, double pointZ, Point3DBasics projectionToPack)
   {
      return EuclidGeometryTools.orthogonalProjectionOnPlane3D(pointX, pointY, pointZ, getPoint(), getNormal(), projectionToPack);
   }

   /**
    * Computes the orthogonal projection of the given 3D point on this 3D plane.
    *
    * @param pointToProject   the point to compute the projection of. Not modified.
    * @param projectionToPack point in which the projection of the point onto the plane is stored.
    *                         Modified.
    * @return whether the method succeeded or not.
    */
   default boolean orthogonalProjection(Point3DReadOnly pointToProject, Point3DBasics projectionToPack)
   {
      return EuclidGeometryTools.orthogonalProjectionOnPlane3D(pointToProject, getPoint(), getNormal(), projectionToPack);
   }

   /**
    * Computes the orthogonal projection of the given 3D point on this 3D line.
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    *
    * @param pointToProject the point to compute the projection of. Not modified.
    * @return the projection of the point onto the plane or {@code null} if the method failed.
    */
   default Point3DBasics orthogonalProjectionCopy(Point3DReadOnly pointToProject)
   {
      return EuclidGeometryTools.orthogonalProjectionOnPlane3D(pointToProject, getPoint(), getNormal());
   }

   /**
    * Tests on a per-component basis on the point and normal if this plane is equal to {@code other}
    * with the tolerance {@code epsilon}. This method will return {@code false} if the two planes are
    * physically the same but either the point or vector of each plane is different. For instance, if
    * {@code this.point == other.point} and {@code this.normal == - other.normal}, the two planes are
    * physically the same but this method returns {@code false}.
    *
    * @param other   the query. Not modified.
    * @param epsilon the tolerance to use.
    * @return {@code true} if the two planes are equal, {@code false} otherwise.
    */
   default boolean epsilonEquals(Plane3DReadOnly other, double epsilon)
   {
      return other.getNormal().epsilonEquals(getNormal(), epsilon) && other.getPoint().epsilonEquals(getPoint(), epsilon);
   }

   /**
    * Tests on a per component basis, if this plane 3D is exactly equal to {@code other}.
    *
    * @param other the other plane 3D to compare against this. Not modified.
    * @return {@code true} if the two planes are exactly equal component-wise, {@code false} otherwise.
    */
   default boolean equals(Plane3DReadOnly other)
   {
      if (other == this)
         return true;
      else if (other == null)
         return false;
      else
         return getPoint().equals(other.getPoint()) && getNormal().equals(other.getNormal());
   }

}
