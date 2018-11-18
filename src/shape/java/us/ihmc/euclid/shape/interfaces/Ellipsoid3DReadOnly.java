package us.ihmc.euclid.shape.interfaces;

import us.ihmc.euclid.geometry.interfaces.Line3DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.shape.tools.EuclidShapeTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public interface Ellipsoid3DReadOnly extends Shape3DReadOnly
{
   Vector3DReadOnly getRadii();

   /** {@inheritDoc} */
   @Override
   default boolean containsNaN()
   {
      return Shape3DReadOnly.super.containsNaN() || getRadii().containsNaN();
   }

   @Override
   default boolean checkIfInside(Point3DReadOnly pointToCheck, Point3DBasics closestPointOnSurfaceToPack, Vector3DBasics normalAtClosestPointToPack)
   {
      Point3DBasics queryInLocal = getIntermediateVariableSupplier().getPoint3D(0);
      getPose().inverseTransform(pointToCheck, queryInLocal);
      boolean isInside = EuclidShapeTools.evaluatePoint3DWithEllipsoid3D(queryInLocal, closestPointOnSurfaceToPack, normalAtClosestPointToPack,
                                                                         getRadii()) <= 0.0;

      if (closestPointOnSurfaceToPack != null)
         transformToWorld(closestPointOnSurfaceToPack);

      if (normalAtClosestPointToPack != null)
         transformToWorld(normalAtClosestPointToPack);

      return isInside;
   }

   @Override
   default double signedDistance(Point3DReadOnly point)
   {
      Point3DBasics queryInLocal = getIntermediateVariableSupplier().getPoint3D(0);
      getPose().inverseTransform(point, queryInLocal);
      return EuclidShapeTools.signedDistanceBetweenPoint3DAndEllipsoid3D(queryInLocal, getRadii());
   }

   @Override
   default boolean isInsideEpsilon(Point3DReadOnly query, double epsilon)
   {
      Point3DBasics queryInLocal = getIntermediateVariableSupplier().getPoint3D(0);
      getPose().inverseTransform(query, queryInLocal);
      return EuclidShapeTools.isPoint3DInsideEllipsoid3D(queryInLocal, getRadii(), epsilon);
   }

   /** {@inheritDoc} */
   @Override
   default boolean orthogonalProjection(Point3DReadOnly pointToProject, Point3DBasics projectionToPack)
   {
      Point3DBasics pointInLocal = getIntermediateVariableSupplier().getPoint3D(0);
      getPose().inverseTransform(pointToProject, pointInLocal);

      boolean isInside = EuclidShapeTools.orthogonalProjectionOntoEllipsoid3D(pointInLocal, projectionToPack, getRadii());

      if (isInside)
      {
         if (projectionToPack != pointToProject)
            projectionToPack.set(pointToProject);
      }
      else
      {
         transformToWorld(projectionToPack);
      }

      return !isInside;
   }

   /**
    * Computes the coordinates of the possible intersections between a line and this ellipsoid.
    * <p>
    * In the case the line and this box do not intersect, this method returns {@code 0} and
    * {@code firstIntersectionToPack} and {@code secondIntersectionToPack} remain unmodified.
    * </p>
    *
    * @param line the line expressed in world coordinates that may intersect this ellipsoid. Not
    *           modified.
    * @param firstIntersectionToPack the coordinate in world of the first intersection. Can be
    *           {@code null}. Modified.
    * @param secondIntersectionToPack the coordinate in world of the second intersection. Can be
    *           {@code null}. Modified.
    * @return the number of intersections between the line and this ellipsoid. It is either equal to 0,
    *         1, or 2.
    */
   default int intersectionWith(Line3DReadOnly line, Point3DBasics firstIntersectionToPack, Point3DBasics secondIntersectionToPack)
   {
      return intersectionWith(line.getPoint(), line.getDirection(), firstIntersectionToPack, secondIntersectionToPack);
   }

   /**
    * Computes the coordinates of the possible intersections between a line and this ellipsoid.
    * <p>
    * In the case the line and this box do not intersect, this method returns {@code 0} and
    * {@code firstIntersectionToPack} and {@code secondIntersectionToPack} are set to
    * {@link Double#NaN}.
    * </p>
    *
    * @param pointOnLine a point expressed in world located on the infinitely long line. Not modified.
    * @param lineDirection the direction expressed in world of the line. Not modified.
    * @param firstIntersectionToPack the coordinate in world of the first intersection. Can be
    *           {@code null}. Modified.
    * @param secondIntersectionToPack the coordinate in world of the second intersection. Can be
    *           {@code null}. Modified.
    * @return the number of intersections between the line and this ellipsoid. It is either equal to 0,
    *         1, or 2.
    */
   default int intersectionWith(Point3DReadOnly pointOnLine, Vector3DReadOnly lineDirection, Point3DBasics firstIntersectionToPack,
                                Point3DBasics secondIntersectionToPack)
   {
      Point3DBasics pointLocal = getIntermediateVariableSupplier().getPoint3D(0);
      getPose().inverseTransform(pointOnLine, pointLocal);
      Vector3DBasics vectorLocal = getIntermediateVariableSupplier().getVector3D(0);
      getPose().inverseTransform(lineDirection, vectorLocal);

      int numberOfIntersections = EuclidGeometryTools.intersectionBetweenLine3DAndEllipsoid3D(getRadiusX(), getRadiusY(), getRadiusZ(), pointLocal, vectorLocal,
                                                                                              firstIntersectionToPack, secondIntersectionToPack);
      if (firstIntersectionToPack != null && numberOfIntersections >= 1)
         transformToWorld(firstIntersectionToPack);
      if (secondIntersectionToPack != null && numberOfIntersections == 2)
         transformToWorld(secondIntersectionToPack);

      return numberOfIntersections;
   }

   /**
    * Packs the 3 radii of this ellipsoid in the given tuple.
    *
    * @param radiiToPack the tuple in which the radii are stored. Modified.
    */
   default void getRadii(Tuple3DBasics radiiToPack)
   {
      radiiToPack.set(getRadii());
   }

   /**
    * Gets the radius along the x-axis.
    *
    * @return the x radius.
    */
   default double getRadiusX()
   {
      return getRadii().getX();
   }

   /**
    * Gets the radius along the y-axis.
    *
    * @return the y radius.
    */
   default double getRadiusY()
   {
      return getRadii().getY();
   }

   /**
    * Gets the radius along the z-axis.
    *
    * @return the z radius.
    */
   default double getRadiusZ()
   {
      return getRadii().getZ();
   }

   /**
    * Tests separately and on a per component basis if the pose and the radii of this ellipsoid and
    * {@code other}'s pose and size are equal to an {@code epsilon}.
    *
    * @param other the other ellipsoid which pose and radii is to be compared against this ellipsoid
    *           pose and radii. Not modified.
    * @param epsilon tolerance to use when comparing each component.
    * @return {@code true} if the two ellipsoids are equal component-wise, {@code false} otherwise.
    */
   default boolean epsilonEquals(Ellipsoid3DReadOnly other, double epsilon)
   {
      return getRadii().epsilonEquals(other.getRadii(), epsilon) && getPosition().epsilonEquals(other.getPosition(), epsilon)
            && getOrientation().epsilonEquals(other.getOrientation(), epsilon);
   }

   /**
    * Compares {@code this} and {@code other} to determine if the two ellipsoids are geometrically
    * similar.
    * <p>
    * This method accounts for the multiple combinations of radii and rotations that generate identical
    * ellipsoids. For instance, two ellipsoids that are identical but one is flipped by 180 degrees are
    * considered geometrically equal.
    * </p>
    *
    * @param other the ellipsoid to compare to. Not modified.
    * @param epsilon the tolerance of the comparison.
    * @return {@code true} if the ellipsoids represent the same geometry, {@code false} otherwise.
    */
   default boolean geometricallyEquals(Ellipsoid3DReadOnly other, double epsilon)
   {
      if (!getPosition().geometricallyEquals(other.getPosition(), epsilon))
         return false;

      boolean areThisRadiiXYEqual = EuclidCoreTools.epsilonEquals(getRadiusX(), getRadiusY(), epsilon);
      boolean areThisRadiiXZEqual = EuclidCoreTools.epsilonEquals(getRadiusX(), getRadiusZ(), epsilon);

      if (areThisRadiiXYEqual && areThisRadiiXZEqual)
      { // This ellipsoid is a sphere.
        // First - assert that the other ellipsoid is also a sphere.
         if (!EuclidCoreTools.epsilonEquals(other.getRadiusX(), other.getRadiusY(), epsilon))
            return false;
         if (!EuclidCoreTools.epsilonEquals(other.getRadiusX(), other.getRadiusZ(), epsilon))
            return false;
         // Second - assert that the radii are the same between the two ellipsoids.
         double thisRadiiSum = getRadiusX() + getRadiusY() + getRadiusZ();
         double otherRadiiSum = other.getRadiusX() + other.getRadiusY() + other.getRadiusZ();
         return Math.abs(thisRadiiSum - otherRadiiSum) <= 3.0 * epsilon; // Comparing the average of each ellipsoid's radii.
      }

      Vector3DBasics otherRadii = getIntermediateVariableSupplier().getVector3D(0);
      other.getPose().transform(other.getRadii(), otherRadii);
      transformToLocal(otherRadii);

      if (areThisRadiiXYEqual)
      {
         if (!EuclidCoreTools.epsilonEquals(getRadiusZ(), otherRadii.getZ(), epsilon))
            return false;

         double thisRadiiXY = EuclidCoreTools.normSquared(getRadiusX(), getRadiusY());
         double otherRadiiXY = EuclidCoreTools.normSquared(otherRadii.getX(), otherRadii.getY());

         return EuclidCoreTools.epsilonEquals(thisRadiiXY, otherRadiiXY, epsilon);
      }

      if (areThisRadiiXZEqual)
      {
         if (!EuclidCoreTools.epsilonEquals(getRadiusY(), otherRadii.getY(), epsilon))
            return false;

         double thisRadiiXZ = EuclidCoreTools.normSquared(getRadiusX(), getRadiusZ());
         double otherRadiiXZ = EuclidCoreTools.normSquared(otherRadii.getX(), otherRadii.getZ());

         return EuclidCoreTools.epsilonEquals(thisRadiiXZ, otherRadiiXZ, epsilon);
      }

      boolean areThisRadiiYZEqual = EuclidCoreTools.epsilonEquals(getRadiusY(), getRadiusZ(), epsilon);

      if (areThisRadiiYZEqual)
      {
         if (!EuclidCoreTools.epsilonEquals(getRadiusX(), otherRadii.getX(), epsilon))
            return false;

         double thisRadiiYZ = EuclidCoreTools.normSquared(getRadiusY(), getRadiusZ());
         double otherRadiiYZ = EuclidCoreTools.normSquared(otherRadii.getY(), otherRadii.getZ());

         return EuclidCoreTools.epsilonEquals(thisRadiiYZ, otherRadiiYZ, epsilon);
      }

      otherRadii.sub(getRadii());

      if (otherRadii.lengthSquared() <= epsilon * epsilon)
         return true;

      return false;
   }
}
