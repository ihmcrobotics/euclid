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

   IntermediateVariableSupplier getIntermediateVariableSupplier();

   /** {@inheritDoc} */
   @Override
   default boolean containsNaN()
   {
      return Shape3DReadOnly.super.containsNaN() || getRadii().containsNaN();
   }

   @Override
   default boolean doPoint3DCollisionTest(Point3DReadOnly pointToCheck, Point3DBasics closestPointOnSurfaceToPack, Vector3DBasics normalAtClosestPointToPack)
   {
      Point3DBasics pointToCheckInLocal = getIntermediateVariableSupplier().requestPoint3D();
      getPose().inverseTransform(pointToCheck, pointToCheckInLocal);

      double distance = EuclidShapeTools.doPoint3DEllipsoid3DCollisionTest(pointToCheckInLocal, getRadii(), closestPointOnSurfaceToPack, normalAtClosestPointToPack);

      transformToWorld(closestPointOnSurfaceToPack);
      transformToWorld(normalAtClosestPointToPack);
      getIntermediateVariableSupplier().releasePoint3D(pointToCheckInLocal);
      
      return distance <= 0.0;
   }

   @Override
   default boolean getSupportingVertex(Vector3DReadOnly supportDirection, Point3DBasics supportingVertexToPack)
   {
      Vector3DBasics supportDirectionInLocal = getIntermediateVariableSupplier().requestVector3D();
      getPose().inverseTransform(supportDirection, supportDirectionInLocal);

      EuclidShapeTools.supportingVertexEllipsoid3D(supportDirectionInLocal, getRadii(), supportingVertexToPack);

      transformToWorld(supportingVertexToPack);

      return true;
   }

   @Override
   default double signedDistance(Point3DReadOnly point)
   {
      Point3DBasics queryInLocal = getIntermediateVariableSupplier().requestPoint3D();
      getPose().inverseTransform(point, queryInLocal);

      double signedDistance = EuclidShapeTools.signedDistanceBetweenPoint3DAndEllipsoid3D(queryInLocal, getRadii());

      getIntermediateVariableSupplier().releasePoint3D(queryInLocal);

      return signedDistance;
   }

   @Override
   default boolean isInsideEpsilon(Point3DReadOnly query, double epsilon)
   {
      Point3DBasics queryInLocal = getIntermediateVariableSupplier().requestPoint3D();
      getPose().inverseTransform(query, queryInLocal);

      boolean isInside = EuclidShapeTools.isPoint3DInsideEllipsoid3D(queryInLocal, getRadii(), epsilon);

      getIntermediateVariableSupplier().releasePoint3D(queryInLocal);

      return isInside;
   }

   /** {@inheritDoc} */
   @Override
   default boolean orthogonalProjection(Point3DReadOnly pointToProject, Point3DBasics projectionToPack)
   {
      Point3DBasics pointToProjectInLocal = getIntermediateVariableSupplier().requestPoint3D();
      getPose().inverseTransform(pointToProject, pointToProjectInLocal);

      boolean hasBeenProjected = EuclidShapeTools.orthogonalProjectionOntoEllipsoid3D(pointToProjectInLocal, getRadii(), projectionToPack);

      if (hasBeenProjected)
      {
         transformToWorld(projectionToPack);
      }

      getIntermediateVariableSupplier().releasePoint3D(pointToProjectInLocal);

      return hasBeenProjected;
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
      Point3DBasics pointOnLineInLocal = getIntermediateVariableSupplier().requestPoint3D();
      Vector3DBasics lineDirectionInLocal = getIntermediateVariableSupplier().requestVector3D();

      getPose().inverseTransform(pointOnLine, pointOnLineInLocal);
      getPose().inverseTransform(lineDirection, lineDirectionInLocal);

      int numberOfIntersections = EuclidGeometryTools.intersectionBetweenLine3DAndEllipsoid3D(getRadiusX(), getRadiusY(), getRadiusZ(), pointOnLineInLocal,
                                                                                              lineDirectionInLocal, firstIntersectionToPack,
                                                                                              secondIntersectionToPack);

      getIntermediateVariableSupplier().releasePoint3D(pointOnLineInLocal);
      getIntermediateVariableSupplier().releaseVector3D(lineDirectionInLocal);

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

      double thisRadiusX = getRadiusX();
      double thisRadiusY = getRadiusY();
      double thisRadiusZ = getRadiusZ();

      boolean areThisRadiiXYEqual = EuclidCoreTools.epsilonEquals(thisRadiusX, thisRadiusY, epsilon);
      boolean areThisRadiiXZEqual = EuclidCoreTools.epsilonEquals(thisRadiusX, thisRadiusZ, epsilon);

      if (areThisRadiiXYEqual && areThisRadiiXZEqual)
      { // This ellipsoid is a sphere.
        // First - assert that the other ellipsoid is also a sphere.
         if (!EuclidCoreTools.epsilonEquals(other.getRadiusX(), other.getRadiusY(), epsilon))
            return false;
         if (!EuclidCoreTools.epsilonEquals(other.getRadiusX(), other.getRadiusZ(), epsilon))
            return false;
         // Second - assert that the radii are the same between the two ellipsoids.
         double thisRadiiSum = thisRadiusX + thisRadiusY + thisRadiusZ;
         double otherRadiiSum = other.getRadiusX() + other.getRadiusY() + other.getRadiusZ();
         return Math.abs(thisRadiiSum - otherRadiiSum) <= 3.0 * epsilon; // Comparing the average of each ellipsoid's radii.
      }

      Vector3DBasics otherRadii = getIntermediateVariableSupplier().requestVector3D();
      other.getPose().transform(other.getRadii(), otherRadii);
      transformToLocal(otherRadii);
      otherRadii.absolute();

      double otherRadiusX = otherRadii.getX();
      double otherRadiusY = otherRadii.getY();
      double otherRadiusZ = otherRadii.getZ();

      getIntermediateVariableSupplier().releaseVector3D(otherRadii);

      if (areThisRadiiXYEqual)
      {
         if (!EuclidCoreTools.epsilonEquals(thisRadiusZ, otherRadiusZ, epsilon))
            return false;

         double thisRadiiXY = EuclidCoreTools.normSquared(thisRadiusX, thisRadiusY);
         double otherRadiiXY = EuclidCoreTools.normSquared(otherRadiusX, otherRadiusY);

         return EuclidCoreTools.epsilonEquals(thisRadiiXY, otherRadiiXY, epsilon);
      }

      if (areThisRadiiXZEqual)
      {
         if (!EuclidCoreTools.epsilonEquals(thisRadiusY, otherRadiusY, epsilon))
            return false;

         double thisRadiiXZ = EuclidCoreTools.normSquared(thisRadiusX, thisRadiusZ);
         double otherRadiiXZ = EuclidCoreTools.normSquared(otherRadiusX, otherRadiusZ);

         return EuclidCoreTools.epsilonEquals(thisRadiiXZ, otherRadiiXZ, epsilon);
      }

      boolean areThisRadiiYZEqual = EuclidCoreTools.epsilonEquals(thisRadiusY, thisRadiusZ, epsilon);

      if (areThisRadiiYZEqual)
      {
         if (!EuclidCoreTools.epsilonEquals(thisRadiusX, otherRadiusX, epsilon))
            return false;

         double thisRadiiYZ = EuclidCoreTools.normSquared(thisRadiusY, thisRadiusZ);
         double otherRadiiYZ = EuclidCoreTools.normSquared(otherRadiusY, otherRadiusZ);

         return EuclidCoreTools.epsilonEquals(thisRadiiYZ, otherRadiiYZ, epsilon);
      }

      double difference = EuclidGeometryTools.distanceSquaredBetweenPoint3Ds(thisRadiusX, thisRadiusY, thisRadiusZ, otherRadiusX, otherRadiusY, otherRadiusZ);

      if (difference <= epsilon * epsilon)
         return true;

      return false;
   }
}
