package us.ihmc.euclid.shape.interfaces;

import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.shape.tools.EuclidShapeTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;

public interface Torus3DReadOnly extends Shape3DReadOnly
{
   /**
    * Gets the radius from the torus center to the tube center.
    *
    * @return this torus main radius.
    */
   double getRadius();

   /**
    * Gets the radius of the tube of this torus.
    *
    * @return the radius of the tube.
    */
   double getTubeRadius();

   /** {@inheritDoc} */
   @Override
   default boolean containsNaN()
   {
      return Shape3DReadOnly.super.containsNaN() || Double.isNaN(getRadius()) || Double.isNaN(getTubeRadius());
   }

   @Override
   default boolean checkIfInside(Point3DReadOnly pointToCheck, Point3DBasics closestPointOnSurfaceToPack, Vector3DBasics normalAtClosestPointToPack)
   {
      Point3DBasics queryInLocal = getIntermediateVariableSupplier().requestPoint3D();
      getPose().inverseTransform(pointToCheck, queryInLocal);
      boolean isInside = EuclidShapeTools.evaluatePoint3DWithTorus3D(getRadius(), getTubeRadius(), queryInLocal, closestPointOnSurfaceToPack,
                                                                     normalAtClosestPointToPack) <= 0.0;

      getIntermediateVariableSupplier().releasePoint3D(queryInLocal);

      if (closestPointOnSurfaceToPack != null)
         transformToWorld(closestPointOnSurfaceToPack);

      if (normalAtClosestPointToPack != null)
         transformToWorld(normalAtClosestPointToPack);

      return isInside;
   }

   @Override
   default double signedDistance(Point3DReadOnly point)
   {
      Point3DBasics queryInLocal = getIntermediateVariableSupplier().requestPoint3D();
      getPose().inverseTransform(point, queryInLocal);
      double signedDistance = EuclidShapeTools.signedDistanceBetweenPoint3DAndTorus3D(getRadius(), getTubeRadius(), queryInLocal);
      getIntermediateVariableSupplier().releasePoint3D(queryInLocal);
      return signedDistance;
   }

   @Override
   default boolean isInsideEpsilon(Point3DReadOnly query, double epsilon)
   {
      Point3DBasics queryInLocal = getIntermediateVariableSupplier().requestPoint3D();
      getPose().inverseTransform(query, queryInLocal);
      boolean isInside = EuclidShapeTools.isPoint3DInsideTorus3D(getRadius(), getTubeRadius(), queryInLocal, epsilon);
      getIntermediateVariableSupplier().releasePoint3D(queryInLocal);
      return isInside;
   }

   /** {@inheritDoc} */
   @Override
   default boolean orthogonalProjection(Point3DReadOnly pointToProject, Point3DBasics projectionToPack)
   {
      // Saving the coordinates in case pointToProject is inside and that pointToProject == projectionToPack.
      double xOriginal = pointToProject.getX();
      double yOriginal = pointToProject.getY();
      double zOriginal = pointToProject.getZ();

      Point3DBasics pointInLocal = getIntermediateVariableSupplier().requestPoint3D();
      getPose().inverseTransform(pointToProject, pointInLocal);

      boolean isInside = EuclidShapeTools.orthogonalProjectionOntoTorus3D(getRadius(), getTubeRadius(), pointInLocal, projectionToPack);

      getIntermediateVariableSupplier().releasePoint3D(pointInLocal);

      if (isInside) // Set the coordinates to the original point to save a transform operation
         projectionToPack.set(xOriginal, yOriginal, zOriginal);
      else
         transformToWorld(projectionToPack);

      return !isInside;
   }

   /**
    * Tests separately and on a per component basis if the pose and the radii of this torus and
    * {@code other}'s pose and radii are equal to an {@code epsilon}.
    *
    * @param other the other torus which pose and radii is to be compared against this torus pose and
    *           radii. Not modified.
    * @param epsilon tolerance to use when comparing each component.
    * @return {@code true} if the two tori are equal component-wise, {@code false} otherwise.
    */
   default boolean epsilonEquals(Torus3DReadOnly other, double epsilon)
   {
      return EuclidCoreTools.epsilonEquals(getRadius(), other.getRadius(), epsilon)
            && EuclidCoreTools.epsilonEquals(getTubeRadius(), other.getTubeRadius(), epsilon) && getPosition().epsilonEquals(other.getPosition(), epsilon)
            && getOrientation().epsilonEquals(other.getOrientation(), epsilon);
   }

   /**
    * Compares {@code this} and {@code other} to determine if the two tori are geometrically similar.
    * <p>
    * This method accounts for the multiple combinations of radii and rotations that generate identical
    * tori. For instance, two tori that are identical but one is rotated around its main axis are
    * considered geometrically equal.
    * </p>
    *
    * @param other the torus to compare to. Not modified.
    * @param epsilon the tolerance of the comparison.
    * @return {@code true} if the two tori represent the same geometry, {@code false} otherwise.
    */
   default boolean geometricallyEquals(Torus3DReadOnly other, double epsilon)
   {
      if (!EuclidCoreTools.epsilonEquals(getRadius(), other.getRadius(), epsilon))
         return false;
      if (!EuclidCoreTools.epsilonEquals(getTubeRadius(), other.getTubeRadius(), epsilon))
         return false;

      if (!getPosition().geometricallyEquals(getPosition(), epsilon))
         return false;

      // The approach used here is the same as in Cylinder3DReadOnly.
      return EuclidGeometryTools.areVector3DsParallel(getOrientation().getM02(), getOrientation().getM12(), getOrientation().getM22(),
                                                      other.getOrientation().getM02(), other.getOrientation().getM12(), other.getOrientation().getM22(),
                                                      epsilon);
   }
}
