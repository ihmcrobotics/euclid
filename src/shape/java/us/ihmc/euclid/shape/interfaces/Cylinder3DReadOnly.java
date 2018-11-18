package us.ihmc.euclid.shape.interfaces;

import us.ihmc.euclid.geometry.interfaces.Line3DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.shape.tools.EuclidShapeTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public interface Cylinder3DReadOnly extends Shape3DReadOnly
{
   /**
    * Gets the radius of this cylinder.
    *
    * @return the value of the radius.
    */
   double getRadius();

   /**
    * Gets the height of this cylinder.
    *
    * @return the value of the height.
    */
   double getHeight();

   /** {@inheritDoc} */
   @Override
   default boolean containsNaN()
   {
      return Shape3DReadOnly.super.containsNaN() || Double.isNaN(getHeight()) || Double.isNaN(getRadius());
   }

   /** {@inheritDoc} */
   @Override
   default boolean checkIfInside(Point3DReadOnly pointToCheck, Point3DBasics closestPointOnSurfaceToPack, Vector3DBasics normalAtClosestPointToPack)
   {
      Point3DBasics queryInLocal = getIntermediateVariableSupplier().requestPoint3D();
      getPose().inverseTransform(pointToCheck, queryInLocal);
      boolean isInside = EuclidShapeTools.evaluatePoint3DWithCylinder3D(getRadius(), getHeight(), queryInLocal, closestPointOnSurfaceToPack,
                                                                        normalAtClosestPointToPack) <= 0.0;

      getIntermediateVariableSupplier().releasePoint3D(queryInLocal);

      if (closestPointOnSurfaceToPack != null)
         transformToWorld(closestPointOnSurfaceToPack);

      if (normalAtClosestPointToPack != null)
         transformToWorld(normalAtClosestPointToPack);

      return isInside;
   }

   /** {@inheritDoc} */
   @Override
   default double signedDistance(Point3DReadOnly point)
   {
      Point3DBasics queryInLocal = getIntermediateVariableSupplier().requestPoint3D();
      getPose().inverseTransform(point, queryInLocal);
      double signedDistance = EuclidShapeTools.signedDistanceBetweenPoint3DAndCylinder3D(getRadius(), getHeight(), queryInLocal);
      getIntermediateVariableSupplier().releasePoint3D(queryInLocal);
      return signedDistance;
   }

   /** {@inheritDoc} */
   @Override
   default boolean isInsideEpsilon(Point3DReadOnly query, double epsilon)
   {
      Point3DBasics queryInLocal = getIntermediateVariableSupplier().requestPoint3D();
      getPose().inverseTransform(query, queryInLocal);
      boolean isInside = EuclidShapeTools.isPoint3DInsideCylinder3D(getRadius(), getHeight(), queryInLocal, epsilon);
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

      boolean isInside = EuclidShapeTools.orthogonalProjectionOntoCylinder3D(getRadius(), getHeight(), pointInLocal, projectionToPack);

      getIntermediateVariableSupplier().releasePoint3D(pointInLocal);

      if (isInside) // Set the coordinates to the original point to save a transform operation
         projectionToPack.set(xOriginal, yOriginal, zOriginal);
      else
         transformToWorld(projectionToPack);

      return !isInside;
   }

   /**
    * Computes the coordinates of the possible intersections between a line and this cylinder.
    * <p>
    * In the case the line and this cylinder do not intersect, this method returns {@code 0} and
    * {@code firstIntersectionToPack} and {@code secondIntersectionToPack} remain unmodified.
    * </p>
    *
    * @param line the line expressed in world coordinates that may intersect this cylinder. Not
    *           modified.
    * @param firstIntersectionToPack the coordinate in world of the first intersection. Can be
    *           {@code null}. Modified.
    * @param secondIntersectionToPack the coordinate in world of the second intersection. Can be
    *           {@code null}. Modified.
    * @return the number of intersections between the line and this cylinder. It is either equal to 0,
    *         1, or 2.
    */
   default int intersectionWith(Line3DReadOnly line, Point3DBasics firstIntersectionToPack, Point3DBasics secondIntersectionToPack)
   {
      return intersectionWith(line.getPoint(), line.getDirection(), firstIntersectionToPack, secondIntersectionToPack);
   }

   /**
    * Computes the coordinates of the possible intersections between a line and this cylinder.
    * <p>
    * In the case the line and this cylinder do not intersect, this method returns {@code 0} and
    * {@code firstIntersectionToPack} and {@code secondIntersectionToPack} remain unmodified.
    * </p>
    *
    * @param pointOnLine a point expressed in world located on the infinitely long line. Not modified.
    * @param lineDirection the direction expressed in world of the line. Not modified.
    * @param firstIntersectionToPack the coordinate in world of the first intersection. Can be
    *           {@code null}. Modified.
    * @param secondIntersectionToPack the coordinate in world of the second intersection. Can be
    *           {@code null}. Modified.
    * @return the number of intersections between the line and this cylinder. It is either equal to 0,
    *         1, or 2.
    */
   default int intersectionWith(Point3DReadOnly pointOnLine, Vector3DReadOnly lineDirection, Point3DBasics firstIntersectionToPack,
                                Point3DBasics secondIntersectionToPack)
   {
      Point3DBasics pointOnLineInLocal = getIntermediateVariableSupplier().requestPoint3D();
      Vector3DBasics lineDirectionInLocal = getIntermediateVariableSupplier().requestVector3D();

      getPose().inverseTransform(pointOnLine, pointOnLineInLocal);
      getPose().inverseTransform(lineDirection, lineDirectionInLocal);

      double halfHeight = 0.5 * getHeight();
      int numberOfIntersections = EuclidGeometryTools.intersectionBetweenLine3DAndCylinder3D(-halfHeight, halfHeight, getRadius(), pointOnLineInLocal,
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
    * Tests separately and on a per component basis if the pose and the size of this cylinder and
    * {@code other}'s pose and size are equal to an {@code epsilon}.
    *
    * @param other the other cylinder which pose and size is to be compared against this cylinder pose
    *           and size. Not modified.
    * @param epsilon tolerance to use when comparing each component.
    * @return {@code true} if the two cylinders are equal component-wise, {@code false} otherwise.
    */
   default boolean epsilonEquals(Cylinder3DReadOnly other, double epsilon)
   {
      return EuclidCoreTools.epsilonEquals(getHeight(), other.getHeight(), epsilon) && EuclidCoreTools.epsilonEquals(getRadius(), other.getRadius(), epsilon)
            && getPosition().epsilonEquals(other.getPosition(), epsilon) && other.getOrientation().epsilonEquals(other.getOrientation(), epsilon);
   }

   /**
    * Compares {@code this} and {@code other} to determine if the two cylinders are geometrically
    * similar.
    * <p>
    * This method accounts for the multiple combinations of radius/height and rotations that generate
    * identical cylinder. For instance, two cylinders that are identical but one is rotated around its
    * main axis are considered geometrically equal.
    * </p>
    *
    * @param other the cylinder to compare to. Not modified.
    * @param epsilon the tolerance of the comparison.
    * @return {@code true} if the cylinders represent the same geometry, {@code false} otherwise.
    */
   default boolean geometricallyEquals(Cylinder3DReadOnly other, double epsilon)
   {
      if (Math.abs(getRadius() - other.getRadius()) > epsilon || Math.abs(getHeight() - other.getHeight()) > epsilon)
         return false;

      if (!getPosition().geometricallyEquals(other.getPosition(), epsilon))
         return false;

      // Here, we check that the axis the cylinder is aligned on (the Z axis, since the cylinder
      // inherently lies on the XY plane) is the same axis that the other cylinder is aligned on using
      // EuclidGeometryTools#areVector3DsParallel(). We could do this by transforming two (0, 0, 1)
      // vectors by each shapePose, but for each:
      // / r00 r01 r02 \   / 0 \   / r02 \ 
      // | r10 r11 r12 | * | 0 | = | r12 |
      // \ r20 r21 r22 /   \ 1 /   \ r22 /
      // So rather than perform this transform, just check that the
      // last column of the rotation matrix of each cylinder (M02, M12, and M22 in shapePose) are aligned
      // vectors.
      return EuclidGeometryTools.areVector3DsParallel(getOrientation().getM02(), getOrientation().getM12(), getOrientation().getM22(),
                                                      other.getOrientation().getM02(), other.getOrientation().getM12(), other.getOrientation().getM22(),
                                                      epsilon);
   }
}
