package us.ihmc.euclid.shape.primitives.interfaces;

import us.ihmc.euclid.geometry.interfaces.BoundingBox3DBasics;
import us.ihmc.euclid.geometry.interfaces.Line3DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.interfaces.Transformable;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.shape.tools.EuclidEllipsoid3DTools;
import us.ihmc.euclid.shape.tools.EuclidShapeTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

/**
 * Read-only interface for ellipsoid 3D.
 * <p>
 * A ellipsoid 3D is represented by its radii, the position of its center, and its orientation.
 * </p>
 *
 * @author Sylvain Bertrand
 */
public interface Ellipsoid3DReadOnly extends Shape3DReadOnly
{
   /**
    * Get the read-only reference to the radii along the three local axes of this ellipsoid.
    *
    * @return the size of this ellipsoid.
    */
   Vector3DReadOnly getRadii();

   /**
    * Gets the read-only reference to the pose of this ellipsoid.
    * <p>
    * The position part describes the coordinates of the center.
    * </p>
    *
    * @return the pose of this ellipsoid.
    */
   Shape3DPoseReadOnly getPose();

   /**
    * Gets the read-only reference to the orientation of this ellipsoid.
    *
    * @return the orientation of this ellipsoid.
    */
   default RotationMatrixReadOnly getOrientation()
   {
      return getPose().getShapeOrientation();
   }

   /**
    * Gets the read-only reference of the position of this ellipsoid.
    *
    * @return the position of this ellipsoid.
    */
   default Point3DReadOnly getPosition()
   {
      return getPose().getShapePosition();
   }

   /**
    * Gets the intermediate variable supplier that can be used for performing operations in either a
    * garbage-free of thread-safe manner.
    *
    * @return the intermediate variable supplier.
    */
   IntermediateVariableSupplier getIntermediateVariableSupplier();

   /** {@inheritDoc} */
   @Override
   default boolean containsNaN()
   {
      return getPose().containsNaN() || getRadii().containsNaN();
   }

   /** {@inheritDoc} */
   @Override
   default boolean evaluatePoint3DCollision(Point3DReadOnly pointToCheck, Point3DBasics closestPointOnSurfaceToPack, Vector3DBasics normalAtClosestPointToPack)
   {
      Point3DBasics pointToCheckInLocal = getIntermediateVariableSupplier().requestPoint3D();
      getPose().inverseTransform(pointToCheck, pointToCheckInLocal);

      double distance = EuclidShapeTools.evaluatePoint3DEllipsoid3DCollision(pointToCheckInLocal,
                                                                             getRadii(),
                                                                             closestPointOnSurfaceToPack,
                                                                             normalAtClosestPointToPack);

      transformToWorld(closestPointOnSurfaceToPack);
      transformToWorld(normalAtClosestPointToPack);
      getIntermediateVariableSupplier().releasePoint3D(pointToCheckInLocal);

      return distance <= 0.0;
   }

   /** {@inheritDoc} */
   @Override
   default boolean getSupportingVertex(Vector3DReadOnly supportDirection, Point3DBasics supportingVertexToPack)
   {
      Vector3DBasics supportDirectionInLocal = getIntermediateVariableSupplier().requestVector3D();
      getPose().inverseTransform(supportDirection, supportDirectionInLocal);

      EuclidShapeTools.supportingVertexEllipsoid3D(supportDirectionInLocal, getRadii(), supportingVertexToPack);

      getIntermediateVariableSupplier().releaseVector3D(supportDirectionInLocal);

      transformToWorld(supportingVertexToPack);

      return true;
   }

   /** {@inheritDoc} */
   @Override
   default double signedDistance(Point3DReadOnly point)
   {
      Point3DBasics queryInLocal = getIntermediateVariableSupplier().requestPoint3D();
      getPose().inverseTransform(point, queryInLocal);

      double signedDistance = EuclidEllipsoid3DTools.distancePoint3DEllipsoid3D(getRadii(), queryInLocal);

      getIntermediateVariableSupplier().releasePoint3D(queryInLocal);

      return signedDistance;
   }

   /** {@inheritDoc} */
   @Override
   default boolean isPointInside(Point3DReadOnly query, double epsilon)
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
    * @param line                     the line expressed in world coordinates that may intersect this
    *                                 ellipsoid. Not modified.
    * @param firstIntersectionToPack  the coordinate in world of the first intersection. Can be
    *                                 {@code null}. Modified.
    * @param secondIntersectionToPack the coordinate in world of the second intersection. Can be
    *                                 {@code null}. Modified.
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
    * @param pointOnLine              a point expressed in world located on the infinitely long line.
    *                                 Not modified.
    * @param lineDirection            the direction expressed in world of the line. Not modified.
    * @param firstIntersectionToPack  the coordinate in world of the first intersection. Can be
    *                                 {@code null}. Modified.
    * @param secondIntersectionToPack the coordinate in world of the second intersection. Can be
    *                                 {@code null}. Modified.
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

      int numberOfIntersections = EuclidGeometryTools.intersectionBetweenLine3DAndEllipsoid3D(getRadiusX(),
                                                                                              getRadiusY(),
                                                                                              getRadiusZ(),
                                                                                              pointOnLineInLocal,
                                                                                              lineDirectionInLocal,
                                                                                              firstIntersectionToPack,
                                                                                              secondIntersectionToPack);

      getIntermediateVariableSupplier().releasePoint3D(pointOnLineInLocal);
      getIntermediateVariableSupplier().releaseVector3D(lineDirectionInLocal);

      if (firstIntersectionToPack != null && numberOfIntersections >= 1)
         transformToWorld(firstIntersectionToPack);
      if (secondIntersectionToPack != null && numberOfIntersections == 2)
         transformToWorld(secondIntersectionToPack);

      return numberOfIntersections;
   }

   /** {@inheritDoc} */
   @Override
   default void getBoundingBox(BoundingBox3DBasics boundingBoxToPack)
   {
      EuclidShapeTools.boundingBoxEllipsoid3D(getPosition(), getOrientation(), getRadii(), boundingBoxToPack);
   }

   /** {@inheritDoc} */
   @Override
   default boolean isConvex()
   {
      return true;
   }

   /** {@inheritDoc} */
   @Override
   default boolean isPrimitive()
   {
      return true;
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
    * @param other   the other ellipsoid which pose and radii is to be compared against this ellipsoid
    *                pose and radii. Not modified.
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
    *
    * @param other   the ellipsoid to compare to. Not modified.
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

   /**
    * Tests on a per component basis, if this ellipsoid 3D is exactly equal to {@code other}.
    *
    * @param other the other ellipsoid 3D to compare against this. Not modified.
    * @return {@code true} if the two ellipsoids are exactly equal component-wise, {@code false}
    *         otherwise.
    */
   default boolean equals(Ellipsoid3DReadOnly other)
   {
      if (other == this)
         return true;
      else if (other == null)
         return false;
      else
         return getPose().equals(other.getPose()) && getRadii().equals(other.getRadii());
   }

   /**
    * Changes the given {@code transformable} from being expressed in world to being expressed in this
    * shape local coordinates.
    *
    * @param transformable the transformable to change the coordinates in which it is expressed.
    *                      Modified.
    */
   default void transformToLocal(Transformable transformable)
   {
      transformable.applyInverseTransform(getPose());
   }

   /**
    * Changes the given {@code transformable} from being expressed in this shape local coordinates to
    * being expressed in world.
    *
    * @param transformable the transformable to change the coordinates in which it is expressed.
    *                      Modified.
    */
   default void transformToWorld(Transformable transformable)
   {
      transformable.applyTransform(getPose());
   }
}
