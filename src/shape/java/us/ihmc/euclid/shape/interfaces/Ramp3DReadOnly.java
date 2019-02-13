package us.ihmc.euclid.shape.interfaces;

import us.ihmc.euclid.shape.tools.EuclidShapeTools;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public interface Ramp3DReadOnly extends Shape3DReadOnly
{
   Vector3DReadOnly getSize();

   IntermediateVariableSupplier getIntermediateVariableSupplier();

   /**
    * Gets the length of this ramp's slope part.
    * <p>
    * Note that this is different than {@link #getSizeX()}. The returned value is equal to:
    * &radic;(this.length<sup>2</sup> + this.height<sup>2</sup>)
    * </p>
    *
    * @return the length of the slope.
    */
   default double getRampLength()
   {
      return EuclidShapeTools.computeRamp3DLength(getSize());
   }

   /**
    * Gets the angle formed by the slope and the bottom face.
    * <p>
    * The angle is positive and in [0, <i>pi</i>].
    * </p>
    *
    * @return the slope angle.
    */
   default double getRampIncline()
   {
      return EuclidShapeTools.computeRamp3DIncline(getSize());
   }

   /** {@inheritDoc} */
   @Override
   default boolean containsNaN()
   {
      return Shape3DReadOnly.super.containsNaN() || getSize().containsNaN();
   }

   @Override
   default boolean doPoint3DCollisionTest(Point3DReadOnly pointToCheck, Point3DBasics closestPointOnSurfaceToPack, Vector3DBasics normalAtClosestPointToPack)
   {
      return EuclidShapeTools.doPoint3DRamp3DCollisionTest(getPose(), getSize(), pointToCheck, closestPointOnSurfaceToPack, normalAtClosestPointToPack) <= 0.0;
   }

   @Override
   default boolean getSupportingVertex(Vector3DReadOnly supportDirection, Point3DBasics supportingVertexToPack)
   {
      Vector3DBasics supportDirectionInLocal = getIntermediateVariableSupplier().requestVector3D();
      getPose().inverseTransform(supportDirection, supportDirectionInLocal);

      if (supportDirectionInLocal.getZ() < 0.0)
      {
         supportingVertexToPack.setX(supportDirectionInLocal.getX() > 0.0 ? getSizeX() : 0.0);
         supportingVertexToPack.setZ(0.0);
      }
      else if (supportDirectionInLocal.getX() > 0.0)
      {
         supportingVertexToPack.setX(getSizeX());
         supportingVertexToPack.setZ(supportDirectionInLocal.getZ() > 0.0 ? getSizeZ() : 0.0);
      }
      else
      { // Look at a the incline of the direction. 
         double directionInclineTanArgument = -supportDirectionInLocal.getX() / supportDirectionInLocal.getZ();
         double rampInclineTanArgument = getSizeZ() / getSizeX();

         if (directionInclineTanArgument > rampInclineTanArgument)
         { // Pointing toward the bottom of the ramp
            supportingVertexToPack.setX(0.0);
            supportingVertexToPack.setZ(0.0);
         }
         else
         { // Pointing toward the top of the ramp
            supportingVertexToPack.setX(getSizeX());
            supportingVertexToPack.setZ(getSizeZ());
         }
      }

      supportingVertexToPack.setY(supportDirectionInLocal.getY() > 0.0 ? 0.5 * getSizeY() : -0.5 * getSizeY());
      transformToWorld(supportingVertexToPack);

      getIntermediateVariableSupplier().releaseVector3D(supportDirectionInLocal);

      return true;
   }

   @Override
   default double signedDistance(Point3DReadOnly point)
   {
      return EuclidShapeTools.signedDistanceBetweenPoint3DAndRamp3D(getPose(), getSize(), point);
   }

   @Override
   default boolean isInsideEpsilon(Point3DReadOnly query, double epsilon)
   {
      return EuclidShapeTools.isPoint3DInsideRamp3D(getPose(), getSize(), query, epsilon);
   }

   /** {@inheritDoc} */
   @Override
   default boolean orthogonalProjection(Point3DReadOnly pointToProject, Point3DBasics projectionToPack)
   {
      return !EuclidShapeTools.orthogonalProjectionOntoRamp3D(getPose(), getSize(), pointToProject, projectionToPack);
   }

   /**
    * Gets the size of this ramp along the x-axis.
    *
    * @return this ramp's length.
    */
   default double getSizeX()
   {
      return getSize().getX();
   }

   /**
    * Gets the size of this ramp along the y-axis.
    *
    * @return this ramp's width.
    */
   default double getSizeY()
   {
      return getSize().getY();
   }

   /**
    * Gets the size of this ramp along the z-axis.
    *
    * @return this ramp's height.
    */
   default double getSizeZ()
   {
      return getSize().getZ();
   }

   /**
    * Computes and packs the surface normal of the slope face of this ramp.
    *
    * @param surfaceNormalToPack the surface normal of the slope. Modified.
    */
   default void getRampSurfaceNormal(Vector3DBasics surfaceNormalToPack)
   {
      surfaceNormalToPack.set(-getSizeZ() / getRampLength(), 0.0, getSizeX() / getRampLength());
      transformToWorld(surfaceNormalToPack);
   }

   /**
    * Tests separately and on a per component basis if the pose and the size of this ramp and
    * {@code other}'s pose and size are equal to an {@code epsilon}.
    *
    * @param other the other ramp which pose and size is to be compared against this ramp pose and
    *           size. Not modified.
    * @param epsilon tolerance to use when comparing each component.
    * @return {@code true} if the two ramps are equal component-wise, {@code false} otherwise.
    */
   default boolean epsilonEquals(Ramp3DReadOnly other, double epsilon)
   {
      return getSize().epsilonEquals(other.getSize(), epsilon) && getPosition().epsilonEquals(other.getPosition(), epsilon)
            && getOrientation().epsilonEquals(other.getOrientation(), epsilon);
   }

   /**
    * Compares {@code this} and {@code other} to determine if the two ramps are geometrically similar,
    * i.e. the difference between their size are less than or equal to {@code epsilon} and their poses
    * are geometrically similar given {@code epsilon}.
    *
    * @param other the ramp to compare to. Not modified.
    * @param epsilon the tolerance of the comparison.
    * @return {@code true} if the ramps represent the same geometry, {@code false} otherwise.
    */
   default boolean geometricallyEquals(Ramp3DReadOnly other, double epsilon)
   {
      return getSize().epsilonEquals(other.getSize(), epsilon) && getPosition().geometricallyEquals(other.getPosition(), epsilon)
            && getOrientation().geometricallyEquals(other.getOrientation(), epsilon);
   }
}
