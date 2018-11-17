package us.ihmc.euclid.shape;

import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.shape.interfaces.IntermediateVariableSupplier;
import us.ihmc.euclid.shape.interfaces.Shape3DBasics;
import us.ihmc.euclid.shape.interfaces.Shape3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;

/**
 * Base implementation for 3D shapes such as: cylinder, box, sphere, etc.
 *
 * @param <S> the final type of this shape.
 */
public abstract class Shape3D implements Shape3DBasics
{
   protected final RigidBodyTransform shapePose = new RigidBodyTransform();
   private IntermediateVariableSupplier supplier = IntermediateVariableSupplier.defaultIntermediateVariableSupplier();

   /**
    * Default constructor for creating a new shape with its local frame aligned with world.
    */
   public Shape3D()
   {
   }

   @Override
   public IntermediateVariableSupplier getIntermediateVariableSupplier()
   {
      return supplier;
   }

   @Override
   public void setIntermediateVariableSupplier(IntermediateVariableSupplier newSupplier)
   {
      supplier = newSupplier;
   }

   /** {@inheritDoc} */
   @Override
   public final boolean checkIfInside(Point3DReadOnly pointToCheck, Point3DBasics closestPointOnSurfaceToPack, Vector3DBasics normalAtClosestPointToPack)
   {
      Point3DBasics queryInLocal = getIntermediateVariableSupplier().getPoint3D(0);
      shapePose.inverseTransform(pointToCheck, queryInLocal);
      boolean isInside = evaluateQuery(queryInLocal, closestPointOnSurfaceToPack, normalAtClosestPointToPack) <= 0.0;

      if (closestPointOnSurfaceToPack != null)
         transformToWorld(closestPointOnSurfaceToPack);

      if (normalAtClosestPointToPack != null)
         transformToWorld(normalAtClosestPointToPack);

      return isInside;
   }

   /** {@inheritDoc} */
   @Override
   public final double signedDistance(Point3DReadOnly point)
   {
      Point3DBasics queryInLocal = getIntermediateVariableSupplier().getPoint3D(0);
      shapePose.inverseTransform(point, queryInLocal);
      return evaluateQuery(queryInLocal, null, null);
   }

   /**
    * Internal generic method used for the public API of any {@code Shape3d}.
    *
    * @param x the x-coordinate of the query expressed in the local coordinates of this shape.
    * @param y the y-coordinate of the query expressed in the local coordinates of this shape.
    * @param z the z-coordinate of the query expressed in the local coordinates of this shape.
    * @param closestPointOnSurfaceToPack closest point to the query expressed in the local coordinates
    *           of this shape. Modified. Can be {@code null}.
    * @param normalAtClosestPointToPack normal of the shape surface at the closest point. Modified. Can
    *           be {@code null}.
    * @return the distance from the query to the closest point on the shape surface. The returned value
    *         is expected to be negative when the query is inside the shape.
    */
   protected abstract double evaluateQuery(Point3DReadOnly query, Point3DBasics closestPointOnSurfaceToPack, Vector3DBasics normalAtClosestPointToPack);

   /** {@inheritDoc} */
   @Override
   public final boolean isInsideEpsilon(Point3DReadOnly query, double epsilon)
   {
      Point3DBasics queryInLocal = getIntermediateVariableSupplier().getPoint3D(0);
      getPose().inverseTransform(query, queryInLocal);
      return isInsideEpsilonShapeFrame(queryInLocal, epsilon);
   }

   /**
    * Tests if the {@code query} is located inside this shape given the tolerance {@code epsilon}.
    * <p>
    * <ul>
    * <li>if {@code epsilon > 0}, the size of this shape is increased by shifting its surface/faces by
    * a distance of {@code epsilon} toward the outside.
    * <li>if {@code epsilon > 0}, the size of this shape is reduced by shifting its surface/faces by a
    * distance of {@code epsilon} toward the inside.
    * </ul>
    * </p>
    *
    * @param x the x-coordinate of the query expressed in the local coordinates of this shape.
    * @param y the y-coordinate of the query expressed in the local coordinates of this shape.
    * @param z the z-coordinate of the query expressed in the local coordinates of this shape.
    * @param epsilon the tolerance to use for this test.
    * @return {@code true} if the query is considered to be inside this shape, {@code false} otherwise.
    */
   protected abstract boolean isInsideEpsilonShapeFrame(Point3DReadOnly queryInLocal, double epsilon);

   /** {@inheritDoc} */
   @Override
   public final boolean orthogonalProjection(Point3DReadOnly pointToProject, Point3DBasics projectionToPack)
   {
      Point3DBasics pointInLocal = getIntermediateVariableSupplier().getPoint3D(0);
      getPose().inverseTransform(pointToProject, pointInLocal);

      boolean isInside = evaluateQuery(pointInLocal, projectionToPack, null) <= 0.0;

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

   @Override
   public RigidBodyTransformBasics getPose()
   {
      return shapePose;
   }

   @Override
   public RotationMatrix getOrientation()
   {
      return shapePose.getRotation();
   }

   /**
    * Tests separately and on a per component basis if the orientation and the position of this shape's
    * pose and {@code other}'s pose are equal to an {@code epsilon}.
    *
    * @param other the other shape which its pose is to be compared against this shape's pose. Not
    *           modified.
    * @param epsilon tolerance to use when comparing each component.
    * @return {@code true} if the two poses are equal component-wise, {@code false} otherwise.
    */
   public boolean epsilonEqualsPose(Shape3DReadOnly other, double epsilon)
   {
      return getPosition().epsilonEquals(other.getPosition(), epsilon) && getOrientation().epsilonEquals(other.getOrientation(), epsilon);
   }

   /**
    * Provides a {@code String} representation of this shape pose as follows: <br>
    * m00, m01, m02 | m03 <br>
    * m10, m11, m12 | m13 <br>
    * m20, m21, m22 | m23
    *
    * @return the {@code String} representing this shape pose.
    */
   public String getPoseString()
   {
      return shapePose.toString();
   }
}
