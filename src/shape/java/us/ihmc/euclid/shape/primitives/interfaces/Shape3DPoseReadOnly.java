package us.ihmc.euclid.shape.primitives.interfaces;

import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

/**
 * Read-only interface for representing the pose of a shape 3D.
 *
 * @author Sylvain Bertrand
 */
public interface Shape3DPoseReadOnly extends RigidBodyTransformReadOnly
{
   /**
    * Gets the read-only reference to the orientation part of this pose.
    *
    * @return the orientation part.
    */
   RotationMatrixReadOnly getShapeOrientation();

   /**
    * Gets the read-only reference to the position part of this pose.
    *
    * @return the position part.
    */
   Point3DReadOnly getShapePosition();

   /**
    * Redirection for {@link #getShapeOrientation()} to comply to {@code RigidBodyTransformReadOnly}.
    */
   @Override
   default RotationMatrixReadOnly getRotation()
   {
      return getShapeOrientation();
   }

   /**
    * Redirection for {@link #getShapePosition()} to comply to {@code RigidBodyTransformReadOnly}.
    */
   @Override
   default Point3DReadOnly getTranslation()
   {
      return getShapePosition();
   }

   /**
    * Gets the read-only reference to the unit-vector representing the local x-axis expressed in world
    * coordinates.
    *
    * @return the x-axis unit-vector.
    */
   Vector3DReadOnly getXAxis();

   /**
    * Gets the read-only reference to the unit-vector representing the local y-axis expressed in world
    * coordinates.
    *
    * @return the y-axis unit-vector.
    */
   Vector3DReadOnly getYAxis();

   /**
    * Gets the read-only reference to the unit-vector representing the local z-axis expressed in world
    * coordinates.
    *
    * @return the z-axis unit-vector.
    */
   Vector3DReadOnly getZAxis();

   /**
    * Tests on a per component basis, if this shape pose 3D is exactly equal to {@code other}.
    *
    * @param other the other shape pose 3D to compare against this. Not modified.
    * @return {@code true} if the two poses are exactly equal component-wise, {@code false} otherwise.
    */
   default boolean equals(Shape3DPoseReadOnly other)
   {
      if (other == this)
         return true;
      else if (other == null)
         return false;
      else
         return getShapePosition().equals(other.getShapePosition()) && getShapeOrientation().equals(other.getShapeOrientation());
   }

   /**
    * Tests on a per-component basis if this shape pose is equal to {@code other} with the tolerance
    * {@code epsilon}.
    *
    * @param other   the query. Not modified.
    * @param epsilon the tolerance to use.
    * @return {@code true} if the two shape poses are equal, {@code false} otherwise.
    */
   default boolean epsilonEquals(Shape3DPoseReadOnly other, double epsilon)
   {
      return getShapePosition().epsilonEquals(other.getShapePosition(), epsilon) && getShapeOrientation().epsilonEquals(other.getShapeOrientation(), epsilon);
   }

   /**
    * Compares {@code this} to {@code other} to determine if the two shape poses are geometrically
    * similar.
    * <p>
    * Two poses are geometrically equal if both their position and orientation are geometrically equal.
    * </p>
    *
    * @param other   the shape pose to compare to. Not modified.
    * @param epsilon the tolerance of the comparison.
    * @return {@code true} if the two shape poses represent the same geometry, {@code false} otherwise.
    */
   default boolean geometricallyEquals(Shape3DPoseReadOnly other, double epsilon)
   {
      return getShapePosition().geometricallyEquals(other.getShapePosition(), epsilon)
            && getShapeOrientation().geometricallyEquals(other.getShapeOrientation(), epsilon);
   }
}
