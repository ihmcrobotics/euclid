package us.ihmc.euclid.shape.primitives.interfaces;

import us.ihmc.euclid.interfaces.EuclidGeometry;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.shape.tools.EuclidShapeIOTools;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

/**
 * Read-only interface for representing the pose of a shape 3D.
 * <p>
 * While the main use-case of a {@code Shape3DPoseReadOnly} is to describe the pose of a shape 3D,
 * it is also used to represent the transform from the shape local coordinate system to the world
 * coordinates, such that it can be used to transform geometry back and forth between the two
 * coordinate systems.
 * </p>
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
    * Tests on a per-component basis if this shape pose is equal to {@code other} with the tolerance
    * {@code epsilon}.
    *
    * @param geometry the query.
    * @param epsilon  the tolerance to use.
    * @return {@code true} if the two shape poses are equal, {@code false} otherwise.
    */
   @Override
   default boolean epsilonEquals(EuclidGeometry geometry, double epsilon)
   {
      if (!(geometry instanceof Shape3DPoseReadOnly))
         return false;
      Shape3DPoseReadOnly other = (Shape3DPoseReadOnly) geometry;
      return getShapePosition().epsilonEquals(other.getShapePosition(), epsilon) && getShapeOrientation().epsilonEquals(other.getShapeOrientation(), epsilon);
   }

   /**
    * Compares {@code this} to {@code other} to determine if the two shape poses are geometrically
    * similar.
    * <p>
    * Two poses are geometrically equal if both their position and orientation are geometrically equal.
    * </p>
    *
    * @param geometry the object to compare to.
    * @param epsilon  the tolerance of the comparison.
    * @return {@code true} if the two shape poses represent the same geometry, {@code false} otherwise.
    */
   @Override
   default boolean geometricallyEquals(EuclidGeometry geometry, double epsilon)
   {
      if (!(geometry instanceof Shape3DPoseReadOnly))
         return false;
      Shape3DPoseReadOnly other = (Shape3DPoseReadOnly) geometry;
      return getShapePosition().geometricallyEquals(other.getShapePosition(), epsilon)
            && getShapeOrientation().geometricallyEquals(other.getShapeOrientation(), epsilon);
   }

   /**
    * Tests on a per component basis, if this shape pose 3D is exactly equal to {@code other}.
    *
    * @param geometry the EuclidGeometry to compare against this. Not modified.
    * @return {@code true} if the two poses are exactly equal component-wise, {@code false} otherwise.
    */
   @Override
   default boolean equals(EuclidGeometry geometry)
   {
      if (geometry == this)
         return true;
      if ((geometry == null) || !(geometry instanceof Shape3DPoseReadOnly))
         return false;
      Shape3DPoseReadOnly other = (Shape3DPoseReadOnly) geometry;
      return getShapePosition().equals(other.getShapePosition()) && getShapeOrientation().equals(other.getShapeOrientation());
   }

   /**
    * Gets the representative {@code String} of this shape 3D pose given a specific format to use.
    * <p>
    * Using the default format {@link #DEFAULT_FORMAT}, this provides a {@code String} as follows:
    *
    * <pre>
    * Shape 3D pose: [position: ( 0.540,  0.110,  0.319 ), yaw-pitch-roll: (-2.061, -0.904, -1.136)]
    * </pre>
    * </p>
    *
    * @param format      the format to use for each number.
    * @param shape3DPose the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   @Override
   default String toString(String format)
   {
      return EuclidShapeIOTools.getShape3DPoseString(format, this);
   }
}
