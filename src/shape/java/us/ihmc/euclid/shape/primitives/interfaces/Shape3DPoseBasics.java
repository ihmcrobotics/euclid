package us.ihmc.euclid.shape.primitives.interfaces;

import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.interfaces.Transformable;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixBasics;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformBasics;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;

/**
 * Write and read interface for representing the pose of a shape 3D.
 *
 * @author Sylvain Bertrand
 */
public interface Shape3DPoseBasics extends Shape3DPoseReadOnly, RigidBodyTransformBasics, Transformable
{
   /**
    * Gets the reference to the orientation part of this pose.
    *
    * @return the orientation part.
    */
   @Override
   RotationMatrixBasics getShapeOrientation();

   /**
    * Gets the reference to the position part of this pose.
    *
    * @return the position part.
    */
   @Override
   Point3DBasics getShapePosition();

   /** {@inheritDoc} */
   @Override
   default RotationMatrixBasics getRotation()
   {
      return getShapeOrientation();
   }

   /** {@inheritDoc} */
   @Override
   default Point3DBasics getTranslation()
   {
      return getShapePosition();
   }

   /**
    * Sets this shape pose 3D to the {@code other} pose 3D.
    *
    * @param pose the pose 3D to set this to. Not modified.
    */
   default void set(Pose3DReadOnly pose)
   {
      set(pose.getOrientation(), pose.getPosition());
   }

   /** {@inheritDoc} */
   @Override
   default void applyTransform(Transform transform)
   {
      transform.transform(this);
   }

   /** {@inheritDoc} */
   @Override
   default void applyInverseTransform(Transform transform)
   {
      transform.inverseTransform(this);
   }
}
