package us.ihmc.euclid.shape.primitives.interfaces;

import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.interfaces.Transformable;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformBasics;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;

public interface Shape3DPoseBasics extends Shape3DPoseReadOnly, RigidBodyTransformBasics, Transformable
{
   RotationMatrix getShapeOrientation();

   Point3DBasics getShapePosition();

   @Override
   default RotationMatrix getRotation()
   {
      return getShapeOrientation();
   }

   @Override
   default Point3DBasics getTranslation()
   {
      return getShapePosition();
   }

   default void set(Pose3DReadOnly pose)
   {
      set(pose.getOrientation(), pose.getPosition());
   }

   @Override
   default void applyTransform(Transform transform)
   {
      transform.transform(getTranslation());
      transform.transform(getRotation());
   }

   @Override
   default void applyInverseTransform(Transform transform)
   {
      transform.inverseTransform(getTranslation());
      transform.inverseTransform(getRotation());
   }
}
