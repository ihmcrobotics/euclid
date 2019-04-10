package us.ihmc.euclid.shape.primitives.interfaces;

import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public interface Shape3DPoseReadOnly extends RigidBodyTransformReadOnly
{
   RotationMatrixReadOnly getShapeOrientation();

   Point3DReadOnly getShapePosition();

   @Override
   default RotationMatrixReadOnly getRotation()
   {
      return getShapeOrientation();
   }

   @Override
   default Point3DReadOnly getTranslation()
   {
      return getShapePosition();
   }

   Vector3DReadOnly getXAxis();

   Vector3DReadOnly getYAxis();

   Vector3DReadOnly getZAxis();

   default boolean equals(Shape3DPoseReadOnly other)
   {
      if (other == this)
         return true;
      else if (other == null)
         return false;
      else
         return getShapePosition().equals(other.getShapePosition()) && getShapeOrientation().equals(other.getShapeOrientation());
   }

   default boolean epsilonEquals(Shape3DPoseReadOnly other, double epsilon)
   {
      return getShapePosition().epsilonEquals(other.getShapePosition(), epsilon) && getShapeOrientation().epsilonEquals(other.getShapeOrientation(), epsilon);
   }

   default boolean geometricallyEquals(Shape3DPoseReadOnly other, double epsilon)
   {
      return getShapePosition().geometricallyEquals(other.getShapePosition(), epsilon)
            && getShapeOrientation().geometricallyEquals(other.getShapeOrientation(), epsilon);
   }
}
