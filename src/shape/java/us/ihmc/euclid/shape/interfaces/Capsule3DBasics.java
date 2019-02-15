package us.ihmc.euclid.shape.interfaces;

import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;

public interface Capsule3DBasics extends Capsule3DReadOnly, Shape3DBasics
{
   void setLength(double length);

   void setRadius(double radius);

   @Override
   Shape3DPoseBasics getPose();

   /**
    * Gets the reference to the orientation of this shape.
    *
    * @return the orientation of this shape.
    */
   @Override
   default RotationMatrix getOrientation()
   {
      return getPose().getShapeOrientation();
   }

   /**
    * Gets the reference of the position of this shape.
    *
    * @return the position of this shape.
    */
   @Override
   default Point3DBasics getPosition()
   {
      return getPose().getShapePosition();
   }

   default void setSize(double length, double radius)
   {
      setLength(length);
      setRadius(radius);
   }

   @Override
   default boolean containsNaN()
   {
      return Capsule3DReadOnly.super.containsNaN();
   }

   /** {@inheritDoc} */
   @Override
   default void setToNaN()
   {
      getPose().setToNaN();
      setSize(Double.NaN, Double.NaN);
   }

   /** {@inheritDoc} */
   @Override
   default void setToZero()
   {
      getPose().setToZero();
      setSize(0.0, 0.0);
   }

   default void set(Capsule3DReadOnly other)
   {
      getPose().set(other.getPose());
      setSize(other.getLength(), other.getRadius());
   }

   default void set(RigidBodyTransformReadOnly pose, double length, double radius)
   {
      getPose().set(pose);
      setSize(length, radius);
   }

   default void set(Pose3DReadOnly pose, double length, double radius)
   {
      getPose().set(pose);
      setSize(length, radius);
   }

   /** {@inheritDoc} */
   @Override
   default void applyInverseTransform(Transform transform)
   {
      transform.inverseTransform(getPose());
   }

   /** {@inheritDoc} */
   @Override
   default void applyTransform(Transform transform)
   {
      transform.transform(getPose());
   }
}
