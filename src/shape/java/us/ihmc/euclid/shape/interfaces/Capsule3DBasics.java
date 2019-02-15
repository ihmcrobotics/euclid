package us.ihmc.euclid.shape.interfaces;

import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;

public interface Capsule3DBasics extends Capsule3DReadOnly, Shape3DBasics
{
   void setLength(double length);

   void setRadius(double radius);

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
      Shape3DBasics.super.setToNaN();
      setSize(Double.NaN, Double.NaN);
   }

   /** {@inheritDoc} */
   @Override
   default void setToZero()
   {
      Shape3DBasics.super.setToZero();
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
}
