package us.ihmc.euclid.shape.primitives.interfaces;

import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public interface Capsule3DBasics extends Capsule3DReadOnly, Shape3DBasics
{
   void setLength(double length);

   void setRadius(double radius);

   /**
    * Gets the reference of the position of this shape.
    *
    * @return the position of this shape.
    */
   @Override
   Point3DBasics getPosition();

   @Override
   Vector3DBasics getAxis();

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
      getPosition().setToNaN();
      getAxis().setToNaN();
      setSize(Double.NaN, Double.NaN);
   }

   /** {@inheritDoc} */
   @Override
   default void setToZero()
   {
      getPosition().setToZero();
      getAxis().setToZero();
      setSize(0.0, 0.0);
   }

   default void set(Capsule3DReadOnly other)
   {
      getPosition().set(other.getPosition());
      getAxis().set(other.getAxis());
      setSize(other.getLength(), other.getRadius());
   }

   default void set(Point3DReadOnly position, Vector3DReadOnly axis, double length, double radius)
   {
      getPosition().set(position);
      getAxis().set(axis);
      getAxis().normalize();
      setSize(length, radius);
   }

   /** {@inheritDoc} */
   @Override
   default void applyInverseTransform(Transform transform)
   {
      transform.inverseTransform(getPosition());
      transform.inverseTransform(getAxis());
      getAxis().normalize();
   }

   /** {@inheritDoc} */
   @Override
   default void applyTransform(Transform transform)
   {
      transform.transform(getPosition());
      transform.transform(getAxis());
      getAxis().normalize();
   }
}
