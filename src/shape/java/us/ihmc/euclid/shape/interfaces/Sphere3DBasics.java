package us.ihmc.euclid.shape.interfaces;

import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

public interface Sphere3DBasics extends Sphere3DReadOnly, Shape3DBasics
{
   void setRadius(double radius);

   /**
    * Gets the reference of the position of this shape.
    *
    * @return the position of this shape.
    */
   @Override
   Point3DBasics getPosition();

   @Override
   default boolean containsNaN()
   {
      return Sphere3DReadOnly.super.containsNaN();
   }

   /** {@inheritDoc} */
   @Override
   default void setToZero()
   {
      getPosition().setToZero();
      setRadius(0.0);
   }

   /** {@inheritDoc} */
   @Override
   default void setToNaN()
   {
      getPosition().setToNaN();
      setRadius(Double.NaN);
   }

   /**
    * Copies the {@code other} sphere data into {@code this}.
    *
    * @param other the other sphere to copy. Not modified.
    */
   default void set(Sphere3DReadOnly other)
   {
      getPosition().set(other.getPosition());
      setRadius(other.getRadius());
   }

   default void set(double centerX, double centerY, double centerZ, double radius)
   {
      getPosition().set(centerX, centerY, centerZ);
      setRadius(radius);
   }

   default void set(Point3DReadOnly center, double radius)
   {
      getPosition().set(center);
      setRadius(radius);
   }

   /** {@inheritDoc} */
   @Override
   default void applyInverseTransform(Transform transform)
   {
      transform.inverseTransform(getPosition());
   }

   /** {@inheritDoc} */
   @Override
   default void applyTransform(Transform transform)
   {
      transform.transform(getPosition());
   }
}
