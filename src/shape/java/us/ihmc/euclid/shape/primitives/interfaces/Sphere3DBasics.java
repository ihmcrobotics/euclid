package us.ihmc.euclid.shape.primitives.interfaces;

import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

/**
 * Write and ready interface for a sphere 3D.
 * <p>
 * A sphere 3D is represented by its position and radius.
 * </p>
 *
 * @author Sylvain Bertrand
 */
public interface Sphere3DBasics extends Sphere3DReadOnly, Shape3DBasics
{
   /**
    * Sets this sphere radius.
    *
    * @param radius the new radius for this sphere.
    * @throws IllegalArgumentException if {@code radius < 0.0}.
    */
   void setRadius(double radius);

   /**
    * Gets the reference of the position of this shape.
    *
    * @return the position of this shape.
    */
   @Override
   Point3DBasics getPosition();

   /** {@inheritDoc} */
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

   /**
    * Sets this sphere properties.
    *
    * @param centerX the x-coordinate of the center.
    * @param centerY the y-coordinate of the center.
    * @param centerZ the z-coordinate of the center.
    * @param radius  the radius for this sphere.
    * @throws IllegalArgumentException if {@code radius < 0.0}.
    */
   default void set(double centerX, double centerY, double centerZ, double radius)
   {
      getPosition().set(centerX, centerY, centerZ);
      setRadius(radius);
   }

   /**
    * Sets this sphere properties.
    *
    * @param center the position of this sphere center. Not modified.
    * @param radius the radius for this sphere.
    * @throws IllegalArgumentException if {@code radius < 0.0}.
    */
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
