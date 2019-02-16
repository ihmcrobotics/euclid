package us.ihmc.euclid.shape.primitives.interfaces;

import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public interface Torus3DBasics extends Torus3DReadOnly, Shape3DBasics
{
   void setRadii(double radius, double tubeRadius);

   /**
    * Gets the reference of the position of this shape.
    *
    * @return the position of this shape.
    */
   @Override
   Point3DBasics getPosition();

   @Override
   Vector3DBasics getAxis();

   @Override
   default boolean containsNaN()
   {
      return Torus3DReadOnly.super.containsNaN();
   }

   /** {@inheritDoc} */
   @Override
   default void setToZero()
   {
      getPosition().setToZero();
      getAxis().setToZero();
      setRadii(0.0, 0.0);
   }

   /** {@inheritDoc} */
   @Override
   default void setToNaN()
   {
      getPosition().setToNaN();
      getAxis().setToNaN();
      setRadii(Double.NaN, Double.NaN);
   }

   /**
    * Copies the {@code other} torus data into {@code this}.
    *
    * @param other the other torus to copy. Not modified.
    */
   default void set(Torus3DReadOnly other)
   {
      getPosition().set(other.getPosition());
      getAxis().set(other.getAxis());
      setRadii(other.getRadius(), other.getTubeRadius());
   }

   /**
    * Sets the pose and radii of this torus 3D.
    *
    * @param pose the position and orientation of this torus. Not modified.
    * @param radius radius from the torus center to the tube center.
    * @param tubeRadius radius of the torus' tube.
    * @throws IllegalArgumentException if {@code tubeRadius} is less than {@value #MIN_TUBE_RADIUS} or
    *            if the resulting inner radius is less than {@value #MIN_INNER_RADIUS}.
    */
   default void set(Point3DReadOnly position, Vector3DReadOnly axis, double radius, double tubeRadius)
   {
      getPosition().set(position);
      getAxis().set(axis);
      getAxis().normalize();
      setRadii(radius, tubeRadius);
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
