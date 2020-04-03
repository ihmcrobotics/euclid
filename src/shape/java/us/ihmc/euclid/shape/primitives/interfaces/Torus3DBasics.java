package us.ihmc.euclid.shape.primitives.interfaces;

import us.ihmc.euclid.Axis;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

/**
 * Write and read interface for a torus 3D.
 * <p>
 * A torus is represented by its position, its axis of revolution, the radius of its tube, and the
 * radius from the torus axis to the tube center.
 * </p>
 *
 * @author Sylvain Bertrand
 */
public interface Torus3DBasics extends Torus3DReadOnly, Shape3DBasics
{
   /**
    * Sets this torus radii.
    *
    * @param radius     the radius for the axis to the tube center.
    * @param tubeRadius the tube radius.
    * @throws IllegalArgumentException if either {@code radius < 0.0} or {@code tubeRadius < 0.0}.
    */
   void setRadii(double radius, double tubeRadius);

   /**
    * Gets the reference of the position of this torus.
    *
    * @return the position of this torus.
    */
   @Override
   Point3DBasics getPosition();

   /**
    * Gets the reference of this torus axis of revolution.
    * <p>
    * Note, the axis should remain a unit-length vector.
    * </p>
    *
    * @return the axis of this torus.
    */
   @Override
   Vector3DBasics getAxis();

   /** {@inheritDoc} */
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
      getAxis().set(Axis.Z);
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
    * @param position   the position of this torus center. Not modified.
    * @param axis       the axis of revolution of this torus. Not modified.
    * @param radius     radius from the torus center to the tube center.
    * @param tubeRadius radius of the torus' tube.
    * @throws IllegalArgumentException if either {@code radius < 0.0} or {@code tubeRadius < 0.0}.
    */
   default void set(Point3DReadOnly position, Vector3DReadOnly axis, double radius, double tubeRadius)
   {
      getPosition().set(position);
      getAxis().set(axis);
      getAxis().normalize();
      setRadii(radius, tubeRadius);
   }

   /**
    * Returns {@code null} as this shape is not defined by a pose.
    */
   @Override
   default Shape3DPoseBasics getPose()
   {
      return null;
   }

   /** {@inheritDoc} */
   @Override
   default void applyInverseTransform(Transform transform)
   {
      transform.inverseTransform(getPosition());
      transform.inverseTransform(getAxis());
   }

   /** {@inheritDoc} */
   @Override
   default void applyTransform(Transform transform)
   {
      transform.transform(getPosition());
      transform.transform(getAxis());
   }
}
