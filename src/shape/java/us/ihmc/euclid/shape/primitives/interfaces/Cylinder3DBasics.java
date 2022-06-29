package us.ihmc.euclid.shape.primitives.interfaces;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.UnitVector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

/**
 * Write and read interface for a cylinder 3D.
 * <p>
 * A cylinder 3D is represented by its length, its radius, the position of its center, and its axis
 * of revolution.
 * </p>
 *
 * @author Sylvain Bertrand
 */
public interface Cylinder3DBasics extends Cylinder3DReadOnly, Shape3DBasics
{
   /**
    * Sets the length of this cylinder.
    *
    * @param length the cylinder length along the z-axis.
    * @throws IllegalArgumentException if {@code length} is negative.
    */
   void setLength(double length);

   /**
    * Sets the radius of this cylinder.
    *
    * @param radius the new radius for this cylinder.
    * @throws IllegalArgumentException if {@code radius} is negative.
    */
   void setRadius(double radius);

   /**
    * Gets the reference of the position of this cylinder.
    *
    * @return the position of this cylinder.
    */
   @Override
   Point3DBasics getPosition();

   /**
    * Gets the reference of this cylinder axis of revolution.
    * <p>
    * Note, the axis should remain a unit-length vector.
    * </p>
    *
    * @return the axis of this cylinder.
    */
   @Override
   UnitVector3DBasics getAxis();

   /**
    * Sets the size of this cylinder.
    *
    * @param length the new length.
    * @param radius the new radius.
    */
   default void setSize(double length, double radius)
   {
      setLength(length);
      setRadius(radius);
   }

   /** {@inheritDoc} */
   @Override
   default boolean containsNaN()
   {
      return Cylinder3DReadOnly.super.containsNaN();
   }

   /** {@inheritDoc} */
   @Override
   default void setToNaN()
   {
      getPosition().setToNaN();
      getAxis().setToNaN();
      setSize(Double.NaN, Double.NaN);
   }

   /**
    * Sets the position to zero, the axis to {@link Axis3D#Z}, and the radius and length to zero.
    */
   @Override
   default void setToZero()
   {
      getPosition().setToZero();
      getAxis().set(Axis3D.Z);
      setSize(0.0, 0.0);
   }

   /**
    * Copies the {@code other} cylinder data into {@code this}.
    *
    * @param other the other cylinder to copy. Not modified.
    */
   default void set(Cylinder3DReadOnly other)
   {
      getPosition().set(other.getPosition());
      getAxis().set(other.getAxis());
      setLength(other.getLength());
      setRadius(other.getRadius());
   }

   /**
    * Sets this cylinder properties.
    *
    * @param position the position of this cylinder center. Not modified.
    * @param axis     the axis of revolution of this cylinder. Not modified.
    * @param length   the new length.
    * @param radius   the new radius.
    */
   default void set(Point3DReadOnly position, Vector3DReadOnly axis, double length, double radius)
   {
      getPosition().set(position);
      getAxis().set(axis);
      setSize(length, radius);
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
