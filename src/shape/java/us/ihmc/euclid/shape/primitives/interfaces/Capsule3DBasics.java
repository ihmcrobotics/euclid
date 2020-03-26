package us.ihmc.euclid.shape.primitives.interfaces;

import us.ihmc.euclid.Axis;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

/**
 * Write and read interface for a capsule 3D.
 * <p>
 * A capsule 3D is represented by its length, i.e. the distance separating the center of the two
 * half-spheres, its radius, the position of its center, and its axis of revolution.
 * </p>
 *
 * @author Sylvain Bertrand
 */
public interface Capsule3DBasics extends Capsule3DReadOnly, Shape3DBasics
{
   /**
    * Sets the length for this capsule.
    *
    * @param length the new length.
    * @throws IllegalArgumentException if {@code length} is negative.
    */
   void setLength(double length);

   /**
    * Sets the radius for this capsule.
    *
    * @param radius the new radius.
    * @throws IllegalArgumentException if {@code radius} is negative.
    */
   void setRadius(double radius);

   /**
    * Gets the reference of the position of this capsule.
    *
    * @return the position of this capsule.
    */
   @Override
   Point3DBasics getPosition();

   /**
    * Gets the reference of this capsule axis of revolution.
    * <p>
    * Note, the axis should remain a unit-length vector.
    * </p>
    *
    * @return the axis of this capsule.
    */
   @Override
   Vector3DBasics getAxis();

   /**
    * Sets the size of this capsule.
    *
    * @param length the new length.
    * @param radius the new radius.
    * @throws IllegalArgumentException if {@code length} or {@code radius} is negative.
    */
   default void setSize(double length, double radius)
   {
      setLength(length);
      setRadius(radius);
   }

   /**
    * Sets this capsule axis of revolution.
    *
    * @param axis the new axis. Not modified.
    */
   default void setAxis(Vector3DReadOnly axis)
   {
      getAxis().set(axis);
      getAxis().normalize();
   }

   /** {@inheritDoc} */
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
      getAxis().set(Axis.Z);
      setSize(0.0, 0.0);
   }

   /**
    * Copies the {@code other} capsule data into {@code this}.
    *
    * @param other the other capsule to copy. Not modified.
    */
   default void set(Capsule3DReadOnly other)
   {
      getPosition().set(other.getPosition());
      getAxis().set(other.getAxis());
      setSize(other.getLength(), other.getRadius());
   }

   /**
    * Sets this capsule properties.
    *
    * @param position the position of this capsule center. Not modified.
    * @param axis     the axis of revolution of this capsule. Not modified.
    * @param length   the new length.
    * @param radius   the new radius.
    * @throws IllegalArgumentException if {@code length} or {@code radius} is negative.
    */
   default void set(Point3DReadOnly position, Vector3DReadOnly axis, double length, double radius)
   {
      getPosition().set(position);
      setAxis(axis);
      setSize(length, radius);
   }

   @Override
   default Shape3DPoseBasics getPose()
   {
      return null;
   }

   @Override
   Capsule3DBasics copy();

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
