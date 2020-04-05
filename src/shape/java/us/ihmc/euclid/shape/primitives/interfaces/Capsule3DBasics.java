package us.ihmc.euclid.shape.primitives.interfaces;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.UnitVector3DBasics;
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
   UnitVector3DBasics getAxis();

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
    * Sets this capsule axis of revolution and normalizes it.
    *
    * @param axis the new axis. Not modified.
    * @deprecated Use {@code this.getAxis().set(axis)} instead.
    */
   @Deprecated
   default void setAxis(Vector3DReadOnly axis)
   {
      getAxis().set(axis);
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
