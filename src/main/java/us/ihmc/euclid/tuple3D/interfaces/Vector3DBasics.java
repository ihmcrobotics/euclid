package us.ihmc.euclid.tuple3D.interfaces;

import us.ihmc.euclid.interfaces.Transformable;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.transform.AffineTransform;
import us.ihmc.euclid.transform.QuaternionBasedTransform;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.Transform;

/**
 * Write and read interface for a 3 dimensional vector.
 * <p>
 * A 3D vector represents a physical quantity with a magnitude and a direction. For instance, it can
 * be used to represent a 3D velocity, force, or translation from one 3D point to another.
 * </p>
 * <p>
 * Although a point and vector hold onto the same type of information, the distinction is made
 * between them as they represent different geometry objects and are typically not handled the same
 * way:
 * <ul>
 * <li>a point represents the coordinate of a location in space. A notable difference with a vector
 * is that the distance between two points has a physical meaning. When a point is transformed with
 * a homogeneous transformation matrix, a point's coordinates are susceptible to be scaled, rotated,
 * and translated.
 * <li>a vector is not constrained to a location in space. Instead, a vector represents some
 * physical quantity that has a direction and a magnitude such as: a velocity, a force, the
 * translation from one point to another, etc. When a vector is transformed with a homogeneous
 * transformation matrix, its components are susceptible to be scaled and rotated, but never to be
 * translated.
 * </ul>
 * </p>
 *
 * @author Sylvain Bertrand
 */
public interface Vector3DBasics extends Tuple3DBasics, Vector3DReadOnly, Transformable
{
   /**
    * Tolerance used in {@link #clipToMaxLength(double)}.
    */
   public static final double EPS_MAX_LENGTH = 1.0e-7;

   /**
    * Normalizes this vector such that its magnitude is equal to 1 after calling this method and its
    * direction remains unchanged.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if this vector contains {@link Double#NaN}, this method is ineffective.
    * </ul>
    * </p>
    */
   default void normalize()
   {
      if (containsNaN())
         return;
      scale(1.0 / length());
   }

   /**
    * Sets this vector to {@code other} and then calls {@link #normalize()}.
    *
    * @param other the other vector to copy the values from. Not modified.
    */
   default void setAndNormalize(Tuple3DReadOnly other)
   {
      set(other);
      normalize();
   }

   /**
    * Sets this vector to the cross product of {@code this} and {@code other}.
    * <p>
    * this = this &times; other
    * </p>
    *
    * @param other the other tuple used in the cross product. Not modified.
    */
   default void cross(Tuple3DReadOnly other)
   {
      cross(this, other);
   }

   /**
    * Sets this vector to the cross product of {@code tuple1} and {@code tuple2}.
    * <p>
    * this = tuple1 &times; tuple2
    * </p>
    *
    * @param tuple1 the first tuple in the cross product. Not modified.
    * @param tuple2 the second tuple in the cross product. Not modified.
    */
   default void cross(Tuple3DReadOnly tuple1, Tuple3DReadOnly tuple2)
   {
      double x = tuple1.getY() * tuple2.getZ() - tuple1.getZ() * tuple2.getY();
      double y = tuple1.getZ() * tuple2.getX() - tuple1.getX() * tuple2.getZ();
      double z = tuple1.getX() * tuple2.getY() - tuple1.getY() * tuple2.getX();
      set(x, y, z);
   }

   /**
    * Limits the magnitude of this vector to {@code maxLength}.
    * <p>
    * If the length of this vector is less than {@code maxLength}, this method does nothing. When it is
    * greater than {@code maxLength}, this vector is scaled such that it length is equal to
    * {@code maxLength} and its direction is preserved.
    * </p>
    * <p>
    * Edge case: if {@code maxLength <} {@value #EPS_MAX_LENGTH}, this vector is set to zero.
    * </p>
    *
    * @param maxLength the maximum allowed length for this vector.
    * @return whether the length of this vector has been changed or not.
    */
   default boolean clipToMaxLength(double maxLength)
   {
      if (maxLength < Vector3DBasics.EPS_MAX_LENGTH)
      {
         setToZero();
         return true;
      }

      double lengthSquared = lengthSquared();

      if (lengthSquared < maxLength * maxLength)
         return false;

      scale(maxLength / EuclidCoreTools.squareRoot(lengthSquared));
      return true;
   }

   /**
    * Transforms this vector by the given {@code transform}.
    * <p>
    * The transformation depends on the implementation of the transform, here are a few examples:
    * <ul>
    * <li>{@link RigidBodyTransform} rotates a vector.
    * <li>{@link QuaternionBasedTransform} rotates a vector.
    * <li>{@link AffineTransform} scales then rotates a vector.
    * </ul>
    * </p>
    *
    * @param transform the geometric transform to apply on this vector. Not modified.
    */
   @Override
   default void applyTransform(Transform transform)
   {
      transform.transform(this);
   }

   /**
    * Transforms this vector by the inverse of the given {@code transform}.
    * <p>
    * The transformation depends on the implementation of the transform, here are a few examples:
    * <ul>
    * <li>{@link RigidBodyTransform} rotates a vector.
    * <li>{@link QuaternionBasedTransform} rotates a vector.
    * <li>{@link AffineTransform} scales then rotates a vector.
    * </ul>
    * </p>
    *
    * @param transform the geometric transform to apply on this vector. Not modified.
    */
   @Override
   default void applyInverseTransform(Transform transform)
   {
      transform.inverseTransform(this);
   }
}
