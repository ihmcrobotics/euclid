package us.ihmc.euclid.tuple3D.interfaces;

import us.ihmc.euclid.interfaces.Transformable;
import us.ihmc.euclid.transform.AffineTransform;
import us.ihmc.euclid.transform.QuaternionBasedTransform;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.Transform;

/**
 * Write and read interface for a 3 dimensional point.
 * <p>
 * A 3D point represents the 3D coordinates of a location in space.
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
public interface Point3DBasics extends Tuple3DBasics, Point3DReadOnly, Transformable
{
   /**
    * Transforms this point by the given {@code transform}.
    * <p>
    * The transformation depends on the implementation of the transform, here are a few examples:
    * <ul>
    * <li>{@link RigidBodyTransform} rotates then translates a point.
    * <li>{@link QuaternionBasedTransform} rotates then translates a point.
    * <li>{@link AffineTransform} scales, rotates, then translates a point.
    * </ul>
    * </p>
    *
    * @param transform the geometric transform to apply on this point. Not modified.
    */
   @Override
   default void applyTransform(Transform transform)
   {
      transform.transform(this);
   }

   /**
    * Transforms this point by the inverse of the given {@code transform}.
    * <p>
    * The transformation depends on the implementation of the transform, here are a few examples:
    * <ul>
    * <li>{@link RigidBodyTransform} rotates then translates a point.
    * <li>{@link QuaternionBasedTransform} rotates then translates a point.
    * <li>{@link AffineTransform} scales, rotates, then translates a point.
    * </ul>
    * </p>
    *
    * @param transform the geometric transform to apply on this point. Not modified.
    */
   @Override
   default void applyInverseTransform(Transform transform)
   {
      transform.inverseTransform(this);
   }
}
