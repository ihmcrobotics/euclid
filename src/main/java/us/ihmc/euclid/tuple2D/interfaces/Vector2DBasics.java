package us.ihmc.euclid.tuple2D.interfaces;

import us.ihmc.euclid.exceptions.NotAMatrix2DException;
import us.ihmc.euclid.transform.AffineTransform;
import us.ihmc.euclid.transform.QuaternionBasedTransform;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;

/**
 * Write and read interface for a 2 dimensional vector.
 * <p>
 * A 2D vector represents a physical quantity with a magnitude and a direction in the XY-plane. For
 * instance, it can be used to represent a 2D velocity, force, or translation from one 2D point to
 * another.
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
public interface Vector2DBasics extends Tuple2DBasics, Vector2DReadOnly
{
   /**
    * Limits the magnitude of this vector to {@code maxLength}.
    * <p>
    * If the length of this vector is less than {@code maxLength}, this method does nothing. When it is
    * greater than {@code maxLength}, this vector is scaled such that it length is equal to
    * {@code maxLength} and its direction is preserved.
    * </p>
    * <p>
    * Edge case: if {@code maxLength <} {@value Vector3DBasics#EPS_MAX_LENGTH}, this vector is set to
    * zero.
    * </p>
    *
    * @param maxLength the maximum allowed length for this vector.
    * @return whether the length of this vector has been changed or not.
    * @deprecated Use {@link Tuple2DBasics#clipToMaxNorm(double)}
    */
   @Deprecated
   default boolean clipToMaxLength(double maxLength)
   {
      return clipToMaxNorm(maxLength);
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
    * @throws NotAMatrix2DException if the rotation part of {@code transform} is not a transformation
    *                               in the XY plane.
    */
   @Override
   default void applyTransform(Transform transform)
   {
      transform.transform(this);
   }

   /**
    * Transforms this vector by the given {@code transform}.
    * <p>
    * Note: transforming a point differs from transforming a vector in the way that the point can be
    * translated, whereas the vector can be only rotated and scaled.
    * </p>
    * <p>
    * The transformation depends on the implementation of the transform, here are a few examples:
    * <ul>
    * <li>{@link RigidBodyTransform} rotates then translates a point.
    * <li>{@link QuaternionBasedTransform} rotates then translates a point.
    * <li>{@link AffineTransform} scales, rotates, then translates a point.
    * </ul>
    * </p>
    *
    * @param transform                 the geometric transform to apply on this vector. Not modified.
    * @param checkIfTransformInXYPlane whether this method should assert that the rotation part of the
    *                                  given transform represents a transformation in the XY plane.
    * @throws NotAMatrix2DException if {@code checkIfTransformInXYPlane == true} and the rotation part
    *                               of {@code transform} is not a transformation in the XY plane.
    */
   @Override
   default void applyTransform(Transform transform, boolean checkIfTransformInXYPlane)
   {
      transform.transform(this, checkIfTransformInXYPlane);
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
    * @throws NotAMatrix2DException if the rotation part of {@code transform} is not a transformation
    *                               in the XY plane.
    */
   @Override
   default void applyInverseTransform(Transform transform)
   {
      transform.inverseTransform(this);
   }

   /**
    * Transforms this vector by the inverse of the given {@code transform}.
    * <p>
    * Note: transforming a point differs from transforming a vector in the way that the point can be
    * translated, whereas the vector can be only rotated and scaled.
    * </p>
    * <p>
    * The transformation depends on the implementation of the transform, here are a few examples:
    * <ul>
    * <li>{@link RigidBodyTransform} rotates then translates a point.
    * <li>{@link QuaternionBasedTransform} rotates then translates a point.
    * <li>{@link AffineTransform} scales, rotates, then translates a point.
    * </ul>
    * </p>
    *
    * @param transform                 the geometric transform to apply on this vector. Not modified.
    * @param checkIfTransformInXYPlane whether this method should assert that the rotation part of the
    *                                  given transform represents a transformation in the XY plane.
    * @throws NotAMatrix2DException if {@code checkIfTransformInXYPlane == true} and the rotation part
    *                               of {@code transform} is not a transformation in the XY plane.
    */
   @Override
   default void applyInverseTransform(Transform transform, boolean checkIfTransformInXYplane)
   {
      transform.inverseTransform(this, checkIfTransformInXYplane);
   }
}
