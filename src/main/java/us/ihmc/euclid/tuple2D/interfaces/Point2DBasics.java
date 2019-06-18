package us.ihmc.euclid.tuple2D.interfaces;

import us.ihmc.euclid.exceptions.NotAMatrix2DException;
import us.ihmc.euclid.transform.AffineTransform;
import us.ihmc.euclid.transform.QuaternionBasedTransform;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.Transform;

/**
 * Write and read interface for a 2 dimensional point.
 * <p>
 * A 2D point represents the 2D coordinates of a location on the XY-plane.
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
public interface Point2DBasics extends Tuple2DBasics, Point2DReadOnly
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
    * @throws NotAMatrix2DException if the rotation part of {@code transform} is not a transformation
    *                               in the XY plane.
    */
   @Override
   default void applyTransform(Transform transform)
   {
      transform.transform(this);
   }

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
    * @param transform                 the geometric transform to apply on this point. Not modified.
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
    * @throws NotAMatrix2DException if the rotation part of {@code transform} is not a transformation
    *                               in the XY plane.
    */
   @Override
   default void applyInverseTransform(Transform transform)
   {
      transform.inverseTransform(this);
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
    * @param transform                 the geometric transform to apply on this point. Not modified.
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
