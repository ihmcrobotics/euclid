package us.ihmc.euclid.geometry.interfaces;

import us.ihmc.euclid.interfaces.Clearable;
import us.ihmc.euclid.interfaces.Transformable;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

/**
 * Write and read interface for a 3D triangle defined by its three vertices A, B, and C.
 *
 * @author Sylvain Bertrand
 */
public interface Triangle3DBasics extends Triangle3DReadOnly, Transformable, Clearable
{
   /**
    * Gets the reference to the first vertex of this triangle.
    *
    * @return the reference to the first vertex of this line segment.
    */
   @Override
   Point3DBasics getA();

   /**
    * Gets the reference to the second vertex of this triangle.
    *
    * @return the reference to the second vertex of this triangle.
    */
   @Override
   Point3DBasics getB();

   /**
    * Gets the reference to the third vertex of this triangle.
    *
    * @return the reference to the second third of this triangle.
    */
   @Override
   Point3DBasics getC();

   /** {@inheritDoc} */
   @Override
   default boolean containsNaN()
   {
      return Triangle3DReadOnly.super.containsNaN();
   }

   /**
    * Sets all vertices of this triangle to {@link Double#NaN}.
    * <p>
    * After calling this method, this triangle becomes invalid. A new set of valid vertices will have
    * to be set so this triangle is again usable.
    * </p>
    */
   @Override
   default void setToNaN()
   {
      getA().setToNaN();
      getB().setToNaN();
      getC().setToNaN();
   }

   /**
    * Sets all vertices of this triangle to zero.
    */
   @Override
   default void setToZero()
   {
      getA().setToZero();
      getB().setToZero();
      getC().setToZero();
   }

   /**
    * Sets this triangle to be the same as the given triangle.
    *
    * @param other the other triangle to copy. Not modified.
    */
   default void set(Triangle3DReadOnly other)
   {
      set(other.getA(), other.getB(), other.getC());
   }

   /**
    * Redefines this triangle's vertices.
    *
    * @param a the first vertex coordinates. Not modified.
    * @param b the second vertex coordinates. Not modified.
    * @param c the third vertex coordinates. Not modified.
    */
   default void set(Point3DReadOnly a, Point3DReadOnly b, Point3DReadOnly c)
   {
      getA().set(a);
      getB().set(b);
      getC().set(c);
   }

   /**
    * Transforms this triangle's vertices using the given homogeneous transformation matrix.
    *
    * @param transform the transform to apply on the vertices of this triangle. Not modified.
    */
   @Override
   default void applyTransform(Transform transform)
   {
      getA().applyTransform(transform);
      getB().applyTransform(transform);
      getC().applyTransform(transform);
   }

   /**
    * Transforms this triangle's vertices using the inverse of the given homogeneous transformation
    * matrix.
    *
    * @param transform the transform to apply on the vertices of this triangle. Not modified.
    */
   @Override
   default void applyInverseTransform(Transform transform)
   {
      getA().applyInverseTransform(transform);
      getB().applyInverseTransform(transform);
      getC().applyInverseTransform(transform);
   }
}
