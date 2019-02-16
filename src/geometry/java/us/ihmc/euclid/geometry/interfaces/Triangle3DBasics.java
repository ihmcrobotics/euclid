package us.ihmc.euclid.geometry.interfaces;

import us.ihmc.euclid.interfaces.Clearable;
import us.ihmc.euclid.interfaces.Transformable;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

public interface Triangle3DBasics extends Triangle3DReadOnly, Transformable, Clearable
{
   @Override
   Point3DBasics getA();

   @Override
   Point3DBasics getB();

   @Override
   Point3DBasics getC();

   @Override
   default boolean containsNaN()
   {
      return Triangle3DReadOnly.super.containsNaN();
   }

   @Override
   default void setToZero()
   {
      getA().setToZero();
      getB().setToZero();
      getC().setToZero();
   }

   @Override
   default void setToNaN()
   {
      getA().setToNaN();
      getB().setToNaN();
      getC().setToNaN();
   }

   default void set(Triangle3DReadOnly other)
   {
      set(other.getA(), other.getB(), other.getC());
   }

   default void set(Point3DReadOnly a, Point3DReadOnly b, Point3DReadOnly c)
   {
      getA().set(a);
      getB().set(b);
      getC().set(c);
   }

   @Override
   default void applyTransform(Transform transform)
   {
      getA().applyTransform(transform);
      getB().applyTransform(transform);
      getC().applyTransform(transform);
   }

   @Override
   default void applyInverseTransform(Transform transform)
   {
      getA().applyInverseTransform(transform);
      getB().applyInverseTransform(transform);
      getC().applyInverseTransform(transform);
   }
}
