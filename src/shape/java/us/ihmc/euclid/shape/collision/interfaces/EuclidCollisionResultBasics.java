package us.ihmc.euclid.shape.collision.interfaces;

import us.ihmc.euclid.interfaces.Clearable;
import us.ihmc.euclid.interfaces.Transformable;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;

public interface EuclidCollisionResultBasics extends EuclidCollisionResultReadOnly, Clearable, Transformable
{
   @Override
   Point3DBasics getPointOnA();

   @Override
   Point3DBasics getPointOnB();

   @Override
   Vector3DBasics getNormalOnA();

   @Override
   Vector3DBasics getNormalOnB();

   @Override
   default boolean containsNaN()
   {
      return EuclidCollisionResultReadOnly.super.containsNaN();
   }

   void setShapesAreColliding(boolean shapesAreColliding);

   void setDistance(double distance);

   @Override
   default void setToZero()
   {
      setShapesAreColliding(false);
      setDistance(0.0);
      getPointOnA().setToZero();
      getNormalOnA().setToZero();
      getPointOnB().setToZero();
      getNormalOnB().setToZero();
   }

   @Override
   default void setToNaN()
   {
      setShapesAreColliding(false);
      setDistance(Double.NaN);
      getPointOnA().setToNaN();
      getNormalOnA().setToNaN();
      getPointOnB().setToNaN();
      getNormalOnB().setToNaN();
   }

   @Override
   default void applyTransform(Transform transform)
   {
      if (!getPointOnA().containsNaN())
         getPointOnA().applyTransform(transform);
      if (!getPointOnB().containsNaN())
         getPointOnB().applyTransform(transform);
      if (!getNormalOnA().containsNaN())
         getNormalOnA().applyTransform(transform);
      if (!getNormalOnB().containsNaN())
         getNormalOnB().applyTransform(transform);
   }

   @Override
   default void applyInverseTransform(Transform transform)
   {
      if (!getPointOnA().containsNaN())
         getPointOnA().applyInverseTransform(transform);
      if (!getPointOnB().containsNaN())
         getPointOnB().applyInverseTransform(transform);
      if (!getNormalOnA().containsNaN())
         getNormalOnA().applyInverseTransform(transform);
      if (!getNormalOnB().containsNaN())
         getNormalOnB().applyInverseTransform(transform);
   }

}
