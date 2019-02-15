package us.ihmc.euclid.shape.interfaces;

import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;

public interface PointShape3DBasics extends PointShape3DReadOnly, Shape3DBasics, Point3DBasics
{
   @Override
   default void setX(double x)
   {
      getPose().setTranslationX(x);
   }

   @Override
   default void setY(double y)
   {
      getPose().setTranslationY(y);
   }

   @Override
   default void setZ(double z)
   {
      getPose().setTranslationZ(z);
   }

   @Override
   default void setToZero()
   {
      getPose().setToZero();
   }

   @Override
   default void setToNaN()
   {
      getPose().setToNaN();
   }

   @Override
   default boolean containsNaN()
   {
      return PointShape3DReadOnly.super.containsNaN();
   }

   /** {@inheritDoc} */
   @Override
   default void applyInverseTransform(Transform transform)
   {
      transform.inverseTransform(getPose());
   }

   /** {@inheritDoc} */
   @Override
   default void applyTransform(Transform transform)
   {
      transform.transform(getPose());
   }
}
