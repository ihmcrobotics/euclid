package us.ihmc.euclid.shape.interfaces;

import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

public interface Sphere3DBasics extends Sphere3DReadOnly, Shape3DBasics
{
   @Override
   default boolean containsNaN()
   {
      return Sphere3DReadOnly.super.containsNaN();
   }

   void setRadius(double radius);

   void setIntermediateVariableSupplier(IntermediateVariableSupplier newSupplier);

   /** {@inheritDoc} */
   @Override
   default void setToZero()
   {
      Shape3DBasics.super.setToZero();
      setRadius(0.0);
   }

   /** {@inheritDoc} */
   @Override
   default void setToNaN()
   {
      Shape3DBasics.super.setToNaN();
      setRadius(Double.NaN);
   }

   /**
    * Copies the {@code other} sphere data into {@code this}.
    *
    * @param other the other sphere to copy. Not modified.
    */
   default void set(Sphere3DReadOnly other)
   {
      getPose().set(other.getPose());
      setRadius(other.getRadius());
   }

   default void set(double centerX, double centerY, double centerZ, double radius)
   {
      getPose().setTranslation(centerX, centerY, centerZ);
      setRadius(radius);
   }

   default void set(Point3DReadOnly center, double radius)
   {
      getPose().setTranslation(center);
      setRadius(radius);
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
