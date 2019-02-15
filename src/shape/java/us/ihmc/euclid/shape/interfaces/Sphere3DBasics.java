package us.ihmc.euclid.shape.interfaces;

import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

public interface Sphere3DBasics extends Sphere3DReadOnly, Shape3DBasics
{
   void setRadius(double radius);

   @Override
   Shape3DPoseBasics getPose();

   /**
    * Gets the reference to the orientation of this shape.
    *
    * @return the orientation of this shape.
    */
   @Override
   default RotationMatrix getOrientation()
   {
      return getPose().getShapeOrientation();
   }

   /**
    * Gets the reference of the position of this shape.
    *
    * @return the position of this shape.
    */
   @Override
   default Point3DBasics getPosition()
   {
      return getPose().getShapePosition();
   }

   void setIntermediateVariableSupplier(IntermediateVariableSupplier newSupplier);

   @Override
   default boolean containsNaN()
   {
      return Sphere3DReadOnly.super.containsNaN();
   }

   /** {@inheritDoc} */
   @Override
   default void setToZero()
   {
      getPose().setToZero();
      setRadius(0.0);
   }

   /** {@inheritDoc} */
   @Override
   default void setToNaN()
   {
      getPose().setToNaN();
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
