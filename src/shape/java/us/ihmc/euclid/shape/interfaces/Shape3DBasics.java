package us.ihmc.euclid.shape.interfaces;

import us.ihmc.euclid.interfaces.Clearable;
import us.ihmc.euclid.interfaces.Transformable;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;

public interface Shape3DBasics extends Shape3DReadOnly, Clearable, Transformable
{
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

   @Override
   default boolean containsNaN()
   {
      return Shape3DReadOnly.super.containsNaN();
   }

   /** {@inheritDoc} */
   @Override
   default void setToNaN()
   {
      getPose().setToNaN();
   }

   /** {@inheritDoc} */
   @Override
   default void setToZero()
   {
      getPose().setToZero();
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
