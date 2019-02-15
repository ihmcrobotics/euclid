package us.ihmc.euclid.shape.interfaces;

import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;

public interface Cylinder3DBasics extends Cylinder3DReadOnly, Shape3DBasics
{
   /**
    * Sets the length of this cylinder.
    *
    * @param length the cylinder length along the z-axis.
    * @throws IllegalArgumentException if {@code length} is negative.
    */
   void setLength(double length);

   /**
    * Sets the radius of this cylinder.
    *
    * @param radius the new radius for this cylinder.
    * @throws IllegalArgumentException if {@code radius} is negative.
    */
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

   default void setSize(double length, double radius)
   {
      setLength(length);
      setRadius(radius);
   }

   void setIntermediateVariableSupplier(IntermediateVariableSupplier newSupplier);

   @Override
   default boolean containsNaN()
   {
      return Cylinder3DReadOnly.super.containsNaN();
   }

   /** {@inheritDoc} */
   @Override
   default void setToNaN()
   {
      getPose().setToNaN();
      setSize(Double.NaN, Double.NaN);
   }

   /** {@inheritDoc} */
   @Override
   default void setToZero()
   {
      getPose().setToZero();
      setSize(0.0, 0.0);
   }

   /**
    * Copies the {@code other} cylinder data into {@code this}.
    *
    * @param other the other cylinder to copy. Not modified.
    */
   default void set(Cylinder3DReadOnly other)
   {
      getPose().set(other.getPose());
      setLength(other.getLength());
      setRadius(other.getRadius());
   }

   default void set(RigidBodyTransformReadOnly pose, double length, double radius)
   {
      getPose().set(pose);
      setSize(length, radius);
   }

   default void set(Pose3DReadOnly pose, double length, double radius)
   {
      getPose().set(pose);
      setSize(length, radius);
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
