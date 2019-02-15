package us.ihmc.euclid.shape.interfaces;

import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;

public interface Ramp3DBasics extends Ramp3DReadOnly, Shape3DBasics
{
   @Override
   Vector3DBasics getSize();

   void setIntermediateVariableSupplier(IntermediateVariableSupplier newSupplier);

   @Override
   default boolean containsNaN()
   {
      return Ramp3DReadOnly.super.containsNaN();
   }

   /** {@inheritDoc} */
   @Override
   default void setToZero()
   {
      Shape3DBasics.super.setToZero();
      getSize().setToZero();
   }

   /** {@inheritDoc} */
   @Override
   default void setToNaN()
   {
      Shape3DBasics.super.setToNaN();
      getSize().setToNaN();
   }

   /**
    * Copies the {@code other} ramp data into {@code this}.
    *
    * @param other the other ramp to copy. Not modified.
    */
   default void set(Ramp3DReadOnly other)
   {
      getPose().set(other.getPose());
      setSize(other.getSize());
   }

   /**
    * Sets the size along the x-axis for this ramp.
    *
    * @param sizeX the size of this ramp along the x-axis.
    * @throws IllegalArgumentException if {@code length} is negative.
    */
   default void setSizeX(double sizeX)
   {
      getSize().setX(sizeX);
   }

   /**
    * Sets the size along the y-axis for this ramp.
    *
    * @param sizeY the size of this ramp along the y-axis.
    * @throws IllegalArgumentException if {@code width} is negative.
    */
   default void setSizeY(double sizeY)
   {
      getSize().setY(sizeY);
   }

   /**
    * Sets the size along the z-axis for this ramp.
    *
    * @param sizeZ the size of this ramp along the z-axis.
    * @throws IllegalArgumentException if {@code height} is negative.
    */
   default void setSizeZ(double sizeZ)
   {
      getSize().setZ(sizeZ);
   }

   /**
    * Sets the size of this ramp.
    *
    * @param size tuple with the new size for this ramp. Not modified.
    * @throws IllegalArgumentException if any of {@code size} components is negative.
    */
   default void setSize(Tuple3DReadOnly size)
   {
      getSize().set(size);
   }

   default void set(Pose3DReadOnly pose, double length, double width, double height)
   {
      getPose().set(pose);
      setSize(length, width, height);
   }

   default void set(RigidBodyTransformReadOnly pose, double length, double width, double height)
   {
      getPose().set(pose);
      setSize(length, width, height);
   }

   /**
    * Sets the size of this ramp.
    *
    * @param sizeX the size of this ramp along the x-axis.
    * @param sizeY the size of this ramp along the y-axis.
    * @param sizeZ the size of this ramp along the z-axis.
    * @throws IllegalArgumentException if any of {@code length}, {@code width}, or {@code height} is
    *            negative.
    */
   default void setSize(double sizeX, double sizeY, double sizeZ)
   {
      getSize().set(sizeX, sizeY, sizeZ);
   }

}
