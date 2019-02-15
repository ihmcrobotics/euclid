package us.ihmc.euclid.shape.interfaces;

import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;

public interface Box3DBasics extends Box3DReadOnly, Shape3DBasics
{
   Vector3DBasics getSize();

   void setIntermediateVariableSupplier(IntermediateVariableSupplier newSupplier);

   @Override
   default boolean containsNaN()
   {
      return Box3DReadOnly.super.containsNaN();
   }

   /** {@inheritDoc} */
   @Override
   default void setToNaN()
   {
      getPose().setToNaN();
      getSize().setToNaN();
   }

   /** {@inheritDoc} */
   @Override
   default void setToZero()
   {
      getPose().setToZero();
      getSize().setToZero();
   }

   /**
    * Copies the {@code other} box data into {@code this}.
    *
    * @param other the other box to copy. Not modified.
    */
   default void set(Box3DReadOnly other)
   {
      getPose().set(other.getPose());
      getSize().set(other.getSize());
   }

   default void set(Point3DReadOnly position, Orientation3DReadOnly orientation, double length, double width, double height)
   {
      getPose().set(orientation, position);
      setSize(length, width, height);
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

   default void set(RigidBodyTransformReadOnly pose, double[] size)
   {
      getPose().set(pose);
      setSize(size[0], size[1], size[2]);
   }

   /**
    * Sets the size of this box.
    *
    * @param sizeX the size of this box along the x-axis.
    * @param sizeY the size of this box along the y-axis.
    * @param sizeZ the size of this box along the z-axis.
    * @throws IllegalArgumentException if any of the three arguments is negative.
    */
   default void setSize(double sizeX, double sizeY, double sizeZ)
   {
      getSize().set(sizeX, sizeY, sizeZ);
   }

   /**
    * Applies the given scale factor to the size of this box.
    *
    * @param scale the scale factor to use.
    * @throws IllegalArgumentException if {@code scale} is negative.
    */
   default void scale(double scale)
   {
      if (scale < 0.0)
         throw new IllegalArgumentException("Cannot apply a negative scale: " + scale);
      getSize().scale(scale);
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
