package us.ihmc.euclid.shape.primitives.interfaces;

import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;

/**
 * Write and read interface for a box 3D.
 * <p>
 * A box 3D is represented by its size, the position of its center, and its orientation.
 * </p>
 *
 * @author Sylvain Bertrand
 */
public interface Box3DBasics extends Box3DReadOnly, Shape3DBasics
{
   /**
    * Get the reference to the size along the three local axes of this box.
    *
    * @return the size of this box.
    */
   @Override
   Vector3DBasics getSize();

   /**
    * Gets the reference to the pose of this box.
    * <p>
    * The position part describes the coordinates of the center.
    * </p>
    *
    * @return the pose of this box.
    */
   @Override
   Shape3DPoseBasics getPose();

   /**
    * Gets the reference to the orientation of this box.
    *
    * @return the orientation of this box.
    */
   @Override
   default RotationMatrix getOrientation()
   {
      return getPose().getShapeOrientation();
   }

   /**
    * Gets the reference of the position of this box.
    *
    * @return the position of this box.
    */
   @Override
   default Point3DBasics getPosition()
   {
      return getPose().getShapePosition();
   }

   /**
    * Changes the variable supplier to use with this shape.
    *
    * @param newSupplier the new variable supplier.
    */
   void setIntermediateVariableSupplier(IntermediateVariableSupplier newSupplier);

   /** {@inheritDoc} */
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

   /**
    * Sets this box properties.
    *
    * @param position    the position of this box center. Not modified.
    * @param orientation the orientation of this box. Not modified.
    * @param sizeX       the size along the x-axis.
    * @param sizeY       the size along the y-axis.
    * @param sizeZ       the size along the z-axis.
    * @throws IllegalArgumentException if any of the three size arguments is negative.
    */
   default void set(Point3DReadOnly position, Orientation3DReadOnly orientation, double sizeX, double sizeY, double sizeZ)
   {
      getPose().set(orientation, position);
      setSize(sizeX, sizeY, sizeZ);
   }

   /**
    * Sets this box properties.
    *
    * @param pose  the pose of this box. Not modified.
    * @param sizeX the size along the x-axis.
    * @param sizeY the size along the y-axis.
    * @param sizeZ the size along the z-axis.
    * @throws IllegalArgumentException if any of the three size arguments is negative.
    */
   default void set(Pose3DReadOnly pose, double sizeX, double sizeY, double sizeZ)
   {
      getPose().set(pose);
      setSize(sizeX, sizeY, sizeZ);
   }

   /**
    * Sets this box properties.
    *
    * @param pose  the pose of this box. Not modified.
    * @param sizeX the size along the x-axis.
    * @param sizeY the size along the y-axis.
    * @param sizeZ the size along the z-axis.
    * @throws IllegalArgumentException if any of the three size arguments is negative.
    */
   default void set(RigidBodyTransformReadOnly pose, double sizeX, double sizeY, double sizeZ)
   {
      getPose().set(pose);
      setSize(sizeX, sizeY, sizeZ);
   }

   /**
    * Sets this box properties.
    *
    * @param pose the pose of this box. Not modified.
    * @param size the size of this box along the x, y, and axes in order. Not modified.
    * @throws IllegalArgumentException if any of the three size arguments is negative.
    */
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
