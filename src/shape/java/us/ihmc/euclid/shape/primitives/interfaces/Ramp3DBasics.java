package us.ihmc.euclid.shape.primitives.interfaces;

import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixBasics;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;

/**
 * Write and read interface for a ramp 3D.
 * <p>
 * A ramp represents a 3D shape with a triangular section in the XZ-plane. Shape description:
 * <ul>
 * <li>The slope face starts from {@code x=0.0}, {@code z=0.0} to end at {@code x=size.getX()},
 * {@code z=size.getZ()}.
 * <li>The bottom face is horizontal (XY-plane) at {@code z=0.0}.
 * <li>The rear face is vertical (YZ-plane) at {@code x=size.getX()}.
 * <li>The left face is vertical (XZ-plane) at {@code y=-size.getY()/2.0}.
 * <li>The right face is vertical (XZ-plane) at {@code y=size.getY()/2.0}.
 * </ul>
 * </p>
 *
 * @author Sylvain Bertrand
 */
public interface Ramp3DBasics extends Ramp3DReadOnly, Shape3DBasics
{
   /**
    * Get the reference to the size along the three local axes of this ramp.
    *
    * @return the size of this ramp.
    */
   @Override
   Vector3DBasics getSize();

   /**
    * Gets the reference to the pose of this ramp.
    *
    * @return the pose of this ramp.
    */
   @Override
   Shape3DPoseBasics getPose();

   /**
    * Gets the reference to the orientation of this ramp.
    *
    * @return the orientation of this ramp.
    */
   @Override
   default RotationMatrixBasics getOrientation()
   {
      return getPose().getShapeOrientation();
   }

   /**
    * Gets the reference of the position of this ramp.
    *
    * @return the position of this ramp.
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
      return Ramp3DReadOnly.super.containsNaN();
   }

   /** {@inheritDoc} */
   @Override
   default void setToZero()
   {
      getPose().setToZero();
      getSize().setToZero();
   }

   /** {@inheritDoc} */
   @Override
   default void setToNaN()
   {
      getPose().setToNaN();
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
      getSize().set(other.getSize());
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
    * Sets this ramp properties.
    *
    * @param position    the position of this ramp. Not modified.
    * @param orientation the orientation of this ramp. Not modified.
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
    * Sets this ramp properties.
    *
    * @param pose  the pose of this ramp. Not modified.
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
    * Sets this ramp properties.
    *
    * @param pose  the pose of this ramp. Not modified.
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
    * Sets this ramp properties.
    *
    * @param pose the pose of this ramp. Not modified.
    * @param size the size of this ramp along the x, y, and axes in order. Not modified.
    * @throws IllegalArgumentException if any of the three size arguments is negative.
    */
   default void set(RigidBodyTransformReadOnly pose, double[] size)
   {
      getPose().set(pose);
      setSize(size[0], size[1], size[2]);
   }

   /**
    * Sets the size of this ramp.
    *
    * @param sizeX the size of this ramp along the x-axis.
    * @param sizeY the size of this ramp along the y-axis.
    * @param sizeZ the size of this ramp along the z-axis.
    * @throws IllegalArgumentException if any of {@code length}, {@code width}, or {@code height} is
    *                                  negative.
    */
   default void setSize(double sizeX, double sizeY, double sizeZ)
   {
      getSize().set(sizeX, sizeY, sizeZ);
   }

   @Override
   Ramp3DBasics copy();

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
