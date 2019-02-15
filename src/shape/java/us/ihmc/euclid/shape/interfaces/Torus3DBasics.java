package us.ihmc.euclid.shape.interfaces;

import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;

public interface Torus3DBasics extends Torus3DReadOnly, Shape3DBasics
{
   void setRadii(double radius, double tubeRadius);

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
      return Torus3DReadOnly.super.containsNaN();
   }

   /** {@inheritDoc} */
   @Override
   default void setToZero()
   {
      getPose().setToZero();
      setRadii(0.0, 0.0);
   }

   /** {@inheritDoc} */
   @Override
   default void setToNaN()
   {
      getPose().setToNaN();
      setRadii(Double.NaN, Double.NaN);
   }

   /**
    * Copies the {@code other} torus data into {@code this}.
    *
    * @param other the other torus to copy. Not modified.
    */
   default void set(Torus3DReadOnly other)
   {
      getPose().set(other.getPose());
      setRadii(other.getRadius(), other.getTubeRadius());
   }

   /**
    * Sets the pose and radii of this torus 3D.
    *
    * @param pose the position and orientation of this torus. Not modified.
    * @param radius radius from the torus center to the tube center.
    * @param tubeRadius radius of the torus' tube.
    * @throws IllegalArgumentException if {@code tubeRadius} is less than {@value #MIN_TUBE_RADIUS} or
    *            if the resulting inner radius is less than {@value #MIN_INNER_RADIUS}.
    */
   default void set(Pose3DReadOnly pose, double radius, double tubeRadius)
   {
      getPose().set(pose);
      setRadii(radius, tubeRadius);
   }

   /**
    * Sets the pose and radii of this torus 3D.
    *
    * @param pose the position and orientation of this torus. Not modified.
    * @param radius radius from the torus center to the tube center.
    * @param tubeRadius radius of the torus' tube.
    * @throws IllegalArgumentException if {@code tubeRadius} is less than {@value #MIN_TUBE_RADIUS} or
    *            if the resulting inner radius is less than {@value #MIN_INNER_RADIUS}.
    */
   default void set(RigidBodyTransformReadOnly pose, double radius, double tubeRadius)
   {
      getPose().set(pose);
      setRadii(radius, tubeRadius);
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
