package us.ihmc.euclid.shape.primitives.interfaces;

import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;

public interface Ellipsoid3DBasics extends Ellipsoid3DReadOnly, Shape3DBasics
{
   @Override
   Vector3DBasics getRadii();

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
      return Ellipsoid3DReadOnly.super.containsNaN();
   }

   /** {@inheritDoc} */
   @Override
   default void setToNaN()
   {
      getPose().setToNaN();
      getRadii().setToNaN();
   }

   /** {@inheritDoc} */
   @Override
   default void setToZero()
   {
      getPose().setToZero();
      getRadii().setToZero();
   }

   /**
    * Copies the {@code other} ellipsoid data into {@code this}.
    *
    * @param other the other ellipsoid to copy. Not modified.
    */
   default void set(Ellipsoid3DReadOnly other)
   {
      getPose().set(other.getPose());
      getRadii().set(other.getRadii());
   }

   /**
    * Sets the pose and radii of this ellipsoid.
    *
    * @param pose the position and orientation of this ellipsoid. Not modified.
    * @param radiusX radius of the ellipsoid along the x-axis.
    * @param radiusY radius of the ellipsoid along the y-axis.
    * @param radiusZ radius of the ellipsoid along the z-axis.
    * @throws IllegalArgumentException if any of the three radii is negative.
    */
   default void set(Pose3DReadOnly pose, double radiusX, double radiusY, double radiusZ)
   {
      getPose().set(pose);
      setRadii(radiusX, radiusY, radiusZ);
   }

   /**
    * Sets the pose and radii of this ellipsoid.
    *
    * @param pose the position and orientation of this ellipsoid. Not modified.
    * @param radiusX radius of the ellipsoid along the x-axis.
    * @param radiusY radius of the ellipsoid along the y-axis.
    * @param radiusZ radius of the ellipsoid along the z-axis.
    * @throws IllegalArgumentException if any of the three radii is negative.
    */
   default void set(RigidBodyTransformReadOnly pose, double radiusX, double radiusY, double radiusZ)
   {
      getPose().set(pose);
      setRadii(radiusX, radiusY, radiusZ);
   }

   /**
    * Sets the radii of this ellipsoid.
    *
    * @param radii tuple holding the 3 radii of the ellipsoid.
    * @throws IllegalArgumentException if any of the three radii is negative.
    */
   default void setRadii(Tuple3DReadOnly radii)
   {
      getRadii().set(radii);
   }

   /**
    * Sets the radii of this ellipsoid.
    *
    * @param radiusX radius of the ellipsoid along the x-axis.
    * @param radiusY radius of the ellipsoid along the y-axis.
    * @param radiusZ radius of the ellipsoid along the z-axis.
    * @throws IllegalArgumentException if any of the three radii is negative.
    */
   default void setRadii(double radiusX, double radiusY, double radiusZ)
   {
      getRadii().set(radiusX, radiusY, radiusZ);
   }

   /**
    * Sets the radius along the x-axis for this ellipsoid.
    *
    * @param radiusX radius of the ellipsoid along the x-axis.
    * @throws IllegalArgumentException if the argument is negative.
    */
   default void setRadiusX(double radiusX)
   {
      getRadii().setX(radiusX);
   }

   /**
    * Sets the radius along the y-axis for this ellipsoid.
    *
    * @param radiusY radius of the ellipsoid along the y-axis.
    * @throws IllegalArgumentException if the argument is negative.
    */
   default void setRadiusY(double radiusY)
   {
      getRadii().setY(radiusY);
   }

   /**
    * Sets the radius along the z-axis for this ellipsoid.
    *
    * @param radiusZ radius of the ellipsoid along the z-axis.
    * @throws IllegalArgumentException if the argument is negative.
    */
   default void setRadiusZ(double radiusZ)
   {
      getRadii().setZ(radiusZ);
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
