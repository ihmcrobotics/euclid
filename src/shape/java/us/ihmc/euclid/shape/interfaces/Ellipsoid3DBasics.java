package us.ihmc.euclid.shape.interfaces;

import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;

public interface Ellipsoid3DBasics extends Ellipsoid3DReadOnly, Shape3DBasics
{
   @Override
   Vector3DBasics getRadii();

   @Override
   default boolean containsNaN()
   {
      return Ellipsoid3DReadOnly.super.containsNaN();
   }

   /** {@inheritDoc} */
   @Override
   default void setToNaN()
   {
      Shape3DBasics.super.setToNaN();
      getRadii().setToNaN();
   }

   /** {@inheritDoc} */
   @Override
   default void setToZero()
   {
      Shape3DBasics.super.setToZero();
      getRadii().setToZero();
   }

   /**
    * Copies the {@code other} ellipsoid data into {@code this}.
    *
    * @param other the other ellipsoid to copy. Not modified.
    */
   default void set(Ellipsoid3DReadOnly other)
   {
      setPose(other);
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
      setPose(pose);
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
      setPose(pose);
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

}
