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
 * Write and read interface for ellipsoid 3D.
 * <p>
 * A ellipsoid 3D is represented by its radii, the position of its center, and its orientation.
 * </p>
 *
 * @author Sylvain Bertrand
 */
public interface Ellipsoid3DBasics extends Ellipsoid3DReadOnly, Shape3DBasics
{
   /**
    * Get the reference to the radii along the three local axes of this ellipsoid.
    *
    * @return the size of this ellipsoid.
    */
   @Override
   Vector3DBasics getRadii();

   /**
    * Gets the reference to the pose of this ellipsoid.
    * <p>
    * The position part describes the coordinates of the center.
    * </p>
    *
    * @return the pose of this ellipsoid.
    */
   @Override
   Shape3DPoseBasics getPose();

   /**
    * Gets the reference to the orientation of this ellipsoid.
    *
    * @return the orientation of this ellipsoid.
    */
   @Override
   default RotationMatrixBasics getOrientation()
   {
      return getPose().getShapeOrientation();
   }

   /**
    * Gets the reference of the position of this ellipsoid.
    *
    * @return the position of this ellipsoid.
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
    * Sets this ellipsoid properties.
    *
    * @param position    the position of this ellipsoid center. Not modified.
    * @param orientation the orientation of this ellipsoid. Not modified.
    * @param radiusX     radius of the ellipsoid along the x-axis.
    * @param radiusY     radius of the ellipsoid along the y-axis.
    * @param radiusZ     radius of the ellipsoid along the z-axis.
    * @throws IllegalArgumentException if any of the three radii is negative.
    */
   default void set(Point3DReadOnly position, Orientation3DReadOnly orientation, double radiusX, double radiusY, double radiusZ)
   {
      getPose().set(orientation, position);
      getRadii().set(radiusX, radiusY, radiusZ);
   }

   /**
    * Sets the pose and radii of this ellipsoid.
    *
    * @param pose    the position and orientation of this ellipsoid. Not modified.
    * @param radiusX radius of the ellipsoid along the x-axis.
    * @param radiusY radius of the ellipsoid along the y-axis.
    * @param radiusZ radius of the ellipsoid along the z-axis.
    * @throws IllegalArgumentException if any of the three radii is negative.
    */
   default void set(Pose3DReadOnly pose, double radiusX, double radiusY, double radiusZ)
   {
      getPose().set(pose);
      getRadii().set(radiusX, radiusY, radiusZ);
   }

   /**
    * Sets the pose and radii of this ellipsoid.
    *
    * @param pose    the position and orientation of this ellipsoid. Not modified.
    * @param radiusX radius of the ellipsoid along the x-axis.
    * @param radiusY radius of the ellipsoid along the y-axis.
    * @param radiusZ radius of the ellipsoid along the z-axis.
    * @throws IllegalArgumentException if any of the three radii is negative.
    */
   default void set(RigidBodyTransformReadOnly pose, double radiusX, double radiusY, double radiusZ)
   {
      getPose().set(pose);
      getRadii().set(radiusX, radiusY, radiusZ);
   }

   /**
    * Sets the pose and radii of this ellipsoid.
    *
    * @param pose  the position and orientation of this ellipsoid. Not modified.
    * @param radii the radii of this ellipsoid along the x, y, and z axes in order. Not modified.
    * @throws IllegalArgumentException if any of the three radii is negative.
    */
   default void set(RigidBodyTransformReadOnly pose, double[] radii)
   {
      getPose().set(pose);
      getRadii().set(radii[0], radii[1], radii[2]);
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

   Ellipsoid3DBasics copy();

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
