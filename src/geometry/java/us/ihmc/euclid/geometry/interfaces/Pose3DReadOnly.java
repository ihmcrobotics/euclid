package us.ihmc.euclid.geometry.interfaces;

import us.ihmc.euclid.geometry.tools.EuclidGeometryIOTools;
import us.ihmc.euclid.interfaces.EuclidGeometry;
import us.ihmc.euclid.orientation.interfaces.Orientation3DBasics;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformBasics;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;

/**
 * Read-only interface for pose 3D.
 * <p>
 * A pose 3D represents a position and orientation in 3 dimensions.
 * </p>
 */
public interface Pose3DReadOnly extends RigidBodyTransformReadOnly, EuclidGeometry
{
   /**
    * Gets the read-only reference of the position part of this pose 3D.
    *
    * @return the read-only position part of this pose 3D.
    */
   Point3DReadOnly getPosition();

   /**
    * Gets the read-only reference to the orientation part of this pose 3D.
    *
    * @return the read-only orientation part of this pose 3D.
    */
   QuaternionReadOnly getOrientation();

   /**
    * Gets the read-only reference of the position part of this pose 3D.
    * <p>
    * Same as {@link #getPosition()}, it is needed only to comply to the
    * {@code RigidBodyTransformReadOnly} interface.
    * </p>
    *
    * @return the read-only position part of this pose 3D.
    */
   @Override
   default Point3DReadOnly getTranslation()
   {
      return getPosition();
   }

   /**
    * Gets the read-only reference to the orientation part of this pose 3D.
    * <p>
    * Same as {@link #getOrientation()}, it is needed only to comply to the
    * {@code RigidBodyTransformReadOnly} interface.
    * </p>
    *
    * @return the read-only orientation part of this pose 3D.
    */
   @Override
   default QuaternionReadOnly getRotation()
   {
      return getOrientation();
   }

   /**
    * Gets the x-coordinate of the position part of this pose 3D.
    *
    * @return the x-coordinate of this pose 3D.
    */
   default double getX()
   {
      return getPosition().getX();
   }

   /**
    * Gets the y-coordinate of the position part of this pose 3D.
    *
    * @return the y-coordinate of this pose 3D.
    */
   default double getY()
   {
      return getPosition().getY();
   }

   /**
    * Gets the z-coordinate of the position part of this pose 3D.
    *
    * @return the z-coordinate of this pose 3D.
    */
   default double getZ()
   {
      return getPosition().getZ();
   }

   /**
    * Computes and returns the yaw angle from the yaw-pitch-roll representation of the orientation part
    * of this pose 3D.
    * <p>
    * WARNING: the Euler angles or yaw-pitch-roll representation is sensitive to gimbal lock and is
    * sometimes undefined.
    * </p>
    *
    * @return the yaw angle around the z-axis.
    */
   default double getYaw()
   {
      return getOrientation().getYaw();
   }

   /**
    * Computes and returns the pitch angle from the yaw-pitch-roll representation of the orientation
    * part of this pose 3D.
    * <p>
    * WARNING: the Euler angles or yaw-pitch-roll representation is sensitive to gimbal lock and is
    * sometimes undefined.
    * </p>
    *
    * @return the pitch angle around the y-axis.
    */
   default double getPitch()
   {
      return getOrientation().getPitch();
   }

   /**
    * Computes and returns the roll angle from the yaw-pitch-roll representation of the orientation
    * part of this pose 3D.
    * <p>
    * WARNING: the Euler angles or yaw-pitch-roll representation is sensitive to gimbal lock and is
    * sometimes undefined.
    * </p>
    *
    * @return the roll angle around the x-axis.
    */
   default double getRoll()
   {
      return getOrientation().getRoll();
   }

   /**
    * Tests if this pose contains a {@link Double#NaN}.
    *
    * @return {@code true} if this pose contains a {@link Double#NaN}, {@code false} otherwise.
    */
   @Override
   default boolean containsNaN()
   {
      return getOrientation().containsNaN() || getPosition().containsNaN();
   }

   /**
    * Gets the position and orientation parts of this pose 3D.
    *
    * @param positionToPack    tuple in which the position is stored. Modified.
    * @param orientationToPack orientation in which the orientation is stored. Modified.
    */
   default void get(Tuple3DBasics positionToPack, Orientation3DBasics orientationToPack)
   {
      positionToPack.set(getPosition());
      orientationToPack.set(getOrientation());
   }

   /**
    * Packs this pose 3D into the given {@code transformToPack}.
    *
    * @param transformToPack the rigid-body transform that is set to represent this pose 3D. Modified.
    */
   default void get(RigidBodyTransformBasics transformToPack)
   {
      transformToPack.set(getOrientation(), getPosition());
   }

   /** {@inheritDoc} */
   @Override
   default boolean epsilonEquals(EuclidGeometry geometry, double epsilon)
   {
      if (geometry == this)
         return true;
      if (geometry == null)
         return false;
      if (!(geometry instanceof Pose3DReadOnly))
         return false;

      Pose3DReadOnly other = (Pose3DReadOnly) geometry;
      return getPosition().epsilonEquals(other.getPosition(), epsilon) && getOrientation().epsilonEquals(other.getOrientation(), epsilon);
   }

   /** {@inheritDoc} */
   @Override
   default boolean equals(EuclidGeometry geometry)
   {
      if (geometry == this)
         return true;
      if (geometry == null)
         return false;
      if (!(geometry instanceof Pose3DReadOnly))
         return false;
      Pose3DReadOnly other = (Pose3DReadOnly) geometry;
      return getPosition().equals(other.getPosition()) && getOrientation().equals(other.getOrientation());
   }

   /**
    * Gets a representative {@code String} of this pose 3D given a specific format to use.
    * <p>
    * Using the default format {@link EuclidCoreIOTools#DEFAULT_FORMAT}, this provides a {@code String}
    * as follows:
    *
    * <pre>
    * Pose 3D: position = ( 0.174, -0.452, -0.222 ), orientation = (-0.052, -0.173, -0.371,  0.087 )
    * </pre>
    * </p>
    */
   @Override
   default String toString(String format)
   {
      return EuclidGeometryIOTools.getPose3DString(format, this);
   }
}
