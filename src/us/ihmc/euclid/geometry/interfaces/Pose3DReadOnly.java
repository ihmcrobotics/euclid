package us.ihmc.euclid.geometry.interfaces;

import us.ihmc.euclid.axisAngle.interfaces.AxisAngleBasics;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.transform.QuaternionBasedTransform;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;

public interface Pose3DReadOnly
{
   /**
    * Gets the read-only reference of the position part of this pose 3D.
    *
    * @return the position part of this pose 3D.
    */
   Point3DReadOnly getPosition();

   /**
    * Gets the read-only reference to the orientation part of this pose 3D.
    *
    * @return the orientation part of this pose 3D.
    */
   QuaternionReadOnly getOrientation();

   /**
    * Gets the x-coordinate of the position part of this pose 3D.
    *
    * @return the x-coordinate of this pose 3D.
    */
   double getX();

   /**
    * Gets the y-coordinate of the position part of this pose 3D.
    *
    * @return the y-coordinate of this pose 3D.
    */
   double getY();

   /**
    * Gets the z-coordinate of the position part of this pose 3D.
    *
    * @return the z-coordinate of this pose 3D.
    */
   double getZ();

   /**
    * Computes and returns the yaw angle from the yaw-pitch-roll representation of the orientation
    * part of this pose 3D.
    * <p>
    * WARNING: the Euler angles or yaw-pitch-roll representation is sensitive to gimbal lock and is
    * sometimes undefined.
    * </p>
    *
    * @return the yaw angle around the z-axis.
    */
   double getYaw();

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
   double getPitch();

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
   double getRoll();

   /**
    * Packs the position part of this pose 3D into the given {@code positionToPack}.
    *
    * @param positionToPack tuple used to store the position coordinates. Modified.
    */
   default void getPosition(Tuple3DBasics positionToPack)
   {
      positionToPack.set(getPosition());
   }

   /**
    * Packs the orientation part of this pose 3D into the given {@code orientationToPack}.
    *
    * @param orientationToPack used to store the orientation of this pose 3D. Modified.
    */
   default void getOrientation(RotationMatrix orientationToPack)
   {
      orientationToPack.set(getOrientation());
   }

   /**
    * Packs the orientation part of this pose 3D into the given {@code orientationToPack}.
    *
    * @param orientationToPack used to store the orientation of this pose 3D. Modified.
    */
   default void getOrientation(QuaternionBasics orientationToPack)
   {
      orientationToPack.set(getOrientation());
   }

   /**
    * Packs the orientation part of this pose 3D into the given {@code orientationToPack}.
    *
    * @param orientationToPack used to store the orientation of this pose 3D. Modified.
    */
   default void getOrientation(AxisAngleBasics orientationToPack)
   {
      orientationToPack.set(getOrientation());
   }

   /**
    * Computes and packs the orientation described by the orientation part of this pose 3D as the
    * yaw-pitch-roll angles.
    * <p>
    * WARNING: the Euler angles or yaw-pitch-roll representation is sensitive to gimbal lock and is
    * sometimes undefined.
    * </p>
    *
    * @param yawPitchRollToPack the array in which the yaw-pitch-roll angles are stored. Modified.
    */
   default void getOrientationYawPitchRoll(double[] yawPitchRollToPack)
   {
      getOrientation().getYawPitchRoll(yawPitchRollToPack);
   }

   /**
    * Computes and packs the orientation described by the orientation part of this pose as a
    * rotation vector.
    * <p>
    * WARNING: a rotation vector is different from a yaw-pitch-roll or Euler angles representation.
    * A rotation vector is equivalent to the axis of an axis-angle that is multiplied by the angle
    * of the same axis-angle.
    * </p>
    *
    * @param rotationVectorToPack the vector in which the rotation vector is stored. Modified.
    */
   default void getRotationVector(Vector3DBasics rotationVectorToPack)
   {
      getOrientation().get(rotationVectorToPack);
   }

   /**
    * Packs this pose 3D into the given {@code transformToPack}.
    *
    * @param transformToPack the rigid-body transform that is set to represent this pose 3D.
    *           Modified.
    */
   default void get(RigidBodyTransform transformToPack)
   {
      transformToPack.set(getOrientation(), getPosition());
   }

   /**
    * Packs this pose 3D into the given {@code transformToPack}.
    *
    * @param transformToPack the quaternion-based transform that is set to represent this pose 3D.
    *           Modified.
    */
   default void get(QuaternionBasedTransform transformToPack)
   {
      transformToPack.set(getOrientation(), getPosition());
   }
   
   /**
    * Computes the distance between the position of this pose 3D and the given {@code point}.
    *
    * @param point the other point used to measure the distance. Not modified.
    * @return the distance between this pose and the given {@code point}.
    */
   default double getPositionDistance(Point3DReadOnly point)
   {
      return getPosition().distance(point);
   }

   /**
    * Computes the distances between the position part of the two poses.
    *
    * @param other the other pose used to measure the distance. Not modified.
    * @return the distance between the position part of the two poses.
    */
   default double getPositionDistance(Pose3DReadOnly other)
   {
      return getPosition().distance(other.getPosition());
   }

   /**
    * Computes the smallest angle representing the difference between the orientation part of this
    * pose 3D and the give {@code orientation}.
    *
    * @param orientation the orientation used to compute the orientation distance. Not modified.
    * @return the angle difference between {@code this} and {@code orientation}, it is contained in
    *         [0, 2<i>pi</i>].
    */
   default double getOrientationDistance(QuaternionReadOnly orientation)
   {
      return getOrientation().distance(orientation);
   }

   /**
    * Computes the absolute angle difference between this pose 3D and {@code other}.
    *
    * @param other the other pose 3D used to compute the orientation distance. Not modified.
    * @return the angle difference between {@code this.orientation} and {@code other.orientation},
    *         it is contained in [0, 2<i>pi</i>].
    */
   default double getOrientationDistance(Pose3DReadOnly other)
   {
      return getOrientation().distance(other.getOrientation());
   }

   /**
    * Tests on a per-component basis if this pose is equal to {@code other} with the tolerance
    * {@code epsilon}.
    *
    * @param other the query. Not modified.
    * @param epsilon the tolerance to use.
    * @return {@code true} if the two poses are equal, {@code false} otherwise.
    */
   default boolean epsilonEquals(Pose3DReadOnly other, double epsilon)
   {
      return epsilonEquals(other, epsilon, epsilon);
   }

   /**
    * Tests on a per-component basis if this pose is equal to {@code other} with separate tolerances
    * for the position {@code positionEpsilon} and the orientation {@code orientationEpsilon}.
    *
    * @param other the query. Not modified.
    * @param positionEpsilon the tolerance to use for comparing the position part.
    * @param orientationEpsilon the tolerance to use for comparing the orientation part.
    * @return {@code true} if the two poses are equal, {@code false} otherwise.
    */
   default boolean epsilonEquals(Pose3DReadOnly other, double positionEpsilon, double orientationEpsilon)
   {
      return getPosition().epsilonEquals(other.getPosition(), positionEpsilon) && getOrientation().epsilonEquals(other.getOrientation(), orientationEpsilon);
   }

   /**
    * Compares {@code this} to {@code other} to determine if the two poses are geometrically
    * similar.
    * <p>
    * Two poses are geometrically equal if both their position and orientation are geometrically
    * equal.
    * </p>
    *
    * @param other the pose to compare to. Not modified.
    * @param epsilon the tolerance of the comparison.
    * @return {@code true} if the two poses represent the same geometry, {@code false} otherwise.
    */
   default boolean geometricallyEquals(Pose3DReadOnly other, double epsilon)
   {
      return this.getPosition().geometricallyEquals(other.getPosition(), epsilon) && this.getOrientation().geometricallyEquals(other.getOrientation(), epsilon);
   }
}
