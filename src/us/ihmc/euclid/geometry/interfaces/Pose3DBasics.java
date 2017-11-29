package us.ihmc.euclid.geometry.interfaces;

import us.ihmc.euclid.axisAngle.interfaces.AxisAngleReadOnly;
import us.ihmc.euclid.interfaces.Clearable;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.transform.QuaternionBasedTransform;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;

public interface Pose3DBasics extends Pose3DReadOnly, Clearable
{
   /**
    * Sets the x-coordinate of the position.
    *
    * @param x the x-coordinate of the position.
    */
   void setX(double x);

   /**
    * Sets the y-coordinate of the position.
    *
    * @param y the y-coordinate of the position.
    */
   void setY(double y);

   /**
    * Sets the z-coordinate of the position.
    *
    * @param z the z-coordinate of the position.
    */
   void setZ(double z);

   @Override
   default boolean containsNaN()
   {
      return getPosition().containsNaN() || getOrientation().containsNaN();
   }

   @Override
   default void setToNaN()
   {
      getPosition().setToNaN();
      getOrientation().setToNaN();
   }

   @Override
   default void setToZero()
   {
      getPosition().setToNaN();
      getOrientation().setToNaN();
   }

   /**
    * Sets the position coordinates.
    *
    * @param x the x-coordinate of the position.
    * @param y the y-coordinate of the position.
    * @param z the z-coordinate of the position.
    */
   default void setPosition(double x, double y, double z)
   {
      setX(x);
      setY(y);
      setZ(z);
   }

   /**
    * Sets the position to the given tuple.
    *
    * @param position the tuple with the new position coordinates. Not modified.
    */
   default void setPosition(Tuple3DReadOnly position)
   {
      setPosition(position.getX(), position.getY(), position.getZ());
   }

   /**
    * Sets the x and y coordinates from the given tuple 2D, the z coordinate remains unchanged.
    *
    * @param position2D the tuple with the new x and y coordinates. Not modified.
    */
   default void setPositionXY(Tuple2DReadOnly position2D)
   {
      setX(position2D.getX());
      setY(position2D.getY());
   }

   /**
    * Sets all the components of this pose 3D.
    * <p>
    * WARNING: the Euler angles or yaw-pitch-roll representation is sensitive to gimbal lock and is
    * sometimes undefined.
    * </p>
    *
    * @param x the x-coordinate of the position.
    * @param y the y-coordinate of the position.
    * @param z the z-coordinate of the position.
    * @param yaw the angle to rotate about the z-axis.
    * @param pitch the angle to rotate about the y-axis.
    * @param roll the angle to rotate about the x-axis.
    */
   default void set(double x, double y, double z, double yaw, double pitch, double roll)
   {
      setPosition(x, y, z);
      setOrientationYawPitchRoll(yaw, pitch, roll);
   }

   /**
    * Sets this pose 3D to the {@code other} pose 3D.
    *
    * @param other the other pose 3D. Not modified.
    */
   default void set(Pose3DReadOnly other)
   {
      setPosition(other.getPosition());
      setOrientation(other.getOrientation());
   }

   /**
    * Sets the orientation part of this pose 3D with the 4 components of a quaternion.
    * <p>
    * The quaternion is normalized internally.
    * </p>
    *
    * @param qx the x-component of the quaternion's vector part.
    * @param qy the y-component of the quaternion's vector part.
    * @param qz the z-component of the quaternion's vector part.
    * @param qs the scalar component of the quaternion.
    */
   default void setOrientation(double qx, double qy, double qz, double qs)
   {
      getOrientation().set(qx, qy, qz, qs);
   }

   /**
    * Sets the orientation part of this pose 3D with the given quaternion.
    *
    * @param orientation the quaternion used to set this pose's orientation. Not modified.
    */
   default void setOrientation(QuaternionReadOnly orientation)
   {
      getOrientation().set(orientation);
   }

   /**
    * Sets the orientation part of this pose 3D with the given rotation matrix.
    *
    * @param orientation the rotation matrix used to set this pose's orientation. Not modified.
    */
   default void setOrientation(RotationMatrixReadOnly orientation)
   {
      getOrientation().set(orientation);
   }

   /**
    * Sets the orientation part of this pose 3D with the given axis-angle.
    *
    * @param orientation the axis-angle used to set this pose's orientation. Not modified.
    */
   default void setOrientation(AxisAngleReadOnly orientation)
   {
      getOrientation().set(orientation);
   }

   /**
    * Sets the orientation part of this pose 3D with the given yaw, pitch, and roll angles.
    * <p>
    * WARNING: the Euler angles or yaw-pitch-roll representation is sensitive to gimbal lock and is
    * sometimes undefined.
    * </p>
    *
    * @param yawPitchRoll array containing the yaw-pitch-roll angles. Not modified.
    */
   default void setOrientationYawPitchRoll(double[] yawPitchRoll)
   {
      getOrientation().setYawPitchRoll(yawPitchRoll);
   }

   /**
    * Sets the orientation part of this pose 3D with the given yaw, pitch, and roll angles.
    * <p>
    * WARNING: the Euler angles or yaw-pitch-roll representation is sensitive to gimbal lock and is
    * sometimes undefined.
    * </p>
    *
    * @param yaw the angle to rotate about the z-axis.
    * @param pitch the angle to rotate about the y-axis.
    * @param roll the angle to rotate about the x-axis.
    */
   default void setOrientationYawPitchRoll(double yaw, double pitch, double roll)
   {
      getOrientation().setYawPitchRoll(yaw, pitch, roll);
   }

   /**
    * Sets this pose 3D to match the given rigid-body transform.
    *
    * @param rigidBodyTransform the transform use to set this pose 3D. Not modified.
    */
   default void set(RigidBodyTransform rigidBodyTransform)
   {
      setPosition(rigidBodyTransform.getTranslationVector());
      setOrientation(rigidBodyTransform.getRotationMatrix());
   }

   /**
    * Sets this pose 3D to match the given quaternion-based transform.
    *
    * @param quaternionBasedTransform the transform use to set this pose 3D. Not modified.
    */
   default void set(QuaternionBasedTransform quaternionBasedTransform)
   {
      setPosition(quaternionBasedTransform.getTranslationVector());
      setOrientation(quaternionBasedTransform.getQuaternion());
   }

   /**
    * Sets both position and orientation.
    *
    * @param position the tuple with the new position coordinates. Not modified.
    * @param orientation the quaternion with the new orientation. Not modified.
    */
   default void set(Tuple3DReadOnly position, QuaternionReadOnly orientation)
   {
      setOrientation(orientation);
      setPosition(position);
   }

   /**
    * Sets both position and orientation.
    *
    * @param position the tuple with the new position coordinates. Not modified.
    * @param orientation the axis-angle with the new orientation. Not modified.
    */
   default void set(Tuple3DReadOnly position, AxisAngleReadOnly orientation)
   {
      setOrientation(orientation);
      setPosition(position);
   }
}