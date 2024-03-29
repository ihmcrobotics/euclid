package us.ihmc.euclid.geometry.interfaces;

import us.ihmc.euclid.interfaces.Clearable;
import us.ihmc.euclid.interfaces.Transformable;
import us.ihmc.euclid.orientation.interfaces.Orientation3DBasics;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.tools.QuaternionTools;
import us.ihmc.euclid.tools.RotationMatrixTools;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformBasics;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;

/**
 * Write and read interface for pose 3D.
 * <p>
 * A pose 3D represents a position and orientation in 3 dimensions.
 * </p>
 */
public interface Pose3DBasics extends Pose3DReadOnly, Transformable, Clearable, RigidBodyTransformBasics
{
   /**
    * Sets the x-coordinate of the position.
    *
    * @param x the x-coordinate of the position.
    */
   default void setX(double x)
   {
      getPosition().setX(x);
   }

   /**
    * Sets the y-coordinate of the position.
    *
    * @param y the y-coordinate of the position.
    */
   default void setY(double y)
   {
      getPosition().setY(y);
   }

   /**
    * Sets the z-coordinate of the position.
    *
    * @param z the z-coordinate of the position.
    */
   default void setZ(double z)
   {
      getPosition().setZ(z);
   }

   /**
    * Gets the reference of the position part of this pose 3D.
    *
    * @return the position part of this pose 3D.
    */
   @Override
   Point3DBasics getPosition();

   /**
    * Gets the reference to the orientation part of this pose 3D.
    *
    * @return the orientation part of this pose 3D.
    */
   @Override
   QuaternionBasics getOrientation();

   /**
    * Gets the reference of the position part of this pose 3D.
    * <p>
    * Same as {@link #getPosition()}, it is needed only to comply to the
    * {@code RigidBodyTransformBasics} interface.
    * </p>
    *
    * @return the position part of this pose 3D.
    */
   @Override
   default Point3DBasics getTranslation()
   {
      return getPosition();
   }

   /**
    * Gets the reference to the orientation part of this pose 3D.
    * <p>
    * Same as {@link #getOrientation()}, it is needed only to comply to the
    * {@code RigidBodyTransformReadOnly} interface.
    * </p>
    *
    * @return the orientation part of this pose 3D.
    */
   @Override
   default QuaternionBasics getRotation()
   {
      return getOrientation();
   }

   /**
    * Sets all the components of this pose 3D.
    * <p>
    * WARNING: the Euler angles or yaw-pitch-roll representation is sensitive to gimbal lock and is
    * sometimes undefined.
    * </p>
    *
    * @param x     the x-coordinate of the position.
    * @param y     the y-coordinate of the position.
    * @param z     the z-coordinate of the position.
    * @param yaw   the angle to rotate about the z-axis.
    * @param pitch the angle to rotate about the y-axis.
    * @param roll  the angle to rotate about the x-axis.
    */
   default void set(double x, double y, double z, double yaw, double pitch, double roll)
   {
      getPosition().set(x, y, z);
      getOrientation().setYawPitchRoll(yaw, pitch, roll);
   }

   /**
    * Sets this pose 3D to the given {@code pose2DReadOnly}.
    *
    * @param pose2DReadOnly the pose 2D used to set this pose 3D. Not modified.
    */
   default void set(Pose2DReadOnly pose2DReadOnly)
   {
      getPosition().set(pose2DReadOnly.getPosition(), 0.0);
      getOrientation().set(pose2DReadOnly.getOrientation());
   }

   /**
    * Sets this pose 3D to the {@code other} pose 3D.
    *
    * @param other the other pose 3D. Not modified.
    */
   default void set(Pose3DReadOnly other)
   {
      getPosition().set(other.getPosition());
      getOrientation().set(other.getOrientation());
   }

   /**
    * Sets this pose 3D to match the given rigid-body transform.
    *
    * @param rigidBodyTransform the transform use to set this pose 3D. Not modified.
    */
   @Override
   default void set(RigidBodyTransformReadOnly rigidBodyTransform)
   {
      getPosition().set(rigidBodyTransform.getTranslation());
      getOrientation().set(rigidBodyTransform.getRotation());
   }

   /**
    * Sets both position and orientation.
    *
    * @param position    the tuple with the new position coordinates. Not modified.
    * @param orientation the new orientation. Not modified.
    */
   default void set(Tuple3DReadOnly position, Orientation3DReadOnly orientation)
   {
      getOrientation().set(orientation);
      getPosition().set(position);
   }

   /** {@inheritDoc} */
   @Override
   default boolean containsNaN()
   {
      return getOrientation().containsNaN() || getPosition().containsNaN();
   }

   /** {@inheritDoc} */
   @Override
   default void setToNaN()
   {
      getOrientation().setToNaN();
      getPosition().setToNaN();
   }

   /**
    * Sets the position to (0, 0) and the orientation to the neutral quaternion, i.e. zero rotation.
    */
   @Override
   default void setToZero()
   {
      getOrientation().setToZero();
      getPosition().setToZero();
   }

   /**
    * Performs a linear interpolation from {@code this} to {@code other} given the percentage
    * {@code alpha}.
    * <p>
    * this.position = (1.0 - alpha) * this.position + alpha * other.position<br>
    * this.orientation = (1.0 - alpha) * this.orientation + alpha * other.orientation
    * </p>
    *
    * @param other the other pose 3D used for the interpolation. Not modified.
    * @param alpha the percentage used for the interpolation. A value of 0 will result in not modifying
    *              {@code this}, while a value of 1 is equivalent to setting {@code this} to
    *              {@code other}.
    */
   default void interpolate(Pose3DReadOnly other, double alpha)
   {
      getPosition().interpolate(other.getPosition(), alpha);
      getOrientation().interpolate(other.getOrientation(), alpha);
   }

   /**
    * Performs a linear interpolation from {@code pose1} to {@code pose2} given the percentage
    * {@code alpha}.
    * <p>
    * this.position = (1.0 - alpha) * pose1.position + alpha * pose2.position<br>
    * this.orientation = (1.0 - alpha) * pose1.orientation + alpha * pose2.orientation
    * </p>
    *
    * @param pose1 the first pose 3D used in the interpolation. Not modified.
    * @param pose2 the second pose 3D used in the interpolation. Not modified.
    * @param alpha the percentage to use for the interpolation. A value of 0 will result in setting
    *              {@code this} to {@code pose1}, while a value of 1 is equivalent to setting
    *              {@code this} to {@code pose2}.
    */
   default void interpolate(Pose3DReadOnly pose1, Pose3DReadOnly pose2, double alpha)
   {
      getPosition().interpolate(pose1.getPosition(), pose2.getPosition(), alpha);
      getOrientation().interpolate(pose1.getOrientation(), pose2.getOrientation(), alpha);
   }

   /**
    * Adds the translation (x, y, z) to this pose 3D assuming it is expressed in the coordinates in
    * which this pose is expressed.
    * <p>
    * If the translation is expressed in the local coordinates described by this pose 3D, use
    * {@link #appendTranslation(double, double, double)}.
    * </p>
    *
    * @param x the translation distance along the x-axis.
    * @param y the translation distance along the y-axis.
    * @param z the translation distance along the z-axis.
    */
   @Override
   default void prependTranslation(double x, double y, double z)
   {
      getPosition().add(x, y, z);
   }

   /**
    * Adds the given {@code translation} to this pose 3D assuming it is expressed in the coordinates in
    * which this pose is expressed.
    * <p>
    * If the {@code translation} is expressed in the local coordinates described by this pose 3D, use
    * {@link #appendTranslation(Tuple3DReadOnly)}.
    * </p>
    *
    * @param translation tuple containing the translation to apply to this pose 3D. Not modified.
    */
   @Override
   default void prependTranslation(Tuple3DReadOnly translation)
   {
      prependTranslation(translation.getX(), translation.getY(), translation.getZ());
   }

   /**
    * Rotates the position part of this pose 3D by the given {@code rotation} and prepends it to the
    * orientation part.
    *
    * @param rotation the rotation to prepend to this pose 3D. Not modified.
    */
   default void prependRotation(Orientation3DReadOnly rotation)
   {
      rotation.transform(getPosition());
      rotation.transform(getOrientation());
   }

   /**
    * Prepends a rotation about the z-axis to this pose 3D: Rotates the position part and prepends the
    * rotation to the orientation part.
    *
    * @param yaw the angle to rotate about the z-axis.
    */
   @Override
   default void prependYawRotation(double yaw)
   {
      RotationMatrixTools.applyYawRotation(yaw, getPosition(), getPosition());
      getOrientation().prependYawRotation(yaw);
   }

   /**
    * Prepends a rotation about the y-axis to this pose 3D: Rotates the position part and prepends the
    * rotation to the orientation part.
    *
    * @param pitch the angle to rotate about the y-axis.
    */
   @Override
   default void prependPitchRotation(double pitch)
   {
      RotationMatrixTools.applyPitchRotation(pitch, getPosition(), getPosition());
      getOrientation().prependPitchRotation(pitch);
   }

   /**
    * Prepends a rotation about the x-axis to this pose 3D: Rotates the position part and prepends the
    * rotation to the orientation part.
    *
    * @param roll the angle to rotate about the x-axis.
    */
   @Override
   default void prependRollRotation(double roll)
   {
      RotationMatrixTools.applyRollRotation(roll, getPosition(), getPosition());
      getOrientation().prependRollRotation(roll);
   }

   /**
    * Prepends the given transform to this pose 3D.
    * <p>
    * This is the same as {@link #applyTransform(Transform)}.
    * </p>
    *
    * @param transform the transform to prepend to this pose 3D. Not modified.
    */
   default void prependTransform(RigidBodyTransformReadOnly transform)
   {
      applyTransform(transform);
   }

   /**
    * Rotates, then adds the translation (x, y, z) to this pose 3D.
    * <p>
    * Use this method if the translation (x, y, z) is expressed in the local coordinates described by
    * this pose 3D. Otherwise, use {@link #prependTranslation(double, double, double)}.
    * </p>
    *
    * @param x the translation distance along the x-axis.
    * @param y the translation distance along the y-axis.
    * @param z the translation distance along the z-axis.
    */
   @Override
   default void appendTranslation(double x, double y, double z)
   {
      double thisX = getX();
      double thisY = getY();
      double thisZ = getZ();

      getPosition().set(x, y, z);
      getOrientation().transform(getPosition());
      getPosition().add(thisX, thisY, thisZ);
   }

   /**
    * Rotates, then adds the given {@code translation} to this pose 3D.
    * <p>
    * Use this method if the {@code translation} is expressed in the local coordinates described by
    * this pose 3D. Otherwise, use {@link #prependTranslation(Tuple3DReadOnly)}.
    * </p>
    *
    * @param translation tuple containing the translation to apply to this pose 3D. Not modified.
    */
   @Override
   default void appendTranslation(Tuple3DReadOnly translation)
   {
      appendTranslation(translation.getX(), translation.getY(), translation.getZ());
   }

   /**
    * Appends the given orientation to this pose 3D.
    * <p>
    * Only the orientation part of this pose is affected by this operation, for more details see
    * {@link Orientation3DBasics#append(Orientation3DReadOnly)}.
    * </p>
    *
    * @param orientation the orientation to append to this pose 3D. Not modified.
    */
   default void appendRotation(Orientation3DReadOnly orientation)
   {
      getOrientation().append(orientation);
   }

   /**
    * Appends a rotation about the z-axis to this pose 3D.
    * <p>
    * More precisely, the position part is unchanged while the orientation part is updated as
    * follows:<br>
    *
    * <pre>
    *                                       / cos(yaw) -sin(yaw) 0 \
    * this.orientation = this.orientation * | sin(yaw)  cos(yaw) 0 |
    *                                       \    0         0     1 /
    * </pre>
    * </p>
    *
    * @param yaw the angle to rotate about the z-axis.
    */
   @Override
   default void appendYawRotation(double yaw)
   {
      getOrientation().appendYawRotation(yaw);
   }

   /**
    * Appends a rotation about the y-axis to this pose 3D.
    * <p>
    * More precisely, the position part is unchanged while the orientation part is updated as
    * follows:<br>
    *
    * <pre>
    *                                       /  cos(pitch) 0 sin(pitch) \
    * this.orientation = this.orientation * |      0      1     0      |
    *                                       \ -sin(pitch) 0 cos(pitch) /
    * </pre>
    * </p>
    *
    * @param pitch the angle to rotate about the y-axis.
    */
   @Override
   default void appendPitchRotation(double pitch)
   {
      getOrientation().appendPitchRotation(pitch);
   }

   /**
    * Appends a rotation about the x-axis to this pose 3D.
    * <p>
    * More precisely, the position part is unchanged while the orientation part is updated as
    * follows:<br>
    *
    * <pre>
    *                                       / 1     0          0     \
    * this.orientation = this.orientation * | 0 cos(roll) -sin(roll) |
    *                                       \ 0 sin(roll)  cos(roll) /
    * </pre>
    * </p>
    *
    * @param roll the angle to rotate about the x-axis.
    */
   @Override
   default void appendRollRotation(double roll)
   {
      getOrientation().appendRollRotation(roll);
   }

   /**
    * Appends the given {@code transform} to this pose 3D.
    *
    * @param transform the rigid-body transform to append to this pose 3D. Not modified.
    */
   default void appendTransform(RigidBodyTransformReadOnly transform)
   {
      QuaternionTools.addTransform(getOrientation(), transform.getTranslation(), getPosition());
      getOrientation().append(transform.getRotation());
   }

   /**
    * Transforms the position and orientation parts of this pose 3D by the given {@code transform}.
    *
    * @param transform the geometric transform to apply on this pose 3D. Not modified.
    */
   @Override
   default void applyTransform(Transform transform)
   {
      transform.transform(getPosition());
      transform.transform(getOrientation());
   }

   /**
    * Transforms the position and orientation parts of this pose 3D by the inverse of the given
    * {@code transform}.
    *
    * @param transform the geometric transform to apply on this pose 3D. Not modified.
    */
   @Override
   default void applyInverseTransform(Transform transform)
   {
      transform.inverseTransform(getPosition());
      transform.inverseTransform(getOrientation());
   }
}