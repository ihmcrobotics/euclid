package us.ihmc.euclid.shape.interfaces;

import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.interfaces.Clearable;
import us.ihmc.euclid.interfaces.Transformable;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformBasics;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;

public interface Shape3DBasics extends Shape3DReadOnly, Clearable, Transformable
{

   RigidBodyTransformBasics getPose();

   /**
    * Gets the read-only reference to the orientation of this shape.
    *
    * @return the orientation of this shape.
    */
   // TODO Not sure what to do here, ideally we'd want Orientation3DBasics but it makes epsilonEquals unavailable.
   RotationMatrix getOrientation();

   /**
    * Gets the read-only reference of the position of this shape.
    *
    * @return the position of this shape.
    */
   Point3DBasics getPosition();

   void setIntermediateVariableSupplier(IntermediateVariableSupplier newSupplier);

   @Override
   default boolean containsNaN()
   {
      return Shape3DReadOnly.super.containsNaN();
   }

   /** {@inheritDoc} */
   @Override
   default void setToNaN()
   {
      getPose().setToNaN();
   }

   /** {@inheritDoc} */
   @Override
   default void setToZero()
   {
      getPose().setToZero();
   }

   /**
    * Sets the orientation of this shape.
    * <p>
    * This method does not affect the position of this shape.
    * </p>
    *
    * @param orientation the new orientation for this shape. Not modified.
    */
   default void setOrientation(Orientation3DReadOnly orientation)
   {
      getPose().setRotation(orientation);
   }

   /**
    * Sets the orientation of this shape.
    * <p>
    * This method does not affect the position of this shape.
    * </p>
    * <p>
    * The given {@code yaw}, {@code pitch}, and {@code roll} angles are composed into a rotation matrix
    * as follows:
    *
    * <pre>
    *     / cos(yaw) -sin(yaw) 0 \   /  cos(pitch) 0 sin(pitch) \   / 1     0          0     \
    * R = | sin(yaw)  cos(yaw) 0 | * |      0      1     0      | * | 0 cos(roll) -sin(roll) |
    *     \    0         0     1 /   \ -sin(pitch) 0 cos(pitch) /   \ 0 sin(roll)  cos(roll) /
    * </pre>
    * </p>
    *
    * @param yaw the angle to rotate about the z-axis.
    * @param pitch the angle to rotate about the y-axis.
    * @param roll the angle to rotate about the x-axis.
    */
   default void setOrientationYawPitchRoll(double yaw, double pitch, double roll)
   {
      getPose().setRotationYawPitchRoll(yaw, pitch, roll);
   }

   /**
    * Sets the orientation of this shape.
    * <p>
    * This method does not affect the position of this shape.
    * </p>
    * <p>
    * The given {@code yaw}, {@code pitch}, and {@code roll} angles are composed into a rotation matrix
    * as follows:
    *
    * <pre>
    *     / cos(yaw) -sin(yaw) 0 \   /  cos(pitch) 0 sin(pitch) \   / 1     0          0     \
    * R = | sin(yaw)  cos(yaw) 0 | * |      0      1     0      | * | 0 cos(roll) -sin(roll) |
    *     \    0         0     1 /   \ -sin(pitch) 0 cos(pitch) /   \ 0 sin(roll)  cos(roll) /
    * </pre>
    * </p>
    *
    * @param yawPitchRoll array containing the yaw-pitch-roll angles. Not modified.
    * @deprecated Use {@link YawPitchRoll} with {@link #setOrientation(Orientation3DReadOnly)}.
    */
   @Deprecated
   default void setOrientationYawPitchRoll(double[] yawPitchRoll)
   {
      getPose().setRotationYawPitchRoll(yawPitchRoll);
   }

   /**
    * Sets the pose, i.e. position and orientation, of this shape.
    *
    * @param pose pose holding the new position and orientation for this shape. Not modified.
    */
   default void setPose(Pose3DReadOnly pose)
   {
      pose.get(getPose());
   }

   /**
    * Sets the pose, i.e. position and orientation, of this shape.
    *
    * @param rigidBodyTransform rigid-body transform holding the new position and orientation for this
    *           shape. Not modified.
    */
   default void setPose(RigidBodyTransformReadOnly rigidBodyTransform)
   {
      getPose().set(rigidBodyTransform);
   }

   /**
    * Sets the pose, i.e. position and orientation, of this shape to the pose of {@code other}.
    *
    * @param other shape holding the pose with the new position and orientation for this shape. Not
    *           modified.
    */
   default void setPose(Shape3DReadOnly other)
   {
      getPose().set(other.getPose());
   }

   /**
    * Sets the pose, i.e. position and orientation, of this shape.
    *
    * @param position the new position for this shape. Not modified.
    * @param orientation the new orientation for this shape. Not modified.
    */
   default void setPose(Tuple3DReadOnly position, Orientation3DReadOnly orientation)
   {
      getPose().set(orientation, position);
   }

   /**
    * Sets the position of this shape.
    * <p>
    * This method does not affect the orientation of this shape.
    * </p>
    *
    * @param x the x-coordinate for this shape position.
    * @param y the y-coordinate for this shape position.
    * @param z the z-coordinate for this shape position.
    */
   default void setPosition(double x, double y, double z)
   {
      getPose().setTranslation(x, y, z);
   }

   /**
    * Sets the position of this shape.
    * <p>
    * This method does not affect the orientation of this shape.
    * </p>
    *
    * @param position the new position for this shape. Not modified.
    */
   default void setPosition(Tuple3DReadOnly position)
   {
      getPose().setTranslation(position);
   }

   /**
    * Sets the x-coordinate of this shape position.
    *
    * @param x the new x-coordinate for this shape.
    */
   default void setPositionX(double x)
   {
      getPose().setTranslationX(x);
   }

   /**
    * Sets the y-coordinate of this shape position.
    *
    * @param y the new y-coordinate for this shape.
    */
   default void setPositionY(double y)
   {
      getPose().setTranslationY(y);
   }

   /**
    * Sets the z-coordinate of this shape position.
    *
    * @param z the new z-coordinate for this shape.
    */
   default void setPositionZ(double z)
   {
      getPose().setTranslationZ(z);
   }

   /**
    * Transforms this shape with a rigid-body transform that is defined in the local coordinates of
    * this shape.
    *
    * @param transform the transform to append to this shape pose. Not modified.
    */
   default void appendTransform(RigidBodyTransformReadOnly transform)
   {
      getPose().multiply(transform);
   }

   /**
    * Translate this shape with the given (x, y, z) components that are expressed in the local
    * coordinates of this shape.
    *
    * @param x the x-component of the translation to append to this shape pose.
    * @param y the y-component of the translation to append to this shape pose.
    * @param z the z-component of the translation to append to this shape pose.
    */
   default void appendTranslation(double x, double y, double z)
   {
      getPose().appendTranslation(x, y, z);
   }

   /**
    * Translate this shape with the given {@code translation} that is expressed in the local
    * coordinates of this shape.
    *
    * @param translation the translation to append to the pose of this shape. Not modified.
    */
   default void appendTranslation(Tuple3DReadOnly translation)
   {
      getPose().appendTranslation(translation);
   }

   /**
    * Rotates this shape by angle of {@code yaw} about the z-axis of this shape local coordinates.
    *
    * @param yaw the angle to rotate about the local z-axis.
    */
   default void appendYawRotation(double yaw)
   {
      getPose().appendYawRotation(yaw);
   }

   /**
    * Rotates this shape by angle of {@code pitch} about the y-axis of this shape local coordinates.
    *
    * @param pitch the angle to rotate about the local y-axis.
    */
   default void appendPitchRotation(double pitch)
   {
      getPose().appendPitchRotation(pitch);
   }

   /**
    * Rotates this shape by angle of {@code roll} about the x-axis of this shape local coordinates.
    *
    * @param roll the angle to rotate about the local x-axis.
    */
   default void appendRollRotation(double roll)
   {
      getPose().appendRollRotation(roll);
   }

   /**
    * Translates this shape with the given (x, y, z) components that are expressed in the world
    * coordinates.
    *
    * @param x the x-component of the translation to prepend to this shape pose.
    * @param y the y-component of the translation to prepend to this shape pose.
    * @param z the z-component of the translation to prepend to this shape pose.
    */
   default void prependTranslation(double x, double y, double z)
   {
      getPose().prependTranslation(x, y, z);
   }

   /**
    * Translate this shape with the given {@code translation} that is expressed in the world
    * coordinates.
    *
    * @param translation the translation to prepend to the pose of this shape. Not modified.
    */
   default void prependTranslation(Tuple3DReadOnly translation)
   {
      getPose().prependTranslation(translation);
   }

   /**
    * Rotates this shape by angle of {@code yaw} about the z-axis of the world coordinates.
    *
    * @param yaw the angle to rotate about the local z-axis.
    */
   default void prependYawRotation(double yaw)
   {
      getPose().prependYawRotation(yaw);
   }

   /**
    * Rotates this shape by angle of {@code pitch} about the y-axis of the world coordinates.
    *
    * @param pitch the angle to rotate about the local y-axis.
    */
   default void prependPitchRotation(double pitch)
   {
      getPose().prependPitchRotation(pitch);
   }

   /**
    * Rotates this shape by angle of {@code roll} about the x-axis of the world coordinates.
    *
    * @param roll the angle to rotate about the local x-axis.
    */
   default void prependRollRotation(double roll)
   {
      getPose().prependRollRotation(roll);
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
