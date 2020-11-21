package us.ihmc.euclid.transform.interfaces;

import us.ihmc.euclid.exceptions.NotARotationMatrixException;
import us.ihmc.euclid.interfaces.Clearable;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.orientation.interfaces.Orientation3DBasics;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.tools.RotationMatrixTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

/**
 * Write and read interface for a rigid-body transform.
 * <p>
 * A rigid-body transform represents a transform that can rotate and/or translate geometries.
 * </p>
 * <p>
 * The data structure used to represents this transform is not enforced here, such that the rotation
 * part can be any implementation of orientation 3D.
 * </p>
 *
 * @author Sylvain Bertrand
 */
public interface RigidBodyTransformBasics extends RigidBodyTransformReadOnly, Clearable
{
   /**
    * Gets the write and read reference to the rotation part of this transform.
    *
    * @return the rotation part of this transform.
    */
   @Override
   Orientation3DBasics getRotation();

   /**
    * Gets the write and read reference of the translation part of this rigid-body transform.
    *
    * @return the translation part of this transform.
    */
   @Override
   Tuple3DBasics getTranslation();

   /**
    * Tests if at least one element of this transform is equal to {@linkplain Double#NaN}.
    *
    * @return {@code true} if at least one element of this transform is equal to
    *         {@linkplain Double#NaN}, {@code false} otherwise.
    */
   @Override
   default boolean containsNaN()
   {
      return RigidBodyTransformReadOnly.super.containsNaN();
   }

   /**
    * Resets this rigid-body transform to identity.
    * <p>
    * When set to identity, this transform has no effect when transforming a geometry object.
    * </p>
    */
   @Override
   default void setToZero()
   {
      getRotation().setToZero();
      getTranslation().setToZero();
   }

   /**
    * Sets the rotation part to represent a 'zero' rotation.
    */
   default void setRotationToZero()
   {
      getRotation().setToZero();
   }

   /**
    * Sets the translation part to zero.
    */
   default void setTranslationToZero()
   {
      getTranslation().setToZero();
   }

   /**
    * Sets all the components of this affine transform making it invalid.
    */
   @Override
   default void setToNaN()
   {
      getRotation().setToNaN();
      getTranslation().setToNaN();
   }

   /**
    * Sets all the components of the rotation matrix to {@link Double#NaN}.
    * <p>
    * See {@link RotationScaleMatrix#setToNaN()}.
    * </p>
    */
   default void setRotationToNaN()
   {
      getRotation().setToNaN();
   }

   /**
    * Sets all the components of the translation vector to {@link Double#NaN}.
    * <p>
    * See {@link Vector3D#setToNaN()}.
    * </p>
    */
   default void setTranslationToNaN()
   {
      getTranslation().setToNaN();
   }

   /**
    * Inverts this rigid-body transform.
    */
   default void invert()
   {
      getRotation().invert();
      if (hasTranslation())
         getRotation().transform(getTranslation());
      getTranslation().negate();
   }

   /**
    * Inverts only the rotation part of this transform, the translation remains unchanged.
    */
   default void invertRotation()
   {
      getRotation().invert();
   }

   /**
    * Normalize the rotation part of this transform.
    */
   default void normalizeRotationPart()
   {
      getRotation().normalize();
   }

   /**
    * Sets this rigid-body transform to {@code other}.
    *
    * @param other the other rigid-body transform to copy the values from. Not modified.
    */
   default void set(RigidBodyTransformReadOnly other)
   {
      getRotation().set(other.getRotation());
      getTranslation().set(other.getTranslation());
   }

   default void set(AffineTransformReadOnly affineTransform)
   {
      getRotation().set(affineTransform.getLinearTransform().getAsQuaternion());
      getTranslation().set(affineTransform.getTranslation());
   }

   /**
    * Sets this rigid-body transform to {@code other} and then inverts it.
    *
    * @param other the other rigid-body transform to copy the values from. Not modified.
    */
   default void setAndInvert(RigidBodyTransformReadOnly other)
   {
      set(other);
      invert();
   }

   /**
    * Sets the rotation and translation parts of this transform separately.
    *
    * @param rotationMatrix the matrix used to set the rotation part of this transform. Not modified.
    * @param translation    the tuple used to set the translation part of this transform. Not modified.
    */
   default void set(RotationMatrixReadOnly rotationMatrix, Tuple3DReadOnly translation)
   {
      getRotation().set(rotationMatrix);
      getTranslation().set(translation);
   }

   /**
    * Sets the rotation and translation parts of this transform separately.
    *
    * @param orientation the orientation used to set the rotation part of this transform. Not modified.
    * @param translation the tuple used to set the translation part of this transform. Not modified.
    */
   default void set(Orientation3DReadOnly orientation, Tuple3DReadOnly translation)
   {
      getRotation().set(orientation);
      getTranslation().set(translation);
   }

   /**
    * Sets the rotation part of this transform to the given axis-angle.
    * <p>
    * This method does not affect the translation part of this transform.
    * </p>
    *
    * @param orientation the orientation used to set the rotation part of this transform. Not modified.
    * @deprecated Use {@code this.getRotation().set(orientation)} instead.
    */
   @Deprecated
   default void setRotation(Orientation3DReadOnly orientation)
   {
      getRotation().set(orientation);
   }

   /**
    * Sets the rotation part of this transform to the given rotation vector.
    * <p>
    * This method does not affect the translation part of this transform.
    * </p>
    * <p>
    * WARNING: a rotation vector is different from a yaw-pitch-roll or Euler angles representation. A
    * rotation vector is equivalent to the axis of an axis-angle that is multiplied by the angle of the
    * same axis-angle.
    * </p>
    *
    * @param rotationVector the rotation vector used to set the rotation part of this transform. Not
    *                       modified.
    * @deprecated Use {@code this.getRotation().setRotationVector(rotationVector)} instead.
    */
   @Deprecated
   default void setRotation(Vector3DReadOnly rotationVector)
   {
      getRotation().setRotationVector(rotationVector);
   }

   /**
    * Sets the rotation part of this transform to the given matrix.
    * <p>
    * This method does not affect the translation part of this transform.
    * </p>
    *
    * @param rotationMatrix the matrix used to set the rotation part of this transform. Not modified.
    * @throws NotARotationMatrixException if the given {@code rotationMatrix} is not a rotation matrix.
    * @deprecated Use {@code this.getRotation().set(rotationMatrix)} instead.
    */
   @Deprecated
   default void setRotation(RotationMatrixReadOnly rotationMatrix)
   {
      getRotation().set(rotationMatrix);
   }

   /**
    * Sets the rotation part of this transform to represent a counter clockwise rotation around the
    * z-axis of an angle {@code yaw}.
    *
    * <pre>
    *     / cos(yaw) -sin(yaw) 0 \
    * R = | sin(yaw)  cos(yaw) 0 |
    *     \    0         0     1 /
    * </pre>
    * <p>
    * This method does not affect the translation part of this transform.
    * </p>
    *
    * @param yaw the angle to rotate about the z-axis.
    * @deprecated Use {@code this.getRotation().setToYawOrientation(yaw)} instead.
    */
   @Deprecated
   default void setRotationYaw(double yaw)
   {
      getRotation().setToYawOrientation(yaw);
   }

   /**
    * Sets the rotation part of this transform to represent a counter clockwise rotation around the
    * y-axis of an angle {@code pitch}.
    *
    * <pre>
    *     /  cos(pitch) 0 sin(pitch) \
    * R = |      0      1     0      |
    *     \ -sin(pitch) 0 cos(pitch) /
    * </pre>
    * <p>
    * This method does not affect the translation part of this transform.
    * </p>
    *
    * @param pitch the angle to rotate about the y-axis.
    * @deprecated Use {@code this.getRotation().setRotationPitch(pitch)} instead.
    */
   @Deprecated
   default void setRotationPitch(double pitch)
   {
      getRotation().setToPitchOrientation(pitch);
   }

   /**
    * Sets the rotation part of this transform to represent a counter clockwise rotation around the
    * x-axis of an angle {@code roll}.
    *
    * <pre>
    *     / 1     0          0     \
    * R = | 0 cos(roll) -sin(roll) |
    *     \ 0 sin(roll)  cos(roll) /
    * </pre>
    * <p>
    * This method does not affect the translation part of this transform.
    * </p>
    *
    * @param roll the angle to rotate about the x-axis.
    * @deprecated Use {@code this.getRotation().setRotationRoll(roll)} instead.
    */
   @Deprecated
   default void setRotationRoll(double roll)
   {
      getRotation().setToRollOrientation(roll);
   }

   /**
    * Sets the rotation part of this transform to represent the same orientation as the given
    * yaw-pitch-roll angles {@code yaw}, {@code pitch}, and {@code roll}.
    *
    * <pre>
    *     / cos(yaw) -sin(yaw) 0 \   /  cos(pitch) 0 sin(pitch) \   / 1     0          0     \
    * R = | sin(yaw)  cos(yaw) 0 | * |      0      1     0      | * | 0 cos(roll) -sin(roll) |
    *     \    0         0     1 /   \ -sin(pitch) 0 cos(pitch) /   \ 0 sin(roll)  cos(roll) /
    * </pre>
    * <p>
    * This method does not affect the translation part of this transform.
    * </p>
    *
    * @param yaw   the angle to rotate about the z-axis.
    * @param pitch the angle to rotate about the y-axis.
    * @param roll  the angle to rotate about the x-axis.
    * @deprecated Use {@code this.getRotation().setRotationYawPitchRoll(yaw, pitch, roll)} instead.
    */
   @Deprecated
   default void setRotationYawPitchRoll(double yaw, double pitch, double roll)
   {
      getRotation().setYawPitchRoll(yaw, pitch, roll);
   }

   /**
    * Sets the rotation part of this transform to represent the same orientation as the given Euler
    * angles {@code eulerAngles}.
    *
    * <pre>
    *     / cos(eulerAngles.z) -sin(eulerAngles.z) 0 \   /  cos(eulerAngles.y) 0 sin(eulerAngles.y) \   / 1         0                   0          \
    * R = | sin(eulerAngles.z)  cos(eulerAngles.z) 0 | * |          0          1         0          | * | 0 cos(eulerAngles.x) -sin(eulerAngles.x) |
    *     \         0                   0          1 /   \ -sin(eulerAngles.y) 0 cos(eulerAngles.y) /   \ 0 sin(eulerAngles.x)  cos(eulerAngles.x) /
    * </pre>
    * <p>
    * This method does not affect the translation part of this transform.
    * </p>
    * <p>
    * This is equivalent to
    * {@code this.setRotationYawPitchRoll(eulerAngles.getZ(), eulerAngles.getY(), eulerAngles.getX())}.
    * </p>
    *
    * @param eulerAngles the Euler angles to copy the orientation from. Not modified.
    * @deprecated Use {@code this.getRotation().setEuler(eulerAngles)} instead.
    */
   @Deprecated
   default void setRotationEuler(Vector3DReadOnly eulerAngles)
   {
      getRotation().setEuler(eulerAngles);
   }

   /**
    * Sets the rotation part of this transform to represent the same orientation as the given Euler
    * angles {@code rotX}, {@code rotY}, and {@code rotZ}.
    *
    * <pre>
    *     / cos(rotZ) -sin(rotZ) 0 \   /  cos(rotY) 0 sin(rotY) \   / 1     0          0     \
    * R = | sin(rotZ)  cos(rotZ) 0 | * |      0     1     0     | * | 0 cos(rotX) -sin(rotX) |
    *     \     0          0     1 /   \ -sin(rotY) 0 cos(rotY) /   \ 0 sin(rotX)  cos(rotX) /
    * </pre>
    * <p>
    * This method does not affect the translation part of this transform.
    * </p>
    * <p>
    * This is equivalent to {@code this.setRotationYawPitchRoll(rotZ, rotY, rotX)}.
    * </p>
    *
    * @param rotX the angle to rotate about the x-axis.
    * @param rotY the angle to rotate about the y-axis.
    * @param rotZ the angle to rotate about the z-axis.
    * @deprecated Use {@code this.getRotation().setEuler(rotX, rotY, rotZ)} instead.
    */
   @Deprecated
   default void setRotationEuler(double rotX, double rotY, double rotZ)
   {
      getRotation().setEuler(rotX, rotY, rotZ);
   }

   /**
    * Sets the rotation part of this transform to the given orientation and sets the translation part
    * to zero.
    *
    * @param orientation the orientation used to set the rotation part of this transform. Not modified.
    */
   default void setRotationAndZeroTranslation(Orientation3DReadOnly orientation)
   {
      getRotation().set(orientation);
      setTranslationToZero();
   }

   /**
    * Sets the rotation part of this transform to the given orientation and sets the translation part
    * to zero.
    *
    * @param rotationMatrix the rotation matrix used to set the rotation part of this transform. Not
    *                       modified.
    */
   default void setRotationAndZeroTranslation(RotationMatrixReadOnly rotationMatrix)
   {
      getRotation().set(rotationMatrix);
      setTranslationToZero();
   }

   /**
    * Sets the rotation part of this transform to the given rotation vector and sets the translation
    * part to zero.
    * <p>
    * WARNING: a rotation vector is different from a yaw-pitch-roll or Euler angles representation. A
    * rotation vector is equivalent to the axis of an axis-angle that is multiplied by the angle of the
    * same axis-angle.
    * </p>
    *
    * @param rotationVector the rotation vector used to set the rotation part of this transform. Not
    *                       modified.
    */
   default void setRotationAndZeroTranslation(Vector3DReadOnly rotationVector)
   {
      getRotation().setRotationVector(rotationVector);
      setTranslationToZero();
   }

   /**
    * Sets the rotation part of this transform to represent a counter clockwise rotation around the
    * z-axis of an angle {@code yaw} and sets the translation part to zero.
    *
    * <pre>
    *     / cos(yaw) -sin(yaw) 0 \
    * R = | sin(yaw)  cos(yaw) 0 |
    *     \    0         0     1 /
    * </pre>
    *
    * @param yaw the angle to rotate about the z-axis.
    */
   default void setRotationYawAndZeroTranslation(double yaw)
   {
      getRotation().setToYawOrientation(yaw);
      setTranslationToZero();
   }

   /**
    * Sets the rotation part of this transform to represent a counter clockwise rotation around the
    * y-axis of an angle {@code pitch} and sets the translation part to zero.
    *
    * <pre>
    *        /  cos(pitch) 0 sin(pitch) \
    * this = |      0      1     0      |
    *        \ -sin(pitch) 0 cos(pitch) /
    * </pre>
    *
    * @param pitch the angle to rotate about the y-axis.
    */
   default void setRotationPitchAndZeroTranslation(double pitch)
   {
      getRotation().setToPitchOrientation(pitch);
      setTranslationToZero();
   }

   /**
    * Sets the rotation part of this transform to represent a counter clockwise rotation around the
    * x-axis of an angle {@code roll} and sets the translation part to zero.
    *
    * <pre>
    *        / 1     0          0     \
    * this = | 0 cos(roll) -sin(roll) |
    *        \ 0 sin(roll)  cos(roll) /
    * </pre>
    *
    * @param roll the angle to rotate about the x-axis.
    */
   default void setRotationRollAndZeroTranslation(double roll)
   {
      getRotation().setToRollOrientation(roll);
      setTranslationToZero();
   }

   /**
    * Sets the rotation part of this transform to represent the same orientation as the given
    * yaw-pitch-roll angles {@code yaw}, {@code pitch}, and {@code roll} and sets the translation part
    * to zero.
    *
    * <pre>
    *     / cos(yaw) -sin(yaw) 0 \   /  cos(pitch) 0 sin(pitch) \   / 1     0          0     \
    * R = | sin(yaw)  cos(yaw) 0 | * |      0      1     0      | * | 0 cos(roll) -sin(roll) |
    *     \    0         0     1 /   \ -sin(pitch) 0 cos(pitch) /   \ 0 sin(roll)  cos(roll) /
    * </pre>
    *
    * @param yaw   the angle to rotate about the z-axis.
    * @param pitch the angle to rotate about the y-axis.
    * @param roll  the angle to rotate about the x-axis.
    */
   default void setRotationYawPitchRollAndZeroTranslation(double yaw, double pitch, double roll)
   {
      getRotation().setYawPitchRoll(yaw, pitch, roll);
      setTranslationToZero();
   }

   /**
    * Sets the rotation part of this transform to represent the same orientation as the given Euler
    * angles {@code eulerAngles} and sets the translation part to zero.
    *
    * <pre>
    *     / cos(eulerAngles.z) -sin(eulerAngles.z) 0 \   /  cos(eulerAngles.y) 0 sin(eulerAngles.y) \   / 1         0                   0          \
    * R = | sin(eulerAngles.z)  cos(eulerAngles.z) 0 | * |          0          1         0          | * | 0 cos(eulerAngles.x) -sin(eulerAngles.x) |
    *     \         0                   0          1 /   \ -sin(eulerAngles.y) 0 cos(eulerAngles.y) /   \ 0 sin(eulerAngles.x)  cos(eulerAngles.x) /
    * </pre>
    * <p>
    * This is equivalent to
    * {@code this.setRotationYawPitchRollAndZeroTranslation(eulerAngles.getZ(), eulerAngles.getY(), eulerAngles.getX())}.
    * </p>
    *
    * @param eulerAngles the Euler angles to copy the orientation from. Not modified.
    */
   default void setRotationEulerAndZeroTranslation(Vector3DReadOnly eulerAngles)
   {
      getRotation().setEuler(eulerAngles);
      setTranslationToZero();
   }

   /**
    * Sets the rotation part of this transform to represent the same orientation as the given Euler
    * angles {@code rotX}, {@code rotY}, and {@code rotZ} and sets the translation part to zero.
    *
    * <pre>
    *     / cos(rotZ) -sin(rotZ) 0 \   /  cos(rotY) 0 sin(rotY) \   / 1     0          0     \
    * R = | sin(rotZ)  cos(rotZ) 0 | * |      0     1     0     | * | 0 cos(rotX) -sin(rotX) |
    *     \     0          0     1 /   \ -sin(rotY) 0 cos(rotY) /   \ 0 sin(rotX)  cos(rotX) /
    * </pre>
    * <p>
    * This is equivalent to {@code this.setRotationYawPitchRollAndZeroTranslation(rotZ, rotY, rotX)}.
    * </p>
    *
    * @param rotX the angle to rotate about the x-axis.
    * @param rotY the angle to rotate about the y-axis.
    * @param rotZ the angle to rotate about the z-axis.
    */
   default void setRotationEulerAndZeroTranslation(double rotX, double rotY, double rotZ)
   {
      getRotation().setEuler(rotX, rotY, rotZ);
      setTranslationToZero();
   }

   /**
    * Sets the x-component of the translation part of this transform.
    * <p>
    * This method does not affect the rotation part of this transform.
    * </p>
    *
    * @param x the x-component of the translation part.
    * @deprecated Use {@code this.getTranslation().setX(x)} instead.
    */
   @Deprecated
   default void setTranslationX(double x)
   {
      getTranslation().setX(x);
   }

   /**
    * Sets the y-component of the translation part of this transform.
    * <p>
    * This method does not affect the rotation part of this transform.
    * </p>
    *
    * @param y the y-component of the translation part.
    * @deprecated Use {@code this.getTranslation().setY(y)} instead.
    */
   @Deprecated
   default void setTranslationY(double y)
   {
      getTranslation().setY(y);
   }

   /**
    * Sets the z-component of the translation part of this transform.
    * <p>
    * This method does not affect the rotation part of this transform.
    * </p>
    *
    * @param z the z-component of the translation part.
    * @deprecated Use {@code this.getTranslation().setZ(z)} instead.
    */
   @Deprecated
   default void setTranslationZ(double z)
   {
      getTranslation().setZ(z);
   }

   /**
    * Sets the translation part of this transform.
    * <p>
    * This method does not affect the rotation part of this transform.
    * </p>
    *
    * @param x the x-component of the translation part.
    * @param y the y-component of the translation part.
    * @param z the z-component of the translation part.
    * @deprecated Use {@code this.getTranslation().set(x, y, z)} instead.
    */
   @Deprecated
   default void setTranslation(double x, double y, double z)
   {
      getTranslation().set(x, y, z);
   }

   /**
    * Sets the translation part of this transform.
    * <p>
    * This method does not affect the rotation part of this transform.
    * </p>
    *
    * @param translation tuple used to set the translation part of this transform. Not modified.
    * @deprecated Use {@code this.getTranslation().set(translation)} instead.
    */
   @Deprecated
   default void setTranslation(Tuple3DReadOnly translation)
   {
      getTranslation().set(translation);
   }

   /**
    * Sets the translation part of this transform and sets the rotation part to identity.
    *
    * @param x the x-component of the translation part.
    * @param y the y-component of the translation part.
    * @param z the z-component of the translation part.
    */
   default void setTranslationAndIdentityRotation(double x, double y, double z)
   {
      getTranslation().set(x, y, z);
      setRotationToZero();
   }

   /**
    * Sets the translation part of this transform and sets the rotation part to identity.
    *
    * @param translation tuple used to set the translation part of this transform. Not modified.
    */
   default void setTranslationAndIdentityRotation(Tuple3DReadOnly translation)
   {
      getTranslation().set(translation);
      setRotationToZero();
   }

   /**
    * Performs the multiplication of this transform with {@code other}.
    * <p>
    * this = this * other
    * </p>
    *
    * @param other the other transform to multiply this with. Not modified.
    */
   default void multiply(RigidBodyTransformReadOnly other)
   {
      if (other.hasTranslation())
         getRotation().addTransform(other.getTranslation(), getTranslation());

      getRotation().append(other.getRotation());
   }

   /**
    * Performs the multiplication of this transform with {@code affineTransform}.
    * <p>
    * Note: the scale part of the given affine transform is not used when performing the multiplication
    * to conserve a proper rigid-body transform describing only a rotation and a translation.
    * </p>
    * <p>
    * this = this * S(affineTransform) <br>
    * where S(affineTransform) is the function selecting only the rotation and translation parts of the
    * affine transform.
    * </p>
    *
    * @param affineTransform the affine transform to multiply this with. Not modified.
    */
   default void multiply(AffineTransformReadOnly affineTransform)
   {
      getRotation().addTransform(affineTransform.getTranslation(), getTranslation());
      getRotation().append(affineTransform.getLinearTransform().getAsQuaternion());
   }

   /**
    * Performs the multiplication of the inverse of this transform with {@code other}.
    * <p>
    * this = this<sup>-1</sup> * other
    * </p>
    *
    * @param other the other transform to multiply this with. Not modified.
    */
   default void multiplyInvertThis(RigidBodyTransformReadOnly other)
   {
      getTranslation().sub(other.getTranslation(), getTranslation());

      getRotation().invert();
      getRotation().transform(getTranslation());
      getRotation().append(other.getRotation());
   }

   /**
    * Performs the multiplication of this transform with the inverse of {@code other}.
    * <p>
    * this = this * other<sup>-1</sup>
    * </p>
    *
    * @param other the other transform to multiply this with. Not modified.
    */
   default void multiplyInvertOther(RigidBodyTransformReadOnly other)
   {
      getRotation().appendInvertOther(other.getRotation());

      if (other.hasTranslation())
         getRotation().subTransform(other.getTranslation(), getTranslation());
   }

   /**
    * Performs the multiplication of the inverse of this transform with {@code affineTransform}.
    * <p>
    * Note: the scale part of the given affine transform is not used when performing the multiplication
    * to conserve a proper rigid-body transform describing only a rotation and a translation.
    * </p>
    * <p>
    * this = this<sup>-1</sup> * S(affineTransform) <br>
    * where S(affineTransform) is the function selecting only the rotation and translation parts of the
    * affine transform.
    * </p>
    *
    * @param affineTransform the affine transform to multiply this with. Not modified.
    */
   default void multiplyInvertThis(AffineTransformReadOnly affineTransform)
   {
      getTranslation().sub(affineTransform.getTranslation(), getTranslation());
      getRotation().inverseTransform(getTranslation());
      getRotation().appendInvertThis(affineTransform.getLinearTransform().getAsQuaternion());
   }

   /**
    * Performs the multiplication of this transform with the inverse of {@code affineTransform}.
    * <p>
    * Note: the scale part of the given affine transform is not used when performing the multiplication
    * to conserve a proper rigid-body transform describing only a rotation and a translation.
    * </p>
    * <p>
    * this = this * S(affineTransform)<sup>-1</sup> <br>
    * where S(affineTransform) is the function selecting only the rotation and translation parts of the
    * affine transform.
    * </p>
    *
    * @param affineTransform the affine transform to multiply this with. Not modified.
    */
   default void multiplyInvertOther(AffineTransformReadOnly affineTransform)
   {
      getRotation().appendInvertOther(affineTransform.getLinearTransform().getAsQuaternion());
      getRotation().subTransform(affineTransform.getTranslation(), getTranslation());
   }

   /**
    * Append a translation transform to this transform.
    *
    * <pre>
    *               / 1 0 0 translation.x \
    * this = this * | 0 1 0 translation.y |
    *               | 0 0 1 translation.z |
    *               \ 0 0 0      1        /
    * </pre>
    * <p>
    * This method does not affect the rotation part of this transform.
    * </p>
    *
    * @param translation the translation to append to this transform. Not modified.
    */
   default void appendTranslation(Tuple3DReadOnly translation)
   {
      getRotation().addTransform(translation, getTranslation());
   }

   /**
    * Append a translation transform to this transform.
    *
    * <pre>
    *               / 1 0 0 x \
    * this = this * | 0 1 0 y |
    *               | 0 0 1 z |
    *               \ 0 0 0 1 /
    * </pre>
    * <p>
    * This method does not affect the rotation part of this transform.
    * </p>
    *
    * @param x the translation along the x-axis to apply.
    * @param y the translation along the y-axis to apply.
    * @param z the translation along the z-axis to apply.
    */
   default void appendTranslation(double x, double y, double z)
   {
      double thisX = getTranslation().getX();
      double thisY = getTranslation().getY();
      double thisZ = getTranslation().getZ();

      getTranslation().set(x, y, z);
      getRotation().transform(getTranslation());
      getTranslation().add(thisX, thisY, thisZ);
   }

   /**
    * Append a rotation about the z-axis to the rotation part of this transform.
    *
    * <pre>
    *         / cos(yaw) -sin(yaw) 0 \
    * R = R * | sin(yaw)  cos(yaw) 0 |
    *         \    0         0     1 /
    * </pre>
    * <p>
    * This method does not affect the translation part of this transform.
    * </p>
    *
    * @param yaw the angle to rotate about the z-axis.
    */
   default void appendYawRotation(double yaw)
   {
      getRotation().appendYawRotation(yaw);
   }

   /**
    * Append a rotation about the y-axis to the rotation part of this transform.
    *
    * <pre>
    *         /  cos(pitch) 0 sin(pitch) \
    * R = R * |      0      1     0      |
    *         \ -sin(pitch) 0 cos(pitch) /
    * </pre>
    * <p>
    * This method does not affect the translation part of this transform.
    * </p>
    *
    * @param pitch the angle to rotate about the y-axis.
    */
   default void appendPitchRotation(double pitch)
   {
      getRotation().appendPitchRotation(pitch);
   }

   /**
    * Append a rotation about the x-axis to the rotation part of this transform.
    *
    * <pre>
    *         / 1     0          0     \
    * R = R * | 0 cos(roll) -sin(roll) |
    *         \ 0 sin(roll)  cos(roll) /
    * </pre>
    * <p>
    * This method does not affect the translation part of this transform.
    * </p>
    *
    * @param roll the angle to rotate about the x-axis.
    */
   default void appendRollRotation(double roll)
   {
      getRotation().appendRollRotation(roll);
   }

   /**
    * Performs the multiplication of {@code other} with this transform.
    * <p>
    * this = other * this
    * </p>
    *
    * @param other the other transform to multiply this with. Not modified.
    */
   default void preMultiply(RigidBodyTransformReadOnly other)
   {
      if (hasTranslation())
      {
         other.getRotation().transform(getTranslation());
         getTranslation().add(other.getTranslation());
      }
      else
      {
         getTranslation().set(other.getTranslation());
      }

      getRotation().prepend(other.getRotation());
   }

   /**
    * Performs the multiplication of {@code affineTransform} with this transform.
    * <p>
    * Note: the scale part of the given affine transform is not used when performing the multiplication
    * to conserve a proper rigid-body transform describing only a rotation and a translation.
    * </p>
    * <p>
    * this = S(affineTransform) * this <br>
    * where S(affineTransform) is the function selecting only the rotation and translation parts of the
    * affine transform.
    * </p>
    *
    * @param affineTransform the affine transform to multiply this with. Not modified.
    */
   default void preMultiply(AffineTransformReadOnly affineTransform)
   {
      affineTransform.getLinearTransform().getAsQuaternion().transform(getTranslation());
      getTranslation().add(affineTransform.getTranslation());
      getRotation().prepend(affineTransform.getLinearTransform().getAsQuaternion());
   }

   /**
    * Performs the multiplication of {@code other} with the inverse of this transform.
    * <p>
    * this = other * this<sup>-1</sup>
    * </p>
    *
    * @param other the other transform to multiply this with. Not modified.
    */
   default void preMultiplyInvertThis(RigidBodyTransformReadOnly other)
   {
      getRotation().invert();
      getRotation().prepend(other.getRotation());

      if (hasTranslation())
         getRotation().transform(getTranslation());
      getTranslation().sub(other.getTranslation(), getTranslation());
   }

   /**
    * Performs the multiplication of the inverse of {@code other} with this transform.
    * <p>
    * this = other<sup>-1</sup> * this
    * </p>
    *
    * @param other the other transform to multiply this with. Not modified.
    */
   default void preMultiplyInvertOther(RigidBodyTransformReadOnly other)
   {
      getTranslation().sub(other.getTranslation());

      if (hasTranslation())
         other.getRotation().inverseTransform(getTranslation());

      getRotation().prependInvertOther(other.getRotation());
   }

   /**
    * Performs the multiplication of {@code affineTransform} with the inverse of this transform.
    * <p>
    * Note: the scale part of the given affine transform is not used when performing the multiplication
    * to conserve a proper rigid-body transform describing only a rotation and a translation.
    * </p>
    * <p>
    * this = S(affineTransform) * this<sup>-1</sup> <br>
    * where S(affineTransform) is the function selecting only the rotation and translation parts of the
    * affine transform.
    * </p>
    *
    * @param affineTransform the affine transform to multiply this with. Not modified.
    */
   default void preMultiplyInvertThis(AffineTransformReadOnly affineTransform)
   {
      getRotation().prependInvertThis(affineTransform.getLinearTransform().getAsQuaternion());
      getRotation().transform(getTranslation());
      getTranslation().sub(affineTransform.getTranslation(), getTranslation());
   }

   /**
    * Performs the multiplication of the inverse of {@code affineTransform} with this transform.
    * <p>
    * Note: the scale part of the given affine transform is not used when performing the multiplication
    * to conserve a proper rigid-body transform describing only a rotation and a translation.
    * </p>
    * <p>
    * this = S(affineTransform)<sup>-1</sup> * this <br>
    * where S(affineTransform) is the function selecting only the rotation and translation parts of the
    * affine transform.
    * </p>
    *
    * @param affineTransform the affine transform to multiply this with. Not modified.
    */
   default void preMultiplyInvertOther(AffineTransformReadOnly affineTransform)
   {
      getTranslation().sub(affineTransform.getTranslation());
      affineTransform.getLinearTransform().getAsQuaternion().inverseTransform(getTranslation());
      getRotation().prependInvertOther(affineTransform.getLinearTransform().getAsQuaternion());
   }

   /**
    * Prepend a translation transform to this transform.
    *
    * <pre>
    *        / 1 0 0 translation.x \
    * this = | 0 1 0 translation.y | * this
    *        | 0 0 1 translation.z |
    *        \ 0 0 0      1        /
    * </pre>
    * <p>
    * This method does not affect the rotation part of this transform.
    * </p>
    *
    * @param translation the translation to prepend to this transform. Not modified.
    */
   default void prependTranslation(Tuple3DReadOnly translation)
   {
      getTranslation().add(translation);
   }

   /**
    * Prepend a translation transform to this transform.
    *
    * <pre>
    *        / 1 0 0 x \
    * this = | 0 1 0 y | * this
    *        | 0 0 1 z |
    *        \ 0 0 0 1 /
    * </pre>
    * <p>
    * This method does not affect the rotation part of this transform.
    * </p>
    *
    * @param x the translation along the x-axis to apply.
    * @param y the translation along the y-axis to apply.
    * @param z the translation along the z-axis to apply.
    */
   default void prependTranslation(double x, double y, double z)
   {
      getTranslation().add(x, y, z);
   }

   /**
    * Prepend a rotation about the z-axis to this transform.
    * <p>
    * This method first rotates the translation part and then prepend the yaw-rotation to the rotation
    * part of this transform.
    * </p>
    *
    * <pre>
    *        / cos(yaw) -sin(yaw)  0   0 \
    * this = | sin(yaw)  cos(yaw)  0   0 | * this
    *        |    0         0      1   0 |
    *        \    0         0      0   1 /
    * </pre>
    *
    * @param yaw the angle to rotate about the z-axis.
    */
   default void prependYawRotation(double yaw)
   {
      RotationMatrixTools.applyYawRotation(yaw, getTranslation(), getTranslation());
      getRotation().prependYawRotation(yaw);
   }

   /**
    * Prepend a rotation about the y-axis to this transform.
    * <p>
    * This method first rotates the translation part and then prepend the pitch-rotation to the
    * rotation part of this transform.
    * </p>
    *
    * <pre>
    *        /  cos(pitch) 0 sin(pitch)  0 \
    * this = |      0      1     0       0 | * this
    *        | -sin(pitch) 0 cos(pitch)  0 |
    *        \      0      0     0       1 /
    * </pre>
    *
    * @param pitch the angle to rotate about the y-axis.
    */
   default void prependPitchRotation(double pitch)
   {
      RotationMatrixTools.applyPitchRotation(pitch, getTranslation(), getTranslation());
      getRotation().prependPitchRotation(pitch);
   }

   /**
    * Prepend a rotation about the x-axis to this transform.
    * <p>
    * This method first rotates the translation part and then prepend the roll-rotation to the rotation
    * part of this transform.
    * </p>
    *
    * <pre>
    *        / 1     0          0     0 \
    * this = | 0 cos(roll) -sin(roll) 0 | * this
    *        | 0 sin(roll)  cos(roll) 0 |
    *        \ 0     0          0     1 /
    * </pre>
    *
    * @param roll the angle to rotate about the x-axis.
    */
   default void prependRollRotation(double roll)
   {
      RotationMatrixTools.applyRollRotation(roll, getTranslation(), getTranslation());
      getRotation().prependRollRotation(roll);
   }
}
