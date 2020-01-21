package us.ihmc.euclid.orientation.interfaces;

import us.ihmc.euclid.interfaces.Clearable;
import us.ihmc.euclid.interfaces.Transformable;
import us.ihmc.euclid.transform.AffineTransform;
import us.ihmc.euclid.transform.QuaternionBasedTransform;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;

/**
 * Write and read interface for a 3D orientation.
 * <p>
 * Even though the representation used is unknown at this level of abstraction, this interface
 * allows to enforce a minimum set of features that all representations of an orientation should
 * provide, such as appending and prepending orientations to each other.
 * </p>
 *
 * @author Sylvain Bertrand
 */
public interface Orientation3DBasics extends Orientation3DReadOnly, Clearable, Transformable
{
   /**
    * For representations of orientations with more variables than degrees of freedom, some or all of
    * the variables are constrained. This method updates the variables considering this constraint.
    * <p>
    * The actual implementation of this function strongly depends on the type of orientation.
    * </p>
    */
   void normalize();

   /**
    * Inverses this orientation.
    * <p>
    * If this orientation describes the orientation of a coordinate system A with respect to a
    * coordinate system B, after this method, the orientation will describe the orientation of B with
    * respect to A.
    * </p>
    * <p>
    * Note that appending or prepending an orientation with its inverse will result into a "zero"
    * orientation.
    * </p>
    */
   void invert();

   /**
    * Sets this orientation to represents the same orientation as a rotation matrix given its 9
    * components.
    *
    * @param m00 the new 1st row 1st column coefficient of the rotation matrix.
    * @param m01 the new 1st row 2nd column coefficient of the rotation matrix.
    * @param m02 the new 1st row 3rd column coefficient of the rotation matrix.
    * @param m10 the new 2nd row 1st column coefficient of the rotation matrix.
    * @param m11 the new 2nd row 2nd column coefficient of the rotation matrix.
    * @param m12 the new 2nd row 3rd column coefficient of the rotation matrix.
    * @param m20 the new 3rd row 1st column coefficient of the rotation matrix.
    * @param m21 the new 3rd row 2nd column coefficient of the rotation matrix.
    * @param m22 the new 3rd row 3rd column coefficient of the rotation matrix.
    */
   void setRotationMatrix(double m00, double m01, double m02, double m10, double m11, double m12, double m20, double m21, double m22);

   /**
    * Sets this orientation to represents the same orientation as an axis-angle given its 4 components.
    *
    * @param x     x-component of the axis part of the axis-angle.
    * @param y     y-component of the axis part of the axis-angle.
    * @param z     z-component of the axis part of the axis-angle.
    * @param angle the angle part of the axis-angle.
    */
   void setAxisAngle(double x, double y, double z, double angle);

   /**
    * Sets this orientation to represents the same orientation as a quaternion given its 4 components.
    *
    * @param x the x-component of the quaternion.
    * @param y the y-component of the quaternion.
    * @param z the z-component of the quaternion.
    * @param s the s-component of the quaternion.
    */
   void setQuaternion(double x, double y, double z, double s);

   /**
    * Sets this orientation to represents the same orientation as a rotation vector given its 3
    * components.
    * <p>
    * WARNING: a rotation vector is different from a yaw-pitch-roll or Euler angles representation. A
    * rotation vector is equivalent to the axis of an axis-angle that is multiplied by the angle of the
    * same axis-angle.
    * </p>
    *
    * @param x the x-component of the rotation vector.
    * @param y the y-component of the rotation vector.
    * @param z the z-component of the rotation vector.
    */
   void setRotationVector(double x, double y, double z);

   /**
    * Sets this orientation to represents the same orientation as a yaw-pitch-roll representation.
    * <p>
    * WARNING: the Euler angles or yaw-pitch-roll representation is sensitive to gimbal lock and is
    * sometimes undefined.
    * </p>
    * <p>
    * The yaw-pitch-roll representation describes a 3D orientation as a succession of three rotations
    * around three axes:
    * <ol>
    * <li>yaw: rotation around the z-axis,
    * <li>pitch: rotation around the y-axis,
    * <li>roll: rotation around the x-axis.
    * </ol>
    * </p>
    * <p>
    * As an example, a rotation matrix can be computed from a yaw-pitch-roll representation as follows:
    *
    * <pre>
    *     / cos(yaw) -sin(yaw) 0 \   /  cos(pitch) 0 sin(pitch) \   / 1     0          0     \
    * R = | sin(yaw)  cos(yaw) 0 | * |      0      1     0      | * | 0 cos(roll) -sin(roll) |
    *     \    0         0     1 /   \ -sin(pitch) 0 cos(pitch) /   \ 0 sin(roll)  cos(roll) /
    * </pre>
    * </p>
    *
    * @param yaw   the angle to rotate about the z-axis.
    * @param pitch the angle to rotate about the y-axis.
    * @param roll  the angle to rotate about the x-axis.
    */
   void setYawPitchRoll(double yaw, double pitch, double roll);

   /**
    * Sets this orientation to represents the same orientation as a yaw-pitch-roll representation.
    * <p>
    * WARNING: the Euler angles or yaw-pitch-roll representation is sensitive to gimbal lock and is
    * sometimes undefined.
    * </p>
    * <p>
    * The yaw-pitch-roll representation describes a 3D orientation as a succession of three rotations
    * around three axes:
    * <ol>
    * <li>yaw: rotation around the z-axis,
    * <li>pitch: rotation around the y-axis,
    * <li>roll: rotation around the x-axis.
    * </ol>
    * </p>
    * <p>
    * As an example, a rotation matrix can be computed from a yaw-pitch-roll representation as follows:
    *
    * <pre>
    *     / cos(yaw) -sin(yaw) 0 \   /  cos(pitch) 0 sin(pitch) \   / 1     0          0     \
    * R = | sin(yaw)  cos(yaw) 0 | * |      0      1     0      | * | 0 cos(roll) -sin(roll) |
    *     \    0         0     1 /   \ -sin(pitch) 0 cos(pitch) /   \ 0 sin(roll)  cos(roll) /
    * </pre>
    * </p>
    *
    * @param yawPitchRoll array containing the yaw-pitch-roll angles. Not modified.
    * @deprecated Use {@link #set(Orientation3DReadOnly)} with {@link YawPitchRoll} instead.
    */
   @Deprecated
   default void setYawPitchRoll(double[] yawPitchRoll)
   {
      setYawPitchRoll(yawPitchRoll[0], yawPitchRoll[1], yawPitchRoll[2]);
   }

   /**
    * Sets this orientation to represent the same orientation as the given {@code rotationVector}.
    * <p>
    * WARNING: a rotation vector is different from a yaw-pitch-roll or Euler angles representation. A
    * rotation vector is equivalent to the axis of an axis-angle that is multiplied by the angle of the
    * same axis-angle.
    * </p>
    *
    * @param rotationVector the rotation vector to set this orientation. Not modified.
    */
   default void setRotationVector(Vector3DReadOnly rotationVector)
   {
      setRotationVector(rotationVector.getX(), rotationVector.getY(), rotationVector.getZ());
   }

   /**
    * Sets this orientation to represent the same orientation as the given Euler angles
    * {@code eulerAngles}.
    * <p>
    * This is equivalent to {@link #setYawPitchRoll(double, double, double)} with
    * {@code yaw = eulerAngles.getZ()}, {@code pitch = eulerAngles.getY()}, and
    * {@code roll = eulerAngles.getX()}.
    * </p>
    *
    * @param eulerAngles the Euler angles to copy the orientation from. Not modified.
    */
   default void setEuler(Vector3DReadOnly eulerAngles)
   {
      setYawPitchRoll(eulerAngles.getZ(), eulerAngles.getY(), eulerAngles.getX());
   }

   /**
    * Sets this orientation to represent the same orientation as the given Euler angles.
    * <p>
    * This is equivalent to {@link #setYawPitchRoll(double, double, double)} with {@code yaw = rotZ},
    * {@code pitch = rotY}, and {@code roll = rotX}.
    * </p>
    *
    * @param rotX the angle to rotate about the x-axis.
    * @param rotY the angle to rotate about the y-axis.
    * @param rotZ the angle to rotate about the z-axis.
    */
   default void setEuler(double rotX, double rotY, double rotZ)
   {
      setYawPitchRoll(rotZ, rotY, rotX);
   }

   /**
    * Sets this orientation to represent a counter clockwise rotation around the z-axis of an angle
    * {@code yaw}.
    *
    * @param yaw the angle to rotate about the z-axis.
    */
   void setToYawOrientation(double yaw);

   /**
    * Sets this orientation to represent a counter clockwise rotation around the y-axis of an angle
    * {@code pitch}.
    *
    * @param pitch the angle to rotate about the y-axis.
    */
   void setToPitchOrientation(double pitch);

   /**
    * Sets this orientation to represent a counter clockwise rotation around the x-axis of an angle
    * {@code roll}.
    *
    * @param roll the angle to rotate about the x-axis.
    */
   void setToRollOrientation(double roll);

   /**
    * Converts, if necessary, and sets this orientation to represents the same orientation as
    * {@code orientation3DReadOnly}.
    *
    * @param orientation3DReadOnly the new orientation. Not modified.
    */
   void set(Orientation3DReadOnly orientation3DReadOnly);

   /**
    * Converts, if necessary, and sets this orientation to represents the same orientation as
    * {@code orientation3DReadOnly} and then normalize this orientation.
    *
    * @param orientation3DReadOnly the new orientation. Not modified.
    */
   default void setAndNormalize(Orientation3DReadOnly orientation3DReadOnly)
   {
      set(orientation3DReadOnly);
      normalize();
   }

   /**
    * Sets this orientation to represent the inverse of the given {@code orientation3DReadOnly}.
    *
    * @param orientation3DReadOnly the new orientation. Not modified.
    */
   default void setAndInvert(Orientation3DReadOnly orientation3DReadOnly)
   {
      set(orientation3DReadOnly);
      invert();
   }

   /**
    * Appends the given orientation to this orientation.
    * <p>
    * Let's consider the following:
    * <ul>
    * <li>three coordinate systems <b>A</b>, <b>B</b>, and <b>C</b>.
    * <li>{@code this} represents the orientation of <b>B</b> relative to <b>A</b>.
    * <li>{@code orientation} represents the orientation of <b>C</b> relative to <b>B</b>.
    * </ul>
    * The result of calling this method will be that {@code this}, represents the orientation of
    * <b>C</b> relative to <b>A</b>.
    * </p>
    * <p>
    * Appending orientations is in some way similar to summing translations. However, while the
    * addition for translation is commutative, the "append" operation on orientation is NOT
    * commutative. Such that: {@code this.append(orientation)} &ne; {@code orientation.append(this)}.
    * </p>
    *
    * @param orientation the orientation to append to this orientation. Not modified.
    */
   void append(Orientation3DReadOnly orientation);

   /**
    * Appends the inverse of the given orientation to this orientation.
    * <p>
    * Let's consider the following:
    * <ul>
    * <li>three coordinate systems <b>A</b>, <b>B</b>, and <b>C</b>.
    * <li>{@code this} represents the orientation of <b>B</b> relative to <b>A</b>.
    * <li>{@code orientation} represents the orientation of <b>B</b> relative to <b>C</b>.
    * </ul>
    * The result of calling this method will be that {@code this}, represents the orientation of
    * <b>C</b> relative to <b>A</b>.
    * </p>
    * <p>
    * This operation is in some way similar to subtracting translations, as in this operation can be
    * seen as subtracting {@code orientation} to {@code this}.
    * </p>
    *
    * @param orientation the orientation which the inverse is to be appended to this orientation. Not
    *                    modified.
    */
   void appendInvertOther(Orientation3DReadOnly orientation);

   /**
    * Inverts {@code this} and then appends the given orientation.
    * <p>
    * Let's consider the following:
    * <ul>
    * <li>three coordinate systems <b>A</b>, <b>B</b>, and <b>C</b>.
    * <li>{@code this} represents the orientation of <b>A</b> relative to <b>B</b>.
    * <li>{@code orientation} represents the orientation of <b>C</b> relative to <b>B</b>.
    * </ul>
    * The result of calling this method will be that {@code this}, represents the orientation of
    * <b>C</b> relative to <b>A</b>.
    * </p>
    * <p>
    * This operation is in some way similar to subtracting translations, as in this operation can be
    * seen as subtracting {@code this} to {@code orientation}.
    * </p>
    *
    * @param orientation the orientation to append to the inverse of this orientation. Not modified.
    */
   default void appendInvertThis(Orientation3DReadOnly orientation)
   {
      invert();
      append(orientation);
   }

   /**
    * Inverts {@code this} and then appends the inverse of the given orientation to this orientation.
    * <p>
    * Let's consider the following:
    * <ul>
    * <li>three coordinate systems <b>A</b>, <b>B</b>, and <b>C</b>.
    * <li>{@code this} represents the orientation of <b>A</b> relative to <b>B</b>.
    * <li>{@code orientation} represents the orientation of <b>B</b> relative to <b>C</b>.
    * </ul>
    * The result of calling this method will be that {@code this}, represents the orientation of
    * <b>C</b> relative to <b>A</b>.
    * </p>
    * <p>
    * This operation is in some way similar to subtracting translations, as in this operation can be
    * seen as subtracting {@code orientation} to {@code this}.
    * </p>
    *
    * @param orientation the orientation which the inverse is to be appended to this orientation. Not
    *                    modified.
    */
   default void appendInvertBoth(Orientation3DReadOnly orientation)
   {
      invert();
      appendInvertOther(orientation);
   }

   /**
    * Appends a rotation {@code R(yaw)} about the z-axis to this orientation.
    *
    * <pre>
    *          / cos(yaw) -sin(yaw) 0 \
    * R(yaw) = | sin(yaw)  cos(yaw) 0 |
    *          \    0         0     1 /
    * </pre>
    * <p>
    * Note that the z-axis refers to the local coordinate system described by this orientation.
    * </p>
    *
    * @param yaw the angle to rotate about the z-axis.
    */
   void appendYawRotation(double yaw);

   /**
    * Appends a rotation {@code R(pitch)} about the y-axis to this orientation.
    *
    * <pre>
    *            /  cos(pitch) 0 sin(pitch) \
    * R(pitch) = |      0      1     0      |
    *            \ -sin(pitch) 0 cos(pitch) /
    * </pre>
    * <p>
    * Note that the y-axis refers to the local coordinate system described by this orientation.
    * </p>
    *
    * @param pitch the angle to rotate about the y-axis.
    */
   void appendPitchRotation(double pitch);

   /**
    * Appends a rotation {@code R(roll)} about the x-axis to this orientation.
    *
    * <pre>
    *           / 1     0          0     \
    * R(roll) = | 0 cos(roll) -sin(roll) |
    *           \ 0 sin(roll)  cos(roll) /
    * </pre>
    * <p>
    * Note that the x-axis refers to the local coordinate system described by this orientation.
    * </p>
    *
    * @param roll the angle to rotate about the x-axis.
    */
   void appendRollRotation(double roll);

   /**
    * Prepends the given orientation to this orientation.
    * <p>
    * Let's consider the following:
    * <ul>
    * <li>three coordinate systems <b>A</b>, <b>B</b>, and <b>C</b>.
    * <li>{@code this} represents the orientation of <b>C</b> relative to <b>B</b>.
    * <li>{@code orientation} represents the orientation of <b>B</b> relative to <b>A</b>.
    * </ul>
    * The result of calling this method will be that {@code this}, represents the orientation of
    * <b>C</b> relative to <b>A</b>.
    * </p>
    * <p>
    * Prepending orientations is in some way similar to summing translations. However, while the
    * addition for translation is commutative, the "prepend" operation on orientation is NOT
    * commutative. Such that: {@code this.prepend(orientation)} &ne; {@code orientation.prepend(this)}.
    * </p>
    *
    * @param orientation the orientation to prepend to this orientation. Not modified.
    */
   void prepend(Orientation3DReadOnly orientation);

   /**
    * Prepends the inverse of the given orientation to this orientation.
    * <p>
    * Let's consider the following:
    * <ul>
    * <li>three coordinate systems <b>A</b>, <b>B</b>, and <b>C</b>.
    * <li>{@code this} represents the orientation of <b>B</b> relative to <b>C</b>.
    * <li>{@code orientation} represents the orientation of <b>B</b> relative to <b>A</b>.
    * </ul>
    * The result of calling this method will be that {@code this}, represents the orientation of
    * <b>C</b> relative to <b>A</b>.
    * </p>
    * <p>
    * This operation is in some way similar to subtracting translations, as in this operation can be
    * seen as subtracting {@code orientation} to {@code this}.
    * </p>
    *
    * @param orientation the orientation which the inverse is to be appended to this orientation. Not
    *                    modified.
    */
   void prependInvertOther(Orientation3DReadOnly orientation);

   /**
    * Inverts {@code this} and then prepends the given orientation.
    * <p>
    * Let's consider the following:
    * <ul>
    * <li>three coordinate systems <b>A</b>, <b>B</b>, and <b>C</b>.
    * <li>{@code this} represents the orientation of <b>C</b> relative to <b>B</b>.
    * <li>{@code orientation} represents the orientation of <b>A</b> relative to <b>B</b>.
    * </ul>
    * The result of calling this method will be that {@code this}, represents the orientation of
    * <b>C</b> relative to <b>A</b>.
    * </p>
    * <p>
    * This operation is in some way similar to subtracting translations, as in this operation can be
    * seen as subtracting {@code this} to {@code orientation}.
    * </p>
    *
    * @param orientation the orientation to append to the inverse of this orientation. Not modified.
    */
   default void prependInvertThis(Orientation3DReadOnly orientation)
   {
      invert();
      prepend(orientation);
   }

   /**
    * Inverts {@code this} and then prepends the inverse of the given orientation to this orientation.
    * <p>
    * Let's consider the following:
    * <ul>
    * <li>three coordinate systems <b>A</b>, <b>B</b>, and <b>C</b>.
    * <li>{@code this} represents the orientation of <b>B</b> relative to <b>C</b>.
    * <li>{@code orientation} represents the orientation of <b>A</b> relative to <b>B</b>.
    * </ul>
    * The result of calling this method will be that {@code this}, represents the orientation of
    * <b>C</b> relative to <b>A</b>.
    * </p>
    * <p>
    * This operation is in some way similar to subtracting translations, as in this operation can be
    * seen as subtracting {@code orientation} to {@code this}.
    * </p>
    *
    * @param orientation the orientation which the inverse is to be appended to this orientation. Not
    *                    modified.
    */
   default void prependInvertBoth(Orientation3DReadOnly orientation)
   {
      invert();
      prependInvertOther(orientation);
   }

   /**
    * Prepends a rotation {@code R(yaw)} about the z-axis to this orientation.
    *
    * <pre>
    *          / cos(yaw) -sin(yaw) 0 \
    * R(yaw) = | sin(yaw)  cos(yaw) 0 |
    *          \    0         0     1 /
    * </pre>
    * <p>
    * Note that the z-axis refers to the base coordinate system in which this orientation is expressed.
    * </p>
    *
    * @param yaw the angle to rotate about the z-axis.
    */
   void prependYawRotation(double yaw);

   /**
    * Prepends a rotation {@code R(pitch)} about the y-axis to this orientation.
    *
    * <pre>
    *            /  cos(pitch) 0 sin(pitch) \
    * R(pitch) = |      0      1     0      |
    *            \ -sin(pitch) 0 cos(pitch) /
    * </pre>
    * <p>
    * Note that the y-axis refers to the base coordinate system in which this orientation is expressed.
    * </p>
    *
    * @param pitch the angle to rotate about the y-axis.
    */
   void prependPitchRotation(double pitch);

   /**
    * Appends a rotation {@code R(roll)} about the x-axis to this orientation.
    *
    * <pre>
    *           / 1     0          0     \
    * R(roll) = | 0 cos(roll) -sin(roll) |
    *           \ 0 sin(roll)  cos(roll) /
    * </pre>
    * <p>
    * Note that the x-axis refers to the base coordinate system in which this orientation is expressed.
    * </p>
    *
    * @param roll the angle to rotate about the x-axis.
    */
   void prependRollRotation(double roll);

   /**
    * Transforms this orientation using the given {@code transform}.
    * <p>
    * {@link RigidBodyTransform}, {@link QuaternionBasedTransform}, and {@link AffineTransform} prepend
    * their rotation part to this. No scale or translation is applied such that the output of this
    * method is still a pure rotation.
    * </p>
    *
    * @param transform the geometric transform to apply on this orientation. Not modified.
    */
   @Override
   default void applyTransform(Transform transform)
   {
      transform.transform(this);
   }

   /**
    * Transforms this orientation using the inverse of the given {@code transform}.
    * <p>
    * {@link RigidBodyTransform}, {@link QuaternionBasedTransform}, and {@link AffineTransform} prepend
    * their rotation part to this. No scale or translation is applied to such that the output of this
    * method is still a pure rotation.
    * </p>
    *
    * @param transform the geometric transform to apply on this orientation. Not modified.
    */
   @Override
   default void applyInverseTransform(Transform transform)
   {
      transform.inverseTransform(this);
   }
}
