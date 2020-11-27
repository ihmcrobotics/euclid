package us.ihmc.euclid.matrix.interfaces;

import us.ihmc.euclid.exceptions.SingularMatrixException;
import us.ihmc.euclid.orientation.interfaces.Orientation3DBasics;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.rotationConversion.RotationMatrixConversion;
import us.ihmc.euclid.tools.Matrix3DTools;
import us.ihmc.euclid.tools.RotationMatrixTools;
import us.ihmc.euclid.tools.SingularValueDecomposition3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;

/**
 * Write and read interface used for a 3-by-3 linear transform.
 * <p>
 * A linear transform matrix behaves mostly like a regular 3D matrix. In addition to the base
 * features from {@link Matrix3DBasics}, the linear transform can be decomposed, using a singular
 * value decomposition, into:
 * 
 * <pre>
 * A = U W V
 * </pre>
 * 
 * where:
 * <ul>
 * <li><tt>A</tt> is this 3D linear transform.
 * <li><tt>U</tt> is the 3D pre-scale rotation.
 * <li><tt>W</tt> is the 3D scale.
 * <li><tt>U</tt> is the 3D post-scale rotation.
 * </ul>
 * The SVD decomposition allows for instance to easily obtain a view of this matrix as a pure
 * rotation transform, i.e. ignoring the scale matrix {@code W}, and use this view to perform
 * operations with other rotations.
 * </p>
 * 
 * @see SingularValueDecomposition3D
 * @author Sylvain Bertrand
 */
public interface LinearTransform3DBasics extends LinearTransform3DReadOnly, Matrix3DBasics
{
   /**
    * Resets the scale part of this linear transform to <tt>(1, 1, 1)</tt> such that this linear
    * transform becomes a pure rotation transform.
    */
   void resetScale();

   /**
    * Sets this linear transform to be a pure rotation transform representing the same rotation as the
    * given rotation vector once converted.
    * <p>
    * WARNING: a rotation vector is different from a yaw-pitch-roll or Euler angles representation. A
    * rotation vector is equivalent to the axis of an axis-angle that is multiplied by the angle of the
    * same axis-angle.
    * </p>
    * 
    * @param rotationVector the rotation vector to set this linear transform. Not modified.
    */
   default void setRotationVector(Vector3DReadOnly rotationVector)
   {
      RotationMatrixConversion.convertRotationVectorToMatrix(rotationVector, this);
   }

   /**
    * Sets this linear transform to be a pure rotation transform representing the same rotation as the
    * given Euler angles once converted.
    * <p>
    * This is equivalent to {@link #set(Orientation3DReadOnly)} with a {@link YawPitchRoll} and
    * {@code yaw = eulerAngles.getZ()}, {@code pitch = eulerAngles.getY()}, and
    * {@code roll = eulerAngles.getX()}.
    * </p>
    *
    * @param eulerAngles the Euler angles to set this linear transform. Not modified.
    */
   default void setEuler(Tuple3DReadOnly eulerAngles)
   {
      RotationMatrixConversion.convertYawPitchRollToMatrix(eulerAngles.getZ(), eulerAngles.getY(), eulerAngles.getX(), this);
   }

   /**
    * Appends the given orientation to this linear transform.
    * <p>
    * Internally, the given orientation is converted into a 3D matrix which is then multiplied to this
    * linear transform. This operation is similar to
    * {@link Orientation3DBasics#append(Orientation3DReadOnly)}.
    * </p>
    * 
    * @param orientation the orientation to append to this linear transform. Not modified.
    */
   default void appendRotation(Orientation3DReadOnly orientation)
   {
      Matrix3DTools.multiply(this, false, false, orientation, false, this);
   }

   /**
    * Appends the inverse of the given orientation to this linear transform.
    * <p>
    * Internally, the given orientation is inverted then converted into a 3D matrix which is then
    * multiplied to this linear transform. This operation is similar to
    * {@link Orientation3DBasics#appendInvertOther(Orientation3DReadOnly)}.
    * </p>
    * 
    * @param orientation the orientation which the inverse is to be appended to this linear transform.
    *                    Not modified.
    */
   default void appendRotationInvertOther(Orientation3DReadOnly orientation)
   {
      Matrix3DTools.multiply(this, false, false, orientation, true, this);
   }

   /**
    * Inverts {@code this} and then appends the given orientation.
    * <p>
    * Internally, the given orientation is converted into a 3D matrix which is then multiplied to this
    * linear transform once inverted. This operation is similar to
    * {@link Orientation3DBasics#appendInvertThis(Orientation3DReadOnly)}.
    * </p>
    * 
    * @param orientation the orientation to append to this linear transform. Not modified.
    * @throws SingularMatrixException if this linear transform is not invertible.
    */
   default void appendRotationInvertThis(Orientation3DReadOnly orientation)
   {
      invert();
      appendRotation(orientation);
   }

   /**
    * Inverts {@code this} and then appends the inverse of the given orientation to this linear
    * transform.
    * <p>
    * Internally, the given orientation is inverted then converted into a 3D matrix which is then
    * multiplied to this linear transform once inverted. This operation is similar to
    * {@link Orientation3DBasics#appendInvertBoth(Orientation3DReadOnly)}.
    * </p>
    * 
    * @param orientation the orientation which the inverse is to be appended to this linear transform.
    *                    Not modified.
    * @throws SingularMatrixException if this linear transform is not invertible.
    */
   default void appendRotationInvertBoth(Orientation3DReadOnly orientation)
   {
      invert();
      appendRotationInvertOther(orientation);
   }

   /**
    * Appends a rotation about the z-axis to this linear transform.
    *
    * <pre>
    *               / cos(yaw) -sin(yaw) 0 \
    * this = this * | sin(yaw)  cos(yaw) 0 |
    *               \    0         0     1 /
    * </pre>
    *
    * @param yaw the angle to rotate about the z-axis.
    */
   default void appendYawRotation(double yaw)
   {
      RotationMatrixTools.appendYawRotation(this, yaw, this);
   }

   /**
    * Appends a rotation about the y-axis to this linear transform.
    *
    * <pre>
    *               /  cos(pitch) 0 sin(pitch) \
    * this = this * |      0      1     0      |
    *               \ -sin(pitch) 0 cos(pitch) /
    * </pre>
    *
    * @param pitch the angle to rotate about the y-axis.
    */
   default void appendPitchRotation(double pitch)
   {
      RotationMatrixTools.appendPitchRotation(this, pitch, this);
   }

   /**
    * Appends a rotation about the x-axis to this linear transform.
    *
    * <pre>
    *               / 1     0          0     \
    * this = this * | 0 cos(roll) -sin(roll) |
    *               \ 0 sin(roll)  cos(roll) /
    * </pre>
    *
    * @param roll the angle to rotate about the x-axis.
    */
   default void appendRollRotation(double roll)
   {
      RotationMatrixTools.appendRollRotation(this, roll, this);
   }

   /**
    * Appends a scale to this linear transform.
    * 
    * <pre>
    *               / scale   0     0   \
    * this = this * |   0   scale   0   |
    *               \   0     0   scale /
    * </pre>
    * 
    * Note that this operation is identical to {@link #scale(double)}.
    * 
    * @param scale the scale to append.
    */
   default void appendScale(double scale)
   {
      scale(scale);
   }

   /**
    * Appends a scale to this linear transform.
    * 
    * <pre>
    *               / scale.getX()      0            0       \
    * this = this * |      0       scale.getY()      0       |
    *               \      0            0       scale.getZ() /
    * </pre>
    * 
    * @param scale the scale to append. Not modified
    */
   default void appendScale(Tuple3DReadOnly scale)
   {
      appendScale(scale.getX(), scale.getY(), scale.getZ());
   }

   /**
    * Appends a scale to this linear transform.
    * 
    * <pre>
    *               / x 0 0 \
    * this = this * | 0 y 0 |
    *               \ 0 0 z /
    * </pre>
    *
    * @param x the scale factor along the x-axis.
    * @param y the scale factor along the y-axis.
    * @param z the scale factor along the z-axis.
    */
   default void appendScale(double x, double y, double z)
   {
      scaleColumns(x, y, z);
   }

   /**
    * Prepends the given orientation to this linear transform.
    * <p>
    * Internally, the given orientation is converted into a 3D matrix which is then pre-multiplied to
    * this linear transform. This operation is similar to
    * {@link Orientation3DBasics#prepend(Orientation3DReadOnly)}.
    * </p>
    * 
    * @param orientation the orientation to prepend to this linear transform. Not modified.
    */
   default void prependRotation(Orientation3DReadOnly orientation)
   {
      Matrix3DTools.multiply(orientation, false, this, false, false, this);
   }

   /**
    * Prepends the inverse of the given orientation to this linear transform.
    * <p>
    * Internally, the given orientation is inverted then converted into a 3D matrix which is then
    * pre-multiplied to this linear transform. This operation is similar to
    * {@link Orientation3DBasics#prependInvertOther(Orientation3DReadOnly)}.
    * </p>
    * 
    * @param orientation the orientation which the inverse is to be prepended to this linear transform.
    *                    Not modified.
    */
   default void prependRotationInvertOther(Orientation3DReadOnly orientation)
   {
      Matrix3DTools.multiply(orientation, true, this, false, false, this);
   }

   /**
    * Inverts {@code this} and then prepends the given orientation.
    * <p>
    * Internally, the given orientation is converted into a 3D matrix which is then pre-multiplied to
    * this linear transform once inverted. This operation is similar to
    * {@link Orientation3DBasics#prependInvertThis(Orientation3DReadOnly)}.
    * </p>
    * 
    * @param orientation the orientation to prepend to this linear transform. Not modified.
    * @throws SingularMatrixException if this linear transform is not invertible.
    */
   default void prependRotationInvertThis(Orientation3DReadOnly orientation)
   {
      invert();
      prependRotation(orientation);
   }

   /**
    * Inverts {@code this} and then prepends the inverse of the given orientation to this linear
    * transform.
    * <p>
    * Internally, the given orientation is inverted then converted into a 3D matrix which is then
    * pre-multiplied to this linear transform once inverted. This operation is similar to
    * {@link Orientation3DBasics#prependInvertBoth(Orientation3DReadOnly)}.
    * </p>
    * 
    * @param orientation the orientation which the inverse is to be prepended to this linear transform.
    *                    Not modified.
    * @throws SingularMatrixException if this linear transform is not invertible.
    */
   default void prependRotationInvertBoth(Orientation3DReadOnly orientation)
   {
      invert();
      prependRotationInvertOther(orientation);
   }

   /**
    * Prepends a rotation about the z-axis to this linear transform.
    *
    * <pre>
    *        / cos(yaw) -sin(yaw) 0 \
    * this = | sin(yaw)  cos(yaw) 0 | * this
    *        \    0         0     1 /
    * </pre>
    *
    * @param yaw the angle to rotate about the z-axis.
    */
   default void prependYawRotation(double yaw)
   {
      RotationMatrixTools.prependYawRotation(yaw, this, this);
   }

   /**
    * Prepends a rotation about the y-axis to this linear transform.
    *
    * <pre>
    *        /  cos(pitch) 0 sin(pitch) \
    * this = |      0      1     0      | * this
    *        \ -sin(pitch) 0 cos(pitch) /
    * </pre>
    *
    * @param pitch the angle to rotate about the y-axis.
    */
   default void prependPitchRotation(double pitch)
   {
      RotationMatrixTools.prependPitchRotation(pitch, this, this);
   }

   /**
    * Prepends a rotation about the x-axis to this linear transform.
    *
    * <pre>
    *        / 1     0          0     \
    * this = | 0 cos(roll) -sin(roll) | * this
    *        \ 0 sin(roll)  cos(roll) /
    * </pre>
    *
    * @param roll the angle to rotate about the x-axis.
    */
   default void prependRollRotation(double roll)
   {
      RotationMatrixTools.prependRollRotation(roll, this, this);
   }

   /**
    * Prepends a scale to this linear transform.
    * 
    * <pre>
    *        / scale   0     0   \
    * this = |   0   scale   0   | * this
    *        \   0     0   scale /
    * </pre>
    * 
    * Note that this operation is identical to {@link #scale(double)}.
    * 
    * @param scale the scale to prepend.
    */
   default void prependScale(double scale)
   {
      prependScale(scale, scale, scale);
   }

   /**
    * Prepends a scale to this linear transform.
    * 
    * <pre>
    *        / scale.getX()      0            0       \
    * this = |      0       scale.getY()      0       | * this
    *        \      0            0       scale.getZ() /
    * </pre>
    * 
    * @param scale the scale to prepend. Not modified
    */
   default void prependScale(Tuple3DReadOnly scale)
   {
      prependScale(scale.getX(), scale.getY(), scale.getZ());
   }

   /**
    * Prepends a scale to this linear transform.
    * 
    * <pre>
    *        / x 0 0 \
    * this = | 0 y 0 | * this
    *        \ 0 0 z /
    * </pre>
    *
    * @param x the scale factor along the x-axis.
    * @param y the scale factor along the y-axis.
    * @param z the scale factor along the z-axis.
    */
   default void prependScale(double x, double y, double z)
   {
      scaleRows(x, y, z);
   }
}