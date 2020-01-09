package us.ihmc.euclid.tuple4D.interfaces;

import us.ihmc.euclid.orientation.interfaces.Orientation3DBasics;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.rotationConversion.QuaternionConversion;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tools.QuaternionTools;

/**
 * Write and read interface for unit-quaternion used to represent 3D orientations.
 * <p>
 * When describing a 4D tuple, its 4 components are often gathered in two groups: the scalar part
 * {@code s} and the vector part ({@code x}, {@code y}, {@code z}).
 * </p>
 * <p>
 * Note on the difference between applying a 3D transform on a quaternion and a 4D vector:
 * <ul>
 * <li>When transformed by a homogeneous transformation matrix, a quaternion is only pre-multiplied
 * by the rotation part of the transform, resulting in concatenating the orientations of the
 * transform and the quaternion.
 * <li>When transformed by a homogeneous transformation matrix, a 4D vector scalar part {@code s}
 * remains unchanged. The vector part ({@code x}, {@code y}, {@code z}) is scaled and rotated, and
 * translated by {@code s} times the translation part of the transform. Note that for {@code s = 0},
 * a 4D vector behaves as a 3D vector, and for {@code s = 1} it behaves as a 3D point.
 * </ul>
 * </p>
 *
 * @author Sylvain Bertrand
 */
public interface QuaternionBasics extends QuaternionReadOnly, Orientation3DBasics, Tuple4DBasics
{
   /**
    * Tolerance used in {@link #pow(double)} to determine if this quaternion is equal to the neutral
    * quaternion.
    */
   public static final double EPS_POW = 1.0e-12;

   /** {@inheritDoc} */
   @Override
   default boolean containsNaN()
   {
      return QuaternionReadOnly.super.containsNaN();
   }

   /**
    * Sets the four components of this quaternion without normalizing.
    * <p>
    * This method is meant for internal usage. Prefer using
    * {@link #set(double, double, double, double)}.
    * </p>
    *
    * @param qx the x-component of this quaternion.
    * @param qy the y-component of this quaternion.
    * @param qz the z-component of this quaternion.
    * @param qs the s-component of this quaternion.
    */
   void setUnsafe(double qx, double qy, double qz, double qs);

   /**
    * Sets this quaternion to the neutral quaternion representing a 'zero' rotation.
    */
   @Override
   default void setToZero()
   {
      setUnsafe(0.0, 0.0, 0.0, 1.0);
   }

   /** {@inheritDoc} */
   @Override
   default void absolute()
   {
      setUnsafe(Math.abs(getX()), Math.abs(getY()), Math.abs(getZ()), Math.abs(getS()));
   }

   /** {@inheritDoc} */
   @Override
   default void negate()
   {
      setUnsafe(-getX(), -getY(), -getZ(), -getS());
   }

   /**
    * Sets this quaternion to its conjugate.
    *
    * <pre>
    *      / -qx \
    * q* = | -qy |
    *      | -qz |
    *      \  qs /
    * </pre>
    */
   default void conjugate()
   {
      setUnsafe(-getX(), -getY(), -getZ(), getS());
   }

   /** {@inheritDoc} */
   @Override
   default void invert()
   {
      conjugate();
   }

   /**
    * Sets this quaternion to its inverse.
    * <p>
    * Essentially calling {@link #conjugate()} and then {@link #normalize()}.
    * </p>
    */
   default void inverse()
   {
      conjugate();
      normalize();
   }

   /**
    * Recomputes this quaternion's components to ensure its norm is equal to 1.
    * <p>
    * Edge cases:
    * <ul>
    * <li>if this axis-angle contains {@link Double#NaN}, this method is ineffective.
    * </ul>
    * </p>
    */
   @Override
   default void normalize()
   {
      if (containsNaN())
         return;

      double invNorm = norm();

      if (invNorm == 0.0)
      {
         setToZero();
         return;
      }

      invNorm = 1.0 / invNorm;
      double qx = getX() * invNorm;
      double qy = getY() * invNorm;
      double qz = getZ() * invNorm;
      double qs = getS() * invNorm;
      setUnsafe(qx, qy, qz, qs);
   }

   /**
    * Normalizes this quaternion and then limits the angle of the rotation it represents to be &in;
    * [-<i>pi</i>;<i>pi</i>].
    */
   default void normalizeAndLimitToPi()
   {
      normalize();

      if (getS() < 0.0)
         negate();
   }

   /**
    * Raises this quaternion to the power {@code alpha}.
    * <p>
    * This is equivalent to converting this quaternion into an axis-angle, scaling the angle by
    * {@code alpha}, and finally converting back to a quaternion.
    * </p>
    * <p>
    * Edge cases:
    * <ul>
    * <li>If this quaternion contains {@link Double#NaN}, this method is ineffective.
    * <li>If the length of the vector part is below {@link #EPS_POW}, the method {@link #setToZero()}
    * is called.
    * </ul>
    * </p>
    *
    * @param alpha the real value of the power.
    */
   default void pow(double alpha)
   {
      if (containsNaN())
         return;

      double uNorm = EuclidCoreTools.norm(getX(), getY(), getZ());

      if (uNorm > EPS_POW)
      {
         double angle = alpha * Math.atan2(uNorm, getS());
         uNorm = Math.sin(angle) / uNorm;
         double qx = getX() * uNorm;
         double qy = getY() * uNorm;
         double qz = getZ() * uNorm;
         set(qx, qy, qz, Math.cos(angle));
      }
      else
      {
         setToZero();
      }
   }

   /** {@inheritDoc} */
   @Override
   default void set(double x, double y, double z, double s)
   {
      setUnsafe(x, y, z, s);
      normalize();
   }

   /** {@inheritDoc} */
   @Override
   default void set(Orientation3DReadOnly orientation3DReadOnly)
   {
      orientation3DReadOnly.get(this);
   }

   /**
    * Sets this quaternion to {@code other}.
    *
    * @param other the other quaternion to copy the values from. Not modified.
    */
   default void set(QuaternionReadOnly other)
   {
      setUnsafe(other.getX(), other.getY(), other.getZ(), other.getS());
   }

   /** {@inheritDoc} */
   @Override
   default void setAndNormalize(Tuple4DReadOnly other)
   {
      set(other);
   }

   /**
    * Sets this tuple to {@code other} and then calls {@link #normalize()}.
    *
    * @param other the other quaternion to copy the values from. Not modified.
    */
   default void setAndNormalize(QuaternionReadOnly other)
   {
      set(other);
      normalize();
   }

   /**
    * Sets this quaternion to the conjugate of {@code other}.
    *
    * <pre>
    *      / -qx \
    * q* = | -qy |
    *      | -qz |
    *      \  qs /
    * </pre>
    *
    * @param other the other quaternion to copy the values from. Not modified.
    */
   default void setAndConjugate(QuaternionReadOnly other)
   {
      set(other);
      conjugate();
   }

   /**
    * Sets this quaternion to {@code other} and then calls {@link #negate()}.
    *
    * @param other the other quaternion to copy the values from. Not modified.
    */
   default void setAndNegate(QuaternionReadOnly other)
   {
      set(other);
      negate();
   }

   /** {@inheritDoc} */
   @Override
   default void setAxisAngle(double x, double y, double z, double angle)
   {
      QuaternionConversion.convertAxisAngleToQuaternion(x, y, z, angle, this);
   }

   /** {@inheritDoc} */
   @Override
   default void setQuaternion(double x, double y, double z, double s)
   {
      setUnsafe(x, y, z, s);
   }

   /** {@inheritDoc} */
   @Override
   default void setRotationVector(double x, double y, double z)
   {
      QuaternionConversion.convertRotationVectorToQuaternion(x, y, z, this);
   }

   /** {@inheritDoc} */
   @Override
   default void setYawPitchRoll(double yaw, double pitch, double roll)
   {
      QuaternionConversion.convertYawPitchRollToQuaternion(yaw, pitch, roll, this);
   }

   /** {@inheritDoc} */
   @Override
   default void setRotationMatrix(double m00, double m01, double m02, double m10, double m11, double m12, double m20, double m21, double m22)
   {
      QuaternionConversion.convertMatrixToQuaternion(m00, m01, m02, m10, m11, m12, m20, m21, m22, this);
   }

   /**
    * Sets this quaternion to represent a counter clockwise rotation around the z-axis of an angle
    * {@code yaw}.
    *
    * @param yaw the angle to rotate about the z-axis.
    * @deprecated Use {@link #setToYawOrientation(double)} instead
    */
   @Deprecated
   default void setToYawQuaternion(double yaw)
   {
      setToYawOrientation(yaw);
   }

   /**
    * Sets this quaternion to represent a counter clockwise rotation around the z-axis of an angle
    * {@code yaw}.
    *
    * @param yaw the angle to rotate about the z-axis.
    */
   @Override
   default void setToYawOrientation(double yaw)
   {
      QuaternionConversion.computeYawQuaternion(yaw, this);
   }

   /**
    * Sets this quaternion to represent a counter clockwise rotation around the y-axis of an angle
    * {@code pitch}.
    *
    * @param pitch the angle to rotate about the y-axis.
    * @deprecated Use {@link #setToPitchOrientation(double)} instead
    */
   @Deprecated
   default void setToPitchQuaternion(double pitch)
   {
      setToPitchOrientation(pitch);
   }

   /**
    * Sets this quaternion to represent a counter clockwise rotation around the y-axis of an angle
    * {@code pitch}.
    *
    * @param pitch the angle to rotate about the y-axis.
    */
   @Override
   default void setToPitchOrientation(double pitch)
   {
      QuaternionConversion.computePitchQuaternion(pitch, this);
   }

   /**
    * Sets this quaternion to represent a counter clockwise rotation around the x-axis of an angle
    * {@code roll}.
    *
    * @param roll the angle to rotate about the x-axis.
    * @deprecated Use {@link #setToRollOrientation(double)} instead
    */
   @Deprecated
   default void setToRollQuaternion(double roll)
   {
      setToRollOrientation(roll);
   }

   /**
    * Sets this quaternion to represent a counter clockwise rotation around the x-axis of an angle
    * {@code roll}.
    *
    * @param roll the angle to rotate about the x-axis.
    */
   @Override
   default void setToRollOrientation(double roll)
   {
      QuaternionConversion.computeRollQuaternion(roll, this);
   }

   /**
    * Sets this quaternion to the difference of {@code q1} and {@code q2}.
    * <p>
    * this = q1<sup>-1</sup> * q2
    * </p>
    *
    * @param q1 the first quaternion in the difference. Not modified.
    * @param q2 the second quaternion in the difference. Not modified.
    */
   default void difference(QuaternionReadOnly q1, QuaternionReadOnly q2)
   {
      QuaternionTools.multiplyConjugateLeft(q1, q2, this);
   }

   /**
    * Multiplies this quaternion by {@code other}.
    * <p>
    * this = this * other
    * </p>
    *
    * @param other the other quaternion to multiply this. Not modified.
    */
   default void multiply(QuaternionReadOnly other)
   {
      QuaternionTools.multiply(this, other, this);
   }

   /**
    * Sets this quaternion to the multiplication of {@code q1} and {@code q2}.
    * <p>
    * this = q1 * q2
    * </p>
    *
    * @param q1 the first quaternion in the multiplication. Not modified.
    * @param q2 the second quaternion in the multiplication. Not modified.
    */
   default void multiply(QuaternionReadOnly q1, QuaternionReadOnly q2)
   {
      QuaternionTools.multiply(q1, q2, this);
   }

   /** {@inheritDoc} */
   @Override
   default void append(Orientation3DReadOnly orientation)
   {
      QuaternionTools.multiply(this, false, orientation, false, this);
   }

   /**
    * Multiplies this quaternion by the conjugate of {@code other}.
    * <p>
    * this = this * other*
    * </p>
    *
    * @param other the other quaternion to multiply this with. Not modified.
    */
   default void multiplyConjugateOther(QuaternionReadOnly other)
   {
      QuaternionTools.multiplyConjugateRight(this, other, this);
   }

   /** {@inheritDoc} */
   @Override
   default void appendInvertOther(Orientation3DReadOnly orientation)
   {
      QuaternionTools.multiply(this, false, orientation, true, this);
   }

   /**
    * Sets this quaternion to the multiplication of the conjugate of {@code this} and {@code other}.
    * <p>
    * this = this* * other
    * </p>
    *
    * @param other the other quaternion to multiply this with. Not modified.
    */
   default void multiplyConjugateThis(QuaternionReadOnly other)
   {
      QuaternionTools.multiplyConjugateLeft(this, other, this);
   }

   /**
    * Sets this quaternion to the multiplication of the conjugate of {@code this} and {@code other}.
    * <p>
    * this = this* * other
    * </p>
    *
    * @param other the other quaternion to multiply this with. Not modified.
    */
   default void multiplyConjugateBoth(QuaternionReadOnly other)
   {
      QuaternionTools.multiplyConjugateBoth(this, other, this);
   }

   /**
    * Append a rotation about the z-axis to this quaternion.
    *
    * <pre>
    *               / qx =     0      \
    * this = this * | qy =     0      |
    *               | qz = sin(yaw/2) |
    *               \ qs = cos(yaw/2) /
    * </pre>
    *
    * @param yaw the angle to rotate about the z-axis.
    */
   @Override
   default void appendYawRotation(double yaw)
   {
      QuaternionTools.appendYawRotation(this, yaw, this);
   }

   /**
    * Append a rotation about the y-axis to this quaternion.
    *
    * <pre>
    *               / qx =      0       \
    * this = this * | qy = sin(pitch/2) |
    *               | qz =      0       |
    *               \ qs = cos(pitch/2) /
    * </pre>
    *
    * @param pitch the angle to rotate about the y-axis.
    */
   @Override
   default void appendPitchRotation(double pitch)
   {
      QuaternionTools.appendPitchRotation(this, pitch, this);
   }

   /**
    * Append a rotation about the x-axis to this quaternion.
    *
    * <pre>
    *               / qx = sin(roll/2) \
    * this = this * | qy =      0      |
    *               | qz =      0      |
    *               \ qs = cos(roll/2) /
    * </pre>
    *
    * @param roll the angle to rotate about the x-axis.
    */
   @Override
   default void appendRollRotation(double roll)
   {
      QuaternionTools.appendRollRotation(this, roll, this);
   }

   /**
    * Pre-multiplies this quaternion by {@code other}.
    * <p>
    * this = other * this
    * </p>
    *
    * @param other the other quaternion to multiply this with. Not modified.
    */
   default void preMultiply(QuaternionReadOnly other)
   {
      QuaternionTools.multiply(other, this, this);
   }

   /** {@inheritDoc} */
   @Override
   default void prepend(Orientation3DReadOnly orientation)
   {
      QuaternionTools.multiply(orientation, false, this, false, this);
   }

   /**
    * Sets this quaternion to the multiplication of the conjugate of {@code other} and {@code this}.
    * <p>
    * this = other* * this
    * </p>
    *
    * @param other the other quaternion to multiply this with. Not modified.
    */
   default void preMultiplyConjugateOther(QuaternionReadOnly other)
   {
      QuaternionTools.multiplyConjugateLeft(other, this, this);
   }

   /** {@inheritDoc} */
   @Override
   default void prependInvertOther(Orientation3DReadOnly orientation)
   {
      QuaternionTools.multiply(orientation, true, this, false, this);
   }

   /**
    * Sets this quaternion to the multiplication of {@code other} and the conjugate of {@code this}.
    * <p>
    * this = other * this*
    * </p>
    *
    * @param other the other quaternion to multiply this with. Not modified.
    */
   default void preMultiplyConjugateThis(QuaternionReadOnly other)
   {
      QuaternionTools.multiplyConjugateRight(other, this, this);
   }

   /**
    * Sets this quaternion to the multiplication of the conjugate of {@code other} and the conjugate of
    * {@code this}.
    * <p>
    * this = other* * this*
    * </p>
    *
    * @param other the other quaternion to multiply this with. Not modified.
    */
   default void preMultiplyConjugateBoth(QuaternionReadOnly other)
   {
      QuaternionTools.multiplyConjugateBoth(other, this, this);
   }

   /**
    * Prepend a rotation about the z-axis to this quaternion.
    *
    * <pre>
    *        / qx =     0      \
    * this = | qy =     0      | * this
    *        | qz = sin(yaw/2) |
    *        \ qs = cos(yaw/2) /
    * </pre>
    *
    * @param yaw the angle to rotate about the z-axis.
    */
   @Override
   default void prependYawRotation(double yaw)
   {
      QuaternionTools.prependYawRotation(yaw, this, this);
   }

   /**
    * Prepend a rotation about the y-axis to this quaternion.
    *
    * <pre>
    *        / qx =      0       \
    * this = | qy = sin(pitch/2) | * this
    *        | qz =      0       |
    *        \ qs = cos(pitch/2) /
    * </pre>
    *
    * @param pitch the angle to rotate about the y-axis.
    */
   @Override
   default void prependPitchRotation(double pitch)
   {
      QuaternionTools.prependPitchRotation(pitch, this, this);
   }

   /**
    * Prepend a rotation about the x-axis to this quaternion.
    *
    * <pre>
    *        / qx = sin(roll/2) \
    * this = | qy =      0      | * this
    *        | qz =      0      |
    *        \ qs = cos(roll/2) /
    * </pre>
    *
    * @param roll the angle to rotate about the x-axis.
    */
   @Override
   default void prependRollRotation(double roll)
   {
      QuaternionTools.prependRollRotation(roll, this, this);
   }

   /**
    * Performs a linear interpolation in SO(3) from {@code this} to {@code qf} given the percentage
    * {@code alpha}.
    * <p>
    * The interpolation method used here is often called a <i>Spherical Linear Interpolation</i> or
    * SLERP.
    * </p>
    *
    * @param qf    the other quaternion used for the interpolation. Not modified.
    * @param alpha the percentage used for the interpolation. A value of 0 will result in not modifying
    *              this quaternion, while a value of 1 is equivalent to setting this quaternion to
    *              {@code qf}.
    */
   default void interpolate(QuaternionReadOnly qf, double alpha)
   {
      interpolate(this, qf, alpha);
   }

   /**
    * Performs a linear interpolation in SO(3) from {@code q0} to {@code qf} given the percentage
    * {@code alpha}.
    * <p>
    * The interpolation method used here is often called a <i>Spherical Linear Interpolation</i> or
    * SLERP.
    * </p>
    *
    * @param q0    the first quaternion used in the interpolation. Not modified.
    * @param qf    the second quaternion used in the interpolation. Not modified.
    * @param alpha the percentage to use for the interpolation. A value of 0 will result in setting
    *              this quaternion to {@code q0}, while a value of 1 is equivalent to setting this
    *              quaternion to {@code qf}.
    */
   default void interpolate(QuaternionReadOnly q0, QuaternionReadOnly qf, double alpha)
   {
      double cosHalfTheta = q0.dot(qf);
      double sign = 1.0;

      if (cosHalfTheta < 0.0)
      {
         sign = -1.0;
         cosHalfTheta = -cosHalfTheta;
      }

      double alpha0 = 1.0 - alpha;
      double alphaf = alpha;

      if (1.0 - cosHalfTheta > 1.0e-12)
      {
         double halfTheta = Math.acos(cosHalfTheta);
         double sinHalfTheta = Math.sin(halfTheta);
         alpha0 = Math.sin(alpha0 * halfTheta) / sinHalfTheta;
         alphaf = Math.sin(alphaf * halfTheta) / sinHalfTheta;
      }

      double qx = alpha0 * q0.getX() + sign * alphaf * qf.getX();
      double qy = alpha0 * q0.getY() + sign * alphaf * qf.getY();
      double qz = alpha0 * q0.getZ() + sign * alphaf * qf.getZ();
      double qs = alpha0 * q0.getS() + sign * alphaf * qf.getS();
      set(qx, qy, qz, qs);
   }
}
