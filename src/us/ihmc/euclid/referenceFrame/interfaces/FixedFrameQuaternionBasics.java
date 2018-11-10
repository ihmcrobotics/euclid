package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;

/**
 * Write and read interface for a quaternion expressed in a constant reference frame, i.e. this
 * quaternion is always expressed in the same reference frame.
 * <p>
 * In addition to representing a {@link QuaternionBasics}, a {@link ReferenceFrame} is associated to
 * a {@code FixedFrameQuaternionBasics}. This allows, for instance, to enforce, at runtime, that
 * operations on quaternions occur in the same coordinate system. Also, via the method
 * {@link FrameChangeable#changeFrame(ReferenceFrame)}, one can easily calculates the value of a
 * quaternion in different reference frames.
 * </p>
 * <p>
 * Because a {@code FixedFrameQuaternionBasics} extends {@code QuaternionBasics}, it is compatible
 * with methods only requiring {@code QuaternionBasics}. However, these methods do NOT assert that
 * the operation occur in the proper coordinate system. Use this feature carefully and always prefer
 * using methods requiring {@code FixedFrameQuaternionBasics}.
 * </p>
 */
public interface FixedFrameQuaternionBasics extends FrameQuaternionReadOnly, FixedFrameTuple4DBasics, FixedFrameOrientation3DBasics, QuaternionBasics
{
   /**
    * Sets this quaternion to represent the orientation from {@code this.getReferenceFrame()} to the
    * given {@code referenceFrame}.
    *
    * @param referenceFrame the reference frame of interest.
    */
   default void setFromReferenceFrame(ReferenceFrame referenceFrame)
   {
      setToZero();
      referenceFrame.transformFromThisToDesiredFrame(getReferenceFrame(), this);
   }

   /**
    * Sets this frame quaternion to {@code orientation3DReadOnly} and checks that its current frame
    * equals {@code referenceFrame}.
    *
    * @param referenceFrame the coordinate system in which the given {@code quaternionReadOnly} is
    *           expressed.
    * @param orientation3DReadOnly the orientation to copy the values from. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this.referenceFrame != referenceFrame}.
    */
   default void set(ReferenceFrame referenceFrame, Orientation3DReadOnly orientation3DReadOnly)
   {
      checkReferenceFrameMatch(referenceFrame);
      set(orientation3DReadOnly);
   }

   /**
    * Sets this frame quaternion to {@code quaternionReadOnly} and checks that its current frame equals
    * {@code referenceFrame}.
    *
    * @param referenceFrame the coordinate system in which the given {@code quaternionReadOnly} is
    *           expressed.
    * @param quaternionReadOnly the quaternion to copy the values from. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this.referenceFrame != referenceFrame}.
    */
   default void set(ReferenceFrame referenceFrame, QuaternionReadOnly quaternionReadOnly)
   {
      checkReferenceFrameMatch(referenceFrame);
      set(quaternionReadOnly);
   }

   /**
    * Sets this frame quaternion to the same orientation described by the given rotation vector
    * {@code rotationVector}.
    * <p>
    * WARNING: a rotation vector is different from a yaw-pitch-roll or Euler angles representation. A
    * rotation vector is equivalent to the axis of an axis-angle that is multiplied by the angle of the
    * same axis-angle.
    * </p>
    *
    * @param rotationVector vector the rotation vector used to set this quaternion. Not modified.
    * @throws ReferenceFrameMismatchException if {@code rotationVector} is not expressed in the same
    *            reference frame as {@code this}.
    */
   default void setRotationVector(FrameVector3DReadOnly rotationVector)
   {
      checkReferenceFrameMatch(rotationVector);
      QuaternionBasics.super.setRotationVector(rotationVector);
   }

   /**
    * Sets this frame quaternion to {@code other}.
    *
    * @param other the other quaternion to copy the values from. Not modified.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same reference
    *            frame as {@code this}.
    */
   default void set(FrameQuaternionReadOnly other)
   {
      checkReferenceFrameMatch(other);
      QuaternionBasics.super.set(other);
   }

   /**
    * Sets this frame quaternion to {@code other}.
    * <p>
    * If {@code other} is expressed in the frame as {@code this}, then this method is equivalent to
    * {@link #set(FrameQuaternionReadOnly)}.
    * </p>
    * <p>
    * If {@code other} is expressed in a different frame than {@code this}, then {@code this} is set to
    * {@code other} and then transformed to be expressed in {@code this.getReferenceFrame()}.
    * </p>
    *
    * @param other the other quaternion to copy the values from. Not modified.
    */
   default void setMatchingFrame(FrameQuaternionReadOnly other)
   {
      QuaternionBasics.super.set(other);
      other.getReferenceFrame().transformFromThisToDesiredFrame(getReferenceFrame(), this);
   }

   /**
    * Sets this frame quaternion to {@code other} and then calls {@link #negate()}.
    *
    * @param other the other frame quaternion to set to. Not modified.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same reference
    *            frame as {@code this}.
    */
   default void setAndNegate(FrameQuaternionReadOnly other)
   {
      checkReferenceFrameMatch(other);
      QuaternionBasics.super.setAndNegate(other);
   }

   /**
    * Sets this frame quaternion to the conjugate of {@code other}.
    *
    * <pre>
    *      / -qx \
    * q* = | -qy |
    *      | -qz |
    *      \  qs /
    * </pre>
    *
    * @param other the other frame quaternion to copy the values from. Not modified.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same reference
    *            frame as {@code this}.
    */
   default void setAndConjugate(FrameQuaternionReadOnly other)
   {
      checkReferenceFrameMatch(other);
      QuaternionBasics.super.setAndConjugate(other);
   }

   /**
    * Sets this frame quaternion to the inverse of {@code other}.
    *
    * @param other the other frame quaternion to copy the values from. Not modified.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same reference
    *            frame as {@code this}.
    * @deprecated Use {@link #setAndInvert(FrameQuaternionReadOnly)} instead
    */
   default void setAndInverse(FrameQuaternionReadOnly other)
   {
      setAndInvert(other);
   }

   /**
    * Sets this frame quaternion to the inverse of {@code other}.
    *
    * @param other the other frame quaternion to copy the values from. Not modified.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same reference
    *            frame as {@code this}.
    */
   default void setAndInvert(FrameQuaternionReadOnly other)
   {
      checkReferenceFrameMatch(other);
      QuaternionBasics.super.setAndInvert(other);
   }

   /**
    * Sets this frame quaternion to {@code other} and then calls {@link #normalize()}.
    *
    * @param other the other frame quaternion to copy the values from. Not modified.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same reference
    *            frame as {@code this}.
    */
   default void setAndNormalize(FrameQuaternionReadOnly other)
   {
      checkReferenceFrameMatch(other);
      QuaternionBasics.super.setAndNormalize(other);
   }

   /**
    * Sets this frame quaternion to represent the same orientation as the given Euler angles
    * {@code eulerAngles}.
    * <p>
    * This is equivalent to
    * {@code this.setYawPitchRoll(eulerAngles.getZ(), eulerAngles.getY(), eulerAngles.getX())}.
    * </p>
    *
    * @param eulerAngles the Euler angles to copy the orientation from. Not modified.
    * @throws ReferenceFrameMismatchException if {@code eulerAngles} is not expressed in the same
    *            reference frame as {@code this}.
    */
   default void setEuler(FrameVector3DReadOnly eulerAngles)
   {
      checkReferenceFrameMatch(eulerAngles);
      QuaternionBasics.super.setEuler(eulerAngles);
   }

   /**
    * Performs a linear interpolation in SO(3) from {@code this} to {@code qf} given the percentage
    * {@code alpha}.
    * <p>
    * The interpolation method used here is often called a <i>Spherical Linear Interpolation</i> or
    * SLERP.
    * </p>
    *
    * @param qf the other frame quaternion used for the interpolation. Not modified.
    * @param alpha the percentage used for the interpolation. A value of 0 will result in not modifying
    *           this frame quaternion, while a value of 1 is equivalent to setting this frame
    *           quaternion to {@code qf}.
    * @throws ReferenceFrameMismatchException if {@code qf} is not expressed in the same reference
    *            frame as {@code this}.
    */
   default void interpolate(FrameQuaternionReadOnly qf, double alpha)
   {
      checkReferenceFrameMatch(qf);
      QuaternionBasics.super.interpolate(qf, alpha);
   }

   /**
    * Performs a linear interpolation in SO(3) from {@code q0} to {@code qf} given the percentage
    * {@code alpha}.
    * <p>
    * The interpolation method used here is often called a <i>Spherical Linear Interpolation</i> or
    * SLERP.
    * </p>
    *
    * @param q0 the first frame quaternion used in the interpolation. Not modified.
    * @param qf the second quaternion used in the interpolation. Not modified.
    * @param alpha the percentage to use for the interpolation. A value of 0 will result in setting
    *           this frame quaternion to {@code q0}, while a value of 1 is equivalent to setting this
    *           frame quaternion to {@code qf}.
    * @throws ReferenceFrameMismatchException if {@code q0} is not expressed in the same reference
    *            frame as {@code this}.
    */
   default void interpolate(FrameQuaternionReadOnly q0, QuaternionReadOnly qf, double alpha)
   {
      checkReferenceFrameMatch(q0);
      QuaternionBasics.super.interpolate(q0, qf, alpha);
   }

   /**
    * Performs a linear interpolation in SO(3) from {@code q0} to {@code qf} given the percentage
    * {@code alpha}.
    * <p>
    * The interpolation method used here is often called a <i>Spherical Linear Interpolation</i> or
    * SLERP.
    * </p>
    *
    * @param q0 the first quaternion used in the interpolation. Not modified.
    * @param qf the second frame quaternion used in the interpolation. Not modified.
    * @param alpha the percentage to use for the interpolation. A value of 0 will result in setting
    *           this frame quaternion to {@code q0}, while a value of 1 is equivalent to setting this
    *           frame quaternion to {@code qf}.
    * @throws ReferenceFrameMismatchException if {@code qf} is not expressed in the same reference
    *            frame as {@code this}.
    */
   default void interpolate(QuaternionReadOnly q0, FrameQuaternionReadOnly qf, double alpha)
   {
      checkReferenceFrameMatch(qf);
      QuaternionBasics.super.interpolate(q0, qf, alpha);
   }

   /**
    * Performs a linear interpolation in SO(3) from {@code q0} to {@code qf} given the percentage
    * {@code alpha}.
    * <p>
    * The interpolation method used here is often called a <i>Spherical Linear Interpolation</i> or
    * SLERP.
    * </p>
    *
    * @param q0 the first quaternion used in the interpolation. Not modified.
    * @param qf the second quaternion used in the interpolation. Not modified.
    * @param alpha the percentage to use for the interpolation. A value of 0 will result in setting
    *           this quaternion to {@code q0}, while a value of 1 is equivalent to setting this
    *           quaternion to {@code qf}.
    * @throws ReferenceFrameMismatchException if either {@code q0} or {@code qf} is not expressed in
    *            the same frame as {@code this}.
    */
   default void interpolate(FrameQuaternionReadOnly q0, FrameQuaternionReadOnly qf, double alpha)
   {
      checkReferenceFrameMatch(q0);
      checkReferenceFrameMatch(qf);
      QuaternionBasics.super.interpolate(q0, qf, alpha);
   }

   /**
    * Multiplies this frame quaternion by {@code other}.
    * <p>
    * this = this * other
    * </p>
    *
    * @param other the other frame quaternion to multiply this. Not modified.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same reference
    *            frame as {@code this}.
    */
   default void multiply(FrameQuaternionReadOnly other)
   {
      checkReferenceFrameMatch(other);
      QuaternionBasics.super.multiply(other);
   }

   /**
    * Multiplies this frame quaternion by {@code other}.
    * <p>
    * this = this * other
    * </p>
    *
    * @param other the other frame quaternion to multiply this. Not modified.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same reference
    *            frame as {@code this}.
    */
   default void append(FrameQuaternionReadOnly other)
   {
      checkReferenceFrameMatch(other);
      QuaternionBasics.super.append(other);
   }

   /**
    * Sets this frame quaternion to the multiplication of {@code q1} and {@code q2}.
    * <p>
    * this = q1 * q2
    * </p>
    *
    * @param q1 the first frame quaternion in the multiplication. Not modified.
    * @param q2 the second quaternion in the multiplication. Not modified.
    * @throws ReferenceFrameMismatchException if {@code q1} is not expressed in the same reference
    *            frame as {@code this}.
    */
   default void multiply(FrameQuaternionReadOnly q1, QuaternionReadOnly q2)
   {
      checkReferenceFrameMatch(q1);
      QuaternionBasics.super.multiply(q1, q2);
   }

   /**
    * Sets this frame quaternion to the multiplication of {@code q1} and {@code q2}.
    * <p>
    * this = q1 * q2
    * </p>
    *
    * @param q1 the first quaternion in the multiplication. Not modified.
    * @param q2 the second frame quaternion in the multiplication. Not modified.
    * @throws ReferenceFrameMismatchException if {@code q2} is not expressed in the same reference
    *            frame as {@code this}.
    */
   default void multiply(QuaternionReadOnly q1, FrameQuaternionReadOnly q2)
   {
      checkReferenceFrameMatch(q2);
      QuaternionBasics.super.multiply(q1, q2);
   }

   /**
    * Sets this frame quaternion to the multiplication of {@code q1} and {@code q2}.
    * <p>
    * this = q1 * q2
    * </p>
    *
    * @param q1 the first frame quaternion in the multiplication. Not modified.
    * @param q2 the second frame quaternion in the multiplication. Not modified.
    * @throws ReferenceFrameMismatchException if either {@code q1} or {@code q2} is not expressed in
    *            the same frame as {@code this}.
    */
   default void multiply(FrameQuaternionReadOnly q1, FrameQuaternionReadOnly q2)
   {
      checkReferenceFrameMatch(q1);
      checkReferenceFrameMatch(q2);
      QuaternionBasics.super.multiply(q1, q2);
   }

   /**
    * Sets this frame quaternion to the difference of {@code q1} and {@code q2}.
    * <p>
    * this = q1<sup>-1</sup> * q2
    * </p>
    *
    * @param q1 the first frame quaternion in the difference. Not modified.
    * @param q2 the second quaternion in the difference. Not modified.
    * @throws ReferenceFrameMismatchException if {@code q1} is not expressed in the same reference
    *            frame as {@code this}.
    */
   default void difference(FrameQuaternionReadOnly q1, QuaternionReadOnly q2)
   {
      checkReferenceFrameMatch(q1);
      QuaternionBasics.super.difference(q1, q2);
   }

   /**
    * Sets this quaternion to the difference of {@code q1} and {@code q2}.
    * <p>
    * this = q1<sup>-1</sup> * q2
    * </p>
    *
    * @param q1 the first quaternion in the difference. Not modified.
    * @param q2 the second frame quaternion in the difference. Not modified.
    * @throws ReferenceFrameMismatchException if {@code q2} is not expressed in the same reference
    *            frame as {@code this}.
    */
   default void difference(QuaternionReadOnly q1, FrameQuaternionReadOnly q2)
   {
      checkReferenceFrameMatch(q2);
      QuaternionBasics.super.difference(q1, q2);
   }

   /**
    * Sets this quaternion to the difference of {@code q1} and {@code q2}.
    * <p>
    * this = q1<sup>-1</sup> * q2
    * </p>
    *
    * @param q1 the first frame quaternion in the difference. Not modified.
    * @param q2 the second frame quaternion in the difference. Not modified.
    * @throws ReferenceFrameMismatchException if either {@code q1} or {@code q2} is not expressed in
    *            the same frame as {@code this}.
    */
   default void difference(FrameQuaternionReadOnly q1, FrameQuaternionReadOnly q2)
   {
      checkReferenceFrameMatch(q1);
      checkReferenceFrameMatch(q2);
      QuaternionBasics.super.difference(q1, q2);
   }

   /**
    * Multiplies this frame quaternion by the conjugate of {@code other}.
    * <p>
    * this = this * other*
    * </p>
    *
    * @param other the other frame quaternion to multiply this with. Not modified.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same reference
    *            frame as {@code this}.
    */
   default void multiplyConjugateOther(FrameQuaternionReadOnly other)
   {
      checkReferenceFrameMatch(other);
      QuaternionBasics.super.multiplyConjugateOther(other);
   }

   /**
    * Multiplies this frame quaternion by the conjugate of {@code other}.
    * <p>
    * this = this * other*
    * </p>
    *
    * @param other the other frame quaternion to multiply this with. Not modified.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same reference
    *            frame as {@code this}.
    */
   default void appendInvertOther(FrameQuaternionReadOnly other)
   {
      checkReferenceFrameMatch(other);
      QuaternionBasics.super.appendInvertOther(other);
   }

   /**
    * Sets this frame quaternion to the multiplication of the conjugate of {@code this} and
    * {@code other}.
    * <p>
    * this = this* * other
    * </p>
    *
    * @param other the other frame quaternion to multiply this with. Not modified.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same reference
    *            frame as {@code this}.
    */
   default void multiplyConjugateThis(FrameQuaternionReadOnly other)
   {
      checkReferenceFrameMatch(other);
      QuaternionBasics.super.multiplyConjugateThis(other);
   }

   /**
    * Sets this frame quaternion to the multiplication of the conjugate of {@code this} and
    * {@code other}.
    * <p>
    * this = this* * other
    * </p>
    *
    * @param other the other frame quaternion to multiply this with. Not modified.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same reference
    *            frame as {@code this}.
    */
   default void appendInvertThis(FrameQuaternionReadOnly other)
   {
      checkReferenceFrameMatch(other);
      QuaternionBasics.super.appendInvertThis(other);
   }

   /**
    * Sets this frame quaternion to the multiplication of the conjugate of {@code this} and the
    * conjugate of {@code other}.
    * <p>
    * this = this* * other*
    * </p>
    *
    * @param other the other frame quaternion to multiply this with. Not modified.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same reference
    *            frame as {@code this}.
    */
   default void multiplyConjugateBoth(FrameQuaternionReadOnly other)
   {
      checkReferenceFrameMatch(other);
      QuaternionBasics.super.multiplyConjugateBoth(other);
   }

   /**
    * Sets this frame quaternion to the multiplication of the conjugate of {@code this} and the
    * conjugate of {@code other}.
    * <p>
    * this = this* * other*
    * </p>
    *
    * @param other the other frame quaternion to multiply this with. Not modified.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same reference
    *            frame as {@code this}.
    */
   default void appendInvertBoth(FrameQuaternionReadOnly other)
   {
      checkReferenceFrameMatch(other);
      QuaternionBasics.super.appendInvertBoth(other);
   }

   /**
    * Pre-multiplies this frame quaternion by {@code other}.
    * <p>
    * this = other * this
    * </p>
    *
    * @param other the other frame quaternion to multiply this with. Not modified.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same reference
    *            frame as {@code this}.
    */
   default void preMultiply(FrameQuaternionReadOnly other)
   {
      checkReferenceFrameMatch(other);
      QuaternionBasics.super.preMultiply(other);
   }

   /**
    * Pre-multiplies this frame quaternion by {@code other}.
    * <p>
    * this = other * this
    * </p>
    *
    * @param other the other frame quaternion to multiply this with. Not modified.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same reference
    *            frame as {@code this}.
    */
   default void prepend(FrameQuaternionReadOnly other)
   {
      checkReferenceFrameMatch(other);
      QuaternionBasics.super.prepend(other);
   }

   /**
    * Sets this frame quaternion to the multiplication of the conjugate of {@code other} and
    * {@code this}.
    * <p>
    * this = other* * this
    * </p>
    *
    * @param other the other frame quaternion to multiply this with. Not modified.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same reference
    *            frame as {@code this}.
    */
   default void preMultiplyConjugateOther(FrameQuaternionReadOnly other)
   {
      checkReferenceFrameMatch(other);
      QuaternionBasics.super.preMultiplyConjugateOther(other);
   }

   /**
    * Sets this frame quaternion to the multiplication of the conjugate of {@code other} and
    * {@code this}.
    * <p>
    * this = other* * this
    * </p>
    *
    * @param other the other frame quaternion to multiply this with. Not modified.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same reference
    *            frame as {@code this}.
    */
   default void prependInvertOther(FrameQuaternionReadOnly other)
   {
      checkReferenceFrameMatch(other);
      QuaternionBasics.super.prependInvertOther(other);
   }

   /**
    * Sets this frame quaternion to the multiplication of {@code other} and the conjugate of
    * {@code this}.
    * <p>
    * this = other * this*
    * </p>
    *
    * @param other the other frame quaternion to multiply this with. Not modified.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same reference
    *            frame as {@code this}.
    */
   default void preMultiplyConjugateThis(FrameQuaternionReadOnly other)
   {
      checkReferenceFrameMatch(other);
      QuaternionBasics.super.preMultiplyConjugateThis(other);
   }

   /**
    * Sets this frame quaternion to the multiplication of the conjugate of {@code other} and
    * {@code this}.
    * <p>
    * this = other* * this
    * </p>
    *
    * @param other the other frame quaternion to multiply this with. Not modified.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same reference
    *            frame as {@code this}.
    */
   default void prependInvertThis(FrameQuaternionReadOnly other)
   {
      checkReferenceFrameMatch(other);
      QuaternionBasics.super.prependInvertThis(other);
   }

   /**
    * Sets this frame quaternion to the multiplication of the conjugate of {@code other} and the
    * conjugate of {@code this}.
    * <p>
    * this = other* * this*
    * </p>
    *
    * @param other the other frame quaternion to multiply this with. Not modified.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same reference
    *            frame as {@code this}.
    */
   default void preMultiplyConjugateBoth(FrameQuaternionReadOnly other)
   {
      checkReferenceFrameMatch(other);
      QuaternionBasics.super.preMultiplyConjugateBoth(other);
   }

   /**
    * Sets this frame quaternion to the multiplication of the conjugate of {@code other} and the
    * conjugate of {@code this}.
    * <p>
    * this = other* * this*
    * </p>
    *
    * @param other the other frame quaternion to multiply this with. Not modified.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same reference
    *            frame as {@code this}.
    */
   default void prependInvertBoth(FrameQuaternionReadOnly other)
   {
      checkReferenceFrameMatch(other);
      QuaternionBasics.super.prependInvertBoth(other);
   }
}
