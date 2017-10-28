package us.ihmc.euclid.referenceFrame;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.euclid.axisAngle.interfaces.AxisAngleReadOnly;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.Tuple4DReadOnly;

/**
 * {@code FrameQuaternion} is a quaternion expressed in a given reference frame.
 * <p>
 * In addition to representing a {@link QuaternionBasics}, a {@link ReferenceFrame} is associated to
 * a {@code FrameQuaternion}. This allows, for instance, to enforce, at runtime, that operations on
 * quaternions occur in the same coordinate system. Also, via the method
 * {@link #changeFrame(ReferenceFrame)}, one can easily calculates the value of a quaternion in
 * different reference frames.
 * </p>
 * <p>
 * Because a {@code FrameQuaternion} extends {@code QuaternionBasics}, it is compatible with methods
 * only requiring {@code QuaternionBasics}. However, these methods do NOT assert that the operation
 * occur in the proper coordinate system. Use this feature carefully and always prefer using methods
 * requiring {@code FrameQuaternion}.
 * </p>
 */
public class FrameQuaternion extends FrameTuple4D<FrameQuaternion, Quaternion> implements FrameQuaternionReadOnly, QuaternionBasics
{
   /**
    * Creates a new frame quaternion and initializes it to the neutral quaternion, i.e. representing
    * a 'zero' rotation, and its reference frame to {@link ReferenceFrame#getWorldFrame()}.
    */
   public FrameQuaternion()
   {
      this(ReferenceFrame.getWorldFrame());
   }

   /**
    * Creates a new frame quaternion and initializes it to the neutral quaternion, i.e. representing
    * a 'zero' rotation, and its reference frame to the {@code referenceFrame}.
    *
    * @param referenceFrame the initial frame for this frame quaternion.
    */
   public FrameQuaternion(ReferenceFrame referenceFrame)
   {
      super(referenceFrame, new Quaternion());
   }

   /**
    * Creates a new frame quaternion and initializes it with the given components and the given
    * reference frame.
    *
    * @param referenceFrame the initial frame for this frame quaternion.
    * @param x the x-component.
    * @param y the y-component.
    * @param z the z-component.
    * @param s the s-component.
    */
   public FrameQuaternion(ReferenceFrame referenceFrame, double x, double y, double z, double s)
   {
      super(referenceFrame, new Quaternion(x, y, z, s));
   }

   /**
    * Creates a new frame quaternion and initializes its component {@code x}, {@code y}, {@code z},
    * {@code s} in order from the given array and initializes its reference frame.
    *
    * @param referenceFrame the initial frame for this frame quaternion.
    * @param quaternionArray the array containing this quaternion's components. Not modified.
    */
   public FrameQuaternion(ReferenceFrame referenceFrame, double[] quaternionArray)
   {
      super(referenceFrame, new Quaternion(quaternionArray));
   }

   /**
    * Creates a new frame quaternion and initializes its component {@code x}, {@code y}, {@code z},
    * {@code s} in order from the given matrix and initializes its reference frame.
    * <p>
    * The quaternion is immediately normalized.
    * </p>
    *
    * @param referenceFrame the initial frame for this frame quaternion.
    * @param matrix the dense-matrix containing this quaternion's components. Not modified.
    */
   public FrameQuaternion(ReferenceFrame referenceFrame, DenseMatrix64F matrix)
   {
      super(referenceFrame, new Quaternion(matrix));
   }

   /**
    * Creates a new quaternion and initializes it to {@code quaternionReadOnly} and to the given
    * reference frame.
    *
    * @param referenceFrame the initial frame for this frame quaternion.
    * @param other the quaternion to copy the components from. Not modified.
    */
   public FrameQuaternion(ReferenceFrame referenceFrame, QuaternionReadOnly quaternionReadOnly)
   {
      super(referenceFrame, new Quaternion(quaternionReadOnly));
   }

   /**
    * Creates a new frame quaternion and initializes it to {@code tuple4DReadOnly} and to the given
    * reference frame.
    *
    * @param referenceFrame the initial frame for this frame quaternion.
    * @param tuple4DReadOnly the tuple to copy the components from. Not modified.
    */
   public FrameQuaternion(ReferenceFrame referenceFrame, Tuple4DReadOnly tuple4DReadOnly)
   {
      super(referenceFrame, new Quaternion(tuple4DReadOnly));
   }

   /**
    * Creates a new frame quaternion and initializes such that it represents the same orientation as
    * the given {@code rotationMatrix} and initializes its reference frame.
    *
    * @param referenceFrame the initial frame for this frame quaternion.
    * @param rotationMatrix the rotation matrix to initialize this quaternion. Not modified.
    */
   public FrameQuaternion(ReferenceFrame referenceFrame, RotationMatrixReadOnly rotationMatrix)
   {
      super(referenceFrame, new Quaternion(rotationMatrix));
   }

   /**
    * Creates a new frame quaternion and initializes such that it represents the same orientation as
    * the given {@code axisAngle} and initializes its reference frame.
    *
    * @param referenceFrame the initial frame for this frame quaternion.
    * @param axisAngle the axis-angle to initialize this quaternion. Not modified.
    */
   public FrameQuaternion(ReferenceFrame referenceFrame, AxisAngleReadOnly axisAngle)
   {
      super(referenceFrame, new Quaternion(axisAngle));
   }

   /**
    * Creates a new frame quaternion and initializes such that it represents the same orientation as
    * the given {@code rotationVector} and initializes its reference frame.
    * <p>
    * WARNING: a rotation vector is different from a yaw-pitch-roll or Euler angles representation.
    * A rotation vector is equivalent to the axis of an axis-angle that is multiplied by the angle
    * of the same axis-angle.
    * </p>
    *
    * @param referenceFrame the initial frame for this frame quaternion.
    * @param rotationVector the rotation vector to initialize this quaternion. Not modified.
    */
   public FrameQuaternion(ReferenceFrame referenceFrame, Vector3DReadOnly rotationVector)
   {
      super(referenceFrame, new Quaternion(rotationVector));
   }

   /**
    * Creates a new frame quaternion and initializes it to {@code other}.
    *
    * @param other the frame quaternion to copy the components and reference frame from. Not
    *           modified.
    */
   public FrameQuaternion(FrameQuaternionReadOnly other)
   {
      super(other.getReferenceFrame(), new Quaternion(other));
   }

   /** {@inheritDoc} */
   @Override
   public final void setUnsafe(double qx, double qy, double qz, double qs)
   {
      tuple.setUnsafe(qx, qy, qz, qs);
   }

   /**
    * Sets this frame quaternion to the same orientation described by the given rotation vector
    * {@code rotationVector}.
    * <p>
    * WARNING: a rotation vector is different from a yaw-pitch-roll or Euler angles representation.
    * A rotation vector is equivalent to the axis of an axis-angle that is multiplied by the angle
    * of the same axis-angle.
    * </p>
    *
    * @param rotation vector the rotation vector used to set this quaternion. Not modified.
    * @throws ReferenceFrameMismatchException if {@code rotationVector} is not expressed in the same
    *            reference frame as {@code this}.
    */
   public final void set(FrameVector3DReadOnly rotationVector)
   {
      checkReferenceFrameMatch(rotationVector);
      tuple.set(rotationVector);
   }

   /**
    * Sets this frame quaternion to {@code other}.
    *
    * @param other the other quaternion to copy the values from. Not modified.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same
    *            reference frame as {@code this}.
    */
   public final void set(FrameQuaternionReadOnly other)
   {
      checkReferenceFrameMatch(other);
      tuple.set(other);
   }

   /**
    * Sets this frame quaternion to {@code other} and then calls {@link #negate()}.
    *
    * @param other the other frame quaternion to set to. Not modified.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same
    *            reference frame as {@code this}.
    */
   public final void setAndNegate(FrameQuaternionReadOnly other)
   {
      checkReferenceFrameMatch(other);
      tuple.setAndNegate(other);
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
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same
    *            reference frame as {@code this}.
    */
   public final void setAndConjugate(FrameQuaternionReadOnly other)
   {
      checkReferenceFrameMatch(other);
      tuple.setAndConjugate(other);
   }

   /**
    * Sets this frame quaternion to the inverse of {@code other}.
    *
    * @param other the other frame quaternion to copy the values from. Not modified.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same
    *            reference frame as {@code this}.
    */
   public final void setAndInverse(FrameQuaternionReadOnly other)
   {
      checkReferenceFrameMatch(other);
      tuple.setAndInverse(other);
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
   public final void setEuler(FrameVector3DReadOnly eulerAngles)
   {
      checkReferenceFrameMatch(eulerAngles);
      tuple.setEuler(eulerAngles);
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
    * @param alpha the percentage used for the interpolation. A value of 0 will result in not
    *           modifying this frame quaternion, while a value of 1 is equivalent to setting this
    *           frame quaternion to {@code qf}.
    * @throws ReferenceFrameMismatchException if {@code qf} is not expressed in the same reference
    *            frame as {@code this}.
    */
   public final void interpolate(FrameQuaternionReadOnly qf, double alpha)
   {
      checkReferenceFrameMatch(qf);
      tuple.interpolate(qf, alpha);
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
    *           this frame quaternion to {@code q0}, while a value of 1 is equivalent to setting
    *           this frame quaternion to {@code qf}.
    * @throws ReferenceFrameMismatchException if {@code q0} is not expressed in the same reference
    *            frame as {@code this}.
    */
   public final void interpolate(FrameQuaternionReadOnly q0, QuaternionReadOnly qf, double alpha)
   {
      checkReferenceFrameMatch(q0);
      tuple.interpolate(q0, qf, alpha);
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
    *           this frame quaternion to {@code q0}, while a value of 1 is equivalent to setting
    *           this frame quaternion to {@code qf}.
    * @throws ReferenceFrameMismatchException if {@code qf} is not expressed in the same reference
    *            frame as {@code this}.
    */
   public final void interpolate(QuaternionReadOnly q0, FrameQuaternionReadOnly qf, double alpha)
   {
      checkReferenceFrameMatch(qf);
      tuple.interpolate(q0, qf, alpha);
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
   public final void interpolate(FrameQuaternionReadOnly q0, FrameQuaternionReadOnly qf, double alpha)
   {
      checkReferenceFrameMatch(q0);
      checkReferenceFrameMatch(qf);
      tuple.interpolate(q0, qf, alpha);
   }

   /**
    * Multiplies this frame quaternion by {@code other}.
    * <p>
    * this = this * other
    * </p>
    *
    * @param other the other frame quaternion to multiply this. Not modified.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same
    *            reference frame as {@code this}.
    */
   public final void multiply(FrameQuaternionReadOnly other)
   {
      checkReferenceFrameMatch(other);
      tuple.multiply(other);
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
   public final void multiply(FrameQuaternionReadOnly q1, QuaternionReadOnly q2)
   {
      checkReferenceFrameMatch(q1);
      tuple.multiply(q1, q2);
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
   public final void multiply(QuaternionReadOnly q1, FrameQuaternionReadOnly q2)
   {
      checkReferenceFrameMatch(q2);
      tuple.multiply(q1, q2);
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
   public final void multiply(FrameQuaternionReadOnly q1, FrameQuaternionReadOnly q2)
   {
      checkReferenceFrameMatch(q1);
      checkReferenceFrameMatch(q2);
      tuple.multiply(q1, q2);
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
   public final void difference(FrameQuaternionReadOnly q1, QuaternionReadOnly q2)
   {
      checkReferenceFrameMatch(q1);
      tuple.difference(q1, q2);
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
   public final void difference(QuaternionReadOnly q1, FrameQuaternionReadOnly q2)
   {
      checkReferenceFrameMatch(q2);
      tuple.difference(q1, q2);
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
   public final void difference(FrameQuaternionReadOnly q1, FrameQuaternionReadOnly q2)
   {
      checkReferenceFrameMatch(q1);
      checkReferenceFrameMatch(q2);
      tuple.difference(q1, q2);
   }

   /**
    * Multiplies this frame quaternion by the conjugate of {@code other}.
    * <p>
    * this = this * other*
    * </p>
    *
    * @param other the other frame quaternion to multiply this with. Not modified.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same
    *            reference frame as {@code this}.
    */
   public final void multiplyConjugateOther(FrameQuaternionReadOnly other)
   {
      checkReferenceFrameMatch(other);
      tuple.multiplyConjugateOther(other);
   }

   /**
    * Sets this frame quaternion to the multiplication of the conjugate of {@code this} and
    * {@code other}.
    * <p>
    * this = this* * other
    * </p>
    *
    * @param other the other frame quaternion to multiply this with. Not modified.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same
    *            reference frame as {@code this}.
    */
   public final void multiplyConjugateThis(FrameQuaternionReadOnly other)
   {
      checkReferenceFrameMatch(other);
      tuple.multiplyConjugateThis(other);
   }

   /**
    * Pre-multiplies this frame quaternion by {@code other}.
    * <p>
    * this = other * this
    * </p>
    *
    * @param other the other frame quaternion to multiply this with. Not modified.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same
    *            reference frame as {@code this}.
    */
   public final void preMultiply(FrameQuaternionReadOnly other)
   {
      checkReferenceFrameMatch(other);
      tuple.preMultiply(other);
   }

   /**
    * Sets this frame quaternion to the multiplication of the conjugate of {@code other} and
    * {@code this}.
    * <p>
    * this = other* * this
    * </p>
    *
    * @param other the other frame quaternion to multiply this with. Not modified.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same
    *            reference frame as {@code this}.
    */
   public final void preMultiplyConjugateOther(FrameQuaternionReadOnly other)
   {
      checkReferenceFrameMatch(other);
      tuple.preMultiplyConjugateOther(other);
   }

   /**
    * Sets this frame quaternion to the multiplication of {@code other} and the conjugate of
    * {@code this}.
    * <p>
    * this = other * this*
    * </p>
    *
    * @param other the other frame quaternion to multiply this with. Not modified.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same
    *            reference frame as {@code this}.
    */
   public final void preMultiplyConjugateThis(FrameQuaternionReadOnly other)
   {
      checkReferenceFrameMatch(other);
      tuple.preMultiplyConjugateThis(other);
   }

   /**
    * Gets the read-only reference to the quaternion used in {@code this}.
    *
    * @return the quaternion of {@code this}.
    */
   public final QuaternionReadOnly getQuaternion()
   {
      return tuple;
   }
}
