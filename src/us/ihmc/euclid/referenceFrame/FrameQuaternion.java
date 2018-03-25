package us.ihmc.euclid.referenceFrame;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.euclid.axisAngle.interfaces.AxisAngleReadOnly;
import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameQuaternionReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple4DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
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
public class FrameQuaternion implements FrameQuaternionBasics, GeometryObject<FrameQuaternion>
{
   /** The reference frame is which this quaternion is currently expressed. */
   private ReferenceFrame referenceFrame;
   /** The quaternion holding the current components of this frame quaternion. */
   private final Quaternion quaternion = new Quaternion();

   /**
    * Creates a new frame quaternion and initializes it to the neutral quaternion, i.e. representing a
    * 'zero' rotation, and its reference frame to {@link ReferenceFrame#getWorldFrame()}.
    */
   public FrameQuaternion()
   {
      setToZero(ReferenceFrame.getWorldFrame());
   }

   /**
    * Creates a new frame quaternion and initializes it to the neutral quaternion, i.e. representing a
    * 'zero' rotation, and its reference frame to the {@code referenceFrame}.
    *
    * @param referenceFrame the initial frame for this frame quaternion.
    */
   public FrameQuaternion(ReferenceFrame referenceFrame)
   {
      setToZero(referenceFrame);
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
      setIncludingFrame(referenceFrame, new Quaternion(x, y, z, s));
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
      setIncludingFrame(referenceFrame, quaternionArray);
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
      setIncludingFrame(referenceFrame, matrix);
   }

   /**
    * Creates a new quaternion and initializes it to {@code quaternionReadOnly} and to the given
    * reference frame.
    *
    * @param referenceFrame the initial frame for this frame quaternion.
    * @param quaternionReadOnly the quaternion to copy the components from. Not modified.
    */
   public FrameQuaternion(ReferenceFrame referenceFrame, QuaternionReadOnly quaternionReadOnly)
   {
      setIncludingFrame(referenceFrame, quaternionReadOnly);
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
      setIncludingFrame(referenceFrame, tuple4DReadOnly);
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
      setIncludingFrame(referenceFrame, rotationMatrix);
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
      setIncludingFrame(referenceFrame, axisAngle);
   }

   /**
    * Creates a new frame quaternion and initializes such that it represents the same orientation as
    * the given {@code rotationVector} and initializes its reference frame.
    * <p>
    * WARNING: a rotation vector is different from a yaw-pitch-roll or Euler angles representation. A
    * rotation vector is equivalent to the axis of an axis-angle that is multiplied by the angle of the
    * same axis-angle.
    * </p>
    *
    * @param referenceFrame the initial frame for this frame quaternion.
    * @param rotationVector the rotation vector to initialize this quaternion. Not modified.
    */
   public FrameQuaternion(ReferenceFrame referenceFrame, Vector3DReadOnly rotationVector)
   {
      setIncludingFrame(referenceFrame, rotationVector);
   }

   /**
    * Creates a new frame quaternion and initializes such that it represents the same orientation as
    * the given yaw-pitch-roll {@code yaw}, {@code pitch}, and {@code roll}.
    *
    * @param referenceFrame the initial frame for this frame quaternion.
    * @param yaw the angle to rotate about the z-axis.
    * @param pitch the angle to rotate about the y-axis.
    * @param roll the angle to rotate about the x-axis.
    */
   public FrameQuaternion(ReferenceFrame referenceFrame, double yaw, double pitch, double roll)
   {
      setYawPitchRollIncludingFrame(referenceFrame, yaw, pitch, roll);
   }

   /**
    * Creates a new frame quaternion and initializes it to {@code frameTuple4DReadOnly}.
    *
    * @param frameTuple4DReadOnly the frame tuple 4D to copy the components and reference frame from.
    *           Not modified.
    */
   public FrameQuaternion(FrameTuple4DReadOnly frameTuple4DReadOnly)
   {
      setIncludingFrame(frameTuple4DReadOnly);
   }

   /**
    * Creates a new frame quaternion and initializes it to {@code other}.
    *
    * @param other the frame quaternion to copy the components and reference frame from. Not modified.
    */
   public FrameQuaternion(FrameQuaternionReadOnly other)
   {
      setIncludingFrame(other);
   }

   /**
    * Sets this frame quaternion to {@code other}.
    *
    * @param other the other quaternion to copy the values from. Not modified.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same reference
    *            frame as {@code this}.
    */
   @Override
   public void set(FrameQuaternion other)
   {
      FrameQuaternionBasics.super.set(other);
   }

   /**
    * Sets the reference frame of this quaternion without updating or modifying its x, y, z, and s
    *
    * @param referenceFrame the new reference frame for this frame vector.
    */
   @Override
   public void setReferenceFrame(ReferenceFrame referenceFrame)
   {
      this.referenceFrame = referenceFrame;
   }

   /** {@inheritDoc} */
   @Override
   public final void setUnsafe(double qx, double qy, double qz, double qs)
   {
      quaternion.setUnsafe(qx, qy, qz, qs);
   }

   /**
    * Gets the reference frame in which this quaternion is currently expressed.
    */
   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return referenceFrame;
   }

   /**
    * Returns the value of the x-component of this quaternion.
    *
    * @return the x-component's value.
    */
   @Override
   public double getX()
   {
      return quaternion.getX();
   }

   /**
    * Returns the value of the y-component of this quaternion.
    *
    * @return the y-component's value.
    */
   @Override
   public double getY()
   {
      return quaternion.getY();
   }

   /**
    * Returns the value of the z-component of this quaternion.
    *
    * @return the z-component's value.
    */
   @Override
   public double getZ()
   {
      return quaternion.getZ();
   }

   /**
    * Returns the value of the s-component of this quaternion.
    *
    * @return the s-component's value.
    */
   @Override
   public double getS()
   {
      return quaternion.getS();
   }

   /**
    * Tests if the given {@code object}'s class is the same as this, in which case the method returns
    * {@link #equals(FrameTuple4DReadOnly)}, it returns {@code false} otherwise.
    * <p>
    * If the two vectors have different frames, this method returns {@code false}.
    * </p>
    *
    * @param object the object to compare against this. Not modified.
    * @return {@code true} if the two tuples are exactly equal component-wise and are expressed in the
    *         same reference frame, {@code false} otherwise.
    */
   @Override
   public boolean equals(Object object)
   {
      try
      {
         return equals((FrameTuple4DReadOnly) object);
      }
      catch (ClassCastException e)
      {
         return false;
      }
   }

   /**
    * Tests on a per component basis if this quaternion is equal to the given {@code other} to an
    * {@code epsilon}.
    * <p>
    * If the two quaternions have different frames, this method returns {@code false}.
    * </p>
    *
    * @param other the other quaternion to compare against this. Not modified.
    * @param epsilon the tolerance to use when comparing each component.
    * @return {@code true} if the two quaternions are equal and are expressed in the same reference
    *         frame, {@code false} otherwise.
    */
   @Override
   public boolean epsilonEquals(FrameQuaternion other, double epsilon)
   {
      return FrameQuaternionBasics.super.epsilonEquals(other, epsilon);
   }

   /**
    * Tests if {@code this} and {@code other} represent the same orientation to an {@code epsilon}.
    * <p>
    * Two quaternions are considered geometrically equal if the magnitude of their difference is less
    * than or equal to {@code epsilon}.
    * </p>
    * <p>
    * Note that two quaternions of opposite sign are considered equal, such that the two quaternions
    * {@code q1 = (x, y, z, s)} and {@code q2 = (-x, -y, -z, -s)} are considered geometrically equal.
    * </p>
    * <p>
    * Note that {@code this.geometricallyEquals(other, epsilon) == true} does not necessarily imply
    * {@code this.epsilonEquals(other, epsilon)} and vice versa.
    * </p>
    *
    * @param other the other quaternion to compare against this. Not modified.
    * @param epsilon the maximum angle of the difference quaternion can be for the two quaternions to
    *           be considered equal.
    * @return {@code true} if the two quaternions represent the same geometry, {@code false} otherwise.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same reference
    *            frame as {@code this}.
    */
   @Override
   public boolean geometricallyEquals(FrameQuaternion other, double epsilon)
   {
      return FrameQuaternionBasics.super.geometricallyEquals(other, epsilon);
   }

   /**
    * Provides a {@code String} representation of this frame quaternion as follows: (x, y, z,
    * s)-worldFrame.
    *
    * @return the {@code String} representing this frame quaternion.
    */
   @Override
   public String toString()
   {
      return EuclidCoreIOTools.getTuple4DString(this) + "-" + referenceFrame;
   }

   /**
    * Calculates and returns a hash code value from the value of each component of this frame
    * quaternion.
    *
    * @return the hash code value for this frame quaternion.
    */
   @Override
   public int hashCode()
   {
      return quaternion.hashCode();
   }
}
