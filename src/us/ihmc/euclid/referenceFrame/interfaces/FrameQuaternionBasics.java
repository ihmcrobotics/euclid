package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.axisAngle.interfaces.AxisAngleReadOnly;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionBasics;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;

/**
 * Write and read interface for a quaternion expressed in a changeable reference frame, i.e. the
 * reference frame in which this quaternion is expressed can be changed.
 * <p>
 * In addition to representing a {@link QuaternionBasics}, a {@link ReferenceFrame} is associated to
 * a {@code FrameQuaternionBasics}. This allows, for instance, to enforce, at runtime, that
 * operations on quaternions occur in the same coordinate system. Also, via the method
 * {@link #changeFrame(ReferenceFrame)}, one can easily calculates the value of a quaternion in
 * different reference frames.
 * </p>
 * <p>
 * Because a {@code FrameQuaternionBasics} extends {@code QuaternionBasics}, it is compatible with
 * methods only requiring {@code QuaternionBasics}. However, these methods do NOT assert that the
 * operation occur in the proper coordinate system. Use this feature carefully and always prefer
 * using methods requiring {@code FrameQuaternionBasics}.
 * </p>
 */
public interface FrameQuaternionBasics extends FixedFrameQuaternionBasics, FrameTuple4DBasics, FrameChangeable
{
   /**
    * Sets this frame quaternion to the same orientation described by the given {@code axisAngle}
    * and sets the frame to the given {@code referenceFrame}.
    *
    * @param referenceFrame the new reference frame for this frame quaternion.
    * @param axisAngle the axis-angle used to set this quaternion. Not modified.
    */
   default void setIncludingFrame(ReferenceFrame referenceFrame, AxisAngleReadOnly axisAngle)
   {
      setReferenceFrame(referenceFrame);
      set(axisAngle);
   }

   /**
    * Sets this frame quaternion to the same orientation described by the given
    * {@code rotationMatrix} and sets the frame to the given {@code referenceFrame}.
    *
    * @param referenceFrame the new reference frame for this frame quaternion.
    * @param rotationMatrix the rotation matrix used to set this quaternion. Not modified.
    */
   default void setIncludingFrame(ReferenceFrame referenceFrame, RotationMatrixReadOnly rotationMatrix)
   {
      setReferenceFrame(referenceFrame);
      set(rotationMatrix);
   }

   /**
    * Sets this frame tuple to {@code quaternionReadOnly} and sets its current frame to
    * {@code referenceFrame}.
    *
    * @param referenceFrame the new reference frame for this frame tuple.
    * @param quaternionReadOnly the quaternion to copy the values from. Not modified.
    */
   default void setIncludingFrame(ReferenceFrame referenceFrame, QuaternionReadOnly quaternionReadOnly)
   {
      setReferenceFrame(referenceFrame);
      set(quaternionReadOnly);
   }

   /**
    * Sets this frame quaternion to the same orientation described by the given rotation vector
    * {@code rotationVector} and sets the frame to the given {@code referenceFrame}.
    * <p>
    * WARNING: a rotation vector is different from a yaw-pitch-roll or Euler angles representation.
    * A rotation vector is equivalent to the axis of an axis-angle that is multiplied by the angle
    * of the same axis-angle.
    * </p>
    *
    * @param referenceFrame the new reference frame for this frame quaternion.
    * @param rotation vector the rotation vector used to set this quaternion. Not modified.
    */
   default void setIncludingFrame(ReferenceFrame referenceFrame, Vector3DReadOnly rotationVector)
   {
      setReferenceFrame(referenceFrame);
      set(rotationVector);
   }

   /**
    * Sets this frame quaternion to the same orientation described by the given rotation vector
    * {@code rotationVector} and sets the frame to {@code rotationVector.getReferenceFrame()}.
    * <p>
    * WARNING: a rotation vector is different from a yaw-pitch-roll or Euler angles representation.
    * A rotation vector is equivalent to the axis of an axis-angle that is multiplied by the angle
    * of the same axis-angle.
    * </p>
    *
    * @param rotation vector the rotation vector used to set this quaternion. Not modified.
    */
   default void setIncludingFrame(FrameVector3DReadOnly rotationVector)
   {
      setReferenceFrame(rotationVector.getReferenceFrame());
      set((Vector3DReadOnly) rotationVector);
   }

   /**
    * Sets this quaternion to represent the same orientation as the given yaw-pitch-roll
    * {@code yawPitchRoll} and sets the frame to the given {@code referenceFrame}.
    *
    * @param yawPitchRoll the yaw-pitch-roll Euler angles to copy the orientation from. Not
    *           modified.
    */
   default void setYawPitchRollIncludingFrame(ReferenceFrame referenceFrame, double[] yawPitchRoll)
   {
      setReferenceFrame(referenceFrame);
      setYawPitchRoll(yawPitchRoll);
   }

   /**
    * Sets this quaternion to represent the same orientation as the given yaw-pitch-roll
    * {@code yaw}, {@code pitch}, and {@code roll} and sets the frame to the given
    * {@code referenceFrame}.
    *
    * @param referenceFrame the new reference frame for this frame quaternion.
    * @param yaw the angle to rotate about the z-axis.
    * @param pitch the angle to rotate about the y-axis.
    * @param roll the angle to rotate about the x-axis.
    */
   default void setYawPitchRollIncludingFrame(ReferenceFrame referenceFrame, double yaw, double pitch, double roll)
   {
      setReferenceFrame(referenceFrame);
      setYawPitchRoll(yaw, pitch, roll);
   }

   /**
    * Sets this quaternion to represent the same orientation as the given Euler angles
    * {@code eulerAngles} and sets the frame to the given {@code referenceFrame}.
    * <p>
    * This is equivalent to
    * {@code this.setYawPitchRollIncludingFrame(referenceFrame, eulerAngles.getZ(), eulerAngles.getY(), eulerAngles.getX())}.
    * </p>
    *
    * @param referenceFrame the new reference frame for this frame quaternion.
    * @param eulerAngles the Euler angles to copy the orientation from. Not modified.
    */
   default void setEulerIncludingFrame(ReferenceFrame referenceFrame, Vector3DReadOnly eulerAngles)
   {
      setReferenceFrame(referenceFrame);
      setEuler(eulerAngles);
   }

   /**
    * Sets this quaternion to represent the same orientation as the given Euler angles {@code rotX},
    * {@code rotY}, and {@code rotZ} and sets the frame to the given {@code referenceFrame}.
    * <p>
    * This is equivalent to
    * {@code this.setYawPitchRollIncludingFrame(referenceFrame, rotZ, rotY, rotX)}.
    * </p>
    *
    * @param referenceFrame the new reference frame for this frame quaternion.
    * @param rotX the angle to rotate about the x-axis.
    * @param rotY the angle to rotate about the y-axis.
    * @param rotZ the angle to rotate about the z-axis.
    */
   default void setEulerIncludingFrame(ReferenceFrame referenceFrame, double rotX, double rotY, double rotZ)
   {
      setReferenceFrame(referenceFrame);
      setEuler(rotX, rotY, rotZ);
   }
}
