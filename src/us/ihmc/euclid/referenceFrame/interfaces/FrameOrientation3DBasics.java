package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;

public interface FrameOrientation3DBasics extends FixedFrameOrientation3DBasics, FrameChangeable
{
   /**
    * Sets this frame orientation to the same orientation described by the given
    * {@code orientation3DReadOnly} and sets the frame to the given {@code referenceFrame}.
    *
    * @param referenceFrame the new reference frame for this frame orientation.
    * @param orientation3DReadOnly the orientation used to set this orientation. Not modified.
    */
   default void setIncludingFrame(ReferenceFrame referenceFrame, Orientation3DReadOnly orientation3DReadOnly)
   {
      setReferenceFrame(referenceFrame);
      set(orientation3DReadOnly);
   }

   /**
    * Sets this frame tuple to {@code other}.
    *
    * @param other the other frame tuple to copy the values and reference frame from. Not modified.
    */
   default void setIncludingFrame(FrameOrientation3DReadOnly other)
   {
      setIncludingFrame(other.getReferenceFrame(), other);
   }

   /**
    * Sets this frame orientation to the same orientation described by the given rotation vector
    * {@code rotationVector} and sets the frame to the given {@code referenceFrame}.
    * <p>
    * WARNING: a rotation vector is different from a yaw-pitch-roll or Euler angles representation. A
    * rotation vector is equivalent to the axis of an axis-angle that is multiplied by the angle of the
    * same axis-angle.
    * </p>
    *
    * @param referenceFrame the new reference frame for this frame orientation.
    * @param rotationVector vector the rotation vector used to set this orientation. Not modified.
    * @deprecated Use {@link #setRotationVectorIncludingFrame(ReferenceFrame,Vector3DReadOnly)} instead
    */
   default void setIncludingFrame(ReferenceFrame referenceFrame, Vector3DReadOnly rotationVector)
   {
      setRotationVectorIncludingFrame(referenceFrame, rotationVector);
   }

   /**
    * Sets this frame orientation to the same orientation described by the given rotation vector
    * {@code rotationVector} and sets the frame to the given {@code referenceFrame}.
    * <p>
    * WARNING: a rotation vector is different from a yaw-pitch-roll or Euler angles representation. A
    * rotation vector is equivalent to the axis of an axis-angle that is multiplied by the angle of the
    * same axis-angle.
    * </p>
    *
    * @param referenceFrame the new reference frame for this frame orientation.
    * @param rotationVector vector the rotation vector used to set this orientation. Not modified.
    */
   default void setRotationVectorIncludingFrame(ReferenceFrame referenceFrame, Vector3DReadOnly rotationVector)
   {
      setReferenceFrame(referenceFrame);
      setRotationVector(rotationVector);
   }

   /**
    * Sets this frame orientation to the same orientation described by the given rotation vector
    * {@code rotationVector} and sets the frame to {@code rotationVector.getReferenceFrame()}.
    * <p>
    * WARNING: a rotation vector is different from a yaw-pitch-roll or Euler angles representation. A
    * rotation vector is equivalent to the axis of an axis-angle that is multiplied by the angle of the
    * same axis-angle.
    * </p>
    *
    * @param rotationVector vector the rotation vector used to set this orientation. Not modified.
    * @deprecated Use {@link #setRotationVectorIncludingFrame(FrameVector3DReadOnly)} instead
    */
   default void setIncludingFrame(FrameVector3DReadOnly rotationVector)
   {
      setRotationVectorIncludingFrame(rotationVector);
   }

   /**
    * Sets this frame orientation to the same orientation described by the given rotation vector
    * {@code rotationVector} and sets the frame to {@code rotationVector.getReferenceFrame()}.
    * <p>
    * WARNING: a rotation vector is different from a yaw-pitch-roll or Euler angles representation. A
    * rotation vector is equivalent to the axis of an axis-angle that is multiplied by the angle of the
    * same axis-angle.
    * </p>
    *
    * @param rotationVector vector the rotation vector used to set this orientation. Not modified.
    */
   default void setRotationVectorIncludingFrame(FrameVector3DReadOnly rotationVector)
   {
      setReferenceFrame(rotationVector.getReferenceFrame());
      setRotationVector((Vector3DReadOnly) rotationVector);
   }

   /**
    * Sets this orientation to represent the same orientation as the given yaw-pitch-roll
    * {@code yawPitchRoll} and sets the frame to the given {@code referenceFrame}.
    *
    * @param referenceFrame the reference frame in which the Euler angles are expressed.
    * @param yawPitchRoll the yaw-pitch-roll Euler angles to copy the orientation from. Not modified.
    * @deprecated Use {@link #setIncludingFrame(ReferenceFrame, Orientation3DReadOnly)} using
    *             {@link YawPitchRoll} for instance.
    */
   default void setYawPitchRollIncludingFrame(ReferenceFrame referenceFrame, double[] yawPitchRoll)
   {
      setReferenceFrame(referenceFrame);
      setYawPitchRoll(yawPitchRoll);
   }

   /**
    * Sets this orientation to represent the same orientation as the given yaw-pitch-roll {@code yaw},
    * {@code pitch}, and {@code roll} and sets the frame to the given {@code referenceFrame}.
    *
    * @param referenceFrame the new reference frame for this frame orientation.
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
    * Sets this orientation to represent the same orientation as the given Euler angles
    * {@code eulerAngles} and sets the frame to the given {@code referenceFrame}.
    * <p>
    * This is equivalent to
    * {@code this.setYawPitchRollIncludingFrame(referenceFrame, eulerAngles.getZ(), eulerAngles.getY(), eulerAngles.getX())}.
    * </p>
    *
    * @param referenceFrame the new reference frame for this frame orientation.
    * @param eulerAngles the Euler angles to copy the orientation from. Not modified.
    */
   default void setEulerIncludingFrame(ReferenceFrame referenceFrame, Vector3DReadOnly eulerAngles)
   {
      setReferenceFrame(referenceFrame);
      setEuler(eulerAngles);
   }

   /**
    * Sets this orientation to represent the same orientation as the given Euler angles {@code rotX},
    * {@code rotY}, and {@code rotZ} and sets the frame to the given {@code referenceFrame}.
    * <p>
    * This is equivalent to
    * {@code this.setYawPitchRollIncludingFrame(referenceFrame, rotZ, rotY, rotX)}.
    * </p>
    *
    * @param referenceFrame the new reference frame for this frame orientation.
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
