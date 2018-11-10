package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.yawPitchRoll.interfaces.YawPitchRollBasics;
import us.ihmc.euclid.yawPitchRoll.interfaces.YawPitchRollReadOnly;

public interface FixedFrameYawPitchRollBasics extends FrameYawPitchRollReadOnly, FixedFrameOrientation3DBasics, YawPitchRollBasics
{
   /**
    * Sets this yaw-pitch-roll to the same value as the given {@code other}.
    * 
    * @param referenceFrame the coordinate system in which the given orientation is expressed.
    * @param yaw the new yaw angle.
    * @param pitch the new pitch angle.
    * @param roll the new roll angle.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same reference
    *            frame as {@code this}.
    */
   default void set(ReferenceFrame referenceFrame, double yaw, double pitch, double roll)
   {
      checkReferenceFrameMatch(referenceFrame);
      set(yaw, pitch, roll);
   }

   /**
    * Sets this yaw-pitch-roll to the same value as the given {@code other}.
    * 
    * @param referenceFrame the coordinate system in which the given {@code yawPitchRollReadOnly} is
    *           expressed.
    * @param yawPitchRollReadOnly the other yaw-pitch-roll. Not modified.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same reference
    *            frame as {@code this}.
    */
   default void set(ReferenceFrame referenceFrame, YawPitchRollReadOnly yawPitchRollReadOnly)
   {
      checkReferenceFrameMatch(referenceFrame);
      set(yawPitchRollReadOnly);
   }

   /**
    * Sets this yaw-pitch-roll to the same value as the given {@code other}.
    * 
    * @param other the other yaw-pitch-roll. Not modified.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same reference
    *            frame as {@code this}.
    */
   default void set(FrameYawPitchRollReadOnly other)
   {
      set(other.getReferenceFrame(), other);
   }

   /**
    * Sets this yaw-pitch-roll to {@code other} and calls {@link #negate()}.
    * 
    * @param other the other yaw-pitch-roll to copy the values from. Not modified.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same reference
    *            frame as {@code this}.
    */
   default void setAndNegate(FrameYawPitchRollReadOnly other)
   {
      checkReferenceFrameMatch(other);
      YawPitchRollBasics.super.setAndNegate(other);
   }
}
