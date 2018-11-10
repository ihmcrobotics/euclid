package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.yawPitchRoll.interfaces.YawPitchRollReadOnly;

public interface FrameYawPitchRollBasics extends FixedFrameYawPitchRollBasics, FrameOrientation3DBasics
{
   /**
    * Sets all the components of this frame yaw-pitch-roll to zero and sets the current reference frame
    * to {@code referenceFrame}.
    *
    * @param referenceFrame the new reference frame to be associated with this yaw-pitch-roll.
    */
   default void setToZero(ReferenceFrame referenceFrame)
   {
      setReferenceFrame(referenceFrame);
      setToZero();
   }

   /**
    * Sets all the components of this yaw-pitch-roll to {@link Double#NaN} and sets the current
    * reference frame to {@code referenceFrame}.
    *
    * @param referenceFrame the new reference frame to be associated with this yaw-pitch-roll.
    */
   default void setToNaN(ReferenceFrame referenceFrame)
   {
      setReferenceFrame(referenceFrame);
      setToNaN();
   }

   /**
    * Sets this yaw-pitch-roll to the same value as the given {@code other}.
    * 
    * @param referenceFrame the new reference frame for this frame yaw-pitch-roll.
    * @param yaw the new yaw angle.
    * @param pitch the new pitch angle.
    * @param roll the new roll angle.
    */
   default void setIncludingFrame(ReferenceFrame referenceFrame, double yaw, double pitch, double roll)
   {
      setReferenceFrame(referenceFrame);
      set(yaw, pitch, roll);
   }

   /**
    * Sets this yaw-pitch-roll to the same value as the given {@code other}.
    * 
    * @param referenceFrame the new reference frame for this frame yaw-pitch-roll.
    * @param yawPitchRollReadOnly the other yaw-pitch-roll. Not modified.
    */
   default void setIncludingFrame(ReferenceFrame referenceFrame, YawPitchRollReadOnly yawPitchRollReadOnly)
   {
      setReferenceFrame(referenceFrame);
      set(yawPitchRollReadOnly);
   }

   /**
    * Sets this yaw-pitch-roll to the same value as the given {@code other} and updates the reference
    * frame.
    * 
    * @param other the other yaw-pitch-roll. Not modified.
    */
   default void setIncludingFrame(FrameYawPitchRollReadOnly other)
   {
      setIncludingFrame(other.getReferenceFrame(), other);
   }

   /**
    * Copies the values in the given array into this yaw-pitch-roll as follows:
    * <ul>
    * <li>{@code this.setYaw(yawPitchRollArray[0]);}
    * <li>{@code this.setPitch(yawPitchRollArray[1]);}
    * <li>{@code this.setRoll(yawPitchRollArray[2]);}
    * </ul>
    *
    * @param referenceFrame the new reference frame for this yaw-pitch-roll.
    * @param yawPitchRollArray the array containing the new values for this yaw-pitch-roll. Not
    *           modified.
    */
   default void setIncludingFrame(ReferenceFrame referenceFrame, double[] yawPitchRollArray)
   {
      setReferenceFrame(referenceFrame);
      set(yawPitchRollArray);
   }

   /**
    * Copies the values in the given array into this yaw-pitch-roll as follows:
    * <ul>
    * <li>{@code this.setYaw(yawPitchRollArray[startIndex + 0]);}
    * <li>{@code this.setPitch(yawPitchRollArray[startIndex + 1]);}
    * <li>{@code this.setRoll(yawPitchRollArray[startIndex + 2]);}
    * </ul>
    *
    * @param referenceFrame the new reference frame for this yaw-pitch-roll.
    * @param startIndex the first index to start reading from in the array.
    * @param yawPitchRollArray the array containing the new values for this yaw-pitch-roll. Not
    *           modified.
    */
   default void setIncludingFrame(ReferenceFrame referenceFrame, int startIndex, double[] yawPitchRollArray)
   {
      setReferenceFrame(referenceFrame);
      set(startIndex, yawPitchRollArray);
   }
}
