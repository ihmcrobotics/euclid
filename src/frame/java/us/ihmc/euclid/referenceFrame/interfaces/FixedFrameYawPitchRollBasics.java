package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.yawPitchRoll.interfaces.YawPitchRollBasics;
import us.ihmc.euclid.yawPitchRoll.interfaces.YawPitchRollReadOnly;

/**
 * Write and read interface for a yaw-pitch-roll object expressed in a constant reference frame,
 * i.e. this quaternion is always expressed in the same reference frame.
 * <p>
 * In addition to representing a {@link YawPitchRollBasics}, a {@link ReferenceFrame} is associated
 * to a {@code FixedFrameYawPitchRollBasics}. This allows, for instance, to enforce, at runtime,
 * that operations on yaw-pitch-rolls occur in the same coordinate system. Also, via the method
 * {@link FrameChangeable#changeFrame(ReferenceFrame)}, one can easily calculates the value of a
 * yaw-pitch-roll in different reference frames.
 * </p>
 * <p>
 * Because a {@code FixedFrameYawPitchRollBasics} extends {@code YawPitchRollBasics}, it is
 * compatible with methods only requiring {@code YawPitchRollBasics}. However, these methods do NOT
 * assert that the operation occur in the proper coordinate system. Use this feature carefully and
 * always prefer using methods requiring {@code FixedFrameYawPitchRollBasics}.
 * </p>
 * <p>
 * Equivalent representation of yaw-pitch-roll as 3-by-3 rotation matrix:
 *
 * <pre>
 *     / cos(yaw) -sin(yaw) 0 \   /  cos(pitch) 0 sin(pitch) \   / 1     0          0     \
 * R = | sin(yaw)  cos(yaw) 0 | * |      0      1     0      | * | 0 cos(roll) -sin(roll) |
 *     \    0         0     1 /   \ -sin(pitch) 0 cos(pitch) /   \ 0 sin(roll)  cos(roll) /
 * </pre>
 * </p>
 *
 * @author Sylvain Bertrand
 */
public interface FixedFrameYawPitchRollBasics extends FrameYawPitchRollReadOnly, FixedFrameOrientation3DBasics, YawPitchRollBasics
{
   /**
    * Sets this yaw-pitch-roll to the same value as the given {@code other}.
    * 
    * @param referenceFrame the coordinate system in which the given orientation is expressed.
    * @param yaw            the new yaw angle.
    * @param pitch          the new pitch angle.
    * @param roll           the new roll angle.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default void set(ReferenceFrame referenceFrame, double yaw, double pitch, double roll)
   {
      checkReferenceFrameMatch(referenceFrame);
      set(yaw, pitch, roll);
   }

   /**
    * Sets this yaw-pitch-roll to the same value as the given {@code other}.
    * 
    * @param referenceFrame       the coordinate system in which the given {@code yawPitchRollReadOnly}
    *                             is expressed.
    * @param yawPitchRollReadOnly the other yaw-pitch-roll. Not modified.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same reference
    *                                         frame as {@code this}.
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
    *                                         frame as {@code this}.
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
    *                                         frame as {@code this}.
    */
   default void setAndNegate(FrameYawPitchRollReadOnly other)
   {
      checkReferenceFrameMatch(other);
      YawPitchRollBasics.super.setAndNegate(other);
   }
}
