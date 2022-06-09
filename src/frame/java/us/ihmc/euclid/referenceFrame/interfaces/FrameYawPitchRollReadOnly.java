package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameIOTools;
import us.ihmc.euclid.yawPitchRoll.interfaces.YawPitchRollReadOnly;

/**
 * Read-only interface for a yaw-pitch-roll object expressed in a given reference frame.
 * <p>
 * In addition to representing a {@link YawPitchRollReadOnly}, a {@link ReferenceFrame} is
 * associated to a {@code FrameYawPitchRollReadOnly}. This allows, for instance, to enforce, at
 * runtime, that operations on yaw-pitch-rolls occur in the same coordinate system. Also, via the
 * method {@link FrameChangeable#changeFrame(ReferenceFrame)}, one can easily calculates the value
 * of a yaw-pitch-roll in different reference frames.
 * </p>
 * <p>
 * Because a {@code FrameYawPitchRollReadOnly} extends {@code YawPitchRollReadOnly}, it is
 * compatible with methods only requiring {@code YawPitchRollReadOnly}. However, these methods do
 * NOT assert that the operation occur in the proper coordinate system. Use this feature carefully
 * and always prefer using methods requiring {@code FrameYawPitchRollReadOnly}.
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
public interface FrameYawPitchRollReadOnly extends FrameOrientation3DReadOnly, YawPitchRollReadOnly
{
   /**
    * Tests on a per component basis, if this yaw-pitch-roll is exactly equal to {@code other}. A
    * failing test does not necessarily mean that the two yaw-pitch-rolls represent two different
    * orientations.
    * <p>
    * If the two yaw-pitch-rolls have different frames, this method returns {@code false}.
    * </p>
    *
    * @param other the other yaw-pitch-roll to compare against this. Not modified.
    * @return {@code true} if the two yaw-pitch-rolls are exactly equal component-wise and are
    *         expressed in the same reference frame, {@code false} otherwise.
    */
   default boolean equals(FrameYawPitchRollReadOnly other)
   {
      if (other == this)
         return true;
      if (other == null || getReferenceFrame() != other.getReferenceFrame())
         return false;

      return YawPitchRollReadOnly.super.equals(other);
   }

   /**
    * Tests on a per component basis, if this yaw-pitch-roll is equal to {@code other} to an
    * {@code epsilon}. A failing test does not necessarily mean that the two yaw-pitch-rolls represent
    * two different orientations.
    * <p>
    * If the two yaw-pitch-rolls have different frames, this method returns {@code false}.
    * </p>
    *
    * @param object  the other object to compare against this. Not modified.
    * @param epsilon the tolerance to use when comparing each component.
    * @return {@code true} if the two yaw-pitch-rolls are equal and are expressed in the same reference
    *         frame, {@code false} otherwise.
    */
   @Override
   default boolean epsilonEquals(Object object, double epsilon)
   {
      if (!(object instanceof FrameYawPitchRollReadOnly))
         return false;
      FrameYawPitchRollReadOnly other = (FrameYawPitchRollReadOnly) object;
      if (getReferenceFrame() != other.getReferenceFrame())
         return false;

      return YawPitchRollReadOnly.super.epsilonEquals(other, epsilon);
   }

   /**
    * Tests if {@code this} and {@code other} represent the same orientation to an {@code epsilon}.
    * <p>
    * Note that {@code this.geometricallyEquals(other, epsilon) == true} does not necessarily imply
    * that the 2 orientations are of the same type nor that they are equal on a per-component bases.
    * Returns false if reference frames don't match.
    * </p>
    *
    * @param object  the object to compare against this. Not modified.
    * @param epsilon the maximum angle for the two orientations to be considered equal.
    * @return {@code true} if the two orientations represent the same geometry, {@code false}
    *         otherwise.
    */
   @Override
   default boolean geometricallyEquals(Object object, double epsilon)
   {
      if (!(object instanceof FrameYawPitchRollReadOnly))
         return false;
      FrameYawPitchRollReadOnly other = (FrameYawPitchRollReadOnly) object;
      if (getReferenceFrame() != other.getReferenceFrame())
         return false;
      return YawPitchRollReadOnly.super.geometricallyEquals(other, epsilon);
   }

   @Override
   default String toString(String format)
   {
      return EuclidFrameIOTools.getFrameYawPitchRollString(format, this);
   }
}
