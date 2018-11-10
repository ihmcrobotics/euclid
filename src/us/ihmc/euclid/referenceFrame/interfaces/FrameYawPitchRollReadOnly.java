package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.yawPitchRoll.interfaces.YawPitchRollReadOnly;

public interface FrameYawPitchRollReadOnly extends FrameOrientation3DReadOnly, YawPitchRollReadOnly
{
   /**
    * Computes and returns the distance from this yaw-pitch-roll to {@code other}.
    *
    * @param other the other yaw-pitch-roll to measure the distance. Not modified.
    * @return the angle representing the distance between the two orientations. It is contained in [0,
    *         2<i>pi</i>]
    * @throws ReferenceFrameMismatchException if reference frame of {@code this} and {@code other} do
    *            not match.
    */
   default double distance(FrameYawPitchRollReadOnly other)
   {
      checkReferenceFrameMatch(other);
      return YawPitchRollReadOnly.super.distance(other);
   }

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
    * @param other the other tuple to compare against this. Not modified.
    * @param epsilon the tolerance to use when comparing each component.
    * @return {@code true} if the two yaw-pitch-rolls are equal and are expressed in the same reference
    *         frame, {@code false} otherwise.
    */
   default boolean epsilonEquals(FrameYawPitchRollReadOnly other, double epsilon)
   {
      if (getReferenceFrame() != other.getReferenceFrame())
         return false;

      return YawPitchRollReadOnly.super.epsilonEquals(other, epsilon);
   }

   /**
    * Tests if {@code this} and {@code other} represent the same orientation to an {@code epsilon}.
    * <p>
    * Two yaw-pitch-roll are considered geometrically equal if the magnitude of their difference is
    * less than or equal to {@code epsilon}.
    * </p>
    * <p>
    * Note that {@code this.geometricallyEquals(other, epsilon) == true} does not necessarily imply
    * {@code this.epsilonEquals(other, epsilon)} and vice versa.
    * </p>
    *
    * @param other the other yaw-pitch-roll to compare against this. Not modified.
    * @param epsilon the maximum angle for the two quaternions to be considered equal.
    * @return {@code true} if the two yaw-pitch-roll represent the same geometry, {@code false}
    *         otherwise.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same reference
    *            frame as {@code this}.
    */
   default boolean geometricallyEquals(FrameYawPitchRollReadOnly other, double epsilon)
   {
      checkReferenceFrameMatch(other);
      return YawPitchRollReadOnly.super.geometricallyEquals(other, epsilon);
   }
}
