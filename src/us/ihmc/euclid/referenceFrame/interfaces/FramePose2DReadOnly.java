package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.geometry.interfaces.Pose2DReadOnly;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;

public interface FramePose2DReadOnly extends Pose2DReadOnly, ReferenceFrameHolder
{
   @Override
   FramePoint2DReadOnly getPosition();

   @Override
   FrameOrientation2DReadOnly getOrientation();

   /**
    * Packs the position part of this pose 2D into the given {@code positionToPack}.
    *
    * @param positionToPack tuple used to store the position coordinates. Modified.
    * @throws ReferenceFrameMismatchException if {@code positionToPack} is not expressed in the same
    *            reference frame as {@code this}.
    */
   default void getPosition(FixedFrameTuple2DBasics positionToPack)
   {
      positionToPack.set(getPosition());
   }

   /**
    * Packs the position part of this pose 2D into the given {@code positionToPack}.
    *
    * @param positionToPack tuple used to store the position coordinates. Modified.
    */
   default void getPosition(FrameTuple2DBasics positionToPack)
   {
      positionToPack.setIncludingFrame(getPosition());
   }

   /**
    * Packs the orientation part of this pose 2D into the given {@code orientationToPack}.
    *
    * @param orientationToPack used to store the orientation of this pose 2D. Modified.
    * @throws ReferenceFrameMismatchException if {@code orientationToPack} is not expressed in the
    *            same reference frame as {@code this}.
    */
   default void getOrientation(FixedFrameOrientation2DBasics orientationToPack)
   {
      orientationToPack.set(getOrientation());
   }

   /**
    * Packs the orientation part of this pose 2D into the given {@code orientationToPack}.
    *
    * @param orientationToPack used to store the orientation of this pose 2D. Modified.
    */
   default void getOrientation(FrameOrientation2DBasics orientationToPack)
   {
      orientationToPack.setIncludingFrame(getOrientation());
   }

   /**
    * Computes the distances between the position part of the two poses.
    *
    * @param other the other pose used to measure the distance. Not modified.
    * @return the distance between the position part of the two poses.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same
    *            reference frame as {@code this}.
    */
   default double getPositionDistance(FramePose2DReadOnly other)
   {
      return getPositionDistance(other.getPosition());
   }

   /**
    * Computes the distance between the position of this pose 2D and the given {@code point}.
    *
    * @param point the other point used to measure the distance. Not modified.
    * @return the distance between this pose and the given {@code point}.
    * @throws ReferenceFrameMismatchException if {@code point} is not expressed in the same
    *            reference frame as {@code this}.
    */
   default double getPositionDistance(FramePoint2DReadOnly point)
   {
      return getPosition().distance(point);
   }

   /**
    * Computes the absolute angle difference between the orientation part of this pose 2D and the
    * give {@code orientation}.
    *
    * @param orientation the orientation used to compute the orientation distance. Not modified.
    * @return the absolute angle difference between {@code this} and {@code orientation}.
    * @throws ReferenceFrameMismatchException if {@code orientation} is not expressed in the same
    *            reference frame as {@code this}.
    */
   default double getOrientationDistance(FrameOrientation2DReadOnly orientation)
   {
      return getOrientation().distance(orientation);
   }

   /**
    * Computes the absolute angle difference between this pose 2D and {@code other}.
    *
    * @param other the other pose 2D used to compute the orientation distance. Not modified.
    * @return the absolute angle difference between {@code this.orientation} and
    *         {@code other.orientation}.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same
    *            reference frame as {@code this}.
    */
   default double getOrientationDistance(FramePose2DReadOnly other)
   {
      return getOrientationDistance(other.getOrientation());
   }

   /**
    * Tests if this pose is equal to the given {@code other} to an {@code epsilon}.
    * <p>
    * If the two poses have different frames, this method returns {@code false}.
    * </p>
    *
    * @param other the other pose to compare against this. Not modified.
    * @param epsilon the tolerance to use when comparing..
    * @return {@code true} if the two poses are equal and are expressed in the same reference frame,
    *         {@code false} otherwise.
    */
   default boolean epsilonEquals(FramePose2DReadOnly other, double epsilon)
   {
      if (getReferenceFrame() != other.getReferenceFrame())
         return false;

      return Pose2DReadOnly.super.epsilonEquals(other, epsilon);
   }

   /**
    * Compares {@code this} to {@code other} to determine if the two poses are geometrically
    * similar.
    * <p>
    * Two poses are geometrically equal if both their position and orientation are geometrically
    * equal.
    * </p>
    *
    * @param other the pose to compare to. Not modified.
    * @param epsilon the tolerance of the comparison.
    * @return {@code true} if the two poses represent the same geometry, {@code false} otherwise.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same
    *            reference frame as {@code this}.
    */
   default boolean geometricallyEquals(FramePose2DReadOnly other, double epsilon)
   {
      checkReferenceFrameMatch(other);
      return Pose2DReadOnly.super.geometricallyEquals(other, epsilon);
   }

   /**
    * Tests if this pose is exactly equal to {@code other}.
    * <p>
    * If the two poses have different frames, this method returns {@code false}.
    * </p>
    *
    * @param other the other pose to compare against this. Not modified.
    * @return {@code true} if the two poses are exactly equal and are expressed in the same
    *         reference frame, {@code false} otherwise.
    */
   default boolean equals(FramePose2DReadOnly other)
   {
      if (other == null || getReferenceFrame() != other.getReferenceFrame())
         return false;

      return Pose2DReadOnly.super.equals(other);
   }
}
