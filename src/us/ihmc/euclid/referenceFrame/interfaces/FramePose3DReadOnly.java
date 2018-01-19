package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;

public interface FramePose3DReadOnly extends Pose3DReadOnly, ReferenceFrameHolder
{
   /**
    * Packs the position part of this pose 3D into the given {@code positionToPack}.
    *
    * @param positionToPack tuple used to store the position coordinates. Modified.
    */
   default void getPosition(FrameTuple3DBasics positionToPack)
   {
      positionToPack.setIncludingFrame(getReferenceFrame(), getPosition());
   }

   /**
    * Packs the orientation part of this pose 3D into the given {@code orientationToPack}.
    *
    * @param orientationToPack used to store the orientation of this pose 3D. Modified.
    */
   default void getOrientation(FrameQuaternionBasics orientationToPack)
   {
      orientationToPack.setIncludingFrame(getReferenceFrame(), getOrientation());
   }

   /**
    * Computes and packs the orientation described by the orientation part of this pose as a
    * rotation vector.
    * <p>
    * WARNING: a rotation vector is different from a yaw-pitch-roll or Euler angles representation.
    * A rotation vector is equivalent to the axis of an axis-angle that is multiplied by the angle
    * of the same axis-angle.
    * </p>
    *
    * @param rotationVectorToPack the vector in which the rotation vector is stored. Modified.
    */
   default void getRotationVector(FrameVector3DBasics rotationVectorToPack)
   {
      rotationVectorToPack.setToZero(getReferenceFrame());
      Pose3DReadOnly.super.getRotationVector(rotationVectorToPack);
   }

   /**
    * Computes the distance between the position of this pose 3D and the given {@code point}.
    *
    * @param point the other point used to measure the distance. Not modified.
    * @return the distance between this pose and the given {@code point}.
    * @throws ReferenceFrameMismatchException if {@code point} is not expressed in the same
    *            reference frame as {@code this}.
    */
   default double getPositionDistance(FramePoint3DReadOnly point)
   {
      checkReferenceFrameMatch(point);
      return Pose3DReadOnly.super.getPositionDistance(point);
   }

   /**
    * Computes the distances between the position part of the two poses.
    *
    * @param other the other pose used to measure the distance. Not modified.
    * @return the distance between the position part of the two poses.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same
    *            reference frame as {@code this}.
    */
   default double getPositionDistance(FramePose3DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      return Pose3DReadOnly.super.getPositionDistance(other);
   }

   /**
    * Computes the smallest angle representing the difference between the orientation part of this
    * pose 3D and the give {@code orientation}.
    *
    * @param orientation the orientation used to compute the orientation distance. Not modified.
    * @return the angle difference between {@code this} and {@code orientation}, it is contained in
    *         [0, 2<i>pi</i>].
    * @throws ReferenceFrameMismatchException if {@code orientation} is not expressed in the same
    *            reference frame as {@code this}.
    */
   default double getOrientationDistance(FrameQuaternionReadOnly orientation)
   {
      checkReferenceFrameMatch(orientation);
      return Pose3DReadOnly.super.getOrientationDistance(orientation);
   }

   /**
    * Computes the absolute angle difference between this pose 3D and {@code other}.
    *
    * @param other the other pose 3D used to compute the orientation distance. Not modified.
    * @return the angle difference between {@code this.orientation} and {@code other.orientation},
    *         it is contained in [0, 2<i>pi</i>].
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same
    *            reference frame as {@code this}.
    */
   default double getOrientationDistance(FramePose3DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      return Pose3DReadOnly.super.getOrientationDistance(other);
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
   default boolean epsilonEquals(FramePose3DReadOnly other, double epsilon)
   {
      if (getReferenceFrame() != other.getReferenceFrame())
         return false;

      return Pose3DReadOnly.super.epsilonEquals(other, epsilon);
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
   default boolean geometricallyEquals(FramePose3DReadOnly other, double epsilon)
   {
      checkReferenceFrameMatch(other);
      return Pose3DReadOnly.super.geometricallyEquals(other, epsilon);
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
   default boolean equals(FramePose3DReadOnly other)
   {
      if (other == null || getReferenceFrame() != other.getReferenceFrame())
         return false;

      return Pose3DReadOnly.super.equals(other);
   }
}
