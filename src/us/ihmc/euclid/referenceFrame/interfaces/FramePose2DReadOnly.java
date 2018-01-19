package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.geometry.interfaces.Pose2DReadOnly;

public interface FramePose2DReadOnly extends Pose2DReadOnly, ReferenceFrameHolder
{
   default boolean geometricallyEquals(FramePose2DReadOnly other, double epsilon)
   {
      if (getReferenceFrame() != other.getReferenceFrame())
         return false;

      return Pose2DReadOnly.super.geometricallyEquals(other, epsilon);
   }

   /**
    * Tests if this pose is equal to the given {@code other} to an {@code epsilon}.
    * <p>
    * If the two poses have different frames, this method returns {@code false}.
    * </p>
    *
    * @param other the other pose to compare against this. Not modified.
    * @param epsilon the tolerance to use when comparing..
    * @return {@code true} if the two poses are equal and are expressed in the same reference
    *         frame, {@code false} otherwise.
    */
   default boolean epsilonEquals(FramePose2DReadOnly other, double epsilon)
   {
      if (getReferenceFrame() != other.getReferenceFrame())
         return false;

      return Pose2DReadOnly.super.epsilonEquals(other, epsilon);
   }

   /**
    * Tests if this pose is exactly equal to {@code other}.
    * <p>
    * If the two poses have different frames, this method returns {@code false}.
    * </p>
    *
    * @param other the other pose to compare against this. Not modified.
    * @return {@code true} if the two poses are exactly equal and are expressed in
    *         the same reference frame, {@code false} otherwise.
    */
   default boolean equals(FramePose2DReadOnly other)
   {
      if (getReferenceFrame() != other.getReferenceFrame())
         return false;

      return Pose2DReadOnly.super.equals(other);
   }
   
   default void getPosition(FrameTuple2DBasics frameTupleToPack)
   {
      frameTupleToPack.setIncludingFrame(getReferenceFrame(), getPosition());
   }
   
   default double getPositionDistance(FramePose2DReadOnly position)
   {
      checkReferenceFrameMatch(position);
      
      return Pose2DReadOnly.super.getPositionDistance(position);
   }
   
   default double getPositionDistance(FramePoint2DReadOnly position)
   {
      checkReferenceFrameMatch(position);
      
      return Pose2DReadOnly.super.getPositionDistance(position);
   }
   
   default double getOrientationDistance(FramePose2DReadOnly orientation)
   {
      checkReferenceFrameMatch(orientation);
      
      return Pose2DReadOnly.super.getOrientationDistance(orientation);
   }
}
