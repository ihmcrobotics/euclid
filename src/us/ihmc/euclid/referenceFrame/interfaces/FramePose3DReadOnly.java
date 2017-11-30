package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FrameQuaternion;
import us.ihmc.euclid.referenceFrame.FrameTuple3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;

public interface FramePose3DReadOnly extends Pose3DReadOnly, ReferenceFrameHolder
{
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
   default boolean epsilonEquals(FramePose3DReadOnly other, double epsilon)
   {
      if (getReferenceFrame() != other.getReferenceFrame())
         return false;

      return Pose3DReadOnly.super.epsilonEquals(other, epsilon);
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
   default boolean equals(FramePose3DReadOnly other)
   {
      if (getReferenceFrame() != other.getReferenceFrame())
         return false;

      return Pose3DReadOnly.super.equals(other);
   }

   default void getPosition(FrameTuple3D frameTupleToPack)
   {
      frameTupleToPack.setIncludingFrame(getReferenceFrame(), getPosition());
   }
   
   default void getOrientation(FrameQuaternion frameQuaternionToPack)
   {
      frameQuaternionToPack.setIncludingFrame(getReferenceFrame(), getOrientation());
   }
   
   default void getRotationVector(FrameVector3D frameVectorToPack)
   {
      
   }
   
   default double getPositionDistance(FramePoint3DReadOnly point)
   {
      checkReferenceFrameMatch(point);
      return Pose3DReadOnly.super.getPositionDistance(point);
   }
   
   default double getOrientationDistance(FrameQuaternionReadOnly orientation)
   {
      checkReferenceFrameMatch(orientation);
      return Pose3DReadOnly.super.getOrientationDistance(orientation);
   }
}
