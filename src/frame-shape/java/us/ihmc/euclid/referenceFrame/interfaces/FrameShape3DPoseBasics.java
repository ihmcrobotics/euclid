package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;

/**
 * Read and write interface for representing the pose of a shape 3D expressed in a changeable
 * reference frame, i.e. the reference frame in which this line is expressed can be changed.
 * <p>
 * While the main use-case of a {@code FixedFrameShape3DPoseBasics} is to describe the pose of a
 * shape 3D, it is also used to represent the transform from the shape local coordinate system to
 * the reference frame coordinates, such that it can be used to transform geometry back and forth
 * between the two coordinate systems.
 * </p>
 *
 * @author Sylvain Bertrand
 */
public interface FrameShape3DPoseBasics extends FixedFrameShape3DPoseBasics, FrameChangeable
{
   /**
    * Sets all the components of this frame pose to zero and sets the current reference frame to
    * {@code referenceFrame}.
    *
    * @param referenceFrame the new reference frame to be associated with this pose.
    */
   default void setToZero(ReferenceFrame referenceFrame)
   {
      setReferenceFrame(referenceFrame);
      setToZero();
   }

   /**
    * Sets all the components of this pose to {@link Double#NaN} and sets the current reference frame
    * to {@code referenceFrame}.
    *
    * @param referenceFrame the new reference frame to be associated with this pose.
    */
   default void setToNaN(ReferenceFrame referenceFrame)
   {
      setReferenceFrame(referenceFrame);
      setToNaN();
   }

   /**
    * Sets this shape pose 3D to the {@code other} pose 3D and updates its reference frame.
    *
    * @param referenceFrame the reference frame in which the argument is expressed.
    * @param pose           the pose 3D to set this to. Not modified.
    */
   default void setIncludingFrame(ReferenceFrame referenceFrame, Pose3DReadOnly pose)
   {
      setReferenceFrame(referenceFrame);
      set(pose);
   }

   /**
    * Sets this shape pose 3D to the {@code other} pose 3D and updates its reference frame.
    *
    * @param pose the pose 3D to set this to. Not modified.
    */
   default void setIncludingFrame(FramePose3DReadOnly pose)
   {
      setIncludingFrame(pose.getReferenceFrame(), pose);
   }

   /**
    * Sets this pose to the given rigid-body transform and update its reference frame.
    *
    * @param referenceFrame the reference frame in which the argument is expressed.
    * @param pose           the rigid-body transform to copy the values from. Not modified.
    */
   default void setIncludingFrame(ReferenceFrame referenceFrame, RigidBodyTransformReadOnly pose)
   {
      setReferenceFrame(referenceFrame);
      set(pose);
   }

   /**
    * Sets this pose to the given rigid-body transform and update its reference frame.
    *
    * @param pose the rigid-body transform to copy the values from. Not modified.
    */
   default void setIncludingFrame(FrameShape3DPoseReadOnly pose)
   {
      setIncludingFrame(pose.getReferenceFrame(), pose);
   }
}
