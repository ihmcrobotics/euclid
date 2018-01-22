package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.axisAngle.interfaces.AxisAngleReadOnly;
import us.ihmc.euclid.geometry.interfaces.Pose2DReadOnly;
import us.ihmc.euclid.geometry.interfaces.Pose3DBasics;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;

/**
 * Write and read interface for a 3D pose expressed in a changeable reference frame, i.e. the
 * reference frame in which this pose is expressed can be changed.
 * <p>
 * In addition to representing a {@link Pose3DBasics}, a {@link ReferenceFrame} is associated to a
 * {@code FramePose3DBasics}. This allows, for instance, to enforce, at runtime, that operations on
 * poses occur in the same coordinate system.
 * </p>
 * <p>
 * Because a {@code FramePose3DBasics} extends {@code Pose3DBasics}, it is compatible with methods
 * only requiring {@code Pose3DBasics}. However, these methods do NOT assert that the operation
 * occur in the proper coordinate system. Use this feature carefully and always prefer using methods
 * requiring {@code FramePose3DBasics}.
 * </p>
 */
public interface FramePose3DBasics extends FixedFramePose3DBasics, FrameChangeable
{
   /**
    * Sets the reference frame of this pose 3D without updating or modifying its position or
    * orientation.
    * 
    * @param referenceFrame the new reference frame for this frame pose 3D.
    */
   void setReferenceFrame(ReferenceFrame referenceFrame);

   /**
    * Sets the position and orientation parts of this pose 3D to zero and sets the current reference
    * frame to {@code referenceFrame}.
    * 
    * @param referenceFrame the new reference frame to be associated with this pose 3D.
    */
   default void setToZero(ReferenceFrame referenceFrame)
   {
      setReferenceFrame(referenceFrame);
      setToZero();
   }

   /**
    * Sets the position and position parts of this pose 3D to {@link Double#NaN} and sets the
    * current reference frame to {@code referenceFrame}.
    * 
    * @param referenceFrame the new reference frame to be associated with this pose 3D.
    */
   default void setToNaN(ReferenceFrame referenceFrame)
   {
      setReferenceFrame(referenceFrame);
      setToNaN();
   }

   /**
    * Sets this pose 3D to represent the same pose as the given {@code pose2DReadOnly} expressed in
    * the given {@code referenceFrame}.
    * 
    * @param referenceFrame the new reference frame.
    * @param pose2DReadOnly the pose 2D used to set this pose. Not modified.
    */
   default void setIncludingFrame(ReferenceFrame referenceFrame, Pose2DReadOnly pose2DReadOnly)
   {
      setReferenceFrame(referenceFrame);
      set(pose2DReadOnly);
   }

   /**
    * Sets this pose 3D from the given pose 3D and sets the reference frame.
    * 
    * @param referenceFrame the new reference frame.
    * @param pose3DReadOnly the pose 3D used to sets the position and orientation. Not modified.
    */
   default void setIncludingFrame(ReferenceFrame referenceFrame, Pose3DReadOnly pose3DReadOnly)
   {
      setReferenceFrame(referenceFrame);
      set(pose3DReadOnly);
   }

   /**
    * Sets this pose 3D from the given reference frame and transform.
    * 
    * @param referenceFrame the new reference frame.
    * @param rigidBodyTransform the transform used to set the pose. Not modified.
    */
   default void setIncludingFrame(ReferenceFrame referenceFrame, RigidBodyTransform rigidBodyTransform)
   {
      setReferenceFrame(referenceFrame);
      set(rigidBodyTransform);
   }

   /**
    * Sets reference frame, position, and orientation.
    *
    * @param referenceFrame the new reference frame.
    * @param position the tuple with the new position coordinates. Not modified.
    * @param orientation the quaternion with the new orientation. Not modified.
    */
   default void setIncludingFrame(ReferenceFrame referenceFrame, Tuple3DReadOnly position, QuaternionReadOnly orientation)
   {
      setReferenceFrame(referenceFrame);
      set(position, orientation);
   }

   /**
    * Sets reference frame, position, and orientation.
    *
    * @param referenceFrame the new reference frame.
    * @param position the tuple with the new position coordinates. Not modified.
    * @param orientation the axis-angle with the new orientation. Not modified.
    */
   default void setIncludingFrame(ReferenceFrame referenceFrame, Tuple3DReadOnly position, AxisAngleReadOnly orientation)
   {
      setReferenceFrame(referenceFrame);
      set(position, orientation);
   }

   /**
    * Sets reference frame, position, and orientation.
    *
    * @param position the tuple with the new position coordinates. Not modified.
    * @param orientation the quaternion with the new orientation. Not modified.
    * @throws ReferenceFrameMismatchException if {@code position} and {@code orientation} are not
    *            expressed in the same reference frame.
    */
   default void setIncludingFrame(FrameTuple3DReadOnly position, FrameQuaternionReadOnly orientation)
   {
      position.checkReferenceFrameMatch(orientation);
      setReferenceFrame(position.getReferenceFrame());
      set((Tuple3DReadOnly) position, (QuaternionReadOnly) orientation);
   }

   /**
    * Sets this frame pose 3D to represent the same pose as the given {@code framePose2DReadOnly}.
    * 
    * @param framePose2DReadOnly the frame pose 2D used to set this frame pose 3D. Not modified.
    */
   default void setIncludingFrame(FramePose2DReadOnly framePose2DReadOnly)
   {
      setIncludingFrame(framePose2DReadOnly.getReferenceFrame(), framePose2DReadOnly);
   }

   /**
    * Sets this frame pose 3D to the {@code other} frame pose 3D.
    *
    * @param other the other frame pose 3D. Not modified.
    */
   default void setIncludingFrame(FramePose3DReadOnly other)
   {
      setIncludingFrame(other.getReferenceFrame(), other);
   }
}
