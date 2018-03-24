package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.exceptions.NotAMatrix2DException;
import us.ihmc.euclid.geometry.interfaces.Orientation2DReadOnly;
import us.ihmc.euclid.geometry.interfaces.Pose2DBasics;
import us.ihmc.euclid.geometry.interfaces.Pose2DReadOnly;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;

/**
 * Write and read interface for a 2D pose expressed in a changeable reference frame, i.e. the
 * reference frame in which this pose is expressed can be changed.
 * <p>
 * In addition to representing a {@link Pose2DBasics}, a {@link ReferenceFrame} is associated to a
 * {@code FramePose2DBasics}. This allows, for instance, to enforce, at runtime, that operations on
 * poses occur in the same coordinate system.
 * </p>
 * <p>
 * Because a {@code FramePose2DBasics} extends {@code Pose2DBasics}, it is compatible with methods
 * only requiring {@code Pose2DBasics}. However, these methods do NOT assert that the operation
 * occur in the proper coordinate system. Use this feature carefully and always prefer using methods
 * requiring {@code FramePose2DBasics}.
 * </p>
 */
public interface FramePose2DBasics extends FixedFramePose2DBasics, FrameChangeable
{
   /**
    * Sets the reference frame of this pose 2D without updating or modifying its position or
    * orientation.
    *
    * @param referenceFrame the new reference frame for this frame pose 2D.
    */
   @Override
   void setReferenceFrame(ReferenceFrame referenceFrame);

   /**
    * Sets the position and orientation parts of this pose 2D to zero and sets the current reference
    * frame to {@code referenceFrame}.
    *
    * @param referenceFrame the new reference frame to be associated with this pose 2D.
    */
   default void setToZero(ReferenceFrame referenceFrame)
   {
      setReferenceFrame(referenceFrame);
      setToZero();
   }

   /**
    * Sets the position and position parts of this pose 2D to {@link Double#NaN} and sets the current
    * reference frame to {@code referenceFrame}.
    *
    * @param referenceFrame the new reference frame to be associated with this pose 2D.
    */
   default void setToNaN(ReferenceFrame referenceFrame)
   {
      setReferenceFrame(referenceFrame);
      setToNaN();
   }

   /**
    * Sets the position, orientation, and reference frame.
    *
    * @param referenceFrame the new reference frame.
    * @param x the x-coordinate for the position.
    * @param y the y-coordinate for the position.
    * @param yaw the angle for the orientation.
    */
   default void setIncludingFrame(ReferenceFrame referenceFrame, double x, double y, double yaw)
   {
      setReferenceFrame(referenceFrame);
      set(x, y, yaw);
   }

   /**
    * Sets this pose 2D from the given pose 3D and sets the reference frame.
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
    * Sets this pose 2D from the given pose 2D and sets the reference frame.
    *
    * @param referenceFrame the new reference frame.
    * @param pose2DReadOnly the pose 2D used to sets the position and orientation. Not modified.
    */
   default void setIncludingFrame(ReferenceFrame referenceFrame, Pose2DReadOnly pose2DReadOnly)
   {
      setReferenceFrame(referenceFrame);
      set(pose2DReadOnly);
   }

   /**
    * Sets the position, orientation, and reference frame.
    *
    * @param referenceFrame the new reference frame.
    * @param position the new position. Not modified.
    * @param orientation the new orientation. Not modified.
    */
   default void setIncludingFrame(ReferenceFrame referenceFrame, Tuple2DReadOnly position, Orientation2DReadOnly orientation)
   {
      setReferenceFrame(referenceFrame);
      set(position, orientation);
   }

   /**
    * Sets this pose 2D to match the given rigid-body transform and sets the reference frame.
    * <p>
    * The given transform has to represent a 2D transformation.
    * </p>
    *
    * @param referenceFrame the new reference frame.
    * @param rigidBodyTransform the transform used to update the position and orientation. Not
    *           modified.
    * @throws NotAMatrix2DException if the rotation part of the transform does not represent a 2D
    *            transformation.
    */
   default void setIncludingFrame(ReferenceFrame referenceFrame, RigidBodyTransform rigidBodyTransform)
   {
      setIncludingFrame(referenceFrame, rigidBodyTransform, true);
   }

   /**
    * Sets this pose 2D to match the given rigid-body transform and sets the reference frame.
    *
    * @param referenceFrame the new reference frame.
    * @param rigidBodyTransform the transform used to update the position and orientation. Not
    *           modified.
    * @param checkIsTransform2D indicates whether or not the method should check that the rotation part
    *           of the given transform represents a 2D rotation in the XY-plane.
    * @throws NotAMatrix2DException if {@code checkIsTransform2D} is {@code true} and if the rotation
    *            part of the transform does not represent a 2D transformation.
    */
   default void setIncludingFrame(ReferenceFrame referenceFrame, RigidBodyTransform rigidBodyTransform, boolean checkIsTransform2D)
   {
      setReferenceFrame(referenceFrame);
      set(rigidBodyTransform, checkIsTransform2D);
   }

   /**
    * Sets this pose 2D from the given {@code position}, {@code yaw} angle and sets the reference
    * frame.
    *
    * @param referenceFrame the new reference frame.
    * @param position the tuple used to initialize this pose's position. Not modified.
    * @param yaw the angle used to initialize the pose's orientation.
    */
   default void setIncludingFrame(ReferenceFrame referenceFrame, Tuple2DReadOnly position, double yaw)
   {
      setReferenceFrame(referenceFrame);
      set(position, yaw);
   }

   /**
    * Sets position, orientation, and reference frame.
    *
    * @param position the tuple with the new position coordinates. Not modified.
    * @param orientation the orientation with the new angle value for this. Not modified.
    * @throws ReferenceFrameMismatchException if {@code position} and {@code orientation} are not
    *            expressed in the same reference frame.
    */
   default void setIncludingFrame(FrameTuple2DReadOnly position, FrameOrientation2DReadOnly orientation)
   {
      position.checkReferenceFrameMatch(orientation);
      setIncludingFrame(position.getReferenceFrame(), position, orientation);
   }

   /**
    * Sets this frame pose 2D to the {@code other} frame pose 2D.
    *
    * @param other the other frame pose 2D. Not modified.
    */
   default void setIncludingFrame(FramePose2DReadOnly other)
   {
      setIncludingFrame(other.getReferenceFrame(), other);
   }

   /**
    * Sets this frame pose 2D to the x, y, yaw components and reference frame of the given
    * {@code framePose3DReadOnly}.
    *
    * @param framePose3DReadOnly the frame pose 3D. Not modified.
    */
   default void setIncludingFrame(FramePose3DReadOnly framePose3DReadOnly)
   {
      setIncludingFrame(framePose3DReadOnly.getReferenceFrame(), framePose3DReadOnly);
   }
}
