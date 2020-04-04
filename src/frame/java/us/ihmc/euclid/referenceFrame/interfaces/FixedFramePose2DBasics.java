package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.geometry.interfaces.Pose2DBasics;
import us.ihmc.euclid.geometry.interfaces.Pose2DReadOnly;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.orientation.interfaces.Orientation2DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;

/**
 * Write and read interface for a 2D pose expressed in a constant reference frame, i.e. the
 * reference frame of this object cannot be changed via this interface.
 * <p>
 * In addition to representing a {@link Pose2DBasics}, a {@link ReferenceFrame} is associated to a
 * {@code FixedFramePose2DBasics}. This allows, for instance, to enforce, at runtime, that
 * operations on poses occur in the same coordinate system.
 * </p>
 * <p>
 * Because a {@code FixedFramePose2DBasics} extends {@code Pose2DBasics}, it is compatible with
 * methods only requiring {@code Pose2DBasics}. However, these methods do NOT assert that the
 * operation occur in the proper coordinate system. Use this feature carefully and always prefer
 * using methods requiring {@code FixedFramePose2DBasics}.
 * </p>
 */
public interface FixedFramePose2DBasics extends FramePose2DReadOnly, Pose2DBasics
{
   /** {@inheritDoc} */
   @Override
   FixedFrameOrientation2DBasics getOrientation();

   /** {@inheritDoc} */
   @Override
   FixedFramePoint2DBasics getPosition();

   /**
    * Sets this pose 2D to represent the pose of the given {@code referenceFrame} expressed in
    * {@code this.getReferenceFrame()}.
    *
    * @param referenceFrame the reference frame of interest.
    */
   default void setFromReferenceFrame(ReferenceFrame referenceFrame)
   {
      setToZero();
      referenceFrame.transformFromThisToDesiredFrame(getReferenceFrame(), this);
   }

   /**
    * Sets the position from the given frame tuple 2D.
    *
    * @param position the tuple with the new position coordinates. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code position} are not expressed in
    *                                         the same reference frame.
    * @deprecated Use {@code this.getPosition().set(position)} instead.
    */
   default void setPosition(FrameTuple2DReadOnly position)
   {
      getPosition().set(position);
   }

   /**
    * Sets the position from the given frame tuple 3D.
    *
    * @param position the tuple with the new position coordinates. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code position} are not expressed in
    *                                         the same reference frame.
    * @deprecated Use {@code this.getPosition().set(position)} instead.
    */
   default void setPosition(FrameTuple3DReadOnly position)
   {
      getPosition().set(position);
   }

   /**
    * Sets the orientation from the given frame orientation 2D.
    *
    * @param orientation the orientation with the new angle value for this. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code orientation} are not expressed
    *                                         in the same reference frame.
    * @deprecated Use {@code this.getOrientation().set(orientation)} instead.
    */
   default void setOrientation(FrameOrientation2DReadOnly orientation)
   {
      getOrientation().set(orientation);
   }

   /**
    * Sets the orientation from the yaw angle of the given {@code orientation}.
    *
    * @param orientation the orientation with the new angle value for this. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code orientation} are not expressed
    *                                         in the same reference frame.
    * @deprecated Use {@code this.getOrientation().set(orientation)} instead.
    */
   default void setOrientation(FrameOrientation3DReadOnly orientation)
   {
      getOrientation().set(orientation);
   }

   /**
    * Sets the pose from the given {@code pose2DReadOnly} that is expressed in the given
    * {@code referenceFrame}.
    *
    * @param referenceFrame the reference frame in which the given {@code pose2DReadOnly} is expressed
    *                       in.
    * @param pose2DReadOnly the pose 2D used to set the pose of this frame pose 2D. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this.getReferenceFrame() != referenceFrame}.
    */
   default void set(ReferenceFrame referenceFrame, Pose2DReadOnly pose2DReadOnly)
   {
      checkReferenceFrameMatch(referenceFrame);
      set(pose2DReadOnly);
   }

   /**
    * Sets the pose from the given {@code pose3DReadOnly} that is expressed in the given
    * {@code referenceFrame}.
    *
    * @param referenceFrame the reference frame in which the given {@code pose3DReadOnly} is expressed
    *                       in.
    * @param pose3DReadOnly the pose 3D used to set the pose of this frame pose 2D. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this.getReferenceFrame() != referenceFrame}.
    */
   default void set(ReferenceFrame referenceFrame, Pose3DReadOnly pose3DReadOnly)
   {
      checkReferenceFrameMatch(referenceFrame);
      set(pose3DReadOnly);
   }

   /**
    * Sets this frame pose 2D to the {@code other} frame pose 2D.
    *
    * @param other the other frame pose 2D. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code other} are not expressed in
    *                                         the same reference frame.
    */
   default void set(FramePose2DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      Pose2DBasics.super.set(other);
   }

   /**
    * Sets this pose 2D to be the same as the given one expressed in the reference frame of this.
    * <p>
    * If {@code other} is expressed in the frame as {@code this}, then this method is equivalent to
    * {@link #set(FramePose2DReadOnly)}.
    * </p>
    * <p>
    * If {@code other} is expressed in a different frame than {@code this}, then {@code this} is set to
    * {@code other} and then transformed to be expressed in {@code this.getReferenceFrame()}.
    * </p>
    *
    * @param other the other frame pose 2D to set this to. Not modified.
    */
   default void setMatchingFrame(FramePose2DReadOnly other)
   {
      Pose2DBasics.super.set(other);
      other.getReferenceFrame().transformFromThisToDesiredFrame(getReferenceFrame(), this);
   }

   /**
    * Sets this frame pose 2D to the x, y, and yaw components of the given {@code framePose3DReadOnly}.
    *
    * @param framePose3DReadOnly the other frame pose 3D. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code framePose3DReadOnly} are not
    *                                         expressed in the same reference frame.
    */
   default void set(FramePose3DReadOnly framePose3DReadOnly)
   {
      checkReferenceFrameMatch(framePose3DReadOnly);
      Pose2DBasics.super.set(framePose3DReadOnly);
   }

   /**
    * Sets this frame pose 2D to the given {@code position} and {@code yaw} angle.
    *
    * @param position the tuple with the new position coordinates. Not modified.
    * @param yaw      the new angle for the orientation.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code position} are not expressed in
    *                                         the same reference frame.
    */
   default void set(FrameTuple2DReadOnly position, double yaw)
   {
      checkReferenceFrameMatch(position);
      Pose2DBasics.super.set(position, yaw);
   }

   /**
    * Sets both position and orientation.
    *
    * @param position    the tuple with the new position coordinates. Not modified.
    * @param orientation the orientation with the new angle value for this. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code position} are not expressed in
    *                                         the same reference frame.
    */
   default void set(FrameTuple2DReadOnly position, Orientation2DReadOnly orientation)
   {
      checkReferenceFrameMatch(position);
      Pose2DBasics.super.set(position, orientation);
   }

   /**
    * Sets both position and orientation.
    *
    * @param position    the tuple with the new position coordinates. Not modified.
    * @param orientation the orientation with the new angle value for this. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code orientation} are not expressed
    *                                         in the same reference frame.
    */
   default void set(Tuple2DReadOnly position, FrameOrientation2DReadOnly orientation)
   {
      checkReferenceFrameMatch(orientation);
      Pose2DBasics.super.set(position, orientation);
   }

   /**
    * Sets both position and orientation.
    *
    * @param position    the tuple with the new position coordinates. Not modified.
    * @param orientation the orientation with the new angle value for this. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this}, {@code position}, and
    *                                         {@code orientation} are not expressed in the same
    *                                         reference frame.
    */
   default void set(FrameTuple2DReadOnly position, FrameOrientation2DReadOnly orientation)
   {
      checkReferenceFrameMatch(position, orientation);
      Pose2DBasics.super.set(position, orientation);
   }

   /**
    * Rotates the position part of this pose 2D by {@code orientation} and adds {@code orientation} to
    * the orientation part.
    * <p>
    * If the rotation should not affect this pose's position, use
    * {@link #appendRotation(Orientation2DReadOnly)}.
    * </p>
    *
    * @param orientation the orientation to prepend to this pose 2D. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code orientation} are not expressed
    *                                         in the same reference frame.
    */
   default void prependRotation(FrameOrientation2DReadOnly orientation)
   {
      checkReferenceFrameMatch(orientation);
      Pose2DBasics.super.prependRotation(orientation);
   }

   /**
    * Adds the given {@code translation} to this pose 2D assuming it is expressed in the coordinates in
    * which this pose is expressed.
    * <p>
    * If the {@code translation} is expressed in the local coordinates described by this pose 2D, use
    * {@link #appendTranslation(FrameTuple2DReadOnly)}.
    * </p>
    *
    * @param translation tuple containing the translation to apply to this pose 2D. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code translation} are not expressed
    *                                         in the same reference frame.
    */
   default void prependTranslation(FrameTuple2DReadOnly translation)
   {
      checkReferenceFrameMatch(translation);
      Pose2DBasics.super.prependTranslation(translation);
   }

   /**
    * Adds the given {@code orientation} to the orientation of this pose 2D.
    * <p>
    * If the position part of this pose 2D is to be rotated by the given {@code orientation}, use
    * {@link #prependRotation(Orientation2DReadOnly)}.
    * </p>
    *
    * @param orientation the orientation to append to this pose 2D. Not modified.
    */
   default void appendRotation(FrameOrientation2DReadOnly orientation)
   {
      checkReferenceFrameMatch(orientation);
      Pose2DBasics.super.appendRotation(orientation);
   }

   /**
    * Rotates, then adds the given {@code translation} to this pose 2D.
    * <p>
    * Use this method if the {@code translation} is expressed in the local coordinates described by
    * this pose 2D. Otherwise, use {@link #prependTranslation(FrameTuple2DReadOnly)}.
    * </p>
    *
    * @param translation tuple containing the translation to apply to this pose 2D. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code translation} are not expressed
    *                                         in the same reference frame.
    */
   default void appendTranslation(FrameTuple2DReadOnly translation)
   {
      checkReferenceFrameMatch(translation);
      Pose2DBasics.super.appendTranslation(translation);
   }

   /**
    * Performs a linear interpolation from {@code this} to {@code other} given the percentage
    * {@code alpha}.
    * <p>
    * this.position = (1.0 - alpha) * this.position + alpha * other.position<br>
    * this.orientation = (1.0 - alpha) * this.orientation + alpha * other.orientation
    * </p>
    *
    * @param other the other pose 2D used for the interpolation. Not modified.
    * @param alpha the percentage used for the interpolation. A value of 0 will result in not modifying
    *              {@code this}, while a value of 1 is equivalent to setting {@code this} to
    *              {@code other}.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code other} are not expressed in
    *                                         the same reference frame.
    */
   default void interpolate(FramePose2DReadOnly other, double alpha)
   {
      checkReferenceFrameMatch(other);
      Pose2DBasics.super.interpolate(other, alpha);
   }

   /**
    * Performs a linear interpolation from {@code pose1} to {@code pose2} given the percentage
    * {@code alpha}.
    * <p>
    * this.position = (1.0 - alpha) * pose1.position + alpha * pose2.position<br>
    * this.orientation = (1.0 - alpha) * pose1.orientation + alpha * pose2.orientation
    * </p>
    *
    * @param pose1 the first pose 2D used in the interpolation. Not modified.
    * @param pose2 the second pose 2D used in the interpolation. Not modified.
    * @param alpha the percentage to use for the interpolation. A value of 0 will result in setting
    *              {@code this} to {@code pose1}, while a value of 1 is equivalent to setting
    *              {@code this} to {@code pose2}.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code pose1} are not expressed in
    *                                         the same reference frame.
    */
   default void interpolate(FramePose2DReadOnly pose1, Pose2DReadOnly pose2, double alpha)
   {
      checkReferenceFrameMatch(pose1);
      Pose2DBasics.super.interpolate(pose1, pose2, alpha);
   }

   /**
    * Performs a linear interpolation from {@code pose1} to {@code pose2} given the percentage
    * {@code alpha}.
    * <p>
    * this.position = (1.0 - alpha) * pose1.position + alpha * pose2.position<br>
    * this.orientation = (1.0 - alpha) * pose1.orientation + alpha * pose2.orientation
    * </p>
    *
    * @param pose1 the first pose 2D used in the interpolation. Not modified.
    * @param pose2 the second pose 2D used in the interpolation. Not modified.
    * @param alpha the percentage to use for the interpolation. A value of 0 will result in setting
    *              {@code this} to {@code pose1}, while a value of 1 is equivalent to setting
    *              {@code this} to {@code pose2}.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code pose2} are not expressed in
    *                                         the same reference frame.
    */
   default void interpolate(Pose2DReadOnly pose1, FramePose2DReadOnly pose2, double alpha)
   {
      checkReferenceFrameMatch(pose2);
      Pose2DBasics.super.interpolate(pose1, pose2, alpha);
   }

   /**
    * Performs a linear interpolation from {@code pose1} to {@code pose2} given the percentage
    * {@code alpha}.
    * <p>
    * this.position = (1.0 - alpha) * pose1.position + alpha * pose2.position<br>
    * this.orientation = (1.0 - alpha) * pose1.orientation + alpha * pose2.orientation
    * </p>
    *
    * @param pose1 the first pose 2D used in the interpolation. Not modified.
    * @param pose2 the second pose 2D used in the interpolation. Not modified.
    * @param alpha the percentage to use for the interpolation. A value of 0 will result in setting
    *              {@code this} to {@code pose1}, while a value of 1 is equivalent to setting
    *              {@code this} to {@code pose2}.
    * @throws ReferenceFrameMismatchException if {@code this}, {@code pose1} and {@code pose2} are not
    *                                         expressed in the same reference frame.
    */
   default void interpolate(FramePose2DReadOnly pose1, FramePose2DReadOnly pose2, double alpha)
   {
      checkReferenceFrameMatch(pose1, pose2);
      Pose2DBasics.super.interpolate(pose1, pose2, alpha);
   }
}
