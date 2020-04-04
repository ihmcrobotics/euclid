package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.geometry.interfaces.Pose2DReadOnly;
import us.ihmc.euclid.geometry.interfaces.Pose3DBasics;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.orientation.interfaces.Orientation3DBasics;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;

/**
 * Write and read interface for a 3D pose expressed in a constant reference frame, i.e. the
 * reference frame of this object cannot be changed via this interface.
 * <p>
 * In addition to representing a {@link Pose3DBasics}, a {@link ReferenceFrame} is associated to a
 * {@code FixedFramePose3DBasics}. This allows, for instance, to enforce, at runtime, that
 * operations on poses occur in the same coordinate system.
 * </p>
 * <p>
 * Because a {@code FixedFramePose3DBasics} extends {@code Pose3DBasics}, it is compatible with
 * methods only requiring {@code Pose3DBasics}. However, these methods do NOT assert that the
 * operation occur in the proper coordinate system. Use this feature carefully and always prefer
 * using methods requiring {@code FixedFramePose3DBasics}.
 * </p>
 */
public interface FixedFramePose3DBasics extends FramePose3DReadOnly, Pose3DBasics
{
   /** {@inheritDoc} */
   @Override
   FixedFramePoint3DBasics getPosition();

   /** {@inheritDoc} */
   @Override
   FixedFrameQuaternionBasics getOrientation();

   /**
    * Sets this pose 3D to represent the pose of the given {@code referenceFrame} expressed in
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
    * <p>
    * The z component remains unchanged.
    * </p>
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
    * Sets the position from the given frame tuple 2D and the given {@code z} coordinate.
    *
    * @param position the tuple with the x and y position coordinates. Not modified.
    * @param z        the new z-coordinate.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code position} are not expressed in
    *                                         the same reference frame.
    * @deprecated Use {@code this.getPosition().set(position, z)} instead.
    */
   default void setPosition(FrameTuple2DReadOnly position, double z)
   {
      getPosition().set(position, z);
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
    * Sets the orientation from the given frame orientation.
    *
    * @param orientation the orientation to set the orientation part of this frame pose. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code orientation} are not expressed
    *                                         in the same reference frame.
    * @deprecated Use {@code this.getOrientation().set(orientation)} instead.
    */
   default void setOrientation(FrameOrientation3DReadOnly orientation)
   {
      getOrientation().set(orientation);
   }

   /**
    * Sets the pose to represent the same pose as the given {@code pose2DReadOnly} that is expressed in
    * the given {@code referenceFrame}.
    *
    * @param referenceFrame the reference frame in which the given {@code pose2DReadOnly} is expressed
    *                       in.
    * @param pose2DReadOnly the pose 2D used to set the pose of this frame pose 3D. Not modified.
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
    * @param pose3DReadOnly the pose 3D used to set the pose of this frame pose 3D. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this.getReferenceFrame() != referenceFrame}.
    */
   default void set(ReferenceFrame referenceFrame, Pose3DReadOnly pose3DReadOnly)
   {
      checkReferenceFrameMatch(referenceFrame);
      set(pose3DReadOnly);
   }

   /**
    * Sets this frame pose 3D to the represent the same pose as the given {@code framePose2DReadOnly}.
    *
    * @param framePose2DReadOnly the other frame pose 2D. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code framePose2DReadOnly} are not
    *                                         expressed in the same reference frame.
    */
   default void set(FramePose2DReadOnly framePose2DReadOnly)
   {
      checkReferenceFrameMatch(framePose2DReadOnly);
      Pose3DBasics.super.set(framePose2DReadOnly);
   }

   /**
    * Sets this frame pose 3D to the {@code other} frame pose 3D.
    *
    * @param other the other frame pose 3D. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code other} are not expressed in
    *                                         the same reference frame.
    */
   default void set(FramePose3DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      Pose3DBasics.super.set(other);
   }

   /**
    * Sets this pose 3D to be the same as the given one expressed in the reference frame of this.
    * <p>
    * If {@code other} is expressed in the frame as {@code this}, then this method is equivalent to
    * {@link #set(FramePose3DReadOnly)}.
    * </p>
    * <p>
    * If {@code other} is expressed in a different frame than {@code this}, then {@code this} is set to
    * {@code other} and then transformed to be expressed in {@code this.getReferenceFrame()}.
    * </p>
    *
    * @param other the other frame pose 3D to set this to. Not modified.
    */
   default void setMatchingFrame(FramePose3DReadOnly other)
   {
      Pose3DBasics.super.set(other);
      other.getReferenceFrame().transformFromThisToDesiredFrame(getReferenceFrame(), this);
   }

   /**
    * Sets both position and orientation.
    *
    * @param position    the tuple with the new position coordinates. Not modified.
    * @param orientation the new orientation. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code position} are not expressed in
    *                                         the same reference frame.
    */
   default void set(FrameTuple3DReadOnly position, Orientation3DReadOnly orientation)
   {
      checkReferenceFrameMatch(position);
      Pose3DBasics.super.set(position, orientation);
   }

   /**
    * Sets both position and orientation.
    *
    * @param position    the tuple with the new position coordinates. Not modified.
    * @param orientation the new orientation. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code orientation} are not expressed
    *                                         in the same reference frame.
    */
   default void set(Tuple3DReadOnly position, FrameOrientation3DReadOnly orientation)
   {
      checkReferenceFrameMatch(orientation);
      Pose3DBasics.super.set(position, orientation);
   }

   /**
    * Sets both position and orientation.
    *
    * @param position    the tuple with the new position coordinates. Not modified.
    * @param orientation the new orientation. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this}, {@code position}, and
    *                                         {@code orientation} are not expressed in the same
    *                                         reference frame.
    */
   default void set(FrameTuple3DReadOnly position, FrameOrientation3DReadOnly orientation)
   {
      checkReferenceFrameMatch(position, orientation);
      Pose3DBasics.super.set(position, orientation);
   }

   /**
    * Adds the given {@code translation} to this pose 3D assuming it is expressed in the coordinates in
    * which this pose is expressed.
    * <p>
    * If the {@code translation} is expressed in the local coordinates described by this pose 3D, use
    * {@link #appendTranslation(FrameTuple3DReadOnly)}.
    * </p>
    *
    * @param translation tuple containing the translation to apply to this pose 3D. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code translation} are not expressed
    *                                         in the same reference frame.
    */
   default void prependTranslation(FrameTuple3DReadOnly translation)
   {
      checkReferenceFrameMatch(translation);
      Pose3DBasics.super.prependTranslation(translation);
   }

   /**
    * Rotates the position part of this pose 3D by the given {@code rotation} and prepends it to the
    * orientation part.
    *
    * @param rotation the rotation to prepend to this pose 3D. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code rotation} are not expressed in
    *                                         the same reference frame.
    * @see Orientation3DBasics#prepend(Orientation3DReadOnly)
    */
   default void prependRotation(FrameOrientation3DReadOnly rotation)
   {
      checkReferenceFrameMatch(rotation);
      Pose3DBasics.super.prependRotation(rotation);
   }

   /**
    * Rotates, then adds the given {@code translation} to this pose 3D.
    * <p>
    * Use this method if the {@code translation} is expressed in the local coordinates described by
    * this pose 3D. Otherwise, use {@link #prependTranslation(FrameTuple3DReadOnly)}.
    * </p>
    *
    * @param translation tuple containing the translation to apply to this pose 3D. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code translation} are not expressed
    *                                         in the same reference frame.
    */
   default void appendTranslation(FrameTuple3DReadOnly translation)
   {
      checkReferenceFrameMatch(translation);
      Pose3DBasics.super.appendTranslation(translation);
   }

   /**
    * Appends the given rotation to this pose 3D.
    * <p>
    * More precisely, the position part is unchanged while the orientation part is updated as
    * follows:<br>
    * {@code this.orientation = this.orientation * rotation}
    * </p>
    *
    * @param rotation the rotation to append to this pose 3D. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code rotation} are not expressed in
    *                                         the same reference frame.
    * @see Orientation3DBasics#append(Orientation3DReadOnly)
    */
   default void appendRotation(FrameOrientation3DReadOnly rotation)
   {
      checkReferenceFrameMatch(rotation);
      Pose3DBasics.super.appendRotation(rotation);
   }

   /**
    * Performs a linear interpolation from {@code this} to {@code other} given the percentage
    * {@code alpha}.
    * <p>
    * this.position = (1.0 - alpha) * this.position + alpha * other.position<br>
    * this.orientation = (1.0 - alpha) * this.orientation + alpha * other.orientation
    * </p>
    *
    * @param other the other pose 3D used for the interpolation. Not modified.
    * @param alpha the percentage used for the interpolation. A value of 0 will result in not modifying
    *              {@code this}, while a value of 1 is equivalent to setting {@code this} to
    *              {@code other}.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code other} are not expressed in
    *                                         the same reference frame.
    */
   default void interpolate(FramePose3DReadOnly other, double alpha)
   {
      checkReferenceFrameMatch(other);
      Pose3DBasics.super.interpolate(other, alpha);
   }

   /**
    * Performs a linear interpolation from {@code pose1} to {@code pose2} given the percentage
    * {@code alpha}.
    * <p>
    * this.position = (1.0 - alpha) * pose1.position + alpha * pose2.position<br>
    * this.orientation = (1.0 - alpha) * pose1.orientation + alpha * pose2.orientation
    * </p>
    *
    * @param pose1 the first pose 3D used in the interpolation. Not modified.
    * @param pose2 the second pose 3D used in the interpolation. Not modified.
    * @param alpha the percentage to use for the interpolation. A value of 0 will result in setting
    *              {@code this} to {@code pose1}, while a value of 1 is equivalent to setting
    *              {@code this} to {@code pose2}.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code pose1} are not expressed in
    *                                         the same reference frame.
    */
   default void interpolate(FramePose3DReadOnly pose1, Pose3DReadOnly pose2, double alpha)
   {
      checkReferenceFrameMatch(pose1);
      Pose3DBasics.super.interpolate(pose1, pose2, alpha);
   }

   /**
    * Performs a linear interpolation from {@code pose1} to {@code pose2} given the percentage
    * {@code alpha}.
    * <p>
    * this.position = (1.0 - alpha) * pose1.position + alpha * pose2.position<br>
    * this.orientation = (1.0 - alpha) * pose1.orientation + alpha * pose2.orientation
    * </p>
    *
    * @param pose1 the first pose 3D used in the interpolation. Not modified.
    * @param pose2 the second pose 3D used in the interpolation. Not modified.
    * @param alpha the percentage to use for the interpolation. A value of 0 will result in setting
    *              {@code this} to {@code pose1}, while a value of 1 is equivalent to setting
    *              {@code this} to {@code pose2}.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code pose2} are not expressed in
    *                                         the same reference frame.
    */
   default void interpolate(Pose3DReadOnly pose1, FramePose3DReadOnly pose2, double alpha)
   {
      checkReferenceFrameMatch(pose2);
      Pose3DBasics.super.interpolate(pose1, pose2, alpha);
   }

   /**
    * Performs a linear interpolation from {@code pose1} to {@code pose2} given the percentage
    * {@code alpha}.
    * <p>
    * this.position = (1.0 - alpha) * pose1.position + alpha * pose2.position<br>
    * this.orientation = (1.0 - alpha) * pose1.orientation + alpha * pose2.orientation
    * </p>
    *
    * @param pose1 the first pose 3D used in the interpolation. Not modified.
    * @param pose2 the second pose 3D used in the interpolation. Not modified.
    * @param alpha the percentage to use for the interpolation. A value of 0 will result in setting
    *              {@code this} to {@code pose1}, while a value of 1 is equivalent to setting
    *              {@code this} to {@code pose2}.
    * @throws ReferenceFrameMismatchException if {@code this}, {@code pose1}, and {@code pose2} are not
    *                                         expressed in the same reference frame.
    */
   default void interpolate(FramePose3DReadOnly pose1, FramePose3DReadOnly pose2, double alpha)
   {
      checkReferenceFrameMatch(pose1, pose2);
      Pose3DBasics.super.interpolate(pose1, pose2, alpha);
   }
}
