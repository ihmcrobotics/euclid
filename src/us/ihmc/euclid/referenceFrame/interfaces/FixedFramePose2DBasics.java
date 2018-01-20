package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.geometry.Orientation2D;
import us.ihmc.euclid.geometry.interfaces.Orientation2DReadOnly;
import us.ihmc.euclid.geometry.interfaces.Pose2DBasics;
import us.ihmc.euclid.geometry.interfaces.Pose2DReadOnly;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.tuple2D.interfaces.Tuple2DReadOnly;

public interface FixedFramePose2DBasics extends FramePose2DReadOnly, Pose2DBasics
{
   @Override
   FixedFrameOrientation2DBasics getOrientation();

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
    * @throws ReferenceFrameMismatchException if {@code this} and {@code position} are not expressed
    *            in the same reference frame.
    */
   default void setPosition(FrameTuple2DReadOnly position)
   {
      checkReferenceFrameMatch(position);
      Pose2DBasics.super.setPosition(position);
   }

   /**
    * Sets the position from the given frame tuple 3D.
    *
    * @param position the tuple with the new position coordinates. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code position} are not expressed
    *            in the same reference frame.
    */
   default void setPosition(FrameTuple3DReadOnly position)
   {
      checkReferenceFrameMatch(position);
      Pose2DBasics.super.setPosition(position);
   }

   /**
    * Sets the orientation from the given frame orientation 2D.
    *
    * @param orientation the orientation with the new angle value for this. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code orientation} are not
    *            expressed in the same reference frame.
    */
   default void setOrientation(FrameOrientation2DReadOnly orientation)
   {
      checkReferenceFrameMatch(orientation);
      Pose2DBasics.super.setOrientation(orientation);
   }

   default void setOrientation(FrameQuaternionReadOnly orientation)
   {
      checkReferenceFrameMatch(orientation);
      Pose2DBasics.super.setOrientation(orientation);
   }

   default void set(ReferenceFrame referenceFrame, Pose2DReadOnly pose2DReadOnly)
   {
      checkReferenceFrameMatch(referenceFrame);
      set(pose2DReadOnly);
   }

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
    *            the same reference frame.
    */
   default void set(FramePose2DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      Pose2DBasics.super.set(other);
   }

   /**
    * Sets this frame pose 2D to the x, y, and yaw components of the given
    * {@code framePose3DReadOnly}.
    *
    * @param framePose3DReadOnly the other frame pose 3D. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code framePose3DReadOnly} are
    *            not expressed in the same reference frame.
    */
   default void set(FramePose3DReadOnly framePose3DReadOnly)
   {
      checkReferenceFrameMatch(framePose3DReadOnly);
      Pose2DBasics.super.set(framePose3DReadOnly);
   }

   /**
    * Sets both position and orientation.
    *
    * @param position the tuple with the new position coordinates. Not modified.
    * @param orientation the orientation with the new angle value for this. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code position} are not expressed
    *            in the same reference frame.
    */
   default void set(FrameTuple2DReadOnly position, Orientation2DReadOnly orientation)
   {
      checkReferenceFrameMatch(position);
      Pose2DBasics.super.set(position, orientation);
   }

   /**
    * Sets both position and orientation.
    *
    * @param position the tuple with the new position coordinates. Not modified.
    * @param orientation the orientation with the new angle value for this. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code orientation} are not
    *            expressed in the same reference frame.
    */
   default void set(Tuple2DReadOnly position, FrameOrientation2DReadOnly orientation)
   {
      checkReferenceFrameMatch(orientation);
      Pose2DBasics.super.set(position, orientation);
   }

   /**
    * Sets both position and orientation.
    *
    * @param position the tuple with the new position coordinates. Not modified.
    * @param orientation the orientation with the new angle value for this. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this}, {@code position}, and
    *            {@code orientation} are not expressed in the same reference frame.
    */
   default void set(FrameTuple2DReadOnly position, FrameOrientation2DReadOnly orientation)
   {
      checkReferenceFrameMatch(position);
      checkReferenceFrameMatch(orientation);
      Pose2DBasics.super.set(position, orientation);
   }

   /**
    * Rotates the position part of this pose 2D by {@code orientation} and adds {@code orientation}
    * to the orientation part.
    * <p>
    * If the rotation should not affect this pose's position, use
    * {@link #appendRotation(Orientation2D)}.
    * </p>
    *
    * @param orientation the orientation to prepend to this pose 2D. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code orientation} are not
    *            expressed in the same reference frame.
    */
   default void prependRotation(FrameOrientation2DReadOnly orientation)
   {
      checkReferenceFrameMatch(orientation);
      Pose2DBasics.super.prependRotation(orientation);
   }

   /**
    * Adds the given {@code translation} to this pose 2D assuming it is expressed in the coordinates
    * in which this pose is expressed.
    * <p>
    * If the {@code translation} is expressed in the local coordinates described by this pose 2D,
    * use {@link #appendTranslation(FrameTuple2DReadOnly)}.
    * </p>
    *
    * @param translation tuple containing the translation to apply to this pose 2D. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code translation} are not
    *            expressed in the same reference frame.
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
    * {@link #prependRotation(Orientation2D)}.
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
    * @throws ReferenceFrameMismatchException if {@code this} and {@code translation} are not
    *            expressed in the same reference frame.
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
    * @param alpha the percentage used for the interpolation. A value of 0 will result in not
    *           modifying {@code this}, while a value of 1 is equivalent to setting {@code this} to
    *           {@code other}.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code other} are not expressed in
    *            the same reference frame.
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
    *           {@code this} to {@code pose1}, while a value of 1 is equivalent to setting
    *           {@code this} to {@code pose2}.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code pose1} are not expressed in
    *            the same reference frame.
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
    *           {@code this} to {@code pose1}, while a value of 1 is equivalent to setting
    *           {@code this} to {@code pose2}.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code pose2} are not expressed in
    *            the same reference frame.
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
    *           {@code this} to {@code pose1}, while a value of 1 is equivalent to setting
    *           {@code this} to {@code pose2}.
    * @throws ReferenceFrameMismatchException if {@code this}, {@code pose1} and {@code pose2} are
    *            not expressed in the same reference frame.
    */
   default void interpolate(FramePose2DReadOnly pose1, FramePose2DReadOnly pose2, double alpha)
   {
      checkReferenceFrameMatch(pose1);
      checkReferenceFrameMatch(pose2);
      Pose2DBasics.super.interpolate(pose1, pose2, alpha);
   }
}
