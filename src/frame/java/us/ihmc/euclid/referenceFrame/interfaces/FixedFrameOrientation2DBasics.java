package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.geometry.interfaces.Orientation2DBasics;
import us.ihmc.euclid.geometry.interfaces.Orientation2DReadOnly;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;

/**
 * Write and read interface for a 2D orientation that is expressed in a constant reference frame,
 * i.e. the reference frame of this object cannot be changed via this interface.
 * <p>
 * A 2D orientation is in the XY-plane, i.e. the yaw angle about the z-axis.
 * </p>
 *
 * @author Sylvain Bertrand
 */
public interface FixedFrameOrientation2DBasics extends FrameOrientation2DReadOnly, Orientation2DBasics
{
   /**
    * Sets this orientation 2D to represent the orientation 2D from {@code this.getReferenceFrame()} to
    * the given {@code referenceFrame}.
    *
    * @param referenceFrame the reference frame of interest.
    */
   default void setFromReferenceFrame(ReferenceFrame referenceFrame)
   {
      setToZero();
      referenceFrame.transformFromThisToDesiredFrame(getReferenceFrame(), this);
   }

   /**
    * Sets this frame orientation to {@code orientation2DReadOnly} and checks that its current frame
    * equals {@code referenceFrame}.
    *
    * @param referenceFrame        the coordinate system in which the given
    *                              {@code orientation2DReadOnly} is expressed.
    * @param orientation2DReadOnly the orientation 2D to copy the value from. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this.referenceFrame != referenceFrame}.
    */
   default void set(ReferenceFrame referenceFrame, Orientation2DReadOnly orientation2DReadOnly)
   {
      checkReferenceFrameMatch(referenceFrame);
      set(orientation2DReadOnly);
   }

   /**
    * Sets this frame orientation to the yaw angle of the given {@code orientationReadOnly} and checks
    * that its current frame equals {@code referenceFrame}.
    *
    * @param referenceFrame        the coordinate system in which the given
    *                              {@code orientation3DReadOnly} is expressed.
    * @param orientation3DReadOnly the orientation to get the yaw angle from. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this.referenceFrame != referenceFrame}.
    */
   default void set(ReferenceFrame referenceFrame, Orientation3DReadOnly orientation3DReadOnly)
   {
      checkReferenceFrameMatch(referenceFrame);
      set(orientation3DReadOnly);
   }

   /**
    * Sets this orientation 2D to the {@code other} orientation 2D.
    *
    * @param other the other orientation 2D. Not modified.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same frame as
    *                                         {@code this}.
    */
   default void set(FrameOrientation2DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      Orientation2DBasics.super.set(other);
   }

   /**
    * Sets this orientation to be the same as the given one expressed in the reference frame of this.
    * <p>
    * If {@code other} is expressed in the frame as {@code this}, then this method is equivalent to
    * {@link #set(FrameOrientation2DReadOnly)}.
    * </p>
    * <p>
    * If {@code other} is expressed in a different frame than {@code this}, then {@code this} is set to
    * {@code other} and then transformed to be expressed in {@code this.getReferenceFrame()}.
    * </p>
    *
    * @param other the other orientation 2D to set this to. Not modified.
    */
   default void setMatchingFrame(FrameOrientation2DReadOnly other)
   {
      Orientation2DBasics.super.set(other);
      other.getReferenceFrame().transformFromThisToDesiredFrame(getReferenceFrame(), this);
   }

   /**
    * Sets this orientation 2D to the yaw angle of the given {@code frameOrientation3DReadOnly}.
    *
    * @param frameOrientation3DReadOnly the orientation to get the yaw angle from. Not modified.
    * @throws ReferenceFrameMismatchException if {@code frameOrientation3DReadOnly} is not expressed in
    *                                         the same frame as {@code this}.
    */
   default void set(FrameOrientation3DReadOnly frameOrientation3DReadOnly)
   {
      checkReferenceFrameMatch(frameOrientation3DReadOnly);
      Orientation2DBasics.super.set(frameOrientation3DReadOnly);
   }

   /**
    * Adds the other orientation 2D to this:<br>
    * {@code this += other}
    * <p>
    * Note that resulting angle is computed such that it is contained in [-<i>pi</i>, <i>pi</pi>].
    * </p>
    *
    * @param other the other orientation 2D to add to this. Not modified.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same frame as
    *                                         {@code this}.
    */
   default void add(FrameOrientation2DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      Orientation2DBasics.super.add(other);
   }

   /**
    * Sets this orientation 2D to the sum of the two given orientation 2Ds:<br>
    * {@code this = orientation1 + orientation2}
    * <p>
    * Note that resulting angle is computed such that it is contained in [-<i>pi</i>, <i>pi</pi>].
    * </p>
    *
    * @param orientation1 the first orientation 2D. Not modified.
    * @param orientation2 the second orientation 2D. Not modified.
    * @throws ReferenceFrameMismatchException if {@code orientation1} is not expressed in the same
    *                                         frame as {@code this}.
    */
   default void add(FrameOrientation2DReadOnly orientation1, Orientation2DReadOnly orientation2)
   {
      checkReferenceFrameMatch(orientation1);
      Orientation2DBasics.super.add(orientation1, orientation2);
   }

   /**
    * Sets this orientation 2D to the sum of the two given orientation 2Ds:<br>
    * {@code this = orientation1 + orientation2}
    * <p>
    * Note that resulting angle is computed such that it is contained in [-<i>pi</i>, <i>pi</pi>].
    * </p>
    *
    * @param orientation1 the first orientation 2D. Not modified.
    * @param orientation2 the second orientation 2D. Not modified.
    * @throws ReferenceFrameMismatchException if {@code orientation2} is not expressed in the same
    *                                         frame as {@code this}.
    */
   default void add(Orientation2DReadOnly orientation1, FrameOrientation2DReadOnly orientation2)
   {
      checkReferenceFrameMatch(orientation2);
      Orientation2DBasics.super.add(orientation1, orientation2);
   }

   /**
    * Sets this orientation 2D to the sum of the two given orientation 2Ds:<br>
    * {@code this = orientation1 + orientation2}
    * <p>
    * Note that resulting angle is computed such that it is contained in [-<i>pi</i>, <i>pi</pi>].
    * </p>
    *
    * @param orientation1 the first orientation 2D. Not modified.
    * @param orientation2 the second orientation 2D. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this}, {@code orientation1}, and
    *                                         {@code orientation2} are not expressed in the same
    *                                         reference frame.
    */
   default void add(FrameOrientation2DReadOnly orientation1, FrameOrientation2DReadOnly orientation2)
   {
      checkReferenceFrameMatch(orientation1, orientation2);
      Orientation2DBasics.super.add(orientation1, orientation2);
   }

   /**
    * Subtracts the other orientation 2D to this:<br>
    * {@code this -= other}
    * <p>
    * Note that resulting angle is computed such that it is contained in [-<i>pi</i>, <i>pi</pi>].
    * </p>
    *
    * @param other the other orientation 2D to add to this. Not modified.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same frame as
    *                                         {@code this}.
    */
   default void sub(FrameOrientation2DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      Orientation2DBasics.super.sub(other);
   }

   /**
    * Sets this orientation 2D to the difference of the two given orientation 2Ds:<br>
    * {@code this = orientation1 - orientation2}
    * <p>
    * Note that resulting angle is computed such that it is contained in [-<i>pi</i>, <i>pi</pi>].
    * </p>
    *
    * @param orientation1 the first orientation 2D. Not modified.
    * @param orientation2 the second orientation 2D. Not modified.
    * @throws ReferenceFrameMismatchException if {@code orientation1} is not expressed in the same
    *                                         frame as {@code this}.
    */
   default void sub(FrameOrientation2DReadOnly orientation1, Orientation2DReadOnly orientation2)
   {
      checkReferenceFrameMatch(orientation1);
      Orientation2DBasics.super.sub(orientation1, orientation2);
   }

   /**
    * Sets this orientation 2D to the difference of the two given orientation 2Ds:<br>
    * {@code this = orientation1 - orientation2}
    * <p>
    * Note that resulting angle is computed such that it is contained in [-<i>pi</i>, <i>pi</pi>].
    * </p>
    *
    * @param orientation1 the first orientation 2D. Not modified.
    * @param orientation2 the second orientation 2D. Not modified.
    * @throws ReferenceFrameMismatchException if {@code orientation2} is not expressed in the same
    *                                         frame as {@code this}.
    */
   default void sub(Orientation2DReadOnly orientation1, FrameOrientation2DReadOnly orientation2)
   {
      checkReferenceFrameMatch(orientation2);
      Orientation2DBasics.super.sub(orientation1, orientation2);
   }

   /**
    * Sets this orientation 2D to the difference of the two given orientation 2Ds:<br>
    * {@code this = orientation1 - orientation2}
    * <p>
    * Note that resulting angle is computed such that it is contained in [-<i>pi</i>, <i>pi</pi>].
    * </p>
    *
    * @param orientation1 the first orientation 2D. Not modified.
    * @param orientation2 the second orientation 2D. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this}, {@code orientation1}, and
    *                                         {@code orientation2} are not expressed in the same
    *                                         reference frame.
    */
   default void sub(FrameOrientation2DReadOnly orientation1, FrameOrientation2DReadOnly orientation2)
   {
      checkReferenceFrameMatch(orientation1, orientation2);
      Orientation2DBasics.super.sub(orientation1, orientation2);
   }

   /**
    * Performs a linear interpolation from {@code this} to {@code other} given the percentage
    * {@code alpha}.
    * <p>
    * this = (1.0 - alpha) * this + alpha * other
    * </p>
    *
    * @param other the other orientation 2D used for the interpolation. Not modified.
    * @param alpha the percentage used for the interpolation. A value of 0 will result in not modifying
    *              {@code this}, while a value of 1 is equivalent to setting {@code this} to
    *              {@code other}.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same frame as
    *                                         {@code this}.
    */
   default void interpolate(FrameOrientation2DReadOnly other, double alpha)
   {
      checkReferenceFrameMatch(other);
      Orientation2DBasics.super.interpolate(other, alpha);
   }

   /**
    * Performs a linear interpolation from {@code orientation1} to {@code orientation2} given the
    * percentage {@code alpha}.
    * <p>
    * this = (1.0 - alpha) * orientation1 + alpha * orientation2
    * </p>
    *
    * @param orientation1 the first orientation 2D used in the interpolation. Not modified.
    * @param orientation2 the second orientation 2D used in the interpolation. Not modified.
    * @param alpha        the percentage to use for the interpolation. A value of 0 will result in
    *                     setting {@code this} to {@code orientation1}, while a value of 1 is
    *                     equivalent to setting {@code this} to {@code orientation2}.
    * @throws ReferenceFrameMismatchException if {@code orientation1} is not expressed in the same
    *                                         frame as {@code this}.
    */
   default void interpolate(FrameOrientation2DReadOnly orientation1, Orientation2DReadOnly orientation2, double alpha)
   {
      checkReferenceFrameMatch(orientation1);
      Orientation2DBasics.super.interpolate(orientation1, orientation2, alpha);
   }

   /**
    * Performs a linear interpolation from {@code orientation1} to {@code orientation2} given the
    * percentage {@code alpha}.
    * <p>
    * this = (1.0 - alpha) * orientation1 + alpha * orientation2
    * </p>
    *
    * @param orientation1 the first orientation 2D used in the interpolation. Not modified.
    * @param orientation2 the second orientation 2D used in the interpolation. Not modified.
    * @param alpha        the percentage to use for the interpolation. A value of 0 will result in
    *                     setting {@code this} to {@code orientation1}, while a value of 1 is
    *                     equivalent to setting {@code this} to {@code orientation2}.
    * @throws ReferenceFrameMismatchException if {@code orientation2} is not expressed in the same
    *                                         frame as {@code this}.
    */
   default void interpolate(Orientation2DReadOnly orientation1, FrameOrientation2DReadOnly orientation2, double alpha)
   {
      checkReferenceFrameMatch(orientation2);
      Orientation2DBasics.super.interpolate(orientation1, orientation2, alpha);
   }

   /**
    * Performs a linear interpolation from {@code orientation1} to {@code orientation2} given the
    * percentage {@code alpha}.
    * <p>
    * this = (1.0 - alpha) * orientation1 + alpha * orientation2
    * </p>
    *
    * @param orientation1 the first orientation 2D used in the interpolation. Not modified.
    * @param orientation2 the second orientation 2D used in the interpolation. Not modified.
    * @param alpha        the percentage to use for the interpolation. A value of 0 will result in
    *                     setting {@code this} to {@code orientation1}, while a value of 1 is
    *                     equivalent to setting {@code this} to {@code orientation2}.
    * @throws ReferenceFrameMismatchException if {@code this}, {@code orientation1}, and
    *                                         {@code orientation2} are not expressed in the same
    *                                         reference frame.
    */
   default void interpolate(FrameOrientation2DReadOnly orientation1, FrameOrientation2DReadOnly orientation2, double alpha)
   {
      checkReferenceFrameMatch(orientation1, orientation2);
      Orientation2DBasics.super.interpolate(orientation1, orientation2, alpha);
   }
}
