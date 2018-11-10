package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.orientation.interfaces.Orientation3DBasics;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;

/**
 * Write and read interface for a 3D orientation expressed in a constant reference frame, i.e. this
 * orientation is always expressed in the same reference frame.
 * <p>
 * Even though the representation used is unknown at this level of abstraction, this interface
 * allows to enforce a minimum set of features that all representations of an orientation should
 * provide, such as appending and prepending orientations to each other.
 * </p>
 * <p>
 * Because a {@code FixedFrameOrientation3DBasics} extends {@code Orientation3DBasics}, it is
 * compatible with methods only requiring {@code Orientation3DBasics}. However, these methods do NOT
 * assert that the operation occur in the proper coordinate system. Use this feature carefully and
 * always prefer using methods requiring {@code FixedFrameOrientation3DBasics}.
 * </p>
 * 
 * @author Sylvain Bertrand
 */
public interface FixedFrameOrientation3DBasics extends FrameOrientation3DReadOnly, Orientation3DBasics
{
   /**
    * Sets this frame quaternion to {@code orientation} and checks that its current frame equals
    * {@code referenceFrame}.
    *
    * @param referenceFrame the coordinate system in which the given {@code quaternionReadOnly} is
    *           expressed.
    * @param orientation the orientation to copy the values from. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this.referenceFrame != referenceFrame}.
    */
   default void set(ReferenceFrame referenceFrame, Orientation3DReadOnly orientation)
   {
      checkReferenceFrameMatch(referenceFrame);
      set(orientation);
   }

   /**
    * Sets this orientation to represent the orientation from {@code this.getReferenceFrame()} to the
    * given {@code referenceFrame}.
    *
    * @param referenceFrame the reference frame of interest.
    */
   default void setFromReferenceFrame(ReferenceFrame referenceFrame)
   {
      setToZero();
      referenceFrame.transformFromThisToDesiredFrame(getReferenceFrame(), this);
   }

   /**
    * Sets this frame orientation to {@code orientation}.
    * <p>
    * If {@code orientation} is expressed in the frame as {@code this}, then this method is equivalent
    * to {@link #set(FrameOrientation3DReadOnly)}.
    * </p>
    * <p>
    * If {@code orientation} is expressed in a different frame than {@code this}, then {@code this} is
    * set to {@code orientation} and then transformed to be expressed in
    * {@code this.getReferenceFrame()}.
    * </p>
    *
    * @param orientation the other orientation to copy the values from. Not modified.
    */
   default void setMatchingFrame(FrameOrientation3DReadOnly orientation)
   {
      set((Orientation3DReadOnly) orientation);
      orientation.getReferenceFrame().transformFromThisToDesiredFrame(getReferenceFrame(), this);
   }

   /**
    * Sets this orientation to represent the same orientation as the given {@code rotationVector}.
    * <p>
    * WARNING: a rotation vector is different from a yaw-pitch-roll or Euler angles representation. A
    * rotation vector is equivalent to the axis of an axis-angle that is multiplied by the angle of the
    * same axis-angle.
    * </p>
    *
    * @param rotationVector the rotation vector to set this orientation. Not modified.
    * @throws ReferenceFrameMismatchException if {@code rotationVector} is not expressed in the same
    *            reference frame as {@code this}.
    */
   default void setRotationVector(FrameVector3DReadOnly rotationVector)
   {
      checkReferenceFrameMatch(rotationVector);
      Orientation3DBasics.super.setRotationVector(rotationVector);
   }

   /**
    * Sets this orientation to represent the same orientation as the given Euler angles
    * {@code eulerAngles}.
    * <p>
    * This is equivalent to {@link #setYawPitchRoll(double, double, double)} with
    * {@code yaw = eulerAngles.getZ()}, {@code pitch = eulerAngles.getY()}, and
    * {@code roll = eulerAngles.getX()}.
    * </p>
    *
    * @param eulerAngles the Euler angles to copy the orientation from. Not modified.
    * @throws ReferenceFrameMismatchException if {@code eulerAngles} is not expressed in the same
    *            reference frame as {@code this}.
    */
   default void setEuler(FrameVector3DReadOnly eulerAngles)
   {
      checkReferenceFrameMatch(eulerAngles);
      Orientation3DBasics.super.setEuler(eulerAngles);
   }

   /**
    * Converts, if necessary, and sets this orientation to represents the same orientation as
    * {@code orientation}.
    *
    * @param orientation the new orientation. Not modified.
    * @throws ReferenceFrameMismatchException if {@code orientation} is not expressed in the same
    *            reference frame as {@code this}.
    */
   default void set(FrameOrientation3DReadOnly orientation)
   {
      checkReferenceFrameMatch(orientation);
      set((Orientation3DReadOnly) orientation);
   }

   /**
    * Converts, if necessary, and sets this orientation to represents the same orientation as
    * {@code orientation} and then normalize this orientation.
    *
    * @param orientation the new orientation. Not modified.
    * @throws ReferenceFrameMismatchException if {@code orientation} is not expressed in the same
    *            reference frame as {@code this}.
    */
   default void setAndNormalize(FrameOrientation3DReadOnly orientation)
   {
      set(orientation);
      normalize();
   }

   /**
    * Sets this orientation to represent the inverse of the given {@code orientation}.
    *
    * @param orientation the new orientation. Not modified.
    * @throws ReferenceFrameMismatchException if {@code orientation} is not expressed in the same
    *            reference frame as {@code this}.
    */
   default void setAndInvert(FrameOrientation3DReadOnly orientation)
   {
      set(orientation);
      invert();
   }

   /**
    * Appends the given orientation to this orientation.
    * <p>
    * Let's consider the following:
    * <ul>
    * <li>three coordinate systems <b>A</b>, <b>B</b>, and <b>C</b>.
    * <li>{@code this} represents the orientation of <b>B</b> relative to <b>A</b>.
    * <li>{@code orientation} represents the orientation of <b>C</b> relative to <b>B</b>.
    * </ul>
    * The result of calling this method will be that {@code this}, represents the orientation of
    * <b>C</b> relative to <b>A</b>.
    * </p>
    * <p>
    * Appending orientations is in some way similar to summing translations. However, while the
    * addition for translation is commutative, the "append" operation on orientation is NOT
    * commutative. Such that: {@code this.append(orientation)} &ne; {@code orientation.append(this)}.
    * </p>
    *
    * @param orientation the orientation to append to this orientation. Not modified.
    * @throws ReferenceFrameMismatchException if {@code orientation} is not expressed in the same
    *            reference frame as {@code this}.
    */
   default void append(FrameOrientation3DReadOnly orientation)
   {
      checkReferenceFrameMatch(orientation);
      append((Orientation3DReadOnly) orientation);
   }

   /**
    * Appends the inverse of the given orientation to this orientation.
    * <p>
    * Let's consider the following:
    * <ul>
    * <li>three coordinate systems <b>A</b>, <b>B</b>, and <b>C</b>.
    * <li>{@code this} represents the orientation of <b>B</b> relative to <b>A</b>.
    * <li>{@code orientation} represents the orientation of <b>B</b> relative to <b>C</b>.
    * </ul>
    * The result of calling this method will be that {@code this}, represents the orientation of
    * <b>C</b> relative to <b>A</b>.
    * </p>
    * <p>
    * This operation is in some way similar to subtracting translations, as in this operation can be
    * seen as subtracting {@code orientation} to {@code this}.
    * </p>
    *
    * @param orientation the orientation which the inverse is to be appended to this orientation. Not
    *           modified.
    * @throws ReferenceFrameMismatchException if {@code orientation} is not expressed in the same
    *            reference frame as {@code this}.
    */
   default void appendInvertOther(FrameOrientation3DReadOnly orientation)
   {
      checkReferenceFrameMatch(orientation);
      appendInvertOther((Orientation3DReadOnly) orientation);
   }

   /**
    * Inverts {@code this} and then appends the given orientation.
    * <p>
    * Let's consider the following:
    * <ul>
    * <li>three coordinate systems <b>A</b>, <b>B</b>, and <b>C</b>.
    * <li>{@code this} represents the orientation of <b>A</b> relative to <b>B</b>.
    * <li>{@code orientation} represents the orientation of <b>C</b> relative to <b>B</b>.
    * </ul>
    * The result of calling this method will be that {@code this}, represents the orientation of
    * <b>C</b> relative to <b>A</b>.
    * </p>
    * <p>
    * This operation is in some way similar to subtracting translations, as in this operation can be
    * seen as subtracting {@code this} to {@code orientation}.
    * </p>
    *
    * @param orientation the orientation to append to the inverse of this orientation. Not modified.
    * @throws ReferenceFrameMismatchException if {@code orientation} is not expressed in the same
    *            reference frame as {@code this}.
    */
   default void appendInvertThis(FrameOrientation3DReadOnly orientation)
   {
      checkReferenceFrameMatch(orientation);
      Orientation3DBasics.super.appendInvertThis(orientation);
   }

   /**
    * Inverts {@code this} and then appends the inverse of the given orientation to this orientation.
    * <p>
    * Let's consider the following:
    * <ul>
    * <li>three coordinate systems <b>A</b>, <b>B</b>, and <b>C</b>.
    * <li>{@code this} represents the orientation of <b>A</b> relative to <b>B</b>.
    * <li>{@code orientation} represents the orientation of <b>B</b> relative to <b>C</b>.
    * </ul>
    * The result of calling this method will be that {@code this}, represents the orientation of
    * <b>C</b> relative to <b>A</b>.
    * </p>
    * <p>
    * This operation is in some way similar to subtracting translations, as in this operation can be
    * seen as subtracting {@code orientation} to {@code this}.
    * </p>
    *
    * @param orientation the orientation which the inverse is to be appended to this orientation. Not
    *           modified.
    * @throws ReferenceFrameMismatchException if {@code orientation} is not expressed in the same
    *            reference frame as {@code this}.
    */
   default void appendInvertBoth(FrameOrientation3DReadOnly orientation)
   {
      checkReferenceFrameMatch(orientation);
      Orientation3DBasics.super.appendInvertBoth(orientation);
   }

   /**
    * Prepends the given orientation to this orientation.
    * <p>
    * Let's consider the following:
    * <ul>
    * <li>three coordinate systems <b>A</b>, <b>B</b>, and <b>C</b>.
    * <li>{@code this} represents the orientation of <b>C</b> relative to <b>B</b>.
    * <li>{@code orientation} represents the orientation of <b>B</b> relative to <b>A</b>.
    * </ul>
    * The result of calling this method will be that {@code this}, represents the orientation of
    * <b>C</b> relative to <b>A</b>.
    * </p>
    * <p>
    * Prepending orientations is in some way similar to summing translations. However, while the
    * addition for translation is commutative, the "prepend" operation on orientation is NOT
    * commutative. Such that: {@code this.prepend(orientation)} &ne; {@code orientation.prepend(this)}.
    * </p>
    *
    * @param orientation the orientation to prepend to this orientation. Not modified.
    * @throws ReferenceFrameMismatchException if {@code orientation} is not expressed in the same
    *            reference frame as {@code this}.
    */
   default void prepend(FrameOrientation3DReadOnly orientation)
   {
      checkReferenceFrameMatch(orientation);
      prepend((Orientation3DReadOnly) orientation);
   }

   /**
    * Prepends the inverse of the given orientation to this orientation.
    * <p>
    * Let's consider the following:
    * <ul>
    * <li>three coordinate systems <b>A</b>, <b>B</b>, and <b>C</b>.
    * <li>{@code this} represents the orientation of <b>B</b> relative to <b>C</b>.
    * <li>{@code orientation} represents the orientation of <b>B</b> relative to <b>A</b>.
    * </ul>
    * The result of calling this method will be that {@code this}, represents the orientation of
    * <b>C</b> relative to <b>A</b>.
    * </p>
    * <p>
    * This operation is in some way similar to subtracting translations, as in this operation can be
    * seen as subtracting {@code orientation} to {@code this}.
    * </p>
    *
    * @param orientation the orientation which the inverse is to be appended to this orientation. Not
    *           modified.
    * @throws ReferenceFrameMismatchException if {@code orientation} is not expressed in the same
    *            reference frame as {@code this}.
    */
   default void prependInvertOther(FrameOrientation3DReadOnly orientation)
   {
      checkReferenceFrameMatch(orientation);
      prependInvertOther((Orientation3DReadOnly) orientation);
   }

   /**
    * Inverts {@code this} and then prepends the given orientation.
    * <p>
    * Let's consider the following:
    * <ul>
    * <li>three coordinate systems <b>A</b>, <b>B</b>, and <b>C</b>.
    * <li>{@code this} represents the orientation of <b>C</b> relative to <b>B</b>.
    * <li>{@code orientation} represents the orientation of <b>A</b> relative to <b>B</b>.
    * </ul>
    * The result of calling this method will be that {@code this}, represents the orientation of
    * <b>C</b> relative to <b>A</b>.
    * </p>
    * <p>
    * This operation is in some way similar to subtracting translations, as in this operation can be
    * seen as subtracting {@code this} to {@code orientation}.
    * </p>
    *
    * @param orientation the orientation to append to the inverse of this orientation. Not modified.
    * @throws ReferenceFrameMismatchException if {@code orientation} is not expressed in the same
    *            reference frame as {@code this}.
    */
   default void prependInvertThis(FrameOrientation3DReadOnly orientation)
   {
      checkReferenceFrameMatch(orientation);
      Orientation3DBasics.super.prependInvertThis(orientation);
   }

   /**
    * Inverts {@code this} and then prepends the inverse of the given orientation to this orientation.
    * <p>
    * Let's consider the following:
    * <ul>
    * <li>three coordinate systems <b>A</b>, <b>B</b>, and <b>C</b>.
    * <li>{@code this} represents the orientation of <b>B</b> relative to <b>C</b>.
    * <li>{@code orientation} represents the orientation of <b>A</b> relative to <b>B</b>.
    * </ul>
    * The result of calling this method will be that {@code this}, represents the orientation of
    * <b>C</b> relative to <b>A</b>.
    * </p>
    * <p>
    * This operation is in some way similar to subtracting translations, as in this operation can be
    * seen as subtracting {@code orientation} to {@code this}.
    * </p>
    *
    * @param orientation the orientation which the inverse is to be appended to this orientation. Not
    *           modified.
    * @throws ReferenceFrameMismatchException if {@code orientation} is not expressed in the same
    *            reference frame as {@code this}.
    */
   default void prependInvertBoth(FrameOrientation3DReadOnly orientation)
   {
      checkReferenceFrameMatch(orientation);
      Orientation3DBasics.super.prependInvertBoth(orientation);
   }
}
