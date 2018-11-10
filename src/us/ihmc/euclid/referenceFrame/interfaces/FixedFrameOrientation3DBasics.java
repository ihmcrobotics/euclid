package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.orientation.interfaces.Orientation3DBasics;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;

public interface FixedFrameOrientation3DBasics extends FrameOrientation3DReadOnly, Orientation3DBasics
{

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
    * {@code orientation3DReadOnly}.
    *
    * @param orientation3DReadOnly the new orientation. Not modified.
    * @throws ReferenceFrameMismatchException if {@code orientation3DReadOnly} is not expressed in the
    *            same reference frame as {@code this}.
    */
   default void set(FrameOrientation3DReadOnly orientation3DReadOnly)
   {
      checkReferenceFrameMatch(orientation3DReadOnly);
      set((Orientation3DReadOnly) orientation3DReadOnly);
   }

   /**
    * Converts, if necessary, and sets this orientation to represents the same orientation as
    * {@code orientation3DReadOnly} and then normalize this orientation.
    *
    * @param other the new orientation. Not modified.
    * @throws ReferenceFrameMismatchException if {@code orientation3DReadOnly} is not expressed in the
    *            same reference frame as {@code this}.
    */
   default void setAndNormalize(FrameOrientation3DReadOnly orientation3DReadOnly)
   {
      set(orientation3DReadOnly);
      normalize();
   }

   /**
    * Sets this orientation to represent the inverse of the given {@code orientation3DReadOnly}.
    *
    * @param other the new orientation. Not modified.
    * @throws ReferenceFrameMismatchException if {@code orientation3DReadOnly} is not expressed in the
    *            same reference frame as {@code this}.
    */
   default void setAndInvert(FrameOrientation3DReadOnly orientation3DReadOnly)
   {
      set(orientation3DReadOnly);
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
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same reference
    *            frame as {@code this}.
    */
   default void append(FrameOrientation3DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      append((Orientation3DReadOnly) other);
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
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same reference
    *            frame as {@code this}.
    */
   default void appendInvertOther(FrameOrientation3DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      appendInvertOther((Orientation3DReadOnly) other);
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
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same reference
    *            frame as {@code this}.
    */
   default void appendInvertThis(FrameOrientation3DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      Orientation3DBasics.super.appendInvertThis(other);
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
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same reference
    *            frame as {@code this}.
    */
   default void appendInvertBoth(FrameOrientation3DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      Orientation3DBasics.super.appendInvertBoth(other);
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
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same reference
    *            frame as {@code this}.
    */
   default void prepend(FrameOrientation3DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      prepend((Orientation3DReadOnly) other);
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
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same reference
    *            frame as {@code this}.
    */
   default void prependInvertOther(FrameOrientation3DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      prependInvertOther((Orientation3DReadOnly) other);
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
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same reference
    *            frame as {@code this}.
    */
   default void prependInvertThis(FrameOrientation3DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      Orientation3DBasics.super.prependInvertThis(other);
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
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same reference
    *            frame as {@code this}.
    */
   default void prependInvertBoth(FrameOrientation3DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      Orientation3DBasics.super.prependInvertBoth(other);
   }
}
