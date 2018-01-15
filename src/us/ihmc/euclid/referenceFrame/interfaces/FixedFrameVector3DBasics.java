package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;

public interface FixedFrameVector3DBasics extends FrameVector3DReadOnly, FixedFrameTuple3DBasics, Vector3DBasics
{

   /**
    * Sets this frame vector to {@code other} and then calls {@link #normalize()}.
    *
    * @param other the other frame vector to copy the values from. Not modified.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same
    *            reference frame as {@code this}.
    */
   default void setAndNormalize(FrameTuple3DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      Vector3DBasics.super.setAndNormalize(other);
   }

   /**
    * Sets this frame vector to the cross product of {@code this} and {@code other}.
    * <p>
    * this = this &times; other
    * </p>
    *
    * @param other the second frame vector in the cross product. Not modified.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same
    *            reference frame as {@code this}.
    */
   default void cross(FrameVector3DReadOnly other)
   {
      cross((FrameTuple3DReadOnly) other);
   }

   /**
    * Sets this frame vector to the cross product of {@code this} and {@code frameTuple3DReadOnly}.
    * <p>
    * this = this &times; frameTuple3DReadOnly
    * </p>
    *
    * @param frameTuple3DReadOnly the second frame tuple in the cross product. Not modified.
    * @throws ReferenceFrameMismatchException if {@code frameTuple3DReadOnly} is not expressed in
    *            the same reference frame as {@code this}.
    */
   default void cross(FrameTuple3DReadOnly frameTuple3DReadOnly)
   {
      checkReferenceFrameMatch(frameTuple3DReadOnly);
      Vector3DBasics.super.cross(frameTuple3DReadOnly);
   }

   /**
    * Sets this frame vector to the cross product of {@code frameVector1} and {@code frameVector2}.
    * <p>
    * this = frameVector1 &times; frameVector2
    * </p>
    *
    * @param frameVector1 the first frame vector in the cross product. Not modified.
    * @param frameVector2 the second frame vector in the cross product. Not modified.
    * @throws ReferenceFrameMismatchException if either {@code frameVector1} or {@code frameVector2}
    *            is not expressed in the same reference frame as {@code this}.
    */
   default void cross(FrameVector3DReadOnly frameVector1, FrameVector3DReadOnly frameVector2)
   {
      cross((FrameTuple3DReadOnly) frameVector1, (FrameTuple3DReadOnly) frameVector2);
   }

   /**
    * Sets this frame vector to the cross product of {@code frameTuple1} and {@code frameTuple2}.
    * <p>
    * this = frameTuple1 &times; frameTuple2
    * </p>
    *
    * @param frameTuple1 the first frame tuple in the cross product. Not modified.
    * @param frameTuple2 the second frame tuple in the cross product. Not modified.
    * @throws ReferenceFrameMismatchException if either {@code frameTuple1} or {@code frameTuple2}
    *            is not expressed in the same reference frame as {@code this}.
    */
   default void cross(FrameTuple3DReadOnly frameTuple1, FrameTuple3DReadOnly frameTuple2)
   {
      checkReferenceFrameMatch(frameTuple1);
      checkReferenceFrameMatch(frameTuple2);
      Vector3DBasics.super.cross(frameTuple1, frameTuple2);
   }

   /**
    * Sets this frame vector to the cross product of {@code frameTuple1} and {@code tuple2}.
    * <p>
    * this = frameTuple1 &times; tuple2
    * </p>
    *
    * @param frameTuple1 the first frame tuple in the cross product. Not modified.
    * @param tuple2 the second tuple in the cross product. Not modified.
    * @throws ReferenceFrameMismatchException if {@code frameTuple1} is not expressed in the same
    *            reference frame as {@code this}.
    */
   default void cross(FrameTuple3DReadOnly frameTuple1, Tuple3DReadOnly tuple2)
   {
      checkReferenceFrameMatch(frameTuple1);
      Vector3DBasics.super.cross(frameTuple1, tuple2);
   }

   /**
    * Sets this frame vector to the cross product of {@code tuple1} and {@code frameTuple2}.
    * <p>
    * this = tuple1 &times; frameTuple2
    * </p>
    *
    * @param tuple1 the first tuple in the cross product. Not modified.
    * @param frameTuple2 the second frame tuple in the cross product. Not modified.
    * @throws ReferenceFrameMismatchException if {@code frameTuple2} is not expressed in the same
    *            reference frame as {@code this}.
    */
   default void cross(Tuple3DReadOnly frameTuple1, FrameTuple3DReadOnly frameTuple2)
   {
      checkReferenceFrameMatch(frameTuple2);
      Vector3DBasics.super.cross(frameTuple1, frameTuple2);
   }

}
