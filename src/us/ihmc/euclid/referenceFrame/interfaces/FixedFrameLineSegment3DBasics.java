package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.geometry.interfaces.LineSegment3DBasics;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

/**
 * Write and read interface for a line segment 3D expressed in a constant reference frame, i.e. this
 * line segment is always expressed in the same reference frame.
 * <p>
 * A line segment 3D is a finite-length line defined in the XY-plane by its two 3D endpoints.
 * </p>
 * <p>
 * In addition to representing a {@link LineSegment3DBasics}, a {@link ReferenceFrame} is associated
 * to a {@code FrameLineSegment3DBasics}. This allows, for instance, to enforce, at runtime, that
 * operations on lines occur in the same coordinate system.
 * </p>
 * <p>
 * Because a {@code FrameLineSegment3DBasics} extends {@code LineSegment3DBasics}, it is compatible
 * with methods only requiring {@code LineSegment3DBasics}. However, these methods do NOT assert
 * that the operation occur in the proper coordinate system. Use this feature carefully and always
 * prefer using methods requiring {@code FrameLineSegment3DBasics}.
 * </p>
 */
public interface FixedFrameLineSegment3DBasics extends FrameLineSegment3DReadOnly, LineSegment3DBasics
{
   /**
    * Gets the reference to the first endpoint of this line segment.
    *
    * @return the reference to the first endpoint of this line segment.
    */
   @Override
   FixedFramePoint3DBasics getFirstEndpoint();

   /**
    * Gets the reference to the second endpoint of this line segment.
    *
    * @return the reference to the second endpoint of this line segment.
    */
   @Override
   FixedFramePoint3DBasics getSecondEndpoint();

   /**
    * Changes the first endpoint of this line segment.
    *
    * @param firstEndpoint new endpoint of this line segment. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code firstEndpoint} are not
    *            expressed in the same reference frame.
    */
   default void setFirstEndpoint(FramePoint3DReadOnly firstEndpoint)
   {
      getFirstEndpoint().set(firstEndpoint);
   }

   /**
    * Changes the second endpoint of this line segment.
    *
    * @param secondEndpoint new second endpoint of this line segment. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code secondEndpoint} are not
    *            expressed in the same reference frame.
    */
   default void setSecondEndpoint(FramePoint3DReadOnly secondEndpoint)
   {
      getSecondEndpoint().set(secondEndpoint);
   }

   /**
    * Sets this line segment to be same as the given line segment.
    *
    * @param other the other line segment to copy. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code other} are not expressed in
    *            the same reference frame.
    */
   default void set(FrameLineSegment3DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      LineSegment3DBasics.super.set(other.getFirstEndpoint(), other.getSecondEndpoint());
   }

   /**
    * Sets this line segment to be the same as the given line segment expressed in the reference frame of this.
    * <p>
    * If {@code other} is expressed in the frame as {@code this}, then this method is equivalent to
    * {@link #set(FrameLineSegment3DReadOnly)}.
    * </p>
    * <p>
    * If {@code other} is expressed in a different frame than {@code this}, then {@code this} is set to
    * {@code other} and then transformed to be expressed in {@code this.getReferenceFrame()}.
    * </p>
    *
    * @param other the other frame line segment to set this to. Not modified.
    */
   default void setMatchingFrame(FrameLineSegment3DReadOnly other)
   {
      LineSegment3DBasics.super.set(other);
      other.getReferenceFrame().transformFromThisToDesiredFrame(getReferenceFrame(), this);
   }

   /**
    * Redefines this line segment with new endpoints.
    *
    * @param firstEndpoint new endpoint of this line segment. Not modified
    * @param secondEndpoint new second endpoint of this line segment. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code firstEndpoint} are not
    *            expressed in the same reference frame.
    */
   default void set(FramePoint3DReadOnly firstEndpoint, Point3DReadOnly secondEndpoint)
   {
      checkReferenceFrameMatch(firstEndpoint);
      LineSegment3DBasics.super.set(firstEndpoint, secondEndpoint);
   }

   /**
    * Redefines this line segment with new endpoints.
    *
    * @param firstEndpoint new endpoint of this line segment. Not modified
    * @param secondEndpoint new second endpoint of this line segment. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code secondEndpoint} are not
    *            expressed in the same reference frame.
    */
   default void set(Point3DReadOnly firstEndpoint, FramePoint3DReadOnly secondEndpoint)
   {
      checkReferenceFrameMatch(secondEndpoint);
      LineSegment3DBasics.super.set(firstEndpoint, secondEndpoint);
   }

   /**
    * Redefines this line segment with new endpoints.
    *
    * @param firstEndpoint new endpoint of this line segment. Not modified
    * @param secondEndpoint new second endpoint of this line segment. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this}, {@code firstEndpoint}, and
    *            {@code secondEndpoint} are not expressed in the same reference frame.
    */
   default void set(FramePoint3DReadOnly firstEndpoint, FramePoint3DReadOnly secondEndpoint)
   {
      checkReferenceFrameMatch(firstEndpoint);
      checkReferenceFrameMatch(secondEndpoint);
      LineSegment3DBasics.super.set(firstEndpoint, secondEndpoint);
   }

   /**
    * Redefines this line segment with a new first endpoint and a vector going from the first to the
    * second endpoint.
    *
    * @param firstEndpoint new first endpoint. Not modified.
    * @param fromFirstToSecondEndpoint vector going from the first to the second endpoint. Not
    *           modified.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code firstEndpoint} are not
    *            expressed in the same reference frame.
    */
   default void set(FramePoint3DReadOnly firstEndpoint, Vector3DReadOnly fromFirstToSecondEndpoint)
   {
      checkReferenceFrameMatch(firstEndpoint);
      LineSegment3DBasics.super.set(firstEndpoint, fromFirstToSecondEndpoint);
   }

   /**
    * Redefines this line segment with a new first endpoint and a vector going from the first to the
    * second endpoint.
    *
    * @param firstEndpoint new first endpoint. Not modified.
    * @param fromFirstToSecondEndpoint vector going from the first to the second endpoint. Not
    *           modified.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code fromFirstToSecondEndpoint} are
    *            not expressed in the same reference frame.
    */
   default void set(Point3DReadOnly firstEndpoint, FrameVector3DReadOnly fromFirstToSecondEndpoint)
   {
      checkReferenceFrameMatch(fromFirstToSecondEndpoint);
      LineSegment3DBasics.super.set(firstEndpoint, fromFirstToSecondEndpoint);
   }

   /**
    * Redefines this line segment with a new first endpoint and a vector going from the first to the
    * second endpoint.
    *
    * @param firstEndpoint new first endpoint. Not modified.
    * @param fromFirstToSecondEndpoint vector going from the first to the second endpoint. Not
    *           modified.
    * @throws ReferenceFrameMismatchException if {@code this}, {@code firstEndpoint}, and
    *            {@code fromFirstToSecondEndpoint} are not expressed in the same reference frame.
    */
   default void set(FramePoint3DReadOnly firstEndpoint, FrameVector3DReadOnly fromFirstToSecondEndpoint)
   {
      checkReferenceFrameMatch(firstEndpoint);
      checkReferenceFrameMatch(fromFirstToSecondEndpoint);
      LineSegment3DBasics.super.set(firstEndpoint, fromFirstToSecondEndpoint);
   }

   /**
    * Translates this line segment by the given (x, y, z) contained in {@code translation}.
    * <p>
    * Note that the length and direction of this line segment remains unchanged.
    * </p>
    *
    * @param translation the translation to add to each endpoint of this line segment. Not modified.
    * @throws ReferenceFrameMismatchException if {@code this} and {@code translation} are not expressed
    *            in the same reference frame.
    */
   default void translate(FrameTuple3DReadOnly translation)
   {
      checkReferenceFrameMatch(translation);
      LineSegment3DBasics.super.translate(translation);
   }
}
