package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.geometry.interfaces.LineSegment2DBasics;
import us.ihmc.euclid.geometry.interfaces.LineSegment2DReadOnly;
import us.ihmc.euclid.geometry.interfaces.LineSegment3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

/**
 * Write and read interface for a line segment 2D expressed in a changeable reference frame, i.e.
 * the reference frame in which this line is expressed can be changed.
 * <p>
 * A line segment 2D is a finite-length line defined in the XY-plane by its two 2D endpoints.
 * </p>
 * <p>
 * In addition to representing a {@link LineSegment2DBasics}, a {@link ReferenceFrame} is associated
 * to a {@code FrameLineSegment2DBasics}. This allows, for instance, to enforce, at runtime, that
 * operations on lines occur in the same coordinate system.
 * </p>
 * <p>
 * Because a {@code FrameLineSegment2DBasics} extends {@code LineSegment2DBasics}, it is compatible
 * with methods only requiring {@code LineSegment2DBasics}. However, these methods do NOT assert
 * that the operation occur in the proper coordinate system. Use this feature carefully and always
 * prefer using methods requiring {@code FrameLineSegment2DBasics}.
 * </p>
 */
public interface FrameLineSegment2DBasics extends FixedFrameLineSegment2DBasics, FrameChangeable
{
   /**
    * Sets the reference frame of this line segment 2D without updating or modifying either of its
    * endpoints.
    *
    * @param referenceFrame the new reference frame for this frame line segment 2D.
    */
   @Override
   void setReferenceFrame(ReferenceFrame referenceFrame);

   /**
    * Sets the point and direction parts of this line segment 2D to zero and sets the current reference
    * frame to {@code referenceFrame}.
    *
    * @param referenceFrame the new reference frame to be associated with this line segment 2D.
    */
   default void setToZero(ReferenceFrame referenceFrame)
   {
      setReferenceFrame(referenceFrame);
      setToZero();
   }

   /**
    * Sets the point and direction parts of this line segment 2D to {@link Double#NaN} and sets the
    * current reference frame to {@code referenceFrame}.
    *
    * @param referenceFrame the new reference frame to be associated with this line segment 2D.
    */
   default void setToNaN(ReferenceFrame referenceFrame)
   {
      setReferenceFrame(referenceFrame);
      setToNaN();
   }

   /**
    * Sets this line segment to be the same as the given line segment including its reference frame.
    *
    * @param other the other line segment to copy. Not modified.
    */
   default void setIncludingFrame(FrameLineSegment2DReadOnly other)
   {
      setIncludingFrame(other.getReferenceFrame(), other);
   }

   /**
    * Sets this line segment to be the same as the given line segment including the given reference
    * frame.
    *
    * @param referenceFrame        the reference frame in which the given line segment is expressed.
    * @param lineSegment2DReadOnly the line segment to copy. Not modified.
    */
   default void setIncludingFrame(ReferenceFrame referenceFrame, LineSegment2DReadOnly lineSegment2DReadOnly)
   {
      setReferenceFrame(referenceFrame);
      set(lineSegment2DReadOnly);
   }

   /**
    * Sets this line segment to be same as the given line segment including its reference frame.
    *
    * @param other the other line segment to copy. Not modified.
    */
   default void setIncludingFrame(FrameLineSegment3DReadOnly other)
   {
      setIncludingFrame(other.getReferenceFrame(), other);
   }

   /**
    * Sets this line segment to be the same as the given line segment projected on the XY-plane.
    *
    * @param referenceFrame        the reference frame in which the given line segment is expressed.
    * @param lineSegment3DReadOnly the other line segment to copy. Not modified.
    */
   default void setIncludingFrame(ReferenceFrame referenceFrame, LineSegment3DReadOnly lineSegment3DReadOnly)
   {
      setReferenceFrame(referenceFrame);
      set(lineSegment3DReadOnly);
   }

   /**
    * Redefines this line segment with new endpoints.
    *
    * @param firstEndpoint  new endpoint of this line segment. Not modified
    * @param secondEndpoint new second endpoint of this line segment. Not modified.
    * @throws ReferenceFrameMismatchException if {@code firstEndpoint} and {@code secondEndpoint} are
    *                                         not expressed in the same reference frame.
    */
   default void setIncludingFrame(FramePoint2DReadOnly firstEndpoint, FramePoint2DReadOnly secondEndpoint)
   {
      firstEndpoint.checkReferenceFrameMatch(secondEndpoint);
      setIncludingFrame(firstEndpoint.getReferenceFrame(), firstEndpoint, secondEndpoint);
   }

   /**
    * Redefines this line segment with new endpoints.
    *
    * @param referenceFrame the reference frame in which the arguments are expressed.
    * @param firstEndpoint  new endpoint of this line segment. Not modified
    * @param secondEndpoint new second endpoint of this line segment. Not modified.
    */
   default void setIncludingFrame(ReferenceFrame referenceFrame, Point2DReadOnly firstEndpoint, Point2DReadOnly secondEndpoint)
   {
      setReferenceFrame(referenceFrame);
      set(firstEndpoint, secondEndpoint);
   }

   /**
    * Redefines this line segment with new endpoints.
    *
    * @param firstEndpoint  new endpoint of this line segment. Not modified
    * @param secondEndpoint new second endpoint of this line segment. Not modified.
    * @throws ReferenceFrameMismatchException if {@code firstEndpoint} and {@code secondEndpoint} are
    *                                         not expressed in the same reference frame.
    */
   default void setIncludingFrame(FramePoint3DReadOnly firstEndpoint, FramePoint3DReadOnly secondEndpoint)
   {
      firstEndpoint.checkReferenceFrameMatch(secondEndpoint);
      setIncludingFrame(firstEndpoint.getReferenceFrame(), firstEndpoint, secondEndpoint);
   }

   /**
    * Redefines this line segment with new endpoints.
    *
    * @param referenceFrame the reference frame in which the arguments are expressed.
    * @param firstEndpoint  new endpoint of this line segment. Not modified
    * @param secondEndpoint new second endpoint of this line segment. Not modified.
    */
   default void setIncludingFrame(ReferenceFrame referenceFrame, Point3DReadOnly firstEndpoint, Point3DReadOnly secondEndpoint)
   {
      setReferenceFrame(referenceFrame);
      set(firstEndpoint, secondEndpoint);
   }

   /**
    * Redefines this line segment with a new first endpoint and a vector going from the first to the
    * second endpoint.
    *
    * @param firstEndpoint             new first endpoint. Not modified.
    * @param fromFirstToSecondEndpoint vector going from the first to the second endpoint. Not
    *                                  modified.
    * @throws ReferenceFrameMismatchException if {@code firstEndpoint} and
    *                                         {@code fromFirstToSecondEndpoint} are not expressed in
    *                                         the same reference frame.
    */
   default void setIncludingFrame(FramePoint2DReadOnly firstEndpoint, FrameVector2DReadOnly fromFirstToSecondEndpoint)
   {
      firstEndpoint.checkReferenceFrameMatch(fromFirstToSecondEndpoint);
      setIncludingFrame(firstEndpoint.getReferenceFrame(), firstEndpoint, fromFirstToSecondEndpoint);
   }

   /**
    * Redefines this line segment with a new first endpoint and a vector going from the first to the
    * second endpoint.
    *
    * @param referenceFrame            the reference frame in which the arguments are expressed.
    * @param firstEndpoint             new first endpoint. Not modified.
    * @param fromFirstToSecondEndpoint vector going from the first to the second endpoint. Not
    *                                  modified.
    */
   default void setIncludingFrame(ReferenceFrame referenceFrame, Point2DReadOnly firstEndpoint, Vector2DReadOnly fromFirstToSecondEndpoint)
   {
      setReferenceFrame(referenceFrame);
      set(firstEndpoint, fromFirstToSecondEndpoint);
   }

   /**
    * Redefines this line segment with a new first endpoint and a vector going from the first to the
    * second endpoint.
    *
    * @param firstEndpoint             new first endpoint. Not modified.
    * @param fromFirstToSecondEndpoint vector going from the first to the second endpoint. Not
    *                                  modified.
    * @throws ReferenceFrameMismatchException if {@code firstEndpoint} and
    *                                         {@code fromFirstToSecondEndpoint} are not expressed in
    *                                         the same reference frame.
    */
   default void setIncludingFrame(FramePoint3DReadOnly firstEndpoint, FrameVector3DReadOnly fromFirstToSecondEndpoint)
   {
      firstEndpoint.checkReferenceFrameMatch(fromFirstToSecondEndpoint);
      setIncludingFrame(firstEndpoint.getReferenceFrame(), firstEndpoint, fromFirstToSecondEndpoint);
   }

   /**
    * Redefines this line segment with a new first endpoint and a vector going from the first to the
    * second endpoint.
    *
    * @param referenceFrame            the reference frame in which the arguments are expressed.
    * @param firstEndpoint             new first endpoint. Not modified.
    * @param fromFirstToSecondEndpoint vector going from the first to the second endpoint. Not
    *                                  modified.
    */
   default void setIncludingFrame(ReferenceFrame referenceFrame, Point3DReadOnly firstEndpoint, Vector3DReadOnly fromFirstToSecondEndpoint)
   {
      setReferenceFrame(referenceFrame);
      set(firstEndpoint, fromFirstToSecondEndpoint);
   }

   /**
    * Performs a transformation of the line segment such that it is expressed in a new frame
    * {@code desireFrame}.
    * <p>
    * Because the transformation between two reference frames is a 3D transformation, the result of
    * transforming this line segment 2D can result in a line segment 3D. This method projects the
    * result of the transformation onto the XY-plane.
    * </p>
    *
    * @param desiredFrame the reference frame in which the line segment is to be expressed.
    */
   void changeFrameAndProjectToXYPlane(ReferenceFrame desiredFrame);
}
