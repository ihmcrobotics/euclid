package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;

/**
 * Read-only interface for a 2D point expressed in a given reference frame.
 * <p>
 * In addition to representing a {@link Point2DReadOnly}, a {@link ReferenceFrame} is associated to
 * a {@code FramePoint2DReadOnly}. This allows, for instance, to enforce, at runtime, that
 * operations on points occur in the same coordinate system.
 * </p>
 * <p>
 * Because a {@code FramePoint2DReadOnly} extends {@code Point2DReadOnly}, it is compatible with
 * methods only requiring {@code Point2DReadOnly}. However, these methods do NOT assert that the
 * operation occur in the proper coordinate system. Use this feature carefully and always prefer
 * using methods requiring {@code FramePoint2DReadOnly}.
 * </p>
 */
public interface FramePoint2DReadOnly extends Point2DReadOnly, FrameTuple2DReadOnly
{
   /**
    * Calculates and returns the distance between this frame point and {@code other}.
    *
    * @param other the other frame point used to measure the distance. Not modified.
    * @return the distance between the two frame points.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same frame as
    *            {@code this}.
    */
   default double distance(FramePoint2DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      return Point2DReadOnly.super.distance(other);
   }

   /**
    * Calculates and returns the square of the distance between this frame point and {@code other}.
    * <p>
    * This method is usually preferred over {@link #distance(FramePoint2DReadOnly)} when calculation
    * speed matters and knowledge of the actual distance does not, i.e. when comparing distances
    * between several pairs of frame points.
    * </p>
    *
    * @param other the other frame point used to measure the square of the distance. Not modified.
    * @return the square of the distance between the two frame points.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same frame as
    *            {@code this}.
    */
   default double distanceSquared(FramePoint2DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      return Point2DReadOnly.super.distanceSquared(other);
   }

   /**
    * Calculates and returns the distance between this frame point and {@code framePoint3DReadOnly} in
    * the XY-plane.
    * <p>
    * Effectively, this calculates the distance as follows:<br>
    * d<sub>xy</sub> = &radic;((this.x - framePoint3DReadOnly.x)<sup>2</sup> + (this.y -
    * framePoint3DReadOnly.y)<sup>2</sup>)
    * </p>
    *
    * @param framePoint3DReadOnly the other frame point used to measure the distance.
    * @return the distance between the two frame points in the XY-plane.
    * @throws ReferenceFrameMismatchException if {@code framePoint3DReadOnly} is not expressed in the
    *            same frame as {@code this}.
    */
   default double distanceXY(FramePoint3DReadOnly framePoint3DReadOnly)
   {
      checkReferenceFrameMatch(framePoint3DReadOnly);
      return Point2DReadOnly.super.distanceXY(framePoint3DReadOnly);
   }

   /**
    * Calculates and returns the square of the distance between this frame point and
    * {@code framePoint3DReadOnly} in the XY-plane.
    * <p>
    * Effectively, this calculates the distance squared as follows:<br>
    * d<sub>xy</sub><sup>2</sup> = (this.x - framePoint3DReadOnly.x)<sup>2</sup> + (this.y -
    * framePoint3DReadOnly.y)<sup>2</sup>
    * </p>
    * <p>
    * This method is usually preferred over {@link #distanceXY(FramePoint3DReadOnly)} when calculation
    * speed matters and knowledge of the actual distance does not, i.e. when comparing distances
    * between several pairs of points.
    * </p>
    *
    * @param framePoint3DReadOnly the frame other point used to measure the square of the distance.
    * @return the square of the distance between the two frame points in the XY-plane.
    * @throws ReferenceFrameMismatchException if {@code framePoint3DReadOnly} is not expressed in the
    *            same frame as {@code this}.
    */
   default double distanceXYSquared(FramePoint3DReadOnly framePoint3DReadOnly)
   {
      checkReferenceFrameMatch(framePoint3DReadOnly);
      return Point2DReadOnly.super.distanceXYSquared(framePoint3DReadOnly);
   }

   /**
    * Compares {@code this} to {@code other} to determine if the two frame points are geometrically
    * similar, i.e. the distance between them is less than or equal to {@code epsilon}.
    *
    * @param other the frame point to compare to. Not modified.
    * @param epsilon the tolerance of the comparison.
    * @return {@code true} if the two frame points represent the same geometry, {@code false}
    *         otherwise.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same reference
    *            frame as {@code this}.
    */
   default boolean geometricallyEquals(FramePoint2DReadOnly other, double epsilon)
   {
      checkReferenceFrameMatch(other);
      return Point2DReadOnly.super.geometricallyEquals(other, epsilon);
   }
}
