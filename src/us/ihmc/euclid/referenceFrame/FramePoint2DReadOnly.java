package us.ihmc.euclid.referenceFrame;

import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;

public interface FramePoint2DReadOnly extends Point2DReadOnly, FrameTuple2DReadOnly, ReferenceFrameHolder
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
    * Calculates and returns the distance between this frame point and {@code framePoint3DReadOnly}
    * in the XY-plane.
    * <p>
    * Effectively, this calculates the distance as follows:<br>
    * d<sub>xy</sub> = &radic;((this.x - framePoint3DReadOnly.x)<sup>2</sup> + (this.y -
    * framePoint3DReadOnly.y)<sup>2</sup>)
    * </p>
    *
    * @param framePoint3DReadOnly the other frame point used to measure the distance.
    * @return the distance between the two frame points in the XY-plane.
    * @throws ReferenceFrameMismatchException if {@code framePoint3DReadOnly} is not expressed in
    *            the same frame as {@code this}.
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
    * This method is usually preferred over {@link #distanceXY(FramePoint3DReadOnly)} when
    * calculation speed matters and knowledge of the actual distance does not, i.e. when comparing
    * distances between several pairs of points.
    * </p>
    *
    * @param framePoint3DReadOnly the frame other point used to measure the square of the distance.
    * @return the square of the distance between the two frame points in the XY-plane.
    * @throws ReferenceFrameMismatchException if {@code framePoint3DReadOnly} is not expressed in
    *            the same frame as {@code this}.
    */
   default double distanceXYSquared(FramePoint3DReadOnly framePoint3DReadOnly)
   {
      checkReferenceFrameMatch(framePoint3DReadOnly);
      return Point2DReadOnly.super.distanceXYSquared(framePoint3DReadOnly);
   }
}
