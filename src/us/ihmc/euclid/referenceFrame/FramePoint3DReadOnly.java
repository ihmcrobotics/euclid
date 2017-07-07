package us.ihmc.euclid.referenceFrame;

import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

public interface FramePoint3DReadOnly extends Point3DReadOnly, FrameTuple3DReadOnly, ReferenceFrameHolder
{
   /**
    * Calculates and returns the distance between this frame point and {@code other}.
    *
    * @param other the other frame point used to measure the distance.
    * @return the distance between the two frame points.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same frame as
    *            {@code this}.
    */
   default double distance(FramePoint3DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      return Point3DReadOnly.super.distance(other);
   }

   /**
    * Calculates and returns the square of the distance between this frame point and {@code other}.
    * <p>
    * This method is usually preferred over {@link #distance(FramePoint3DReadOnly)} when calculation
    * speed matters and knowledge of the actual distance does not, i.e. when comparing distances
    * between several pairs of frame points.
    * </p>
    *
    * @param other the other frame point used to measure the square of the distance.
    * @return the square of the distance between the two frame points.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same frame as
    *            {@code this}.
    */
   default double distanceSquared(FramePoint3DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      return Point3DReadOnly.super.distanceSquared(other);
   }

   /**
    * Calculates and returns the distance between this frame point and {@code other} in the
    * XY-plane.
    * <p>
    * Effectively, this calculates the distance as follows:<br>
    * d<sub>xy</sub> = &radic;((this.x - other.x)<sup>2</sup> + (this.y - other.y)<sup>2</sup>)
    * </p>
    *
    * @param other the other frame point used to measure the distance.
    * @return the distance between the two frame points in the XY-plane.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same frame as
    *            {@code this}.
    */
   default double distanceXY(FramePoint3DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      return Point3DReadOnly.super.distanceXY(other);
   }

   /**
    * Calculates and returns the square of the distance between this frame point and {@code other}
    * in the XY-plane.
    * <p>
    * Effectively, this calculates the distance squared as follows:<br>
    * d<sub>xy</sub><sup>2</sup> = (this.x - other.x)<sup>2</sup> + (this.y - other.y)<sup>2</sup>
    * </p>
    * <p>
    * This method is usually preferred over {@link #distanceXY(FramePoint3DReadOnly)} when
    * calculation speed matters and knowledge of the actual distance does not, i.e. when comparing
    * distances between several pairs of frame points.
    * </p>
    *
    * @param other the other frame point used to measure the square of the distance.
    * @return the square of the distance between the two frame points in the XY-plane.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same frame as
    *            {@code this}.
    */
   default double distanceXYSquared(FramePoint3DReadOnly other)
   {
      checkReferenceFrameMatch(other);
      return Point3DReadOnly.super.distanceXYSquared(other);
   }

   /**
    * Calculates and returns the distance between this frame point and {@code framePoint2DReadOnly}
    * in the XY-plane.
    * <p>
    * Effectively, this calculates the distance as follows:<br>
    * d<sub>xy</sub> = &radic;((this.x - framePoint2DReadOnly.x)<sup>2</sup> + (this.y -
    * framePoint2DReadOnly.y)<sup>2</sup>)
    * </p>
    *
    * @param framePoint2DReadOnly the other frame point used to measure the distance.
    * @return the distance between the two frame points in the XY-plane.
    * @throws ReferenceFrameMismatchException if {@code framePoint2DReadOnly} is not expressed in
    *            the same frame as {@code this}.
    */
   default double distanceXY(FramePoint2DReadOnly framePoint2DReadOnly)
   {
      checkReferenceFrameMatch(framePoint2DReadOnly);
      return Point3DReadOnly.super.distanceXY(framePoint2DReadOnly);
   }

   /**
    * Calculates and returns the square of the distance between this frame point and
    * {@code framePoint2DReadOnly} in the XY-plane.
    * <p>
    * Effectively, this calculates the distance squared as follows:<br>
    * d<sub>xy</sub><sup>2</sup> = (this.x - framePoint2DReadOnly.x)<sup>2</sup> + (this.y -
    * framePoint2DReadOnly.y)<sup>2</sup>
    * </p>
    * <p>
    * This method is usually preferred over {@link #distanceXY(FramePoint2DReadOnly)} when
    * calculation speed matters and knowledge of the actual distance does not, i.e. when comparing
    * distances between several pairs of points.
    * </p>
    *
    * @param framePoint2DReadOnly the other frame point used to measure the square of the distance.
    * @return the square of the distance between the two frame points in the XY-plane.
    * @throws ReferenceFrameMismatchException if {@code framePoint2DReadOnly} is not expressed in
    *            the same frame as {@code this}.
    */
   default double distanceXYSquared(FramePoint2DReadOnly framePoint2DReadOnly)
   {
      checkReferenceFrameMatch(framePoint2DReadOnly);
      return Point3DReadOnly.super.distanceXYSquared(framePoint2DReadOnly);
   }
}
