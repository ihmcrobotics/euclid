package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

/**
 * Read-only interface for a 3D point expressed in a given reference frame.
 * <p>
 * In addition to representing a {@link Point3DReadOnly}, a {@link ReferenceFrame} is associated to
 * a {@code FramePoint3DReadOnly}. This allows, for instance, to enforce, at runtime, that
 * operations on points occur in the same coordinate system.
 * </p>
 * <p>
 * Because a {@code FramePoint3DReadOnly} extends {@code Point3DReadOnly}, it is compatible with
 * methods only requiring {@code Point3DReadOnly}. However, these methods do NOT assert that the
 * operation occur in the proper coordinate system. Use this feature carefully and always prefer
 * using methods requiring {@code FramePoint3DReadOnly}.
 * </p>
 */
public interface FramePoint3DReadOnly extends Point3DReadOnly, FrameTuple3DReadOnly
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
    * Calculates and returns the distance between this frame point and {@code other} in the XY-plane.
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
    * Calculates and returns the square of the distance between this frame point and {@code other} in
    * the XY-plane.
    * <p>
    * Effectively, this calculates the distance squared as follows:<br>
    * d<sub>xy</sub><sup>2</sup> = (this.x - other.x)<sup>2</sup> + (this.y - other.y)<sup>2</sup>
    * </p>
    * <p>
    * This method is usually preferred over {@link #distanceXY(FramePoint3DReadOnly)} when calculation
    * speed matters and knowledge of the actual distance does not, i.e. when comparing distances
    * between several pairs of frame points.
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
    * Calculates and returns the distance between this frame point and {@code framePoint2DReadOnly} in
    * the XY-plane.
    * <p>
    * Effectively, this calculates the distance as follows:<br>
    * d<sub>xy</sub> = &radic;((this.x - framePoint2DReadOnly.x)<sup>2</sup> + (this.y -
    * framePoint2DReadOnly.y)<sup>2</sup>)
    * </p>
    *
    * @param framePoint2DReadOnly the other frame point used to measure the distance.
    * @return the distance between the two frame points in the XY-plane.
    * @throws ReferenceFrameMismatchException if {@code framePoint2DReadOnly} is not expressed in the
    *            same frame as {@code this}.
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
    * This method is usually preferred over {@link #distanceXY(FramePoint2DReadOnly)} when calculation
    * speed matters and knowledge of the actual distance does not, i.e. when comparing distances
    * between several pairs of points.
    * </p>
    *
    * @param framePoint2DReadOnly the other frame point used to measure the square of the distance.
    * @return the square of the distance between the two frame points in the XY-plane.
    * @throws ReferenceFrameMismatchException if {@code framePoint2DReadOnly} is not expressed in the
    *            same frame as {@code this}.
    */
   default double distanceXYSquared(FramePoint2DReadOnly framePoint2DReadOnly)
   {
      checkReferenceFrameMatch(framePoint2DReadOnly);
      return Point3DReadOnly.super.distanceXYSquared(framePoint2DReadOnly);
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
   default boolean geometricallyEquals(FramePoint3DReadOnly other, double epsilon)
   {
      checkReferenceFrameMatch(other);
      return Point3DReadOnly.super.geometricallyEquals(other, epsilon);
   }
}
