package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.geometry.interfaces.Pose2DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameIOTools;

/**
 * Read-only interface for a 2D pose expressed in a given reference frame.
 * <p>
 * In addition to representing a {@link Pose2DReadOnly}, a {@link ReferenceFrame} is associated to a
 * {@code FramePose2DReadOnly}. This allows, for instance, to enforce, at runtime, that operations
 * on poses occur in the same coordinate system.
 * </p>
 * <p>
 * Because a {@code FramePose2DReadOnly} extends {@code Pose2DReadOnly}, it is compatible with
 * methods only requiring {@code Pose2DReadOnly}. However, these methods do NOT assert that the
 * operation occur in the proper coordinate system. Use this feature carefully and always prefer
 * using methods requiring {@code FramePose2DReadOnly}.
 * </p>
 */
public interface FramePose2DReadOnly extends Pose2DReadOnly, EuclidFrameGeometry
{
   /** {@inheritDoc} */
   @Override
   FramePoint2DReadOnly getPosition();

   /** {@inheritDoc} */
   @Override
   FrameOrientation2DReadOnly getOrientation();

   /**
    * Computes the distances between the position part of the two poses.
    *
    * @param other the other pose used to measure the distance. Not modified.
    * @return the distance between the position part of the two poses.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default double getPositionDistance(FramePose2DReadOnly other)
   {
      return getPositionDistance(other.getPosition());
   }

   /**
    * Computes the distance between the position of this pose 2D and the given {@code point}.
    *
    * @param point the other point used to measure the distance. Not modified.
    * @return the distance between this pose and the given {@code point}.
    * @throws ReferenceFrameMismatchException if {@code point} is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default double getPositionDistance(FramePoint2DReadOnly point)
   {
      return getPosition().distance(point);
   }

   /**
    * Computes the absolute angle difference between the orientation part of this pose 2D and the give
    * {@code orientation}.
    *
    * @param orientation the orientation used to compute the orientation distance. Not modified.
    * @return the absolute angle difference between {@code this} and {@code orientation}.
    * @throws ReferenceFrameMismatchException if {@code orientation} is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default double getOrientationDistance(FrameOrientation2DReadOnly orientation)
   {
      return getOrientation().distance(orientation);
   }

   /**
    * Computes the absolute angle difference between this pose 2D and {@code other}.
    *
    * @param other the other pose 2D used to compute the orientation distance. Not modified.
    * @return the absolute angle difference between {@code this.orientation} and
    *         {@code other.orientation}.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default double getOrientationDistance(FramePose2DReadOnly other)
   {
      return getOrientationDistance(other.getOrientation());
   }

   @Override
   default String toString(String format)
   {
      return EuclidFrameIOTools.getFramePose2DString(format, this);
   }
}
