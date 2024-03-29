package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.orientation.interfaces.Orientation3DBasics;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameIOTools;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;

/**
 * Read-only interface for a 3D pose expressed in a given reference frame.
 * <p>
 * In addition to representing a {@link Pose3DReadOnly}, a {@link ReferenceFrame} is associated to a
 * {@code FramePose3DReadOnly}. This allows, for instance, to enforce, at runtime, that operations
 * on poses occur in the same coordinate system.
 * </p>
 * <p>
 * Because a {@code FramePose3DReadOnly} extends {@code Pose3DReadOnly}, it is compatible with
 * methods only requiring {@code Pose3DReadOnly}. However, these methods do NOT assert that the
 * operation occur in the proper coordinate system. Use this feature carefully and always prefer
 * using methods requiring {@code FramePose3DReadOnly}.
 * </p>
 */
public interface FramePose3DReadOnly extends Pose3DReadOnly, EuclidFrameGeometry
{
   /** {@inheritDoc} */
   @Override
   FramePoint3DReadOnly getPosition();

   /** {@inheritDoc} */
   @Override
   FrameQuaternionReadOnly getOrientation();

   /** {@inheritDoc} */
   @Override
   default FramePoint3DReadOnly getTranslation()
   {
      return getPosition();
   }

   /** {@inheritDoc} */
   @Override
   default FrameQuaternionReadOnly getRotation()
   {
      return getOrientation();
   }

   /**
    * Gets the position and orientation.
    *
    * @param positionToPack    the tuple used to store the position. Modified.
    * @param orientationToPack the orientation used to store the orientation. Modified.
    * @throws ReferenceFrameMismatchException if {@code positionToPack} and/or
    *                                         {@code orientationToPack} are not expressed in the same
    *                                         reference frame as this frame pose.
    */
   default void get(FixedFrameTuple3DBasics positionToPack, FixedFrameOrientation3DBasics orientationToPack)
   {
      positionToPack.set(getPosition());
      orientationToPack.set(getOrientation());
   }

   /**
    * Gets the position and orientation.
    *
    * @param positionToPack    the tuple used to store the position. Modified.
    * @param orientationToPack the orientation used to store the orientation. Modified.
    */
   default void get(FrameTuple3DBasics positionToPack, FrameOrientation3DBasics orientationToPack)
   {
      positionToPack.setIncludingFrame(getPosition());
      orientationToPack.setIncludingFrame(getOrientation());
   }

   /**
    * Gets the position and orientation.
    *
    * @param positionToPack    the tuple used to store the position. Modified.
    * @param orientationToPack the orientation used to store the orientation. Modified.
    * @throws ReferenceFrameMismatchException if {@code positionToPack} is not expressed in the same
    *                                         reference frame as this frame pose.
    */
   default void get(FixedFrameTuple3DBasics positionToPack, Orientation3DBasics orientationToPack)
   {
      positionToPack.set(getPosition());
      orientationToPack.set(getOrientation());
   }

   /**
    * Gets the position and orientation.
    *
    * @param positionToPack    the tuple used to store the position. Modified.
    * @param orientationToPack the orientation used to store the orientation. Modified.
    */
   default void get(FrameTuple3DBasics positionToPack, Orientation3DBasics orientationToPack)
   {
      positionToPack.setIncludingFrame(getPosition());
      orientationToPack.set(getOrientation());
   }

   /**
    * Gets the position and orientation.
    *
    * @param positionToPack    the tuple used to store the position. Modified.
    * @param orientationToPack the orientation used to store the orientation. Modified.
    * @throws ReferenceFrameMismatchException if {@code orientationToPack} is not expressed in the same
    *                                         reference frame as this frame pose.
    */
   default void get(Tuple3DBasics positionToPack, FixedFrameOrientation3DBasics orientationToPack)
   {
      positionToPack.set(getPosition());
      orientationToPack.set(getOrientation());
   }

   /**
    * Gets the position and orientation.
    *
    * @param positionToPack    the tuple used to store the position. Modified.
    * @param orientationToPack the orientation used to store the orientation. Modified.
    */
   default void get(Tuple3DBasics positionToPack, FrameOrientation3DBasics orientationToPack)
   {
      positionToPack.set(getPosition());
      orientationToPack.setIncludingFrame(getOrientation());
   }

   /**
    * Computes and packs the orientation described by the orientation part of this pose as a rotation
    * vector.
    * <p>
    * WARNING: a rotation vector is different from a yaw-pitch-roll or Euler angles representation. A
    * rotation vector is equivalent to the axis of an axis-angle that is multiplied by the angle of the
    * same axis-angle.
    * </p>
    *
    * @param rotationVectorToPack the vector in which the rotation vector is stored. Modified.
    * @throws ReferenceFrameMismatchException if {@code rotationVectorToPack} is not expressed in the
    *                                         same reference frame as {@code this}.
    */
   default void getRotationVector(FixedFrameVector3DBasics rotationVectorToPack)
   {
      getOrientation().getRotationVector(rotationVectorToPack);
   }

   /**
    * Computes and packs the orientation described by the orientation part of this pose as a rotation
    * vector.
    * <p>
    * WARNING: a rotation vector is different from a yaw-pitch-roll or Euler angles representation. A
    * rotation vector is equivalent to the axis of an axis-angle that is multiplied by the angle of the
    * same axis-angle.
    * </p>
    *
    * @param rotationVectorToPack the vector in which the rotation vector is stored. Modified.
    */
   default void getRotationVector(FrameVector3DBasics rotationVectorToPack)
   {
      getOrientation().getRotationVector(rotationVectorToPack);
   }

   /**
    * Computes the distance between the position of this pose 3D and the given {@code point}.
    *
    * @param point the other point used to measure the distance. Not modified.
    * @return the distance between this pose and the given {@code point}.
    * @throws ReferenceFrameMismatchException if {@code point} is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default double getPositionDistance(FramePoint3DReadOnly point)
   {
      return getPosition().distance(point);
   }

   /**
    * Computes the distances between the position part of the two poses.
    *
    * @param other the other pose used to measure the distance. Not modified.
    * @return the distance between the position part of the two poses.
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default double getPositionDistance(FramePose3DReadOnly other)
   {
      return getPositionDistance(other.getPosition());
   }

   /**
    * Computes the smallest angle representing the difference between the orientation part of this pose
    * 3D and the give {@code orientation}.
    *
    * @param orientation the orientation used to compute the orientation distance. Not modified.
    * @return the angle difference between {@code this} and {@code orientation}, it is contained in [0,
    *         2<i>pi</i>].
    * @throws ReferenceFrameMismatchException if {@code orientation} is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default double getOrientationDistance(FrameQuaternionReadOnly orientation)
   {
      return getOrientation().distance(orientation);
   }

   /**
    * Computes the absolute angle difference between this pose 3D and {@code other}.
    *
    * @param other the other pose 3D used to compute the orientation distance. Not modified.
    * @return the angle difference between {@code this.orientation} and {@code other.orientation}, it
    *         is contained in [0, 2<i>pi</i>].
    * @throws ReferenceFrameMismatchException if {@code other} is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default double getOrientationDistance(FramePose3DReadOnly other)
   {
      return getOrientationDistance(other.getOrientation());
   }

   /**
    * Gets a representative {@code String} of this pose 3D given a specific format to use.
    * <p>
    * Using the default format {@link EuclidCoreIOTools#DEFAULT_FORMAT}, this provides a {@code String} as follows:
    *
    * <pre>
    * Pose 3D: position = ( 0.174, -0.452, -0.222 ), orientation = (-0.052, -0.173, -0.371,  0.087 ), worldFrame
    * </pre>
    * </p>
    */
   @Override
   default String toString(String format)
   {
      return EuclidFrameIOTools.getFramePose3DString(format, this);
   }
}
