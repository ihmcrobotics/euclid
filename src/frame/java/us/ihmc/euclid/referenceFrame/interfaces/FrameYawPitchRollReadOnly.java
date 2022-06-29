package us.ihmc.euclid.referenceFrame.interfaces;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameIOTools;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.yawPitchRoll.interfaces.YawPitchRollReadOnly;

/**
 * Read-only interface for a yaw-pitch-roll object expressed in a given reference frame.
 * <p>
 * In addition to representing a {@link YawPitchRollReadOnly}, a {@link ReferenceFrame} is
 * associated to a {@code FrameYawPitchRollReadOnly}. This allows, for instance, to enforce, at
 * runtime, that operations on yaw-pitch-rolls occur in the same coordinate system. Also, via the
 * method {@link FrameChangeable#changeFrame(ReferenceFrame)}, one can easily calculates the value
 * of a yaw-pitch-roll in different reference frames.
 * </p>
 * <p>
 * Because a {@code FrameYawPitchRollReadOnly} extends {@code YawPitchRollReadOnly}, it is
 * compatible with methods only requiring {@code YawPitchRollReadOnly}. However, these methods do
 * NOT assert that the operation occur in the proper coordinate system. Use this feature carefully
 * and always prefer using methods requiring {@code FrameYawPitchRollReadOnly}.
 * </p>
 * <p>
 * Equivalent representation of yaw-pitch-roll as 3-by-3 rotation matrix:
 *
 * <pre>
 *     / cos(yaw) -sin(yaw) 0 \   /  cos(pitch) 0 sin(pitch) \   / 1     0          0     \
 * R = | sin(yaw)  cos(yaw) 0 | * |      0      1     0      | * | 0 cos(roll) -sin(roll) |
 *     \    0         0     1 /   \ -sin(pitch) 0 cos(pitch) /   \ 0 sin(roll)  cos(roll) /
 * </pre>
 * </p>
 *
 * @author Sylvain Bertrand
 */
public interface FrameYawPitchRollReadOnly extends FrameOrientation3DReadOnly, YawPitchRollReadOnly
{
   /**
    * Gets a representative {@code String} of this yaw-pitch-roll given a specific format to use.
    * <p>
    * Using the default format {@link EuclidCoreIOTools#DEFAULT_FORMAT}, this provides a {@code String}
    * as follows:
    *
    * <pre>
    * yaw-pitch-roll: ( 0.674,  0.455,  0.582 ) - worldFrame
    * </pre>
    * </p>
    */
   @Override
   default String toString(String format)
   {
      return EuclidFrameIOTools.getFrameYawPitchRollString(format, this);
   }
}
