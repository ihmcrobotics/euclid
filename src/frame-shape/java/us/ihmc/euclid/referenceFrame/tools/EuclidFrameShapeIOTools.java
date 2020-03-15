package us.ihmc.euclid.referenceFrame.tools;

import static us.ihmc.euclid.tools.EuclidCoreIOTools.DEFAULT_FORMAT;

import us.ihmc.euclid.referenceFrame.interfaces.FrameShape3DPoseReadOnly;
import us.ihmc.euclid.shape.tools.EuclidShapeIOTools;

public class EuclidFrameShapeIOTools
{
   private EuclidFrameShapeIOTools()
   {
      // Suppresses default constructor, ensuring non-instantiability.
   }

   /**
    * Gets the representative {@code String} of {@code shape3DPose} as follows:
    *
    * <pre>
    * Shape 3D pose: [position: ( 0.540,  0.110,  0.319 ), yaw-pitch-roll: (-2.061, -0.904, -1.136)] - worldFrame
    * </pre>
    *
    * @param shape3DPose the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getFrameShape3DPoseString(FrameShape3DPoseReadOnly shape3DPose)
   {
      return getFrameShape3DPoseString(DEFAULT_FORMAT, shape3DPose);
   }

   /**
    * Gets the representative {@code String} of {@code shape3DPose} given a specific format to use.
    * <p>
    * Using the default format {@link #DEFAULT_FORMAT}, this provides a {@code String} as follows:
    *
    * <pre>
    * Shape 3D pose: [position: ( 0.540,  0.110,  0.319 ), yaw-pitch-roll: (-2.061, -0.904, -1.136)] - worldFrame
    * </pre>
    * </p>
    *
    * @param format      the format to use for each number.
    * @param shape3DPose the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getFrameShape3DPoseString(String format, FrameShape3DPoseReadOnly shape3DPose)
   {
      if (shape3DPose == null)
         return "null";
      else
         return EuclidShapeIOTools.getShape3DPoseString(format, shape3DPose) + " - " + shape3DPose.getReferenceFrame();
   }

}
