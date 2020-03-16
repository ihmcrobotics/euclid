package us.ihmc.euclid.referenceFrame.tools;

import static us.ihmc.euclid.shape.tools.EuclidShapeIOTools.getBox3DString;
import static us.ihmc.euclid.shape.tools.EuclidShapeIOTools.getCapsule3DString;
import static us.ihmc.euclid.shape.tools.EuclidShapeIOTools.getPointShape3DString;
import static us.ihmc.euclid.shape.tools.EuclidShapeIOTools.getShape3DPoseString;
import static us.ihmc.euclid.tools.EuclidCoreIOTools.DEFAULT_FORMAT;

import us.ihmc.euclid.referenceFrame.interfaces.*;

public class EuclidFrameShapeIOTools
{
   private EuclidFrameShapeIOTools()
   {
      // Suppresses default constructor, ensuring non-instantiability.
   }

   /**
    * Gets the representative {@code String} of {@code box3D} as follows:
    *
    * <pre>
    * Box 3D: [position: ( 0.540,  0.110,  0.319 ), yaw-pitch-roll: (-2.061, -0.904, -1.136), size: ( 0.191,  0.719,  0.479 )] - worldFrame
    * </pre>
    *
    * @param box3D the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getFrameBox3DString(FrameBox3DReadOnly box3D)
   {
      return getFrameBox3DString(DEFAULT_FORMAT, box3D);
   }

   /**
    * Gets the representative {@code String} of {@code box3D} given a specific format to use.
    * <p>
    * Using the default format {@link #DEFAULT_FORMAT}, this provides a {@code String} as follows:
    *
    * <pre>
    * Box 3D: [position: ( 0.540,  0.110,  0.319 ), yaw-pitch-roll: (-2.061, -0.904, -1.136), size: ( 0.191,  0.719,  0.479 )] - worldFrame
    * </pre>
    * </p>
    *
    * @param format the format to use for each number.
    * @param box3D  the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getFrameBox3DString(String format, FrameBox3DReadOnly box3D)
   {
      if (box3D == null)
         return "null";
      else
         return getBox3DString(format, box3D) + " - " + box3D.getReferenceFrame();
   }

   /**
    * Gets a representative {@code String} of {@code capsule3D} as follows:
    *
    * <pre>
    * Capsule 3D: [position: (-0.362, -0.617,  0.066 ), axis: ( 0.634, -0.551, -0.543 ), length:  0.170, radius:  0.906] - worldFrame
    * </pre>
    *
    * @param capsule3D the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getFrameCapsule3DString(FrameCapsule3DReadOnly capsule3D)
   {
      return getFrameCapsule3DString(DEFAULT_FORMAT, capsule3D);
   }

   /**
    * Gets a representative {@code String} of {@code capsule3D} given a specific format to use.
    * <p>
    * Using the default format {@link #DEFAULT_FORMAT}, this provides a {@code String} as follows:
    *
    * <pre>
    * Capsule 3D: [position: (-0.362, -0.617,  0.066 ), axis: ( 0.634, -0.551, -0.543 ), length:  0.170, radius:  0.906] - worldFrame
    * </pre>
    * </p>
    *
    * @param format    the format to use for each number.
    * @param capsule3D the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getFrameCapsule3DString(String format, FrameCapsule3DReadOnly capsule3D)
   {
      return getCapsule3DString(format, capsule3D) + " - " + capsule3D.getReferenceFrame();
   }

   /**
    * Gets a representative {@code String} of {@code cylinder3D} as follows:
    *
    * <pre>
    * Cylinder 3D: [position: (-0.362, -0.617,  0.066 ), axis: ( 0.634, -0.551, -0.543 ), length:  0.170, radius:  0.906] - worldFrame
    * </pre>
    *
    * @param cylinder3D the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getFrameCylinder3DString(FrameCylinder3DReadOnly cylinder3D)
   {
      return getFrameCylinder3DString(DEFAULT_FORMAT, cylinder3D);
   }

   /**
    * Gets a representative {@code String} of {@code cylinder3D} given a specific format to use.
    * <p>
    * Using the default format {@link #DEFAULT_FORMAT}, this provides a {@code String} as follows:
    *
    * <pre>
    * Cylinder 3D: [position: (-0.362, -0.617,  0.066 ), axis: ( 0.634, -0.551, -0.543 ), length:  0.170, radius:  0.906] - worldFrame
    * </pre>
    * </p>
    *
    * @param format     the format to use for each number.
    * @param cylinder3D the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getFrameCylinder3DString(String format, FrameCylinder3DReadOnly cylinder3D)
   {
      return getFrameCylinder3DString(format, cylinder3D) + " - " + cylinder3D.getReferenceFrame();
   }

   /**
    * Gets the representative {@code String} of {@code ellipsoid3D} as follows:
    *
    * <pre>
    * Ellipsoid 3D: [position: ( 0.540,  0.110,  0.319 ), yaw-pitch-roll: (-2.061, -0.904, -1.136), radii: ( 0.191,  0.719,  0.479 )] - worldFrame
    * </pre>
    *
    * @param ellipsoid3D the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getFrameEllipsoid3DString(FrameEllipsoid3DReadOnly ellipsoid3D)
   {
      return getFrameEllipsoid3DString(DEFAULT_FORMAT, ellipsoid3D);
   }

   /**
    * Gets the representative {@code String} of {@code ellipsoid3D} given a specific format to use.
    * <p>
    * Using the default format {@link #DEFAULT_FORMAT}, this provides a {@code String} as follows:
    *
    * <pre>
    * Ellipsoid 3D: [position: ( 0.540,  0.110,  0.319 ), yaw-pitch-roll: (-2.061, -0.904, -1.136), radii: ( 0.191,  0.719,  0.479 )] - worldFrame
    * </pre>
    * </p>
    *
    * @param format      the format to use for each number.
    * @param ellipsoid3D the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getFrameEllipsoid3DString(String format, FrameEllipsoid3DReadOnly ellipsoid3D)
   {
      if (ellipsoid3D == null)
         return "null";
      else
         return getFrameEllipsoid3DString(format, ellipsoid3D) + " - " + ellipsoid3D.getReferenceFrame();
   }

   /**
    * Gets a representative {@code String} of {@code pointShape3D} as follows:
    *
    * <pre>
    * Point shape 3D: (-0.362, -0.617,  0.066 ) - worldFrame
    * </pre>
    *
    * @param pointShape3D the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getFramePointShape3DString(FramePointShape3DReadOnly pointShape3D)
   {
      return getFramePointShape3DString(DEFAULT_FORMAT, pointShape3D);
   }

   /**
    * Gets a representative {@code String} of {@code pointShape3D} given a specific format to use.
    * <p>
    * Using the default format {@link #DEFAULT_FORMAT}, this provides a {@code String} as follows:
    *
    * <pre>
    * Point shape 3D: (-0.362, -0.617,  0.066 ) - worldFrame
    * </pre>
    * </p>
    *
    * @param format       the format to use for each number.
    * @param pointShape3D the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getFramePointShape3DString(String format, FramePointShape3DReadOnly pointShape3D)
   {
      return getPointShape3DString(format, pointShape3D) + " - " + pointShape3D.getReferenceFrame();
   }

   /**
    * Gets the representative {@code String} of {@code ramp3D} as follows:
    *
    * <pre>
    * Ramp 3D: [position: ( 0.540,  0.110,  0.319 ), yaw-pitch-roll: (-2.061, -0.904, -1.136), size: ( 0.191,  0.719,  0.479 )] - worldFrame
    * </pre>
    *
    * @param ramp3D the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getFrameRamp3DString(FrameRamp3DReadOnly ramp3D)
   {
      return getFrameRamp3DString(DEFAULT_FORMAT, ramp3D);
   }

   /**
    * Gets the representative {@code String} of {@code ramp3D} given a specific format to use.
    * <p>
    * Using the default format {@link #DEFAULT_FORMAT}, this provides a {@code String} as follows:
    *
    * <pre>
    * Ramp 3D: [position: ( 0.540,  0.110,  0.319 ), yaw-pitch-roll: (-2.061, -0.904, -1.136), size: ( 0.191,  0.719,  0.479 )] - worldFrame
    * </pre>
    * </p>
    *
    * @param format the format to use for each number.
    * @param ramp3D the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getFrameRamp3DString(String format, FrameRamp3DReadOnly ramp3D)
   {
      if (ramp3D == null)
         return "null";
      else
         return getFrameRamp3DString(format, ramp3D) + " - " + ramp3D.getReferenceFrame();
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
         return getShape3DPoseString(format, shape3DPose) + " - " + shape3DPose.getReferenceFrame();
   }

}
