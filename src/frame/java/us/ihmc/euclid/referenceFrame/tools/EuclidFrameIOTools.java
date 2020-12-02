package us.ihmc.euclid.referenceFrame.tools;

import static us.ihmc.euclid.geometry.tools.EuclidGeometryIOTools.getBoundingBox2DString;
import static us.ihmc.euclid.geometry.tools.EuclidGeometryIOTools.getBoundingBox3DString;
import static us.ihmc.euclid.geometry.tools.EuclidGeometryIOTools.getConvexPolygon2DString;
import static us.ihmc.euclid.geometry.tools.EuclidGeometryIOTools.getLine2DString;
import static us.ihmc.euclid.geometry.tools.EuclidGeometryIOTools.getLine3DString;
import static us.ihmc.euclid.geometry.tools.EuclidGeometryIOTools.getLineSegment2DString;
import static us.ihmc.euclid.geometry.tools.EuclidGeometryIOTools.getLineSegment3DString;
import static us.ihmc.euclid.geometry.tools.EuclidGeometryIOTools.getPose2DString;
import static us.ihmc.euclid.geometry.tools.EuclidGeometryIOTools.getPose3DString;
import static us.ihmc.euclid.geometry.tools.EuclidGeometryIOTools.getVertex2DSupplierString;
import static us.ihmc.euclid.geometry.tools.EuclidGeometryIOTools.getVertex3DSupplierString;
import static us.ihmc.euclid.tools.EuclidCoreIOTools.DEFAULT_FORMAT;
import static us.ihmc.euclid.tools.EuclidCoreIOTools.getMatrix3DString;
import static us.ihmc.euclid.tools.EuclidCoreIOTools.getOrientation2DString;
import static us.ihmc.euclid.tools.EuclidCoreIOTools.getStringAsYawPitchRoll;
import static us.ihmc.euclid.tools.EuclidCoreIOTools.getTuple2DString;
import static us.ihmc.euclid.tools.EuclidCoreIOTools.getTuple3DString;
import static us.ihmc.euclid.tools.EuclidCoreIOTools.getTuple4DString;
import static us.ihmc.euclid.tools.EuclidCoreIOTools.getYawPitchRollString;

import us.ihmc.euclid.referenceFrame.interfaces.FrameBoundingBox2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameBoundingBox3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameLine2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameLine3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameLineSegment2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameLineSegment3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameMatrix3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameOrientation2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameOrientation3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple4DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVertex2DSupplier;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVertex3DSupplier;
import us.ihmc.euclid.referenceFrame.interfaces.FrameYawPitchRollReadOnly;

/**
 * {@code EuclidCoreIOTools} is intended to gather the input & output tools for printing, saving,
 * and loading geometry objects.
 * <p>
 * At this time, only a few print tools are offered, additional features will come in future
 * releases.
 * </p>
 *
 * @author Sylvain Bertrand
 */
public class EuclidFrameIOTools
{
   private EuclidFrameIOTools()
   {
      // Suppresses default constructor, ensuring non-instantiability.
   }

   /**
    * Gets a representative {@code String} of {@code tuple} as follows:
    *
    * <pre>
    * (-0.675, -0.102 ) - worldFrame
    * </pre>
    *
    * @param tuple the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getFrameTuple2DString(FrameTuple2DReadOnly tuple)
   {
      return getFrameTuple2DString(DEFAULT_FORMAT, tuple);
   }

   /**
    * Gets a representative {@code String} of {@code tuple} given a specific format to use.
    * <p>
    * Using the default format {@link #DEFAULT_FORMAT}, this provides a {@code String} as follows:
    *
    * <pre>
    * (-0.675, -0.102 ) - worldFrame
    * </pre>
    * </p>
    *
    * @param format the format to use for each number.
    * @param tuple  the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getFrameTuple2DString(String format, FrameTuple2DReadOnly tuple)
   {
      if (tuple == null)
         return "null";
      else
         return getTuple2DString(format, tuple) + " - " + tuple.getReferenceFrame();
   }

   /**
    * Gets a representative {@code String} of {@code tuple} as follows:
    *
    * <pre>
    * (-0.558, -0.380,  0.130 ) - worldFrame
    * </pre>
    *
    * @param tuple the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getFrameTuple3DString(FrameTuple3DReadOnly tuple)
   {
      return getFrameTuple3DString(DEFAULT_FORMAT, tuple);
   }

   /**
    * Gets a representative {@code String} of {@code tuple} given a specific format to use.
    * <p>
    * Using the default format {@link #DEFAULT_FORMAT}, this provides a {@code String} as follows:
    *
    * <pre>
    * (-0.558, -0.380,  0.130 ) - worldFrame
    * </pre>
    * </p>
    *
    * @param format the format to use for each number.
    * @param tuple  the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getFrameTuple3DString(String format, FrameTuple3DReadOnly tuple)
   {
      if (tuple == null)
         return "null";
      else
         return getTuple3DString(format, tuple) + " - " + tuple.getReferenceFrame();
   }

   /**
    * Gets a representative {@code String} of {@code tuple} as follows:
    *
    * <pre>
    * (-0.052, -0.173, -0.371,  0.087 ) - worldFrame
    * </pre>
    *
    * @param tuple the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getFrameTuple4DString(FrameTuple4DReadOnly tuple)
   {
      return getFrameTuple4DString(DEFAULT_FORMAT, tuple);
   }

   /**
    * Gets a representative {@code String} of {@code tuple} given a specific format to use.
    * <p>
    * Using the default format {@link #DEFAULT_FORMAT}, this provides a {@code String} as follows:
    *
    * <pre>
    * (-0.052, -0.173, -0.371,  0.087 ) - worldFrame
    * </pre>
    * </p>
    *
    * @param format the format to use for each number.
    * @param tuple  the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getFrameTuple4DString(String format, FrameTuple4DReadOnly tuple)
   {
      if (tuple == null)
         return "null";
      else
         return getTuple4DString(format, tuple) + " - " + tuple.getReferenceFrame();
   }

   /**
    * Gets a representative {@code String} of {@code matrix} as follows:
    *
    * <pre>
    * /-0.576, -0.784,  0.949 \
    * | 0.649, -0.542, -0.941 |
    * \-0.486, -0.502, -0.619 /
    * worldFrame
    * </pre>
    *
    * @param matrix the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getFrameMatrix3DString(FrameMatrix3DReadOnly matrix)
   {
      return getFrameMatrix3DString(DEFAULT_FORMAT, matrix);
   }

   /**
    * Gets a representative {@code String} of {@code matrix} given a specific format to use.
    * <p>
    * Using the default format {@link #DEFAULT_FORMAT}, this provides a {@code String} as follows:
    *
    * <pre>
    * /-0.576, -0.784,  0.949 \
    * | 0.649, -0.542, -0.941 |
    * \-0.486, -0.502, -0.619 /
    * worldFrame
    * </pre>
    * </p>
    *
    * @param format the format to use for each number.
    * @param matrix the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getFrameMatrix3DString(String format, FrameMatrix3DReadOnly matrix)
   {
      if (matrix == null)
         return "null";
      else
         return getMatrix3DString(format, matrix) + "\n" + matrix.getReferenceFrame();
   }

   /**
    * Gets a representative {@code String} of {@code yawPitchRoll} as follows:
    *
    * <pre>
    * yaw-pitch-roll: ( 0.674,  0.455,  0.582 ) - worldFrame
    * </pre>
    *
    * @param yawPitchRoll the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getFrameYawPitchRollString(FrameYawPitchRollReadOnly yawPitchRoll)
   {
      return getFrameYawPitchRollString(DEFAULT_FORMAT, yawPitchRoll);
   }

   /**
    * Gets a representative {@code String} of {@code yawPitchRoll} given a specific format to use.
    * <p>
    * Using the default format {@link #DEFAULT_FORMAT}, this provides a {@code String} as follows:
    *
    * <pre>
    * yaw-pitch-roll: ( 0.674,  0.455,  0.582 ) - worldFrame
    * </pre>
    * </p>
    *
    * @param format       the format to use for each number.
    * @param yawPitchRoll the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getFrameYawPitchRollString(String format, FrameYawPitchRollReadOnly yawPitchRoll)
   {
      if (yawPitchRoll == null)
         return "null";
      else
         return getYawPitchRollString(format, yawPitchRoll) + " - " + yawPitchRoll.getReferenceFrame();
   }

   /**
    * Gets a representative {@code String} of {@code orientation} using a yaw-pitch-roll representation
    * as follows:
    *
    * <pre>
    * yaw-pitch-roll: ( 0.674,  0.455,  0.582 ) - worldFrame
    * </pre>
    *
    * @param orientation the orientation to get the {@code String} of using a yaw-pitch-roll
    *                    representation. Not modified.
    * @return the representative {@code String}.
    */
   public static String getStringAsFrameYawPitchRoll(FrameOrientation3DReadOnly orientation)
   {
      return getStringAsFrameYawPitchRoll(DEFAULT_FORMAT, orientation);
   }

   /**
    * Gets a representative {@code String} of {@code orientation} using a yaw-pitch-roll representation
    * and given a specific format to use.
    * <p>
    * Using the default format {@link #DEFAULT_FORMAT}, this provides a {@code String} as follows:
    *
    * <pre>
    * yaw-pitch-roll: ( 0.674,  0.455,  0.582 ) - worldFrame
    * </pre>
    * </p>
    *
    * @param format      the format to use for each number.
    * @param orientation the orientation to get the {@code String} of using a yaw-pitch-roll
    *                    representation. Not modified.
    * @return the representative {@code String}.
    */
   public static String getStringAsFrameYawPitchRoll(String format, FrameOrientation3DReadOnly orientation)
   {
      if (orientation == null)
         return "null";
      else
         return getStringAsYawPitchRoll(format, orientation) + " - " + orientation.getReferenceFrame();
   }

   /**
    * Gets a representative {@code String} of {@code line2D} as follows:
    *
    * <pre>
    * Line 2D: point = ( 0.174,  0.732 ), direction = (-0.380,  0.130 ), worldFrame
    * </pre>
    *
    * @param line2D the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getFrameLine2DString(FrameLine2DReadOnly line2D)
   {
      return getFrameLine2DString(DEFAULT_FORMAT, line2D);
   }

   /**
    * Gets a representative {@code String} of {@code line2D} given a specific format to use.
    * <p>
    * Using the default format {@link #DEFAULT_FORMAT}, this provides a {@code String} as follows:
    *
    * <pre>
    * Line 2D: point = ( 0.174,  0.732 ), direction = (-0.380,  0.130 ), worldFrame
    * </pre>
    * </p>
    *
    * @param format the format to use for each number.
    * @param line2D the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getFrameLine2DString(String format, FrameLine2DReadOnly line2D)
   {
      if (line2D == null)
         return "null";
      else
         return getLine2DString(format, line2D) + ", " + line2D.getReferenceFrame();
   }

   /**
    * Gets a representative {@code String} of {@code line3D} as follows:
    *
    * <pre>
    * Line 3D: point = ( 0.174,  0.732, -0.222 ), direction = (-0.558, -0.380,  0.130 ), worldFrame
    * </pre>
    *
    * @param line3D the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getFrameLine3DString(FrameLine3DReadOnly line3D)
   {
      return getFrameLine3DString(DEFAULT_FORMAT, line3D);
   }

   /**
    * Gets a representative {@code String} of {@code line3D} given a specific format to use.
    * <p>
    * Using the default format {@link #DEFAULT_FORMAT}, this provides a {@code String} as follows:
    *
    * <pre>
    * Line 3D: point = ( 0.174,  0.732, -0.222 ), direction = (-0.558, -0.380,  0.130 ), worldFrame
    * </pre>
    * </p>
    *
    * @param format the format to use for each number.
    * @param line3D the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getFrameLine3DString(String format, FrameLine3DReadOnly line3D)
   {
      if (line3D == null)
         return "null";
      else
         return getLine3DString(format, line3D) + ", " + line3D.getReferenceFrame();
   }

   /**
    * Gets a representative {@code String} of {@code lineSegment2D} as follows:
    *
    * <pre>
    * Line segment 2D: 1st endpoint = ( 0.174,  0.732 ), 2nd endpoint = (-0.558,  0.130 ), worldFrame
    * </pre>
    *
    * @param lineSegment2D the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getFrameLineSegment2DString(FrameLineSegment2DReadOnly lineSegment2D)
   {
      return getFrameLineSegment2DString(DEFAULT_FORMAT, lineSegment2D);
   }

   /**
    * Gets a representative {@code String} of {@code lineSegment2D} given a specific format to use.
    * <p>
    * Using the default format {@link #DEFAULT_FORMAT}, this provides a {@code String} as follows:
    *
    * <pre>
    * Line segment 2D: 1st endpoint = ( 0.174,  0.732 ), 2nd endpoint = (-0.558,  0.130 ), worldFrame
    * </pre>
    * </p>
    *
    * @param format        the format to use for each number.
    * @param lineSegment2D the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getFrameLineSegment2DString(String format, FrameLineSegment2DReadOnly lineSegment2D)
   {
      if (lineSegment2D == null)
         return "null";
      else
         return getLineSegment2DString(format, lineSegment2D) + ", " + lineSegment2D.getReferenceFrame();
   }

   /**
    * Gets a representative {@code String} of {@code lineSegment3D} as follows:
    *
    * <pre>
    * Line segment 3D: 1st endpoint = ( 0.174,  0.732, -0.222 ), 2nd endpoint = (-0.558, -0.380,  0.130 ), worldFrame
    * </pre>
    *
    * @param lineSegment3D the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getFrameLineSegment3DString(FrameLineSegment3DReadOnly lineSegment3D)
   {
      return getFrameLineSegment3DString(DEFAULT_FORMAT, lineSegment3D);
   }

   /**
    * Gets a representative {@code String} of {@code lineSegment3D} given a specific format to use.
    * <p>
    * Using the default format {@link #DEFAULT_FORMAT}, this provides a {@code String} as follows:
    *
    * <pre>
    * Line segment 3D: 1st endpoint = ( 0.174,  0.732, -0.222 ), 2nd endpoint = (-0.558, -0.380,  0.130 ), worldFrame
    * </pre>
    * </p>
    *
    * @param format        the format to use for each number.
    * @param lineSegment3D the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getFrameLineSegment3DString(String format, FrameLineSegment3DReadOnly lineSegment3D)
   {
      if (lineSegment3D == null)
         return "null";
      else
         return getLineSegment3DString(format, lineSegment3D) + ", " + lineSegment3D.getReferenceFrame();
   }

   /**
    * Gets a representative {@code String} of {@code boundingBox2D} as follows:
    *
    * <pre>
    * Bounding Box 2D: min = ( 0.174,  0.732 ), max = (-0.558, -0.380 ), worldFrame
    * </pre>
    *
    * @param boundingBox2D the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getFrameBoundingBox2DString(FrameBoundingBox2DReadOnly boundingBox2D)
   {
      return getFrameBoundingBox2DString(DEFAULT_FORMAT, boundingBox2D);
   }

   /**
    * Gets a representative {@code String} of {@code boundingBox2D} given a specific format to use.
    * <p>
    * Using the default format {@link #DEFAULT_FORMAT}, this provides a {@code String} as follows:
    *
    * <pre>
    * Bounding Box 2D: min = ( 0.174,  0.732 ), max = (-0.558, -0.380 ), worldFrame
    * </pre>
    * </p>
    *
    * @param format        the format to use for each number.
    * @param boundingBox2D the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getFrameBoundingBox2DString(String format, FrameBoundingBox2DReadOnly boundingBox2D)
   {
      if (boundingBox2D == null)
         return "null";
      else
         return getBoundingBox2DString(format, boundingBox2D) + ", " + boundingBox2D.getReferenceFrame();
   }

   /**
    * Gets a representative {@code String} of {@code boundingBox3D} as follows:
    *
    * <pre>
    * Bounding Box 3D: min = ( 0.174,  0.732, -0.222 ), max = (-0.558, -0.380,  0.130 ), worldFrame
    * </pre>
    *
    * @param boundingBox3D the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getFrameBoundingBox3DString(FrameBoundingBox3DReadOnly boundingBox3D)
   {
      return getFrameBoundingBox3DString(DEFAULT_FORMAT, boundingBox3D);
   }

   /**
    * Gets a representative {@code String} of {@code boundingBox3D} given a specific format to use.
    * <p>
    * Using the default format {@link #DEFAULT_FORMAT}, this provides a {@code String} as follows:
    *
    * <pre>
    * Bounding Box 3D: min = ( 0.174,  0.732, -0.222 ), max = (-0.558, -0.380,  0.130 ), worldFrame
    * </pre>
    * </p>
    *
    * @param format        the format to use for each number.
    * @param boundingBox3D the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getFrameBoundingBox3DString(String format, FrameBoundingBox3DReadOnly boundingBox3D)
   {
      if (boundingBox3D == null)
         return "null";
      else
         return getBoundingBox3DString(format, boundingBox3D) + ", " + boundingBox3D.getReferenceFrame();
   }

   /**
    * Gets a representative {@code String} of {@code orientation2D} as follows:
    *
    * <pre>
    * (0.174) - worldFrame
    * </pre>
    *
    * @param orientation2D the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getFrameOrientation2DString(FrameOrientation2DReadOnly orientation2D)
   {
      return getFrameOrientation2DString(DEFAULT_FORMAT, orientation2D);
   }

   /**
    * Gets a representative {@code String} of {@code orientation2D} given a specific format to use.
    * <p>
    * Using the default format {@link #DEFAULT_FORMAT}, this provides a {@code String} as follows:
    *
    * <pre>
    * (0.174) - worldFrame
    * </pre>
    * </p>
    *
    * @param format        the format to use for each number.
    * @param orientation2D the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getFrameOrientation2DString(String format, FrameOrientation2DReadOnly orientation2D)
   {
      if (orientation2D == null)
         return "null";
      else
         return getOrientation2DString(format, orientation2D) + " - " + orientation2D.getReferenceFrame();
   }

   /**
    * Gets a representative {@code String} of {@code pose2D} as follows:
    *
    * <pre>
    * Pose 2D: position = ( 0.174, -0.222 ), orientation = (-0.130 ), worldFrame
    * </pre>
    *
    * @param pose2D the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getFramePose2DString(FramePose2DReadOnly pose2D)
   {
      return getFramePose2DString(DEFAULT_FORMAT, pose2D);
   }

   /**
    * Gets a representative {@code String} of {@code pose2D} given a specific format to use.
    * <p>
    * Using the default format {@link #DEFAULT_FORMAT}, this provides a {@code String} as follows:
    *
    * <pre>
    * Pose 2D: position = ( 0.174, -0.222 ), orientation = (-0.130 ), worldFrame
    * </pre>
    * </p>
    *
    * @param format the format to use for each number.
    * @param pose2D the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getFramePose2DString(String format, FramePose2DReadOnly pose2D)
   {
      if (pose2D == null)
         return "null";
      else
         return getPose2DString(format, pose2D) + ", " + pose2D.getReferenceFrame();
   }

   /**
    * Gets a representative {@code String} of {@code pose3D} as follows:
    *
    * <pre>
    * Pose 3D: position = ( 0.174, -0.452, -0.222 ), orientation = (-0.052, -0.173, -0.371,  0.087 ), worldFrame
    * </pre>
    *
    * @param pose3D the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getFramePose3DString(FramePose3DReadOnly pose3D)
   {
      return getFramePose3DString(DEFAULT_FORMAT, pose3D);
   }

   /**
    * Gets a representative {@code String} of {@code pose3D} given a specific format to use.
    * <p>
    * Using the default format {@link #DEFAULT_FORMAT}, this provides a {@code String} as follows:
    *
    * <pre>
    * Pose 3D: position = ( 0.174, -0.452, -0.222 ), orientation = (-0.052, -0.173, -0.371,  0.087 ), worldFrame
    * </pre>
    * </p>
    *
    * @param format the format to use for each number.
    * @param pose3D the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getFramePose3DString(String format, FramePose3DReadOnly pose3D)
   {
      if (pose3D == null)
         return "null";
      else
         return getPose3DString(format, pose3D) + ", " + pose3D.getReferenceFrame();
   }

   /**
    * Gets a representative {@code String} of {@code convexPolygon2D} as follows:
    *
    * <pre>
    * Convex Polygon 2D: vertices = [
    * ( 0.174, -0.452 ),
    * (-0.052, -0.173 ) ]
    * worldFrame
    * </pre>
    *
    * @param convexPolygon2D the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getFrameConvexPolygon2DString(FrameConvexPolygon2DReadOnly convexPolygon2D)
   {
      return getFrameConvexPolygon2DString(DEFAULT_FORMAT, convexPolygon2D);
   }

   /**
    * Gets a representative {@code String} of {@code convexPolygon2D} given a specific format to use.
    * <p>
    * Using the default format {@link #DEFAULT_FORMAT}, this provides a {@code String} as follows:
    *
    * <pre>
    * Convex Polygon 2D: vertices = [
    * ( 0.174, -0.452 ),
    * (-0.052, -0.173 ) ]
    * worldFrame
    * </pre>
    * </p>
    *
    * @param format          the format to use for each number.
    * @param convexPolygon2D the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getFrameConvexPolygon2DString(String format, FrameConvexPolygon2DReadOnly convexPolygon2D)
   {
      if (convexPolygon2D == null)
         return "null";
      else
         return getConvexPolygon2DString(format, convexPolygon2D) + "\n" + convexPolygon2D.getReferenceFrame();
   }

   /**
    * Gets a representative {@code String} of {@code vertex2DSupplier} as follows:
    *
    * <pre>
    * Vertex 2D supplier: vertices = [
    * ( 0.174, -0.452 ),
    * (-0.052, -0.173 ) ]
    * worldFrame
    * </pre>
    *
    * @param vertex2DSupplier the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getFrameVertex2DSupplierString(FrameVertex2DSupplier vertex2DSupplier)
   {
      return getFrameVertex2DSupplierString(DEFAULT_FORMAT, vertex2DSupplier);
   }

   /**
    * Gets a representative {@code String} of {@code vertex2DSupplier} given a specific format to use.
    * <p>
    * Using the default format {@link #DEFAULT_FORMAT}, this provides a {@code String} as follows:
    *
    * <pre>
    * Vertex 2D supplier: vertices = [
    * ( 0.174, -0.452 ),
    * (-0.052, -0.173 ) ]
    * worldFrame
    * </pre>
    * </p>
    *
    * @param format           the format to use for each number.
    * @param vertex2DSupplier the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getFrameVertex2DSupplierString(String format, FrameVertex2DSupplier vertex2DSupplier)
   {
      if (vertex2DSupplier == null)
         return "null";
      else
         return getVertex2DSupplierString(format, vertex2DSupplier) + "\n" + vertex2DSupplier.getReferenceFrame();
   }

   /**
    * Gets a representative {@code String} of {@code vertex3DSupplier} as follows:
    *
    * <pre>
    * Vertex 3D supplier: vertices = [
    * ( 0.174, -0.452, -0.222 ),
    * (-0.052, -0.173, -0.371 ) ]
    * worldFrame
    * </pre>
    *
    * @param vertex3DSupplier the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getFrameVertex3DSupplierString(FrameVertex3DSupplier vertex3DSupplier)
   {
      return getFrameVertex3DSupplierString(DEFAULT_FORMAT, vertex3DSupplier);
   }

   /**
    * Gets a representative {@code String} of {@code vertex3DSupplier} given a specific format to use.
    * <p>
    * Using the default format {@link #DEFAULT_FORMAT}, this provides a {@code String} as follows:
    *
    * <pre>
    * Vertex 3D supplier: vertices = [
    * ( 0.174, -0.452, -0.222 ),
    * (-0.052, -0.173, -0.371 ) ]
    * worldFrame
    * </pre>
    * </p>
    *
    * @param format           the format to use for each number.
    * @param vertex3DSupplier the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getFrameVertex3DSupplierString(String format, FrameVertex3DSupplier vertex3DSupplier)
   {
      if (vertex3DSupplier == null)
         return "null";
      else
         return getVertex3DSupplierString(format, vertex3DSupplier) + "\n" + vertex3DSupplier.getReferenceFrame();
   }
}
