package us.ihmc.euclid.geometry.tools;

import static us.ihmc.euclid.tools.EuclidCoreIOTools.DEFAULT_FORMAT;
import static us.ihmc.euclid.tools.EuclidCoreIOTools.getStringOf;
import static us.ihmc.euclid.tools.EuclidCoreIOTools.getTuple2DString;
import static us.ihmc.euclid.tools.EuclidCoreIOTools.getTuple3DString;
import static us.ihmc.euclid.tools.EuclidCoreIOTools.getTuple4DString;

import java.util.List;

import us.ihmc.euclid.geometry.LineSegment1D;
import us.ihmc.euclid.geometry.interfaces.BoundingBox2DReadOnly;
import us.ihmc.euclid.geometry.interfaces.BoundingBox3DReadOnly;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.geometry.interfaces.Line2DReadOnly;
import us.ihmc.euclid.geometry.interfaces.Line3DReadOnly;
import us.ihmc.euclid.geometry.interfaces.LineSegment2DReadOnly;
import us.ihmc.euclid.geometry.interfaces.LineSegment3DReadOnly;
import us.ihmc.euclid.geometry.interfaces.Plane3DReadOnly;
import us.ihmc.euclid.geometry.interfaces.Pose2DReadOnly;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.geometry.interfaces.Triangle3DReadOnly;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.geometry.interfaces.Vertex3DSupplier;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;

/**
 * {@code EuclidGeometryIOTools}, as {@link EuclidCoreIOTools}, is intended to gather the input &
 * output tools for printing, saving, and loading geometry objects.
 * <p>
 * At this time, only a few print tools are offered, additional features will come in future
 * releases.
 * </p>
 *
 * @author Sylvain Bertrand
 */
public class EuclidGeometryIOTools
{
   private EuclidGeometryIOTools()
   {
      // Suppresses default constructor, ensuring non-instantiability.
   }

   /**
    * Gets a representative {@code String} of {@code line2D} as follows:
    *
    * <pre>
    * Line 2D: point = ( 0.174,  0.732 ), direction = (-0.380,  0.130 )
    * </pre>
    *
    * @param line2D the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getLine2DString(Line2DReadOnly line2D)
   {
      return getLine2DString(DEFAULT_FORMAT, line2D);
   }

   /**
    * Gets a representative {@code String} of {@code line2D} given a specific format to use.
    * <p>
    * Using the default format {@link #DEFAULT_FORMAT}, this provides a {@code String} as follows:
    *
    * <pre>
    * Line 2D: point = ( 0.174,  0.732 ), direction = (-0.380,  0.130 )
    * </pre>
    * </p>
    *
    * @param format the format to use for each number.
    * @param line2D the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getLine2DString(String format, Line2DReadOnly line2D)
   {
      if (line2D == null)
         return "null";
      else
         return getLine2DString(format, line2D.getPoint(), line2D.getDirection());
   }

   /**
    * Gets a representative {@code String} of {@code line2D} given a specific format to use.
    * <p>
    * Using the default format {@link #DEFAULT_FORMAT}, this provides a {@code String} as follows:
    *
    * <pre>
    * Line 2D: point = ( 0.174,  0.732 ), direction = (-0.380,  0.130 )
    * </pre>
    * </p>
    *
    * @param format        the format to use for each number.
    * @param pointOnLine   a point located on the line to get the {@code String} of. Not modified.
    * @param lineDirection the direction of the line to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getLine2DString(String format, Point2DReadOnly pointOnLine, Vector2DReadOnly lineDirection)
   {
      return "Line 2D: point = " + getTuple2DString(format, pointOnLine) + ", direction = " + getTuple2DString(format, lineDirection);
   }

   /**
    * Gets a representative {@code String} of {@code line3D} as follows:
    *
    * <pre>
    * Line 3D: point = ( 0.174,  0.732, -0.222 ), direction = (-0.558, -0.380,  0.130 )
    * </pre>
    *
    * @param line3D the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getLine3DString(Line3DReadOnly line3D)
   {
      return getLine3DString(DEFAULT_FORMAT, line3D);
   }

   /**
    * Gets a representative {@code String} of {@code line3D} given a specific format to use.
    * <p>
    * Using the default format {@link #DEFAULT_FORMAT}, this provides a {@code String} as follows:
    *
    * <pre>
    * Line 3D: point = ( 0.174,  0.732, -0.222 ), direction = (-0.558, -0.380,  0.130 )
    * </pre>
    * </p>
    *
    * @param format the format to use for each number.
    * @param line3D the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getLine3DString(String format, Line3DReadOnly line3D)
   {
      if (line3D == null)
         return "null";
      else
         return getLine3DString(format, line3D.getPoint(), line3D.getDirection());
   }

   /**
    * Gets a representative {@code String} of {@code line3D} given a specific format to use.
    * <p>
    * Using the default format {@link #DEFAULT_FORMAT}, this provides a {@code String} as follows:
    *
    * <pre>
    * Line 3D: point = ( 0.174,  0.732, -0.222 ), direction = (-0.558, -0.380,  0.130 )
    * </pre>
    * </p>
    *
    * @param format        the format to use for each number.
    * @param pointOnLine   a point located on the line to get the {@code String} of. Not modified.
    * @param lineDirection the direction of the line to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getLine3DString(String format, Point3DReadOnly pointOnLine, Vector3DReadOnly lineDirection)
   {
      return "Line 3D: point = " + getTuple3DString(format, pointOnLine) + ", direction = " + getTuple3DString(format, lineDirection);
   }

   /**
    * Gets a representative {@code String} of {@code lineSegment1D} as follows:
    *
    * <pre>
    * Line segment 1D: 1st endpoint = ( 0.732 ), 2nd endpoint = (-0.558 )
    * </pre>
    *
    * @param lineSegment1D the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getLineSegment1DString(LineSegment1D lineSegment1D)
   {
      return getLineSegment1DString(DEFAULT_FORMAT, lineSegment1D);
   }

   /**
    * Gets a representative {@code String} of {@code lineSegment1D} given a specific format to use.
    * <p>
    * Using the default format {@link #DEFAULT_FORMAT}, this provides a {@code String} as follows:
    *
    * <pre>
    * Line segment 1D: 1st endpoint = ( 0.732 ), 2nd endpoint = (-0.558 )
    * </pre>
    * </p>
    *
    * @param format        the format to use for each number.
    * @param lineSegment1D the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getLineSegment1DString(String format, LineSegment1D lineSegment1D)
   {
      if (lineSegment1D == null)
         return "null";
      else
         return getLineSegment1DString(format, lineSegment1D.getFirstEndpoint(), lineSegment1D.getSecondEndpoint());
   }

   /**
    * Gets a representative {@code String} of {@code lineSegment1D} given a specific format to use.
    * <p>
    * Using the default format {@link #DEFAULT_FORMAT}, this provides a {@code String} as follows:
    *
    * <pre>
    * Line segment 1D: 1st endpoint = ( 0.732 ), 2nd endpoint = (-0.558 )
    * </pre>
    * </p>
    *
    * @param format           the format to use for each number.
    * @param lineSegmentStart the first endpoint of the line segment to get the {@code String} of. Not
    *                         modified.
    * @param lineSegmentEnd   the second endpoint of the line segment to get the {@code String} of. Not
    *                         modified.
    * @return the representative {@code String}.
    */
   public static String getLineSegment1DString(String format, double lineSegmentStart, double lineSegmentEnd)
   {
      return "Line segment 1D: 1st endpoint = " + getStringOf("(", " )", ", ", format, lineSegmentStart) + ", 2nd endpoint = "
            + getStringOf("(", " )", ", ", format, lineSegmentEnd);
   }

   /**
    * Gets a representative {@code String} of {@code lineSegment2D} as follows:
    *
    * <pre>
    * Line segment 2D: 1st endpoint = ( 0.174,  0.732 ), 2nd endpoint = (-0.558,  0.130 )
    * </pre>
    *
    * @param lineSegment2D the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getLineSegment2DString(LineSegment2DReadOnly lineSegment2D)
   {
      return getLineSegment2DString(DEFAULT_FORMAT, lineSegment2D);
   }

   /**
    * Gets a representative {@code String} of {@code lineSegment2D} given a specific format to use.
    * <p>
    * Using the default format {@link #DEFAULT_FORMAT}, this provides a {@code String} as follows:
    *
    * <pre>
    * Line segment 2D: 1st endpoint = ( 0.174,  0.732 ), 2nd endpoint = (-0.558,  0.130 )
    * </pre>
    * </p>
    *
    * @param format        the format to use for each number.
    * @param lineSegment2D the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getLineSegment2DString(String format, LineSegment2DReadOnly lineSegment2D)
   {
      if (lineSegment2D == null)
         return "null";
      else
         return getLineSegment2DString(format, lineSegment2D.getFirstEndpoint(), lineSegment2D.getSecondEndpoint());
   }

   /**
    * Gets a representative {@code String} of {@code lineSegment2D} given a specific format to use.
    * <p>
    * Using the default format {@link #DEFAULT_FORMAT}, this provides a {@code String} as follows:
    *
    * <pre>
    * Line segment 2D: 1st endpoint = ( 0.174,  0.732 ), 2nd endpoint = (-0.558,  0.130 )
    * </pre>
    * </p>
    *
    * @param format           the format to use for each number.
    * @param lineSegmentStart the first endpoint of the line segment to get the {@code String} of. Not
    *                         modified.
    * @param lineSegmentEnd   the second endpoint of the line segment to get the {@code String} of. Not
    *                         modified.
    * @return the representative {@code String}.
    */
   public static String getLineSegment2DString(String format, Point2DReadOnly lineSegmentStart, Point2DReadOnly lineSegmentEnd)
   {
      return "Line segment 2D: 1st endpoint = " + getTuple2DString(format, lineSegmentStart) + ", 2nd endpoint = " + getTuple2DString(format, lineSegmentEnd);
   }

   /**
    * Gets a representative {@code String} of {@code lineSegment3D} as follows:
    *
    * <pre>
    * Line segment 3D: 1st endpoint = ( 0.174,  0.732, -0.222 ), 2nd endpoint = (-0.558, -0.380,  0.130 )
    * </pre>
    *
    * @param lineSegment3D the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getLineSegment3DString(LineSegment3DReadOnly lineSegment3D)
   {
      return getLineSegment3DString(DEFAULT_FORMAT, lineSegment3D);
   }

   /**
    * Gets a representative {@code String} of {@code lineSegment3D} given a specific format to use.
    * <p>
    * Using the default format {@link #DEFAULT_FORMAT}, this provides a {@code String} as follows:
    *
    * <pre>
    * Line segment 3D: 1st endpoint = ( 0.174,  0.732, -0.222 ), 2nd endpoint = (-0.558, -0.380,  0.130 )
    * </pre>
    * </p>
    *
    * @param format        the format to use for each number.
    * @param lineSegment3D the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getLineSegment3DString(String format, LineSegment3DReadOnly lineSegment3D)
   {
      if (lineSegment3D == null)
         return "null";
      else
         return getLineSegment3DString(format, lineSegment3D.getFirstEndpoint(), lineSegment3D.getSecondEndpoint());
   }

   /**
    * Gets a representative {@code String} of {@code lineSegment3D} given a specific format to use.
    * <p>
    * Using the default format {@link #DEFAULT_FORMAT}, this provides a {@code String} as follows:
    *
    * <pre>
    * Line segment 3D: 1st endpoint = ( 0.174,  0.732, -0.222 ), 2nd endpoint = (-0.558, -0.380,  0.130 )
    * </pre>
    * </p>
    *
    * @param format           the format to use for each number.
    * @param lineSegmentStart the first endpoint of the line segment to get the {@code String} of. Not
    *                         modified.
    * @param lineSegmentEnd   the second endpoint of the line segment to get the {@code String} of. Not
    *                         modified.
    * @return the representative {@code String}.
    */
   public static String getLineSegment3DString(String format, Point3DReadOnly lineSegmentStart, Point3DReadOnly lineSegmentEnd)
   {
      return "Line segment 3D: 1st endpoint = " + getTuple3DString(format, lineSegmentStart) + ", 2nd endpoint = " + getTuple3DString(format, lineSegmentEnd);
   }

   /**
    * Gets a representative {@code String} of {@code boundingBox2D} as follows:
    *
    * <pre>
    * Bounding Box 2D: min = ( 0.174,  0.732 ), max = (-0.558, -0.380 )
    * </pre>
    *
    * @param boundingBox2D the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getBoundingBox2DString(BoundingBox2DReadOnly boundingBox2D)
   {
      return getBoundingBox2DString(DEFAULT_FORMAT, boundingBox2D);
   }

   /**
    * Gets a representative {@code String} of {@code boundingBox2D} given a specific format to use.
    * <p>
    * Using the default format {@link #DEFAULT_FORMAT}, this provides a {@code String} as follows:
    *
    * <pre>
    * Bounding Box 2D: min = ( 0.174,  0.732 ), max = (-0.558, -0.380 )
    * </pre>
    * </p>
    *
    * @param format        the format to use for each number.
    * @param boundingBox2D the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getBoundingBox2DString(String format, BoundingBox2DReadOnly boundingBox2D)
   {
      if (boundingBox2D == null)
         return "null";
      else
         return getBoundingBox2DString(format, boundingBox2D.getMinPoint(), boundingBox2D.getMaxPoint());
   }

   /**
    * Gets a representative {@code String} of {@code boundingBox2D} given a specific format to use.
    * <p>
    * Using the default format {@link #DEFAULT_FORMAT}, this provides a {@code String} as follows:
    *
    * <pre>
    * Bounding Box 2D: min = ( 0.174,  0.732 ), max = (-0.558, -0.380 )
    * </pre>
    * </p>
    *
    * @param format         the format to use for each number.
    * @param boundingBoxMin the minimum coordinate of the bounding box to get the {@code String} of.
    *                       Not modified.
    * @param boundingBoxMax the maximum coordinate of the bounding box to get the {@code String} of.
    *                       Not modified.
    * @return the representative {@code String}.
    */
   public static String getBoundingBox2DString(String format, Point2DReadOnly boundingBoxMin, Point2DReadOnly boundingBoxMax)
   {
      return "Bounding Box 2D: min = " + getTuple2DString(format, boundingBoxMin) + ", max = " + getTuple2DString(format, boundingBoxMax);
   }

   /**
    * Gets a representative {@code String} of {@code boundingBox3D} as follows:
    *
    * <pre>
    * Bounding Box 3D: min = ( 0.174,  0.732, -0.222 ), max = (-0.558, -0.380,  0.130 )
    * </pre>
    *
    * @param boundingBox3D the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getBoundingBox3DString(BoundingBox3DReadOnly boundingBox3D)
   {
      return getBoundingBox3DString(DEFAULT_FORMAT, boundingBox3D);
   }

   /**
    * Gets a representative {@code String} of {@code boundingBox3D} given a specific format to use.
    * <p>
    * Using the default format {@link #DEFAULT_FORMAT}, this provides a {@code String} as follows:
    *
    * <pre>
    * Bounding Box 3D: min = ( 0.174,  0.732, -0.222 ), max = (-0.558, -0.380,  0.130 )
    * </pre>
    * </p>
    *
    * @param format        the format to use for each number.
    * @param boundingBox3D the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getBoundingBox3DString(String format, BoundingBox3DReadOnly boundingBox3D)
   {
      if (boundingBox3D == null)
         return "null";
      else
         return getBoundingBox3DString(format, boundingBox3D.getMinPoint(), boundingBox3D.getMaxPoint());
   }

   /**
    * Gets a representative {@code String} of {@code boundingBox3D} given a specific format to use.
    * <p>
    * Using the default format {@link #DEFAULT_FORMAT}, this provides a {@code String} as follows:
    *
    * <pre>
    * Bounding Box 3D: min = ( 0.174,  0.732, -0.222 ), max = (-0.558, -0.380,  0.130 )
    * </pre>
    * </p>
    *
    * @param format         the format to use for each number.
    * @param boundingBoxMin the minimum coordinate of the bounding box to get the {@code String} of.
    *                       Not modified.
    * @param boundingBoxMax the maximum coordinate of the bounding box to get the {@code String} of.
    *                       Not modified.
    * @return the representative {@code String}.
    */
   public static String getBoundingBox3DString(String format, Point3DReadOnly boundingBoxMin, Point3DReadOnly boundingBoxMax)
   {
      return "Bounding Box 3D: min = " + getTuple3DString(format, boundingBoxMin) + ", max = " + getTuple3DString(format, boundingBoxMax);
   }

   /**
    * Gets a representative {@code String} of {@code plane3D} as follows:
    *
    * <pre>
    * Plane 3D: point = ( 0.174,  0.732, -0.222 ), normal = (-0.558, -0.380,  0.130 )
    * </pre>
    *
    * @param plane3D the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getPlane3DString(Plane3DReadOnly plane3D)
   {
      return getPlane3DString(DEFAULT_FORMAT, plane3D);
   }

   /**
    * Gets a representative {@code String} of {@code plane3D} given a specific format to use.
    * <p>
    * Using the default format {@link #DEFAULT_FORMAT}, this provides a {@code String} as follows:
    *
    * <pre>
    * Plane 3D: point = ( 0.174,  0.732, -0.222 ), normal = (-0.558, -0.380,  0.130 )
    * </pre>
    * </p>
    *
    * @param format  the format to use for each number.
    * @param plane3D the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getPlane3DString(String format, Plane3DReadOnly plane3D)
   {
      if (plane3D == null)
         return "null";
      else
         return getPlane3DString(format, plane3D.getPoint(), plane3D.getNormal());
   }

   /**
    * Gets a representative {@code String} of {@code plane3D} given a specific format to use.
    * <p>
    * Using the default format {@link #DEFAULT_FORMAT}, this provides a {@code String} as follows:
    *
    * <pre>
    * Plane 3D: point = ( 0.174,  0.732, -0.222 ), normal = (-0.558, -0.380,  0.130 )
    * </pre>
    * </p>
    *
    * @param format       the format to use for each number.
    * @param pointOnPlane a point located on the plane to get the {@code String} of. Not modified.
    * @param planeNormal  the normal of the plane to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getPlane3DString(String format, Point3DReadOnly pointOnPlane, Vector3DReadOnly planeNormal)
   {
      return "Plane 3D: point = " + getTuple3DString(format, pointOnPlane) + ", normal = " + getTuple3DString(format, planeNormal);
   }

   /**
    * Gets a representative {@code String} of {@code pose2D} as follows:
    *
    * <pre>
    * Pose 2D: position = ( 0.174, -0.222 ), orientation = (-0.130 )
    * </pre>
    *
    * @param pose2D the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getPose2DString(Pose2DReadOnly pose2D)
   {
      return getPose2DString(DEFAULT_FORMAT, pose2D);
   }

   /**
    * Gets a representative {@code String} of {@code pose2D} given a specific format to use.
    * <p>
    * Using the default format {@link #DEFAULT_FORMAT}, this provides a {@code String} as follows:
    *
    * <pre>
    * Pose 2D: position = ( 0.174, -0.222 ), orientation = (-0.130 )
    * </pre>
    * </p>
    *
    * @param format the format to use for each number.
    * @param pose2D the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getPose2DString(String format, Pose2DReadOnly pose2D)
   {
      if (pose2D == null)
         return "null";
      else
         return getPose2DString(format, pose2D.getPosition(), pose2D.getYaw());
   }

   /**
    * Gets a representative {@code String} of {@code pose2D} given a specific format to use.
    * <p>
    * Using the default format {@link #DEFAULT_FORMAT}, this provides a {@code String} as follows:
    *
    * <pre>
    * Pose 2D: position = ( 0.174, -0.222 ), orientation = (-0.130 )
    * </pre>
    * </p>
    *
    * @param format      the format to use for each number.
    * @param position    the position part of the pose to get the {@code String} of. Not modified.
    * @param orientation the orientation part of the pose to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getPose2DString(String format, Point2DReadOnly position, double orientation)
   {
      return "Pose 2D: position = " + getTuple2DString(format, position) + ", orientation = " + EuclidCoreIOTools.getOrientation2DString(format, orientation);
   }

   /**
    * Gets a representative {@code String} of {@code pose3D} as follows:
    *
    * <pre>
    * Pose 3D: position = ( 0.174, -0.452, -0.222 ), orientation = (-0.052, -0.173, -0.371,  0.087 )
    * </pre>
    *
    * @param pose3D the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getPose3DString(Pose3DReadOnly pose3D)
   {
      return getPose3DString(DEFAULT_FORMAT, pose3D);
   }

   /**
    * Gets a representative {@code String} of {@code pose3D} given a specific format to use.
    * <p>
    * Using the default format {@link #DEFAULT_FORMAT}, this provides a {@code String} as follows:
    *
    * <pre>
    * Pose 3D: position = ( 0.174, -0.452, -0.222 ), orientation = (-0.052, -0.173, -0.371,  0.087 )
    * </pre>
    * </p>
    *
    * @param format the format to use for each number.
    * @param pose3D the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getPose3DString(String format, Pose3DReadOnly pose3D)
   {
      if (pose3D == null)
         return "null";
      else
         return getPose3DString(format, pose3D.getPosition(), pose3D.getOrientation());
   }

   /**
    * Gets a representative {@code String} of {@code pose3D} given a specific format to use.
    * <p>
    * Using the default format {@link #DEFAULT_FORMAT}, this provides a {@code String} as follows:
    *
    * <pre>
    * Pose 3D: position = ( 0.174, -0.452, -0.222 ), orientation = (-0.052, -0.173, -0.371,  0.087 )
    * </pre>
    * </p>
    *
    * @param format      the format to use for each number.
    * @param position    the position part of the pose to get the {@code String} of. Not modified.
    * @param orientation the orientation part of the pose to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getPose3DString(String format, Point3DReadOnly position, QuaternionReadOnly orientation)
   {
      return "Pose 3D: position = " + getTuple3DString(format, position) + ", orientation = " + getTuple4DString(format, orientation);
   }

   /**
    * Gets a representative {@code String} of {@code convexPolygon2D} as follows:
    *
    * <pre>
    * Convex Polygon 2D: vertices = [
    * ( 0.174, -0.452 ),
    * (-0.052, -0.173 ) ]
    * </pre>
    *
    * @param convexPolygon2D the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getConvexPolygon2DString(ConvexPolygon2DReadOnly convexPolygon2D)
   {
      return getConvexPolygon2DString(DEFAULT_FORMAT, convexPolygon2D);
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
    * </pre>
    * </p>
    *
    * @param format          the format to use for each number.
    * @param convexPolygon2D the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getConvexPolygon2DString(String format, ConvexPolygon2DReadOnly convexPolygon2D)
   {
      if (convexPolygon2D == null)
         return "null";
      else
         return getConvexPolygon2DString(format, convexPolygon2D.getPolygonVerticesView(), convexPolygon2D.getNumberOfVertices());
   }

   /**
    * Gets a representative {@code String} of a convex polygon given a specific format to use.
    * <p>
    * Using the default format {@link #DEFAULT_FORMAT}, this provides a {@code String} as follows:
    *
    * <pre>
    * Convex Polygon 2D: vertices = [
    * ( 0.174, -0.452 ),
    * (-0.052, -0.173 ) ]
    * </pre>
    * </p>
    *
    * @param format           the format to use for each number.
    * @param vertices         the list of vertices of the polygon to get the string of. Not modified.
    * @param numberOfVertices the polygon size.
    * @return the representative {@code String}.
    */
   public static String getConvexPolygon2DString(String format, List<? extends Point2DReadOnly> vertices, int numberOfVertices)
   {
      if (numberOfVertices == 0)
         return "Convex Polygon 2D: vertices = []";

      String string = "Convex Polygon 2D: vertices = [\n";
      for (int i = 0; i < numberOfVertices - 1; i++)
         string += getTuple2DString(format, vertices.get(i)) + ",\n";
      string += getTuple2DString(format, vertices.get(numberOfVertices - 1)) + " ]";
      return string;
   }

   /**
    * Gets a representative {@code String} of a triangle as follows:
    *
    * <pre>
    * Triangle 3D: [( 0.174, -0.452, -0.222 ), (-0.052, -0.173, -0.371 ), (-0.558, -0.380,  0.130 )]
    * </pre>
    *
    * @param triangle3D the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getTriangle3DString(Triangle3DReadOnly triangle3D)
   {
      return getTriangle3DString(DEFAULT_FORMAT, triangle3D);
   }

   /**
    * Gets a representative {@code String} of a triangle as follows:
    * <p>
    * Using the default format {@link #DEFAULT_FORMAT}, this provides a {@code String} as follows:
    *
    * <pre>
    * Triangle 3D: [( 0.174, -0.452, -0.222 ), (-0.052, -0.173, -0.371 ), (-0.558, -0.380,  0.130 )]
    * </pre>
    * </p>
    *
    * @param format     the format to use for each number.
    * @param triangle3D the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getTriangle3DString(String format, Triangle3DReadOnly triangle3D)
   {
      if (triangle3D == null)
         return "null";
      else
         return getTriangle3DString(format, triangle3D.getA(), triangle3D.getB(), triangle3D.getC());
   }

   /**
    * Gets a representative {@code String} of a triangle as follows:
    * <p>
    * Using the default format {@link #DEFAULT_FORMAT}, this provides a {@code String} as follows:
    *
    * <pre>
    * Triangle 3D: [( 0.174, -0.452, -0.222 ), (-0.052, -0.173, -0.371 ), (-0.558, -0.380,  0.130 )]
    * </pre>
    * </p>
    *
    * @param format the format to use for each number.
    * @param a      the first vertex of the triangle to get the {@code String} of. Not modified.
    * @param b      the second vertex of the triangle to get the {@code String} of. Not modified.
    * @param c      the third vertex of the triangle to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getTriangle3DString(String format, Point3DReadOnly a, Point3DReadOnly b, Point3DReadOnly c)
   {
      return "Triangle 3D: [" + getTuple3DString(format, a) + ", " + getTuple3DString(format, b) + ", " + getTuple3DString(format, c) + "]";
   }

   /**
    * Gets a representative {@code String} of {@code vertex2DSupplier} as follows:
    *
    * <pre>
    * Vertex 2D supplier: vertices = [
    * ( 0.174, -0.452 ),
    * (-0.052, -0.173 ) ]
    * </pre>
    *
    * @param vertex2DSupplier the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getVertex2DSupplierString(Vertex2DSupplier vertex2DSupplier)
   {
      return getVertex2DSupplierString(DEFAULT_FORMAT, vertex2DSupplier);
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
    * </pre>
    * </p>
    *
    * @param format           the format to use for each number.
    * @param vertex2DSupplier the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getVertex2DSupplierString(String format, Vertex2DSupplier vertex2DSupplier)
   {
      if (vertex2DSupplier == null)
         return "null";

      if (vertex2DSupplier.getNumberOfVertices() == 0)
         return "Vertex 2D supplier: vertices = []";

      String string = "Vertex 2D supplier: vertices = [\n";
      for (int i = 0; i < vertex2DSupplier.getNumberOfVertices() - 1; i++)
         string += getTuple2DString(format, vertex2DSupplier.getVertex(i)) + ",\n";
      string += getTuple2DString(format, vertex2DSupplier.getVertex(vertex2DSupplier.getNumberOfVertices() - 1)) + " ]";
      return string;
   }

   /**
    * Gets a representative {@code String} of {@code vertex3DSupplier} as follows:
    *
    * <pre>
    * Vertex 3D supplier: vertices = [
    * ( 0.174, -0.452, -0.222 ),
    * (-0.052, -0.173, -0.371 ) ]
    * </pre>
    *
    * @param vertex3DSupplier the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getVertex3DSupplierString(Vertex3DSupplier vertex3DSupplier)
   {
      return getVertex3DSupplierString(DEFAULT_FORMAT, vertex3DSupplier);
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
    * </pre>
    * </p>
    *
    * @param format           the format to use for each number.
    * @param vertex3DSupplier the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getVertex3DSupplierString(String format, Vertex3DSupplier vertex3DSupplier)
   {
      if (vertex3DSupplier == null)
         return "null";

      if (vertex3DSupplier.getNumberOfVertices() == 0)
         return "Vertex 3D supplier: vertices = []";

      String string = "Vertex 3D supplier: vertices = [\n";
      for (int i = 0; i < vertex3DSupplier.getNumberOfVertices() - 1; i++)
         string += getTuple3DString(format, vertex3DSupplier.getVertex(i)) + ",\n";
      string += getTuple3DString(format, vertex3DSupplier.getVertex(vertex3DSupplier.getNumberOfVertices() - 1)) + " ]";
      return string;
   }
}
