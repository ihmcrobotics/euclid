package us.ihmc.euclid.referenceFrame.tools;

import static us.ihmc.euclid.referenceFrame.tools.EuclidFrameIOTools.getFrameTuple3DString;
import static us.ihmc.euclid.shape.tools.EuclidShapeIOTools.*;
import static us.ihmc.euclid.tools.EuclidCoreIOTools.DEFAULT_FORMAT;

import us.ihmc.euclid.referenceFrame.collision.interfaces.EuclidFrameShape3DCollisionResultReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameBox3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameCapsule3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameCylinder3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameEllipsoid3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePointShape3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameRamp3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameShape3DPoseReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameShape3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameSphere3DReadOnly;
import us.ihmc.euclid.referenceFrame.polytope.interfaces.FrameConvexPolytope3DReadOnly;
import us.ihmc.euclid.referenceFrame.polytope.interfaces.FrameFace3DReadOnly;
import us.ihmc.euclid.referenceFrame.polytope.interfaces.FrameHalfEdge3DReadOnly;
import us.ihmc.euclid.referenceFrame.polytope.interfaces.FrameVertex3DReadOnly;
import us.ihmc.euclid.shape.tools.EuclidShapeIOTools;

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
      if (capsule3D == null)
         return "null";
      else
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
      if (cylinder3D == null)
         return "null";
      else
         return getCylinder3DString(format, cylinder3D) + " - " + cylinder3D.getReferenceFrame();
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
         return getEllipsoid3DString(format, ellipsoid3D) + " - " + ellipsoid3D.getReferenceFrame();
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
      if (pointShape3D == null)
         return "null";
      else
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
         return getRamp3DString(format, ramp3D) + " - " + ramp3D.getReferenceFrame();
   }

   /**
    * Gets a representative {@code String} of {@code sphere3D} as follows:
    *
    * <pre>
    * Sphere 3D: [position: (-0.362, -0.617,  0.066 ), radius:  0.906] - worldFrame
    * </pre>
    *
    * @param sphere3D the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getFrameSphere3DString(FrameSphere3DReadOnly sphere3D)
   {
      return getFrameSphere3DString(DEFAULT_FORMAT, sphere3D);
   }

   /**
    * Gets a representative {@code String} of {@code sphere3D} given a specific format to use.
    * <p>
    * Using the default format {@link #DEFAULT_FORMAT}, this provides a {@code String} as follows:
    *
    * <pre>
    * Sphere 3D: [position: (-0.362, -0.617,  0.066 ), radius:  0.906] - worldFrame
    * </pre>
    * </p>
    *
    * @param format   the format to use for each number.
    * @param sphere3D the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getFrameSphere3DString(String format, FrameSphere3DReadOnly sphere3D)
   {
      if (sphere3D == null)
         return "null";
      else
         return getSphere3DString(format, sphere3D) + " - " + sphere3D.getReferenceFrame();
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

   /**
    * Gets the representative {@code String} of {@code euclidShape3DCollisionResult} as follows:<br>
    * When shapes are colliding:
    *
    * <pre>
    * Collision test result: colliding, depth: 0.539
    * Shape A: Box3D - shapeFrameA, location: ( 0.540,  0.110,  0.319 ) - locationFrameA, normal: ( 0.540,  0.110,  0.319 ) - normalFrameB
    * Shape B: Capsule3D - shapeFrameB, location: ( 0.540,  0.110,  0.319 ) - locationFrameB, normal: ( 0.540,  0.110,  0.319 ) - normalFrameB
    * </pre>
    *
    * When shapes are not colliding:
    *
    * <pre>
    * Collision test result: non-colliding, separating distance: 0.539
    * Shape A: Box3D - shapeFrameA, location: ( 0.540,  0.110,  0.319 ) - locationFrameA, normal: ( 0.540,  0.110,  0.319 ) - normalFrameB
    * Shape B: Capsule3D - shapeFrameB, location: ( 0.540,  0.110,  0.319 ) - locationFrameB, normal: ( 0.540,  0.110,  0.319 ) - normalFrameB
    * </pre>
    *
    * @param euclidShape3DCollisionResult the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getEuclidFrameShape3DCollisionResultString(EuclidFrameShape3DCollisionResultReadOnly euclidShape3DCollisionResult)
   {
      return getEuclidFrameShape3DCollisionResultString(DEFAULT_FORMAT, euclidShape3DCollisionResult);
   }

   /**
    * Gets the representative {@code String} of {@code euclidShape3DCollisionResult} given a specific
    * format to use.
    * <p>
    * Using the default format {@link #DEFAULT_FORMAT}, this provides a {@code String} as follows:<br>
    * When shapes are colliding:
    *
    * <pre>
    * Collision test result: colliding, depth: 0.539
    * Shape A: Box3D - shapeFrameA, location: ( 0.540,  0.110,  0.319 ) - locationFrameA, normal: ( 0.540,  0.110,  0.319 ) - normalFrameB
    * Shape B: Capsule3D - shapeFrameB, location: ( 0.540,  0.110,  0.319 ) - locationFrameB, normal: ( 0.540,  0.110,  0.319 ) - normalFrameB
    * </pre>
    *
    * When shapes are not colliding:
    *
    * <pre>
    * Collision test result: non-colliding, separating distance: 0.539
    * Shape A: Box3D - shapeFrameA, location: ( 0.540,  0.110,  0.319 ) - locationFrameA, normal: ( 0.540,  0.110,  0.319 ) - normalFrameB
    * Shape B: Capsule3D - shapeFrameB, location: ( 0.540,  0.110,  0.319 ) - locationFrameB, normal: ( 0.540,  0.110,  0.319 ) - normalFrameB
    * </pre>
    * </p>
    *
    * @param format                       the format to use for each number.
    * @param euclidShape3DCollisionResult the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getEuclidFrameShape3DCollisionResultString(String format, EuclidFrameShape3DCollisionResultReadOnly euclidShape3DCollisionResult)
   {
      if (euclidShape3DCollisionResult == null)
         return "null";

      String string = "Collision test result: ";
      if (euclidShape3DCollisionResult.areShapesColliding())
      {
         string += "colliding, depth: " + euclidShape3DCollisionResult.getSignedDistance() + "\n";
      }
      else
      {
         string += "non-colliding, separating distance: " + euclidShape3DCollisionResult.getSignedDistance() + "\n";
      }

      FrameShape3DReadOnly shapeA = euclidShape3DCollisionResult.getShapeA();
      FrameShape3DReadOnly shapeB = euclidShape3DCollisionResult.getShapeB();

      string += "Shape A: " + (shapeA == null ? "null" : (shapeA.getClass().getSimpleName() + " - " + shapeA.getReferenceFrame().getName()));
      string += ", location: " + getFrameTuple3DString(format, euclidShape3DCollisionResult.getPointOnA());
      string += ", normal: " + getFrameTuple3DString(format, euclidShape3DCollisionResult.getNormalOnA()) + "\n";
      string += "Shape B: " + (shapeB == null ? "null" : (shapeB.getClass().getSimpleName() + " - " + shapeB.getReferenceFrame().getName()));
      string += ", location: " + getFrameTuple3DString(format, euclidShape3DCollisionResult.getPointOnB());
      string += ", normal: " + getFrameTuple3DString(format, euclidShape3DCollisionResult.getNormalOnB());
      return string;
   }

   /**
    * Gets the representative {@code String} of {@code vertex3D} as follows:
    *
    * <pre>
    * Vertex 3D: (-1.004, -3.379, -0.387 ), number of edges: 3
    *         [(-1.004, -3.379, -0.387 ); ( 1.372, -3.150,  0.556 )]
    *         [(-1.004, -3.379, -0.387 ); (-0.937, -3.539, -0.493 )]
    *         [(-1.004, -3.379, -0.387 ); (-1.046, -3.199, -0.303 )]
    *         worldFrame
    * </pre>
    *
    * @param vertex3D the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getFrameVertex3DString(FrameVertex3DReadOnly vertex3D)
   {
      return getFrameVertex3DString(DEFAULT_FORMAT, vertex3D);
   }

   /**
    * Gets the representative {@code String} of {@code vertex3D} given a specific format to use.
    * <p>
    * Using the default format {@link #DEFAULT_FORMAT}, this provides a {@code String} as follows:
    *
    * <pre>
    * Vertex 3D: (-1.004, -3.379, -0.387 ), number of edges: 3
    *         [(-1.004, -3.379, -0.387 ); ( 1.372, -3.150,  0.556 )]
    *         [(-1.004, -3.379, -0.387 ); (-0.937, -3.539, -0.493 )]
    *         [(-1.004, -3.379, -0.387 ); (-1.046, -3.199, -0.303 )]
    *         worldFrame
    * </pre>
    * </p>
    *
    * @param format   the format to use for each number.
    * @param vertex3D the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getFrameVertex3DString(String format, FrameVertex3DReadOnly vertex3D)
   {
      if (vertex3D == null)
         return "null";
      else
         return EuclidShapeIOTools.getVertex3DString(format, vertex3D) + "\n\t" + vertex3D.getReferenceFrame();
   }

   /**
    * Gets the representative {@code String} of {@code halfEdge3D} as follows:
    *
    * <pre>
    * Half-edge 3D: [( 2.350,  4.284,  0.427 ); ( 3.310,  6.118, -3.108 )]
    *    Twin    : [( 3.310,  6.118, -3.108 ); ( 2.350,  4.284,  0.427 )]
    *    Next    : [( 3.310,  6.118, -3.108 ); ( 3.411,  2.581, -3.144 )]
    *    Previous: [( 3.411,  2.581, -3.144 ); ( 2.350,  4.284,  0.427 )]
    *    Face: centroid: ( 3.024,  4.328, -1.941 ), normal: ( 0.961,  0.025,  0.274 )
    *    worldFrame
    * </pre>
    *
    * @param halfEdge3D the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getFrameHalfEdge3DString(FrameHalfEdge3DReadOnly halfEdge3D)
   {
      return getFrameHalfEdge3DString(DEFAULT_FORMAT, halfEdge3D);
   }

   /**
    * Gets the representative {@code String} of {@code halfEdge3D} given a specific format to use.
    * <p>
    * Using the default format {@link #DEFAULT_FORMAT}, this provides a {@code String} as follows:
    *
    * <pre>
    * Half-edge 3D: [( 2.350,  4.284,  0.427 ); ( 3.310,  6.118, -3.108 )]
    *    Twin    : [( 3.310,  6.118, -3.108 ); ( 2.350,  4.284,  0.427 )]
    *    Next    : [( 3.310,  6.118, -3.108 ); ( 3.411,  2.581, -3.144 )]
    *    Previous: [( 3.411,  2.581, -3.144 ); ( 2.350,  4.284,  0.427 )]
    *    Face: centroid: ( 3.024,  4.328, -1.941 ), normal: ( 0.961,  0.025,  0.274 )
    *    worldFrame
    * </pre>
    * </p>
    *
    * @param format     the format to use for each number.
    * @param halfEdge3D the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getFrameHalfEdge3DString(String format, FrameHalfEdge3DReadOnly halfEdge3D)
   {
      if (halfEdge3D == null)
         return "null";
      else
         return EuclidShapeIOTools.getHalfEdge3DString(format, halfEdge3D) + "\n\t" + halfEdge3D.getReferenceFrame();
   }

   /**
    * Gets the representative {@code String} of {@code face3D} as follows:
    *
    * <pre>
    * Face 3D: centroid: ( 2.621, -0.723, -1.355 ), normal: ( 0.903, -0.202,  0.378 ), area:  0.180, number of edges: 4
    *    [( 2.590, -0.496, -1.161 ); ( 2.746, -0.536, -1.554 )]
    *    [( 2.746, -0.536, -1.554 ); ( 2.651, -0.950, -1.549 )]
    *    [( 2.651, -0.950, -1.549 ); ( 2.496, -0.910, -1.157 )]
    *    [( 2.496, -0.910, -1.157 ); ( 2.590, -0.496, -1.161 )]
    *    worldFrame
    * </pre>
    *
    * @param face3D the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getFrameFace3DString(FrameFace3DReadOnly face3D)
   {
      return getFrameFace3DString(DEFAULT_FORMAT, face3D);
   }

   /**
    * Gets the representative {@code String} of {@code face3D} given a specific format to use.
    * <p>
    * Using the default format {@link #DEFAULT_FORMAT}, this provides a {@code String} as follows:
    *
    * <pre>
    * Face 3D: centroid: ( 2.621, -0.723, -1.355 ), normal: ( 0.903, -0.202,  0.378 ), area:  0.180, number of edges: 4
    *    [( 2.590, -0.496, -1.161 ); ( 2.746, -0.536, -1.554 )]
    *    [( 2.746, -0.536, -1.554 ); ( 2.651, -0.950, -1.549 )]
    *    [( 2.651, -0.950, -1.549 ); ( 2.496, -0.910, -1.157 )]
    *    [( 2.496, -0.910, -1.157 ); ( 2.590, -0.496, -1.161 )]
    *    worldFrame
    * </pre>
    * </p>
    *
    * @param format the format to use for each number.
    * @param face3D the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getFrameFace3DString(String format, FrameFace3DReadOnly face3D)
   {
      if (face3D == null)
         return "null";
      else
         return EuclidShapeIOTools.getFace3DString(format, face3D) + "\n\t" + face3D.getReferenceFrame();
   }

   /**
    * Gets the representative {@code String} of {@code convexPolytope3D} as follows:
    *
    * <pre>
    * Convex polytope 3D: number of: [faces: 4, edges: 12, vertices: 4
    * Face list:
    *    centroid: ( 0.582, -0.023,  0.160 ), normal: ( 0.516, -0.673,  0.530 )
    *    centroid: ( 0.420,  0.176,  0.115 ), normal: (-0.038,  0.895, -0.444 )
    *    centroid: ( 0.264, -0.253, -0.276 ), normal: ( 0.506,  0.225, -0.833 )
    *    centroid: ( 0.198, -0.176, -0.115 ), normal: (-0.643, -0.374,  0.668 )
    * Edge list:
    *    [( 0.674,  0.482,  0.712 ); ( 0.870,  0.251,  0.229 )]
    *    [( 0.870,  0.251,  0.229 ); ( 0.204, -0.803, -0.461 )]
    *    [( 0.204, -0.803, -0.461 ); ( 0.674,  0.482,  0.712 )]
    *    [( 0.870,  0.251,  0.229 ); ( 0.674,  0.482,  0.712 )]
    *    [( 0.674,  0.482,  0.712 ); (-0.283, -0.207, -0.595 )]
    *    [(-0.283, -0.207, -0.595 ); ( 0.870,  0.251,  0.229 )]
    *    [( 0.204, -0.803, -0.461 ); ( 0.870,  0.251,  0.229 )]
    *    [( 0.870,  0.251,  0.229 ); (-0.283, -0.207, -0.595 )]
    *    [(-0.283, -0.207, -0.595 ); ( 0.204, -0.803, -0.461 )]
    *    [( 0.674,  0.482,  0.712 ); ( 0.204, -0.803, -0.461 )]
    *    [( 0.204, -0.803, -0.461 ); (-0.283, -0.207, -0.595 )]
    *    [(-0.283, -0.207, -0.595 ); ( 0.674,  0.482,  0.712 )]
    * Vertex list:
    *    ( 0.674,  0.482,  0.712 )
    *    ( 0.870,  0.251,  0.229 )
    *    ( 0.204, -0.803, -0.461 )
    *    (-0.283, -0.207, -0.595 )
    * worldFrame
    * </pre>
    *
    * @param convexPolytope3D the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getFrameConvexPolytope3DString(FrameConvexPolytope3DReadOnly convexPolytope3D)
   {
      return getFrameConvexPolytope3DString(DEFAULT_FORMAT, convexPolytope3D);
   }

   /**
    * Gets the representative {@code String} of {@code convexPolytope3D} given a specific format to
    * use.
    * <p>
    * Using the default format {@link #DEFAULT_FORMAT}, this provides a {@code String} as follows:
    *
    * <pre>
    * Convex polytope 3D: number of: [faces: 4, edges: 12, vertices: 4
    * Face list:
    *    centroid: ( 0.582, -0.023,  0.160 ), normal: ( 0.516, -0.673,  0.530 )
    *    centroid: ( 0.420,  0.176,  0.115 ), normal: (-0.038,  0.895, -0.444 )
    *    centroid: ( 0.264, -0.253, -0.276 ), normal: ( 0.506,  0.225, -0.833 )
    *    centroid: ( 0.198, -0.176, -0.115 ), normal: (-0.643, -0.374,  0.668 )
    * Edge list:
    *    [( 0.674,  0.482,  0.712 ); ( 0.870,  0.251,  0.229 )]
    *    [( 0.870,  0.251,  0.229 ); ( 0.204, -0.803, -0.461 )]
    *    [( 0.204, -0.803, -0.461 ); ( 0.674,  0.482,  0.712 )]
    *    [( 0.870,  0.251,  0.229 ); ( 0.674,  0.482,  0.712 )]
    *    [( 0.674,  0.482,  0.712 ); (-0.283, -0.207, -0.595 )]
    *    [(-0.283, -0.207, -0.595 ); ( 0.870,  0.251,  0.229 )]
    *    [( 0.204, -0.803, -0.461 ); ( 0.870,  0.251,  0.229 )]
    *    [( 0.870,  0.251,  0.229 ); (-0.283, -0.207, -0.595 )]
    *    [(-0.283, -0.207, -0.595 ); ( 0.204, -0.803, -0.461 )]
    *    [( 0.674,  0.482,  0.712 ); ( 0.204, -0.803, -0.461 )]
    *    [( 0.204, -0.803, -0.461 ); (-0.283, -0.207, -0.595 )]
    *    [(-0.283, -0.207, -0.595 ); ( 0.674,  0.482,  0.712 )]
    * Vertex list:
    *    ( 0.674,  0.482,  0.712 )
    *    ( 0.870,  0.251,  0.229 )
    *    ( 0.204, -0.803, -0.461 )
    *    (-0.283, -0.207, -0.595 )
    * worldFrame
    * </pre>
    * </p>
    *
    * @param format           the format to use for each number.
    * @param convexPolytope3D the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getFrameConvexPolytope3DString(String format, FrameConvexPolytope3DReadOnly convexPolytope3D)
   {
      if (convexPolytope3D == null)
         return "null";
      else
         return EuclidShapeIOTools.getConvexPolytope3DString(format, convexPolytope3D) + "\n" + convexPolytope3D.getReferenceFrame();
   }
}
