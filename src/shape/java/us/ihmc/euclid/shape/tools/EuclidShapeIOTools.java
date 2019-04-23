package us.ihmc.euclid.shape.tools;

import static us.ihmc.euclid.tools.EuclidCoreIOTools.*;

import java.util.Collection;

import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.shape.collision.interfaces.EuclidShape3DCollisionResultReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.ConvexPolytope3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Face3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.HalfEdge3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Vertex3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Box3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Capsule3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Cylinder3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Ellipsoid3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.PointShape3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Ramp3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Shape3DPoseReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Sphere3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Torus3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

/**
 * {@code EuclidShapeIOTools}, as {@link EuclidCoreIOTools}, is intended to gather the input &
 * output tools for printing, saving, and loading geometry objects.
 * <p>
 * At this time, only a few print tools are offered, additional features will come in future
 * releases.
 * </p>
 *
 * @author Sylvain Bertrand
 */
public class EuclidShapeIOTools
{
   private EuclidShapeIOTools()
   {
      // Suppresses default constructor, ensuring non-instantiability.
   }

   /**
    * Gets the representative {@code String} of {@code box3D} as follows:
    * 
    * <pre>
    * Box 3D: [position: ( 0.540,  0.110,  0.319 ), yaw-pitch-roll: (-2.061, -0.904, -1.136), size: ( 0.191,  0.719,  0.479 )]
    * </pre>
    * 
    * @param box3D the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getBox3DString(Box3DReadOnly box3D)
   {
      return getBox3DString(DEFAULT_FORMAT, box3D);
   }

   /**
    * Gets the representative {@code String} of {@code box3D} given a specific format to use.
    * <p>
    * Using the default format {@link #DEFAULT_FORMAT}, this provides a {@code String} as follows:
    * 
    * <pre>
    * Box 3D: [position: ( 0.540,  0.110,  0.319 ), yaw-pitch-roll: (-2.061, -0.904, -1.136), size: ( 0.191,  0.719,  0.479 )]
    * </pre>
    * </p>
    * 
    * @param format the format to use for each number.
    * @param box3D the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getBox3DString(String format, Box3DReadOnly box3D)
   {
      if (box3D == null)
         return "null";
      else
         return getBox3DString(format, box3D.getPosition(), box3D.getOrientation(), box3D.getSize());
   }

   /**
    * Gets the representative {@code String} of {@code box3D} given a specific format to use.
    * <p>
    * Using the default format {@link #DEFAULT_FORMAT}, this provides a {@code String} as follows:
    * 
    * <pre>
    * Box 3D: [position: ( 0.540,  0.110,  0.319 ), yaw-pitch-roll: (-2.061, -0.904, -1.136), size: ( 0.191,  0.719,  0.479 )]
    * </pre>
    * </p>
    * 
    * @param format the format to use for each number.
    * @param position the location of the box. Not modified.
    * @param orientation the orientation of the box. Not modified.
    * @param size the box's size. Not modified.
    * @return the representative {@code String}.
    */
   public static String getBox3DString(String format, Tuple3DReadOnly position, Orientation3DReadOnly orientation, Vector3DReadOnly size)
   {
      return "Box 3D: [position: " + getTuple3DString(format, position) + ", " + getStringAsYawPitchRoll(format, orientation) + ", size: "
            + getTuple3DString(format, size) + "]";
   }

   /**
    * Gets a representative {@code String} of {@code capsule3D} as follows:
    * 
    * <pre>
    * Capsule 3D: [position: (-0.362, -0.617,  0.066 ), axis: ( 0.634, -0.551, -0.543 ), length:  0.170, radius:  0.906]
    * </pre>
    * 
    * @param capsule3D the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getCapsule3DString(Capsule3DReadOnly capsule3D)
   {
      return getCapsule3DString(DEFAULT_FORMAT, capsule3D);
   }

   /**
    * Gets a representative {@code String} of {@code capsule3D} given a specific format to use.
    * <p>
    * Using the default format {@link #DEFAULT_FORMAT}, this provides a {@code String} as follows:
    * 
    * <pre>
    * Capsule 3D: [position: (-0.362, -0.617,  0.066 ), axis: ( 0.634, -0.551, -0.543 ), length:  0.170, radius:  0.906]
    * </pre>
    * </p>
    * 
    * @param format the format to use for each number.
    * @param capsule3D the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getCapsule3DString(String format, Capsule3DReadOnly capsule3D)
   {
      return getCapsule3DString(format, capsule3D.getPosition(), capsule3D.getAxis(), capsule3D.getLength(), capsule3D.getRadius());
   }

   /**
    * Gets a representative {@code String} of {@code capsule3D} given a specific format to use.
    * <p>
    * Using the default format {@link #DEFAULT_FORMAT}, this provides a {@code String} as follows:
    * 
    * <pre>
    * Capsule 3D: [position: (-0.362, -0.617,  0.066 ), axis: ( 0.634, -0.551, -0.543 ), length:  0.170, radius:  0.906]
    * </pre>
    * </p>
    * 
    * @param format the format to use for each number.
    * @param position the location of the capsule. Not modified.
    * @param axis the capsule's axis. Not modified.
    * @param length the capsule's length.
    * @param radius the capsule's radius.
    * @return the representative {@code String}.
    */
   public static String getCapsule3DString(String format, Tuple3DReadOnly position, Vector3DReadOnly axis, double length, double radius)
   {
      return "Capsule 3D: [position: " + getTuple3DString(format, position) + ", axis: " + getTuple3DString(format, axis) + ", length: "
            + String.format(format, length) + ", radius: " + String.format(format, radius) + "]";
   }

   /**
    * Gets a representative {@code String} of {@code cylinder3D} as follows:
    * 
    * <pre>
    * Cylinder 3D: [position: (-0.362, -0.617,  0.066 ), axis: ( 0.634, -0.551, -0.543 ), length:  0.170, radius:  0.906]
    * </pre>
    * 
    * @param cylinder3D the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getCylinder3DString(Cylinder3DReadOnly cylinder3D)
   {
      return getCylinder3DString(DEFAULT_FORMAT, cylinder3D);
   }

   /**
    * Gets a representative {@code String} of {@code cylinder3D} given a specific format to use.
    * <p>
    * Using the default format {@link #DEFAULT_FORMAT}, this provides a {@code String} as follows:
    * 
    * <pre>
    * Cylinder 3D: [position: (-0.362, -0.617,  0.066 ), axis: ( 0.634, -0.551, -0.543 ), length:  0.170, radius:  0.906]
    * </pre>
    * </p>
    * 
    * @param format the format to use for each number.
    * @param cylinder3D the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getCylinder3DString(String format, Cylinder3DReadOnly cylinder3D)
   {
      return getCylinder3DString(format, cylinder3D.getPosition(), cylinder3D.getAxis(), cylinder3D.getLength(), cylinder3D.getRadius());
   }

   /**
    * Gets a representative {@code String} of {@code cylinder3D} given a specific format to use.
    * <p>
    * Using the default format {@link #DEFAULT_FORMAT}, this provides a {@code String} as follows:
    * 
    * <pre>
    * Cylinder 3D: [position: (-0.362, -0.617,  0.066 ), axis: ( 0.634, -0.551, -0.543 ), length:  0.170, radius:  0.906]
    * </pre>
    * </p>
    * 
    * @param format the format to use for each number.
    * @param position the location of the cylinder. Not modified.
    * @param axis the cylinder's axis. Not modified.
    * @param length the cylinder's length.
    * @param radius the cylinder's radius.
    * @return the representative {@code String}.
    */
   public static String getCylinder3DString(String format, Tuple3DReadOnly position, Vector3DReadOnly axis, double length, double radius)
   {
      return "Cylinder 3D: [position: " + getTuple3DString(format, position) + ", axis: " + getTuple3DString(format, axis) + ", length: "
            + String.format(format, length) + ", radius: " + String.format(format, radius) + "]";
   }

   /**
    * Gets the representative {@code String} of {@code ellipsoid3D} as follows:
    * 
    * <pre>
    * Ellipsoid 3D: [position: ( 0.540,  0.110,  0.319 ), yaw-pitch-roll: (-2.061, -0.904, -1.136), radii: ( 0.191,  0.719,  0.479 )]
    * </pre>
    * 
    * @param ellipsoid3D the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getEllipsoid3DString(Ellipsoid3DReadOnly ellipsoid3D)
   {
      return getEllipsoid3DString(DEFAULT_FORMAT, ellipsoid3D);
   }

   /**
    * Gets the representative {@code String} of {@code ellipsoid3D} given a specific format to use.
    * <p>
    * Using the default format {@link #DEFAULT_FORMAT}, this provides a {@code String} as follows:
    * 
    * <pre>
    * Ellipsoid 3D: [position: ( 0.540,  0.110,  0.319 ), yaw-pitch-roll: (-2.061, -0.904, -1.136), radii: ( 0.191,  0.719,  0.479 )]
    * </pre>
    * </p>
    * 
    * @param format the format to use for each number.
    * @param ellipsoid3D the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getEllipsoid3DString(String format, Ellipsoid3DReadOnly ellipsoid3D)
   {
      if (ellipsoid3D == null)
         return "null";
      else
         return getEllipsoid3DString(format, ellipsoid3D.getPosition(), ellipsoid3D.getOrientation(), ellipsoid3D.getRadii());
   }

   /**
    * Gets the representative {@code String} of {@code ellipsoid3D} given a specific format to use.
    * <p>
    * Using the default format {@link #DEFAULT_FORMAT}, this provides a {@code String} as follows:
    * 
    * <pre>
    * Ellipsoid 3D: [position: ( 0.540,  0.110,  0.319 ), yaw-pitch-roll: (-2.061, -0.904, -1.136), radii: ( 0.191,  0.719,  0.479 )]
    * </pre>
    * </p>
    * 
    * @param format the format to use for each number.
    * @param position the location of the ellipsoid. Not modified.
    * @param orientation the orientation of the ellipsoid. Not modified.
    * @param radii the ellipsoid's radii. Not modified.
    * @return the representative {@code String}.
    */
   public static String getEllipsoid3DString(String format, Tuple3DReadOnly position, Orientation3DReadOnly orientation, Vector3DReadOnly radii)
   {
      return "Ellipsoid 3D: [position: " + getTuple3DString(format, position) + ", " + getStringAsYawPitchRoll(format, orientation) + ", radii: "
            + getTuple3DString(format, radii) + "]";
   }

   /**
    * Gets a representative {@code String} of {@code pointShape3D} as follows:
    * 
    * <pre>
    * Point shape 3D: (-0.362, -0.617,  0.066 )
    * </pre>
    * 
    * @param pointShape3D the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getPointShape3DString(PointShape3DReadOnly pointShape3D)
   {
      return getPointShape3DString(DEFAULT_FORMAT, pointShape3D);
   }

   /**
    * Gets a representative {@code String} of {@code pointShape3D} given a specific format to use.
    * <p>
    * Using the default format {@link #DEFAULT_FORMAT}, this provides a {@code String} as follows:
    * 
    * <pre>
    * Point shape 3D: (-0.362, -0.617,  0.066 )
    * </pre>
    * </p>
    * 
    * @param format the format to use for each number.
    * @param pointShape3D the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getPointShape3DString(String format, PointShape3DReadOnly pointShape3D)
   {
      return getPointShape3DString(format, (Tuple3DReadOnly) pointShape3D);
   }

   /**
    * Gets a representative {@code String} of {@code pointShape3D} given a specific format to use.
    * <p>
    * Using the default format {@link #DEFAULT_FORMAT}, this provides a {@code String} as follows:
    * 
    * <pre>
    * Point shape 3D: (-0.362, -0.617,  0.066 )
    * </pre>
    * </p>
    * 
    * @param format the format to use for each number.
    * @param position the location of the point shape. Not modified.
    * @return the representative {@code String}.
    */
   public static String getPointShape3DString(String format, Tuple3DReadOnly position)
   {
      return "Point shape 3D: " + getTuple3DString(format, position);
   }

   /**
    * Gets the representative {@code String} of {@code ramp3D} as follows:
    * 
    * <pre>
    * Ramp 3D: [position: ( 0.540,  0.110,  0.319 ), yaw-pitch-roll: (-2.061, -0.904, -1.136), size: ( 0.191,  0.719,  0.479 )]
    * </pre>
    * 
    * @param ramp3D the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getRamp3DString(Ramp3DReadOnly ramp3D)
   {
      return getRamp3DString(DEFAULT_FORMAT, ramp3D);
   }

   /**
    * Gets the representative {@code String} of {@code ramp3D} given a specific format to use.
    * <p>
    * Using the default format {@link #DEFAULT_FORMAT}, this provides a {@code String} as follows:
    * 
    * <pre>
    * Ramp 3D: [position: ( 0.540,  0.110,  0.319 ), yaw-pitch-roll: (-2.061, -0.904, -1.136), size: ( 0.191,  0.719,  0.479 )]
    * </pre>
    * </p>
    * 
    * @param format the format to use for each number.
    * @param ramp3D the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getRamp3DString(String format, Ramp3DReadOnly ramp3D)
   {
      if (ramp3D == null)
         return "null";
      else
         return getRamp3DString(format, ramp3D.getPosition(), ramp3D.getOrientation(), ramp3D.getSize());
   }

   /**
    * Gets the representative {@code String} of {@code ramp3D} given a specific format to use.
    * <p>
    * Using the default format {@link #DEFAULT_FORMAT}, this provides a {@code String} as follows:
    * 
    * <pre>
    * Ramp 3D: [position: ( 0.540,  0.110,  0.319 ), yaw-pitch-roll: (-2.061, -0.904, -1.136), size: ( 0.191,  0.719,  0.479 )]
    * </pre>
    * </p>
    * 
    * @param format the format to use for each number.
    * @param position the location of the ramp. Not modified.
    * @param orientation the orientation of the ramp. Not modified.
    * @param size the ramp's size. Not modified.
    * @return the representative {@code String}.
    */
   public static String getRamp3DString(String format, Tuple3DReadOnly position, Orientation3DReadOnly orientation, Vector3DReadOnly size)
   {
      return "Ramp 3D: [position: " + getTuple3DString(format, position) + ", " + getStringAsYawPitchRoll(format, orientation) + ", size: "
            + getTuple3DString(format, size) + "]";
   }

   /**
    * Gets a representative {@code String} of {@code sphere3D} as follows:
    * 
    * <pre>
    * Sphere 3D: [position: (-0.362, -0.617,  0.066 ), radius:  0.906]
    * </pre>
    * 
    * @param sphere3D the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getSphere3DString(Sphere3DReadOnly sphere3D)
   {
      return getSphere3DString(DEFAULT_FORMAT, sphere3D);
   }

   /**
    * Gets a representative {@code String} of {@code sphere3D} given a specific format to use.
    * <p>
    * Using the default format {@link #DEFAULT_FORMAT}, this provides a {@code String} as follows:
    * 
    * <pre>
    * Sphere 3D: [position: (-0.362, -0.617,  0.066 ), radius:  0.906]
    * </pre>
    * </p>
    * 
    * @param format the format to use for each number.
    * @param sphere3D the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getSphere3DString(String format, Sphere3DReadOnly sphere3D)
   {
      return getSphere3DString(format, sphere3D.getPosition(), sphere3D.getRadius());
   }

   /**
    * Gets a representative {@code String} of {@code sphere3D} given a specific format to use.
    * <p>
    * Using the default format {@link #DEFAULT_FORMAT}, this provides a {@code String} as follows:
    * 
    * <pre>
    * Sphere 3D: [position: (-0.362, -0.617,  0.066 ), radius:  0.906]
    * </pre>
    * </p>
    * 
    * @param format the format to use for each number.
    * @param position the location of the sphere. Not modified.
    * @param radius the capsule's radius.
    * @return the representative {@code String}.
    */
   public static String getSphere3DString(String format, Tuple3DReadOnly position, double radius)
   {
      return "Sphere 3D: [position: " + getTuple3DString(format, position) + ", radius: " + String.format(format, radius) + "]";
   }

   /**
    * Gets a representative {@code String} of {@code torus3D} as follows:
    * 
    * <pre>
    * Torus 3D: [position: (-0.362, -0.617,  0.066 ), axis: ( 0.634, -0.551, -0.543 ), radius:  0.170, tube radius:  0.906]
    * </pre>
    * 
    * @param torus3D the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getTorus3DString(Torus3DReadOnly torus3D)
   {
      return getTorus3DString(DEFAULT_FORMAT, torus3D);
   }

   /**
    * Gets a representative {@code String} of {@code torus3D} given a specific format to use.
    * <p>
    * Using the default format {@link #DEFAULT_FORMAT}, this provides a {@code String} as follows:
    * 
    * <pre>
    * Torus 3D: [position: (-0.362, -0.617,  0.066 ), axis: ( 0.634, -0.551, -0.543 ), radius:  0.170, tube radius:  0.906]
    * </pre>
    * </p>
    * 
    * @param format the format to use for each number.
    * @param torus3D the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getTorus3DString(String format, Torus3DReadOnly torus3D)
   {
      return getTorus3DString(format, torus3D.getPosition(), torus3D.getAxis(), torus3D.getRadius(), torus3D.getTubeRadius());
   }

   /**
    * Gets a representative {@code String} of {@code capsule3D} given a specific format to use.
    * <p>
    * Using the default format {@link #DEFAULT_FORMAT}, this provides a {@code String} as follows:
    * 
    * <pre>
    * Torus 3D: [position: (-0.362, -0.617,  0.066 ), axis: ( 0.634, -0.551, -0.543 ), radius:  0.170, tube radius:  0.906]
    * </pre>
    * </p>
    * 
    * @param format the format to use for each number.
    * @param position the location of the torus. Not modified.
    * @param axis the torus' axis. Not modified.
    * @param radius the torus' radius.
    * @param tubeRadius the torus' tube radius.
    * @return the representative {@code String}.
    */
   public static String getTorus3DString(String format, Tuple3DReadOnly position, Vector3DReadOnly axis, double radius, double tubeRadius)
   {
      return "Torus 3D: [position: " + getTuple3DString(format, position) + ", axis: " + getTuple3DString(format, axis) + ", radius: "
            + String.format(format, radius) + ", tube radius: " + String.format(format, tubeRadius) + "]";
   }

   /**
    * Gets the representative {@code String} of {@code shape3DPose} as follows:
    * 
    * <pre>
    * Shape 3D pose: [position: ( 0.540,  0.110,  0.319 ), yaw-pitch-roll: (-2.061, -0.904, -1.136)]
    * </pre>
    * 
    * @param shape3DPose the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getShape3DPoseString(Shape3DPoseReadOnly shape3DPose)
   {
      return getShape3DPoseString(DEFAULT_FORMAT, shape3DPose.getShapeOrientation(), shape3DPose.getShapePosition());
   }

   /**
    * Gets the representative {@code String} of {@code shape3DPose} given a specific format to use.
    * <p>
    * Using the default format {@link #DEFAULT_FORMAT}, this provides a {@code String} as follows:
    * 
    * <pre>
    * Shape 3D pose: [position: ( 0.540,  0.110,  0.319 ), yaw-pitch-roll: (-2.061, -0.904, -1.136)]
    * </pre>
    * </p>
    * 
    * @param format the format to use for each number.
    * @param shape3DPose the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getShape3DPoseString(String format, Shape3DPoseReadOnly shape3DPose)
   {
      return getShape3DPoseString(format, shape3DPose.getShapeOrientation(), shape3DPose.getShapePosition());
   }

   /**
    * Gets the representative {@code String} of {@code shape3DPose} given a specific format to use.
    * <p>
    * Using the default format {@link #DEFAULT_FORMAT}, this provides a {@code String} as follows:
    * 
    * <pre>
    * Shape 3D pose: [position: ( 0.540,  0.110,  0.319 ), yaw-pitch-roll: (-2.061, -0.904, -1.136)]
    * </pre>
    * </p>
    * 
    * @param format the format to use for each number.
    * @param orientation the orientation part of the {@code shapePose3D}. Not modified.
    * @param position the position part of the {@code shapePose3D}. Not modified.
    * @return the representative {@code String}.
    */
   public static String getShape3DPoseString(String format, RotationMatrixReadOnly orientation, Point3DReadOnly position)
   {
      return "Shape 3D pose: [position: " + getTuple3DString(format, position) + ", " + getStringAsYawPitchRoll(format, orientation) + "]";
   }

   /**
    * Gets the representative {@code String} of {@code euclidShape3DCollisionResult} as follows:<br>
    * When shapes are colliding:
    * 
    * <pre>
    * Collision test result: colliding, depth: 0.539
    * Shape A: Box3D, location: ( 0.540,  0.110,  0.319 ), normal: ( 0.540,  0.110,  0.319 )
    * Shape B: Capsule3D, location: ( 0.540,  0.110,  0.319 ), normal: ( 0.540,  0.110,  0.319 )
    * </pre>
    * 
    * When shapes are not colliding:
    * 
    * <pre>
    * Collision test result: non-colliding, separating distance: 0.539
    * Shape A: Box3D, location: ( 0.540,  0.110,  0.319 ), normal: ( 0.540,  0.110,  0.319 )
    * Shape B: Capsule3D, location: ( 0.540,  0.110,  0.319 ), normal: ( 0.540,  0.110,  0.319 )
    * </pre>
    * 
    * @param euclidShape3DCollisionResult the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getEuclidShape3DCollisionResultString(EuclidShape3DCollisionResultReadOnly euclidShape3DCollisionResult)
   {
      return getEuclidShape3DCollisionResultString(DEFAULT_FORMAT, euclidShape3DCollisionResult);
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
    * Shape A: Box3D, location: ( 0.540,  0.110,  0.319 ), normal: ( 0.540,  0.110,  0.319 )
    * Shape B: Capsule3D, location: ( 0.540,  0.110,  0.319 ), normal: ( 0.540,  0.110,  0.319 )
    * </pre>
    * 
    * When shapes are not colliding:
    * 
    * <pre>
    * Collision test result: non-colliding, separating distance: 0.539
    * Shape A: Box3D, location: ( 0.540,  0.110,  0.319 ), normal: ( 0.540,  0.110,  0.319 )
    * Shape B: Capsule3D, location: ( 0.540,  0.110,  0.319 ), normal: ( 0.540,  0.110,  0.319 )
    * </pre>
    * </p>
    * 
    * @param format the format to use for each number.
    * @param euclidShape3DCollisionResult the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getEuclidShape3DCollisionResultString(String format, EuclidShape3DCollisionResultReadOnly euclidShape3DCollisionResult)
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
      string += "Shape A: " + (euclidShape3DCollisionResult.getShapeA() == null ? "null" : euclidShape3DCollisionResult.getShapeA().getClass().getSimpleName());
      string += ", location: " + getTuple3DString(format, euclidShape3DCollisionResult.getPointOnA());
      string += ", normal: " + getTuple3DString(format, euclidShape3DCollisionResult.getNormalOnA()) + "\n";
      string += "Shape B: " + (euclidShape3DCollisionResult.getShapeB() == null ? "null" : euclidShape3DCollisionResult.getShapeB().getClass().getSimpleName());
      string += ", location: " + getTuple3DString(format, euclidShape3DCollisionResult.getPointOnB());
      string += ", normal: " + getTuple3DString(format, euclidShape3DCollisionResult.getNormalOnB());
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
    * </pre>
    * 
    * @param vertex3D the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getVertex3DString(Vertex3DReadOnly vertex3D)
   {
      return getVertex3DString(DEFAULT_FORMAT, vertex3D);
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
    * </pre>
    * </p>
    * 
    * @param format the format to use for each number.
    * @param vertex3D the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getVertex3DString(String format, Vertex3DReadOnly vertex3D)
   {
      if (vertex3D == null)
         return "null";
      String string = "Vertex 3D: " + getTuple3DString(format, vertex3D) + ", number of edges: " + vertex3D.getNumberOfAssociatedEdges();
      string += getHalfEdge3DCollectionString(format, "\n\t", vertex3D.getAssociatedEdges());
      return string;
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
    * </pre>
    * 
    * @param halfEdge3D the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getHalfEdge3DString(HalfEdge3DReadOnly halfEdge3D)
   {
      return getHalfEdge3DString(DEFAULT_FORMAT, halfEdge3D);
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
    * </pre>
    * </p>
    * 
    * @param format the format to use for each number.
    * @param halfEdge3D the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getHalfEdge3DString(String format, HalfEdge3DReadOnly halfEdge3D)
   {
      if (halfEdge3D == null)
         return "null";
      String string = "Half-edge 3D: " + getHalfEdge3DShortString(format, halfEdge3D);
      string += "\n\tTwin    : " + getHalfEdge3DShortString(format, halfEdge3D.getTwin());
      string += "\n\tNext    : " + getHalfEdge3DShortString(format, halfEdge3D.getNext());
      string += "\n\tPrevious: " + getHalfEdge3DShortString(format, halfEdge3D.getPrevious());
      string += "\n\tFace: " + getFace3DShortString(format, halfEdge3D.getFace());
      return string;
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
    * </pre>
    * 
    * @param face3D the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getFace3DString(Face3DReadOnly face3D)
   {
      return getFace3DString(DEFAULT_FORMAT, face3D);
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
    * </pre>
    * </p>
    * 
    * @param format the format to use for each number.
    * @param face3D the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getFace3DString(String format, Face3DReadOnly face3D)
   {
      if (face3D == null)
         return "null";

      return "Face 3D: " + getFace3DShortString(format, face3D) + ", area: " + String.format(format, face3D.getArea()) + ", number of edges: "
            + face3D.getNumberOfEdges() + getHalfEdge3DCollectionString(format, "\n\t", face3D.getEdges());
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
    * </pre>
    * 
    * @param convexPolytope3D the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getConvexPolytope3DString(ConvexPolytope3DReadOnly convexPolytope3D)
   {
      return getConvexPolytope3DString(DEFAULT_FORMAT, convexPolytope3D);
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
    * 	centroid: ( 0.582, -0.023,  0.160 ), normal: ( 0.516, -0.673,  0.530 )
    * 	centroid: ( 0.420,  0.176,  0.115 ), normal: (-0.038,  0.895, -0.444 )
    * 	centroid: ( 0.264, -0.253, -0.276 ), normal: ( 0.506,  0.225, -0.833 )
    * 	centroid: ( 0.198, -0.176, -0.115 ), normal: (-0.643, -0.374,  0.668 )
    * Edge list: 
    * 	[( 0.674,  0.482,  0.712 ); ( 0.870,  0.251,  0.229 )]
    * 	[( 0.870,  0.251,  0.229 ); ( 0.204, -0.803, -0.461 )]
    * 	[( 0.204, -0.803, -0.461 ); ( 0.674,  0.482,  0.712 )]
    * 	[( 0.870,  0.251,  0.229 ); ( 0.674,  0.482,  0.712 )]
    * 	[( 0.674,  0.482,  0.712 ); (-0.283, -0.207, -0.595 )]
    * 	[(-0.283, -0.207, -0.595 ); ( 0.870,  0.251,  0.229 )]
    * 	[( 0.204, -0.803, -0.461 ); ( 0.870,  0.251,  0.229 )]
    * 	[( 0.870,  0.251,  0.229 ); (-0.283, -0.207, -0.595 )]
    * 	[(-0.283, -0.207, -0.595 ); ( 0.204, -0.803, -0.461 )]
    * 	[( 0.674,  0.482,  0.712 ); ( 0.204, -0.803, -0.461 )]
    * 	[( 0.204, -0.803, -0.461 ); (-0.283, -0.207, -0.595 )]
    * 	[(-0.283, -0.207, -0.595 ); ( 0.674,  0.482,  0.712 )]
    * Vertex list: 
    * 	( 0.674,  0.482,  0.712 )
    * 	( 0.870,  0.251,  0.229 )
    * 	( 0.204, -0.803, -0.461 )
    * 	(-0.283, -0.207, -0.595 )
    * </pre>
    * </p>
    * 
    * @param format the format to use for each number.
    * @param convexPolytope3D the object to get the {@code String} of. Not modified.
    * @return the representative {@code String}.
    */
   public static String getConvexPolytope3DString(String format, ConvexPolytope3DReadOnly convexPolytope3D)
   {
      if (convexPolytope3D == null)
         return "null";

      String string = "Convex polytope 3D: number of: [faces: " + convexPolytope3D.getNumberOfFaces() + ", edges: " + convexPolytope3D.getNumberOfEdges()
            + ", vertices: " + convexPolytope3D.getNumberOfVertices();
      String linePrefix = "\n\t";
      string += "\nFace list: " + getFace3DCollectionString(format, linePrefix, convexPolytope3D.getFaces());
      string += "\nEdge list: " + getHalfEdge3DCollectionString(format, linePrefix, convexPolytope3D.getHalfEdges());
      string += "\nVertex list: " + getVertex3DCollectionString(format, linePrefix, convexPolytope3D.getVertices());
      return string;
   }

   private static String getVertex3DCollectionString(String format, String linePrefix, Collection<? extends Vertex3DReadOnly> vertices)
   {
      return EuclidCoreIOTools.getCollectionString(linePrefix, null, linePrefix, vertices, vertex -> getTuple3DString(format, vertex));
   }

   private static String getHalfEdge3DCollectionString(String format, String linePrefix, Collection<? extends HalfEdge3DReadOnly> halfEdges)
   {
      return EuclidCoreIOTools.getCollectionString(linePrefix, null, linePrefix, halfEdges, halfEdge -> getHalfEdge3DShortString(format, halfEdge));
   }

   private static String getFace3DCollectionString(String format, String linePrefix, Collection<? extends Face3DReadOnly> faces)
   {
      return EuclidCoreIOTools.getCollectionString(linePrefix, null, linePrefix, faces, face -> getFace3DShortString(format, face));
   }

   private static String getHalfEdge3DShortString(String format, HalfEdge3DReadOnly halfEdge)
   {
      if (halfEdge == null)
         return null;
      return "[" + getTuple3DString(format, halfEdge.getOrigin()) + "; " + getTuple3DString(format, halfEdge.getDestination()) + "]";
   }

   private static String getFace3DShortString(String format, Face3DReadOnly face3D)
   {
      if (face3D == null)
         return "null";
      return "centroid: " + getTuple3DString(format, face3D.getCentroid()) + ", normal: " + getTuple3DString(format, face3D.getNormal());
   }
}
