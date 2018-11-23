package us.ihmc.euclid.shape.tools;

import static us.ihmc.euclid.tools.EuclidCoreIOTools.*;

import us.ihmc.euclid.matrix.interfaces.RotationMatrixReadOnly;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.shape.interfaces.Box3DReadOnly;
import us.ihmc.euclid.shape.interfaces.Capsule3DReadOnly;
import us.ihmc.euclid.shape.interfaces.Cylinder3DReadOnly;
import us.ihmc.euclid.shape.interfaces.Ellipsoid3DReadOnly;
import us.ihmc.euclid.shape.interfaces.PointShape3DReadOnly;
import us.ihmc.euclid.shape.interfaces.Ramp3DReadOnly;
import us.ihmc.euclid.shape.interfaces.Shape3DPoseReadOnly;
import us.ihmc.euclid.shape.interfaces.Sphere3DReadOnly;
import us.ihmc.euclid.shape.interfaces.Torus3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

public class EuclidShapeIOTools
{
   public static String getBox3DString(Box3DReadOnly box3D)
   {
      return getBox3DString(DEFAULT_FORMAT, box3D);
   }

   public static String getBox3DString(String format, Box3DReadOnly box3D)
   {
      if (box3D == null)
         return "null";
      else
         return getBox3DString(format, box3D.getPosition(), box3D.getOrientation(), box3D.getSize());
   }

   public static String getBox3DString(Tuple3DReadOnly position, Orientation3DReadOnly orientation, Vector3DReadOnly size)
   {
      return getBox3DString(DEFAULT_FORMAT, position, orientation, size);
   }

   /**
    * 
    * <pre>
    * Box 3D: [position: ( 0.540,  0.110,  0.319 ), yaw-pitch-roll: (-2.061, -0.904, -1.136), size: ( 0.191,  0.719,  0.479 )]
    * </pre>
    */
   public static String getBox3DString(String format, Tuple3DReadOnly position, Orientation3DReadOnly orientation, Vector3DReadOnly size)
   {
      return "Box 3D: [position: " + getTuple3DString(format, position) + ", " + getStringAsYawPitchRoll(format, orientation) + ", size: "
            + getTuple3DString(format, size) + "]";
   }

   public static String getCapsule3DString(Capsule3DReadOnly capsule3D)
   {
      return getCapsule3DString(DEFAULT_FORMAT, capsule3D);
   }

   public static String getCapsule3DString(String format, Capsule3DReadOnly capsule3D)
   {
      return getCapsule3DString(format, capsule3D.getPosition(), capsule3D.getAxis(), capsule3D.getLength(), capsule3D.getRadius());
   }

   public static String getCapsule3DString(Tuple3DReadOnly position, Vector3DReadOnly axis, double length, double radius)
   {
      return getCapsule3DString(DEFAULT_FORMAT, position, axis, length, radius);
   }

   /**
    * <pre>
    * Capsule 3D: [position: (-0.362, -0.617,  0.066 ), axis: ( 0.634, -0.551, -0.543 ), length:  0.170, radius:  0.906]
    * </pre>
    */
   public static String getCapsule3DString(String format, Tuple3DReadOnly position, Vector3DReadOnly axis, double length, double radius)
   {
      return "Capsule 3D: [position: " + getTuple3DString(format, position) + ", axis: " + getTuple3DString(format, axis) + ", length: "
            + String.format(format, length) + ", radius: " + String.format(format, radius) + "]";
   }

   public static String getCylinder3DString(Cylinder3DReadOnly cylinder3D)
   {
      return getCylinder3DString(DEFAULT_FORMAT, cylinder3D);
   }

   public static String getCylinder3DString(String format, Cylinder3DReadOnly cylinder3D)
   {
      return getCylinder3DString(format, cylinder3D.getPosition(), cylinder3D.getAxis(), cylinder3D.getLength(), cylinder3D.getRadius());
   }

   public static String getCylinder3DString(Tuple3DReadOnly position, Vector3DReadOnly axis, double length, double radius)
   {
      return getCylinder3DString(DEFAULT_FORMAT, position, axis, length, radius);
   }

   /**
    * <pre>
    * Cylinder 3D: [position: (-0.362, -0.617,  0.066 ), axis: ( 0.634, -0.551, -0.543 ), length:  0.170, radius:  0.906]
    * </pre>
    */
   public static String getCylinder3DString(String format, Tuple3DReadOnly position, Vector3DReadOnly axis, double length, double radius)
   {
      return "Cylinder 3D: [position: " + getTuple3DString(format, position) + ", axis: " + getTuple3DString(format, axis) + ", length: "
            + String.format(format, length) + ", radius: " + String.format(format, radius) + "]";
   }

   public static String getEllipsoid3DString(Ellipsoid3DReadOnly box3D)
   {
      return getEllipsoid3DString(DEFAULT_FORMAT, box3D);
   }

   public static String getEllipsoid3DString(String format, Ellipsoid3DReadOnly ellipsoid3D)
   {
      if (ellipsoid3D == null)
         return "null";
      else
         return getEllipsoid3DString(format, ellipsoid3D.getPosition(), ellipsoid3D.getOrientation(), ellipsoid3D.getRadii());
   }

   public static String getEllipsoid3DString(Tuple3DReadOnly position, Orientation3DReadOnly orientation, Vector3DReadOnly radii)
   {
      return getEllipsoid3DString(DEFAULT_FORMAT, position, orientation, radii);
   }

   /**
    * 
    * <pre>
    * Ellipsoid 3D: [position: ( 0.540,  0.110,  0.319 ), yaw-pitch-roll: (-2.061, -0.904, -1.136), radii: ( 0.191,  0.719,  0.479 )]
    * </pre>
    */
   public static String getEllipsoid3DString(String format, Tuple3DReadOnly position, Orientation3DReadOnly orientation, Vector3DReadOnly radii)
   {
      return "Ellipsoid 3D: [position: " + getTuple3DString(format, position) + ", " + getStringAsYawPitchRoll(format, orientation) + ", radii: "
            + getTuple3DString(format, radii) + "]";
   }

   public static String getPointShape3DString(PointShape3DReadOnly pointShape3D)
   {
      return getPointShape3DString(DEFAULT_FORMAT, pointShape3D);
   }

   public static String getPointShape3DString(String format, PointShape3DReadOnly pointShape3D)
   {
      return getPointShape3DString(format, (Tuple3DReadOnly) pointShape3D);
   }

   public static String getPointShape3DString(String format, Tuple3DReadOnly position)
   {
      return "Point shape 3D: " + getTuple3DString(format, position);
   }

   public static String getRamp3DString(Ramp3DReadOnly ramp3D)
   {
      return getRamp3DString(DEFAULT_FORMAT, ramp3D);
   }

   public static String getRamp3DString(String format, Ramp3DReadOnly ramp3D)
   {
      if (ramp3D == null)
         return "null";
      else
         return getRamp3DString(format, ramp3D.getPosition(), ramp3D.getOrientation(), ramp3D.getSize());
   }

   public static String getRamp3DString(Tuple3DReadOnly position, Orientation3DReadOnly orientation, Vector3DReadOnly size)
   {
      return getRamp3DString(DEFAULT_FORMAT, position, orientation, size);
   }

   /**
    * 
    * <pre>
    * Ramp 3D: [position: ( 0.540,  0.110,  0.319 ), yaw-pitch-roll: (-2.061, -0.904, -1.136), size: ( 0.191,  0.719,  0.479 )]
    * </pre>
    */
   public static String getRamp3DString(String format, Tuple3DReadOnly position, Orientation3DReadOnly orientation, Vector3DReadOnly size)
   {
      return "Ramp 3D: [position: " + getTuple3DString(format, position) + ", " + getStringAsYawPitchRoll(format, orientation) + ", size: "
            + getTuple3DString(format, size) + "]";
   }

   public static String getSphere3DString(Sphere3DReadOnly sphere3D)
   {
      return getSphere3DString(DEFAULT_FORMAT, sphere3D);
   }

   public static String getSphere3DString(String format, Sphere3DReadOnly sphere3D)
   {
      return getSphere3DString(format, sphere3D.getPosition(), sphere3D.getRadius());
   }

   public static String getSphere3DString(Tuple3DReadOnly position, double radius)
   {
      return getSphere3DString(DEFAULT_FORMAT, position, radius);
   }

   /**
    * <pre>
    * Sphere 3D: [position: (-0.362, -0.617,  0.066 ), radius:  0.906]
    * </pre>
    */
   public static String getSphere3DString(String format, Tuple3DReadOnly position, double radius)
   {
      return "Sphere 3D: [position: " + getTuple3DString(format, position) + ", radius: " + String.format(format, radius) + "]";
   }

   public static String getTorus3DString(Torus3DReadOnly torus3D)
   {
      return getTorus3DString(DEFAULT_FORMAT, torus3D);
   }

   public static String getTorus3DString(String format, Torus3DReadOnly torus3D)
   {
      return getTorus3DString(format, torus3D.getPosition(), torus3D.getAxis(), torus3D.getRadius(), torus3D.getTubeRadius());
   }

   public static String getTorus3DString(Tuple3DReadOnly position, Vector3DReadOnly axis, double radius, double tubeRadius)
   {
      return getTorus3DString(DEFAULT_FORMAT, position, axis, radius, tubeRadius);
   }

   /**
    * <pre>
    * Torus 3D: [position: (-0.362, -0.617,  0.066 ), axis: ( 0.634, -0.551, -0.543 ), radius:  0.971, tube radius:  0.113]
    * </pre>
    */
   public static String getTorus3DString(String format, Tuple3DReadOnly position, Vector3DReadOnly axis, double radius, double tubeRadius)
   {
      return "Torus 3D: [position: " + getTuple3DString(format, position) + ", axis: " + getTuple3DString(format, axis) + ", radius: "
            + String.format(format, radius) + ", tube radius: " + String.format(format, tubeRadius) + "]";
   }

   public static String getShape3DPoseString(Shape3DPoseReadOnly shape3DPose)
   {
      return getShape3DPoseString(DEFAULT_FORMAT, shape3DPose.getShapeOrientation(), shape3DPose.getShapePosition());
   }

   public static String getShape3DPoseString(String format, Shape3DPoseReadOnly shape3DPose)
   {
      return getShape3DPoseString(format, shape3DPose.getShapeOrientation(), shape3DPose.getShapePosition());
   }

   public static String getShape3DPoseString(String format, RotationMatrixReadOnly orientation, Point3DReadOnly position)
   {
      return "Shape 3D pose: [position: " + getTuple3DString(format, position) + ", " + getStringAsYawPitchRoll(format, orientation) + "]";
   }
}
