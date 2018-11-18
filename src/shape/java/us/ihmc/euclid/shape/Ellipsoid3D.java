package us.ihmc.euclid.shape;

import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.shape.interfaces.Ellipsoid3DBasics;
import us.ihmc.euclid.shape.tools.EuclidShapeTools;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

/**
 * {@code Ellipsoid3D} represents a 3D ellipsoid defined by its three main radii and with its origin
 * at its center.
 */
public class Ellipsoid3D extends Shape3D implements Ellipsoid3DBasics, GeometryObject<Ellipsoid3D>
{
   /** The three radii of this ellipsoid. */
   private final Vector3D radii = new Vector3D()
   {
      @Override
      public void setX(double x)
      {
         if (x < 0.0)
            throw new IllegalArgumentException("The x-radius of an Ellipsoid3D cannot be negative: " + x);
         super.setX(x);
      }

      @Override
      public void setY(double y)
      {
         if (y < 0.0)
            throw new IllegalArgumentException("The y-radius of an Ellipsoid3D cannot be negative: " + y);
         super.setY(y);
      }

      @Override
      public void setZ(double z)
      {
         if (z < 0.0)
            throw new IllegalArgumentException("The z-radius of an Ellipsoid3D cannot be negative: " + z);
         super.setZ(z);
      }
   };

   /**
    * Creates a new ellipsoid 3D with its 3 radii initialized to {@code 1}.
    */
   public Ellipsoid3D()
   {
      this(1.0, 1.0, 1.0);
   }

   /**
    * Creates a new ellipsoid 3D identical to {@code other}.
    *
    * @param other the other ellipsoid to copy. Not modified.
    */
   public Ellipsoid3D(Ellipsoid3D other)
   {
      set(other);
   }

   /**
    * Creates a new 3D ellipsoid and initializes its radii.
    *
    * @param radiusX radius of the ellipsoid along the x-axis.
    * @param radiusY radius of the ellipsoid along the y-axis.
    * @param radiusZ radius of the ellipsoid along the z-axis.
    * @throws IllegalArgumentException if any of the three radii is negative.
    */
   public Ellipsoid3D(double radiusX, double radiusY, double radiusZ)
   {
      setRadii(radiusX, radiusY, radiusZ);
   }

   /**
    * Creates a new 3D ellipsoid and initializes its pose and radii.
    *
    * @param pose the position and orientation of this ellipsoid. Not modified.
    * @param radiusX radius of the ellipsoid along the x-axis.
    * @param radiusY radius of the ellipsoid along the y-axis.
    * @param radiusZ radius of the ellipsoid along the z-axis.
    * @throws IllegalArgumentException if any of the three radii is negative.
    */
   public Ellipsoid3D(Pose3DReadOnly pose, double radiusX, double radiusY, double radiusZ)
   {
      set(pose, radiusX, radiusY, radiusZ);
   }

   /**
    * Creates a new 3D ellipsoid and initializes its pose and radii.
    *
    * @param pose the position and orientation of this ellipsoid. Not modified.
    * @param radiusX radius of the ellipsoid along the x-axis.
    * @param radiusY radius of the ellipsoid along the y-axis.
    * @param radiusZ radius of the ellipsoid along the z-axis.
    * @throws IllegalArgumentException if any of the three radii is negative.
    */
   public Ellipsoid3D(RigidBodyTransformReadOnly pose, double radiusX, double radiusY, double radiusZ)
   {
      set(pose, radiusX, radiusY, radiusZ);
   }

   /** {@inheritDoc} */
   @Override
   protected double evaluateQuery(Point3DReadOnly query, Point3DBasics closestPointToPack, Vector3DBasics normalToPack)
   {
      return EuclidShapeTools.evaluatePoint3DWithEllipsoid3D(query, closestPointToPack, normalToPack, getRadii());
   }

   @Override
   public Vector3DReadOnly getRadii()
   {
      return radii;
   }

   /** {@inheritDoc} */
   @Override
   protected boolean isInsideEpsilonShapeFrame(Point3DReadOnly query, double epsilon)
   {
      return EuclidShapeTools.isPoint3DInsideEllipsoid3D(query, getRadii(), epsilon);
   }

   /**
    * Copies the {@code other} ellipsoid data into {@code this}.
    *
    * @param other the other ellipsoid to copy. Not modified.
    */
   @Override
   public void set(Ellipsoid3D other)
   {
      setPose(other);
      radii.set(other.radii);
   }

   /** {@inheritDoc} */
   @Override
   public void setToNaN()
   {
      super.setToNaN();
      radii.setToNaN();
   }

   /** {@inheritDoc} */
   @Override
   public void setToZero()
   {
      super.setToZero();
      radii.setToZero();
   }

   /**
    * Sets the pose and radii of this ellipsoid.
    *
    * @param pose the position and orientation of this ellipsoid. Not modified.
    * @param radiusX radius of the ellipsoid along the x-axis.
    * @param radiusY radius of the ellipsoid along the y-axis.
    * @param radiusZ radius of the ellipsoid along the z-axis.
    * @throws IllegalArgumentException if any of the three radii is negative.
    */
   public void set(Pose3DReadOnly pose, double radiusX, double radiusY, double radiusZ)
   {
      setPose(pose);
      setRadii(radiusX, radiusY, radiusZ);
   }

   /**
    * Sets the pose and radii of this ellipsoid.
    *
    * @param pose the position and orientation of this ellipsoid. Not modified.
    * @param radiusX radius of the ellipsoid along the x-axis.
    * @param radiusY radius of the ellipsoid along the y-axis.
    * @param radiusZ radius of the ellipsoid along the z-axis.
    * @throws IllegalArgumentException if any of the three radii is negative.
    */
   public void set(RigidBodyTransformReadOnly pose, double radiusX, double radiusY, double radiusZ)
   {
      setPose(pose);
      setRadii(radiusX, radiusY, radiusZ);
   }

   /**
    * Sets the radii of this ellipsoid.
    *
    * @param radii tuple holding the 3 radii of the ellipsoid.
    * @throws IllegalArgumentException if any of the three radii is negative.
    */
   public void setRadii(Tuple3DReadOnly radii)
   {
      this.radii.set(radii);
   }

   /**
    * Sets the radii of this ellipsoid.
    *
    * @param radiusX radius of the ellipsoid along the x-axis.
    * @param radiusY radius of the ellipsoid along the y-axis.
    * @param radiusZ radius of the ellipsoid along the z-axis.
    * @throws IllegalArgumentException if any of the three radii is negative.
    */
   public void setRadii(double radiusX, double radiusY, double radiusZ)
   {
      radii.set(radiusX, radiusY, radiusZ);
   }

   /**
    * Sets the radius along the x-axis for this ellipsoid.
    *
    * @param radiusX radius of the ellipsoid along the x-axis.
    * @throws IllegalArgumentException if the argument is negative.
    */
   public void setRadiusX(double radiusX)
   {
      radii.setX(radiusX);
   }

   /**
    * Sets the radius along the y-axis for this ellipsoid.
    *
    * @param radiusY radius of the ellipsoid along the y-axis.
    * @throws IllegalArgumentException if the argument is negative.
    */
   public void setRadiusY(double radiusY)
   {
      radii.setY(radiusY);
   }

   /**
    * Sets the radius along the z-axis for this ellipsoid.
    *
    * @param radiusZ radius of the ellipsoid along the z-axis.
    * @throws IllegalArgumentException if the argument is negative.
    */
   public void setRadiusZ(double radiusZ)
   {
      radii.setZ(radiusZ);
   }

   /**
    * Tests separately and on a per component basis if the pose and the radii of this ellipsoid and
    * {@code other}'s pose and size are equal to an {@code epsilon}.
    *
    * @param other the other ellipsoid which pose and radii is to be compared against this ellipsoid
    *           pose and radii. Not modified.
    * @param epsilon tolerance to use when comparing each component.
    * @return {@code true} if the two ellipsoids are equal component-wise, {@code false} otherwise.
    */
   @Override
   public boolean epsilonEquals(Ellipsoid3D other, double epsilon)
   {
      return Ellipsoid3DBasics.super.epsilonEquals(other, epsilon);
   }

   /**
    * Compares {@code this} and {@code other} to determine if the two ellipsoids are geometrically
    * similar.
    * <p>
    * This method accounts for the multiple combinations of radii and rotations that generate identical
    * ellipsoids. For instance, two ellipsoids that are identical but one is flipped by 180 degrees are
    * considered geometrically equal.
    * </p>
    *
    * @param other the ellipsoid to compare to. Not modified.
    * @param epsilon the tolerance of the comparison.
    * @return {@code true} if the ellipsoids represent the same geometry, {@code false} otherwise.
    */
   @Override
   public boolean geometricallyEquals(Ellipsoid3D other, double epsilon)
   {
      return Ellipsoid3DBasics.super.geometricallyEquals(other, epsilon);
   }

   /**
    * Provides a {@code String} representation of this ellipsoid 3D as follows:<br>
    * Ellipsoid 3D: radii = (rx, ry, rz), pose = <br>
    * m00, m01, m02 | m03 <br>
    * m10, m11, m12 | m13 <br>
    * m20, m21, m22 | m23
    *
    * @return the {@code String} representing this ellipsoid 3D.
    */
   @Override
   public String toString()
   {
      return "Ellipsoid 3D: radii = " + radii + ", pose =\n" + getPoseString();
   }
}
