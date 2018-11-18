package us.ihmc.euclid.shape;

import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.shape.interfaces.Ellipsoid3DBasics;
import us.ihmc.euclid.shape.interfaces.IntermediateVariableSupplier;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;

/**
 * {@code Ellipsoid3D} represents a 3D ellipsoid defined by its three main radii and with its origin
 * at its center.
 */
public class Ellipsoid3D implements Ellipsoid3DBasics, GeometryObject<Ellipsoid3D>
{
   private final RigidBodyTransform pose = new RigidBodyTransform();
   private IntermediateVariableSupplier supplier = IntermediateVariableSupplier.defaultIntermediateVariableSupplier();

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

   @Override
   public RigidBodyTransform getPose()
   {
      return pose;
   }

   @Override
   public RotationMatrix getOrientation()
   {
      return pose.getRotation();
   }

   @Override
   public Vector3DBasics getPosition()
   {
      return pose.getTranslation();
   }

   @Override
   public Vector3DBasics getRadii()
   {
      return radii;
   }

   @Override
   public IntermediateVariableSupplier getIntermediateVariableSupplier()
   {
      return supplier;
   }

   @Override
   public void setIntermediateVariableSupplier(IntermediateVariableSupplier newSupplier)
   {
      this.supplier = newSupplier;
   }

   /**
    * Copies the {@code other} ellipsoid data into {@code this}.
    *
    * @param other the other ellipsoid to copy. Not modified.
    */
   @Override
   public void set(Ellipsoid3D other)
   {
      Ellipsoid3DBasics.super.set(other);
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
      return "Ellipsoid 3D: radii = " + radii + ", pose =\n" + pose;
   }
}
