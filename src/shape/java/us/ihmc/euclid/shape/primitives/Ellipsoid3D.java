package us.ihmc.euclid.shape.primitives;

import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Ellipsoid3DBasics;
import us.ihmc.euclid.shape.primitives.interfaces.Ellipsoid3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.IntermediateVariableSupplier;
import us.ihmc.euclid.shape.tools.EuclidShapeIOTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;

/**
 * Implementation of an ellipsoid 3D.
 * <p>
 * A ellipsoid 3D is represented by its radii, the position of its center, and its orientation.
 * </p>
 *
 * @author Sylvain Bertrand
 */
public class Ellipsoid3D implements Ellipsoid3DBasics, GeometryObject<Ellipsoid3D>
{
   /** Pose of this ellipsoid. */
   private final Shape3DPose pose = new Shape3DPose();
   /** Current supplier to use for storing intermediate results. */
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
    * Creates a new 3D ellipsoid and initializes its radii.
    *
    * @param radiusX radius of the ellipsoid along the x-axis.
    * @param radiusY radius of the ellipsoid along the y-axis.
    * @param radiusZ radius of the ellipsoid along the z-axis.
    * @throws IllegalArgumentException if any of the three radii is negative.
    */
   public Ellipsoid3D(double radiusX, double radiusY, double radiusZ)
   {
      getRadii().set(radiusX, radiusY, radiusZ);
   }

   /**
    * Creates a new ellipsoid 3D and initializes its pose and size.
    *
    * @param position    the position of this ellipsoid. Not modified.
    * @param orientation the orientation of this ellipsoid. Not modified.
    * @param radiusX     the size of the ellipsoid along the x-axis.
    * @param radiusY     the size of the ellipsoid along the y-axis.
    * @param radiusZ     the size of the ellipsoid along the z-axis.
    * @throws IllegalArgumentException if any of the three radii is negative.
    */
   public Ellipsoid3D(Point3DReadOnly position, Orientation3DReadOnly orientation, double radiusX, double radiusY, double radiusZ)
   {
      set(position, orientation, radiusX, radiusY, radiusZ);
   }

   /**
    * Creates a new 3D ellipsoid and initializes its pose and radii.
    *
    * @param pose    the position and orientation of this ellipsoid. Not modified.
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
    * @param pose    the position and orientation of this ellipsoid. Not modified.
    * @param radiusX radius of the ellipsoid along the x-axis.
    * @param radiusY radius of the ellipsoid along the y-axis.
    * @param radiusZ radius of the ellipsoid along the z-axis.
    * @throws IllegalArgumentException if any of the three radii is negative.
    */
   public Ellipsoid3D(RigidBodyTransformReadOnly pose, double radiusX, double radiusY, double radiusZ)
   {
      set(pose, radiusX, radiusY, radiusZ);
   }

   /**
    * Creates a new ellipsoid 3D identical to {@code other}.
    *
    * @param other the other ellipsoid to copy. Not modified.
    */
   public Ellipsoid3D(Ellipsoid3DReadOnly other)
   {
      set(other);
   }

   /** {@inheritDoc} */
   @Override
   public Shape3DPose getPose()
   {
      return pose;
   }

   /** {@inheritDoc} */
   @Override
   public Vector3DBasics getRadii()
   {
      return radii;
   }

   /** {@inheritDoc} */
   @Override
   public IntermediateVariableSupplier getIntermediateVariableSupplier()
   {
      return supplier;
   }

   /** {@inheritDoc} */
   @Override
   public void setIntermediateVariableSupplier(IntermediateVariableSupplier newSupplier)
   {
      supplier = newSupplier;
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
    * Tests on a per component basis if {@code other} and {@code this} are equal to an {@code epsilon}.
    *
    * @param other   the other ellipsoid to compare against this. Not modified.
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
    *
    * @param other   the ellipsoid to compare to. Not modified.
    * @param epsilon the tolerance of the comparison.
    * @return {@code true} if the ellipsoids represent the same geometry, {@code false} otherwise.
    */
   @Override
   public boolean geometricallyEquals(Ellipsoid3D other, double epsilon)
   {
      return Ellipsoid3DBasics.super.geometricallyEquals(other, epsilon);
   }

   /**
    * Tests if the given {@code object}'s class is the same as this, in which case the method returns
    * {@link #equals(Ellipsoid3DReadOnly)}, it returns {@code false} otherwise.
    *
    * @param object the object to compare against this. Not modified.
    * @return {@code true} if {@code object} and this are exactly equal, {@code false} otherwise.
    */
   @Override
   public boolean equals(Object object)
   {
      if (object instanceof Ellipsoid3DReadOnly)
         return Ellipsoid3DBasics.super.equals((Ellipsoid3DReadOnly) object);
      else
         return false;
   }

   /**
    * Calculates and returns a hash code value from the value of each component of this ellipsoid 3D.
    *
    * @return the hash code value for this ellipsoid 3D.
    */
   @Override
   public int hashCode()
   {
      long hash = EuclidHashCodeTools.combineHashCode(pose.hashCode(), radii.hashCode());
      return EuclidHashCodeTools.toIntHashCode(hash);
   }

   /**
    * Provides a {@code String} representation of this ellipsoid 3D as follows:<br>
    * Ellipsoid 3D: [position: ( 0.540, 0.110, 0.319 ), yaw-pitch-roll: (-2.061, -0.904, -1.136),
    * radii: ( 0.191, 0.719, 0.479 )]
    *
    * @return the {@code String} representing this ellipsoid 3D.
    */
   @Override
   public String toString()
   {
      return EuclidShapeIOTools.getEllipsoid3DString(this);
   }
}
