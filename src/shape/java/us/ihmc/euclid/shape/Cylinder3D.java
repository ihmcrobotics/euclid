package us.ihmc.euclid.shape;

import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.shape.interfaces.Cylinder3DBasics;
import us.ihmc.euclid.shape.interfaces.Cylinder3DReadOnly;
import us.ihmc.euclid.shape.interfaces.IntermediateVariableSupplier;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

/**
 * {@code Cylinder3D} represents a cylinder defined by its radius and height.
 * <p>
 * Shape description:
 * <ul>
 * <li>The cylinder's axis is the z-axis.
 * <li>The cylinder's origin is its centroid.
 * </ul>
 * </p>
 */
public class Cylinder3D implements Cylinder3DBasics, GeometryObject<Cylinder3D>
{
   private final RigidBodyTransform pose = new RigidBodyTransform();
   private IntermediateVariableSupplier supplier = IntermediateVariableSupplier.defaultIntermediateVariableSupplier();

   /** Radius of the cylinder part. */
   private double radius;
   /**
    * Overall height of the cylinder, i.e. the top face is at {@code 0.5 * height} and the bottom face
    * at {@code - 0.5 * height}.
    */
   private double height;

   private final Vector3DReadOnly axis = new Vector3DReadOnly()
   {
      @Override
      public double getX()
      {
         return pose.getM02();
      }

      @Override
      public double getY()
      {
         return pose.getM12();
      }

      @Override
      public double getZ()
      {
         return pose.getM22();
      }
   };

   /**
    * Creates a new cylinder with height of {@code 1} and radius of {@code 0.5}.
    */
   public Cylinder3D()
   {
      this(1.0, 0.5);
   }

   /**
    * Creates a new cylinder 3D identical to {@code other}.
    *
    * @param other the other cylinder to copy. Not modified.
    */
   public Cylinder3D(Cylinder3DReadOnly other)
   {
      set(other);
   }

   /**
    * Creates a new cylinder 3D and initializes its height and radius.
    *
    * @param height the cylinder length along the z-axis.
    * @param radius the radius of the cylinder.
    * @throws IllegalArgumentException if either {@code height} or {@code radius} is negative.
    */
   public Cylinder3D(double height, double radius)
   {
      setSize(height, radius);
   }

   /**
    * Creates a new cylinder 3D and initializes its pose, height, and radius.
    *
    * @param pose the position and orientation of this cylinder. Not modified.
    * @param height the cylinder length along the z-axis.
    * @param radius the radius of the cylinder.
    * @throws IllegalArgumentException if either {@code height} or {@code radius} is negative.
    */
   public Cylinder3D(RigidBodyTransformReadOnly pose, double height, double radius)
   {
      set(pose, height, radius);
   }

   /**
    * Creates a new cylinder 3D and initializes its pose, height, and radius.
    *
    * @param pose the position and orientation of this cylinder. Not modified.
    * @param height the cylinder length along the z-axis.
    * @param radius the radius of the cylinder.
    * @throws IllegalArgumentException if either {@code height} or {@code radius} is negative.
    */
   public Cylinder3D(Pose3DReadOnly pose, double height, double radius)
   {
      set(pose, height, radius);
   }

   /**
    * Copies the {@code other} cylinder data into {@code this}.
    *
    * @param other the other cylinder to copy. Not modified.
    */
   @Override
   public void set(Cylinder3D other)
   {
      Cylinder3DBasics.super.set(other);
   }

   /**
    * Sets the radius of this cylinder.
    *
    * @param radius the new radius for this cylinder.
    * @throws IllegalArgumentException if {@code radius} is negative.
    */
   public void setRadius(double radius)
   {
      if (radius < 0.0)
         throw new IllegalArgumentException("The radius of a Cylinder3D cannot be negative: " + radius);
      this.radius = radius;
   }

   /**
    * Sets the height of this cylinder.
    *
    * @param height the cylinder length along the z-axis.
    * @throws IllegalArgumentException if {@code height} is negative.
    */
   public void setHeight(double height)
   {
      if (height < 0.0)
         throw new IllegalArgumentException("The height of a Cylinder3D cannot be negative: " + height);
      this.height = height;
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

   /**
    * Gets the radius of this cylinder.
    *
    * @return the value of the radius.
    */
   @Override
   public double getRadius()
   {
      return radius;
   }

   /**
    * Gets the height of this cylinder.
    *
    * @return the value of the height.
    */
   @Override
   public double getHeight()
   {
      return height;
   }

   @Override
   public Vector3DReadOnly getAxis()
   {
      return axis;
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
    * Tests separately and on a per component basis if the pose and the size of this cylinder and
    * {@code other}'s pose and size are equal to an {@code epsilon}.
    *
    * @param other the other cylinder which pose and size is to be compared against this cylinder pose
    *           and size. Not modified.
    * @param epsilon tolerance to use when comparing each component.
    * @return {@code true} if the two cylinders are equal component-wise, {@code false} otherwise.
    */
   @Override
   public boolean epsilonEquals(Cylinder3D other, double epsilon)
   {
      return Cylinder3DBasics.super.epsilonEquals(other, epsilon);
   }

   /**
    * Compares {@code this} and {@code other} to determine if the two cylinders are geometrically
    * similar.
    * <p>
    * This method accounts for the multiple combinations of radius/height and rotations that generate
    * identical cylinder. For instance, two cylinders that are identical but one is rotated around its
    * main axis are considered geometrically equal.
    * </p>
    *
    * @param other the cylinder to compare to. Not modified.
    * @param epsilon the tolerance of the comparison.
    * @return {@code true} if the cylinders represent the same geometry, {@code false} otherwise.
    */
   @Override
   public boolean geometricallyEquals(Cylinder3D other, double epsilon)
   {
      return Cylinder3DBasics.super.geometricallyEquals(other, epsilon);
   }

   /**
    * Provides a {@code String} representation of this cylinder 3D as follows:<br>
    * Cylinder 3D: height = h, radius = r, pose = <br>
    * m00, m01, m02 | m03 <br>
    * m10, m11, m12 | m13 <br>
    * m20, m21, m22 | m23
    *
    * @return the {@code String} representing this cylinder 3D.
    */
   @Override
   public String toString()
   {
      return "Cylinder 3D: height = " + height + ", radius = " + radius + ", pose=\n" + pose;
   }
}
