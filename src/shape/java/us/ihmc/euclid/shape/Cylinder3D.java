package us.ihmc.euclid.shape;

import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.shape.interfaces.Cylinder3DBasics;
import us.ihmc.euclid.shape.interfaces.Cylinder3DReadOnly;
import us.ihmc.euclid.shape.interfaces.IntermediateVariableSupplier;
import us.ihmc.euclid.shape.tools.EuclidShapeIOTools;
import us.ihmc.euclid.tools.EuclidCoreFactories;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

/**
 * {@code Cylinder3D} represents a cylinder defined by its radius and length.
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
   private final Shape3DPose pose = new Shape3DPose();
   private IntermediateVariableSupplier supplier = IntermediateVariableSupplier.defaultIntermediateVariableSupplier();

   /** Radius of the cylinder part. */
   private double radius;
   /**
    * Overall length of the cylinder, i.e. the top face is at {@code 0.5 * length} and the bottom face
    * at {@code - 0.5 * length}.
    */
   private double length;

   private final Point3DReadOnly topCenter = EuclidCoreFactories.newLinkedPoint3DReadOnly(() -> getHalfLength() * getAxis().getX() + getPose().getTranslationX(),
                                                                                          () -> getHalfLength() * getAxis().getY() + getPose().getTranslationY(),
                                                                                          () -> getHalfLength() * getAxis().getZ() + getPose().getTranslationZ());
   private final Point3DReadOnly bottomCenter = EuclidCoreFactories.newLinkedPoint3DReadOnly(() -> -getHalfLength() * getAxis().getX() + getPose().getTranslationX(),
                                                                                             () -> -getHalfLength() * getAxis().getY() + getPose().getTranslationY(),
                                                                                             () -> -getHalfLength() * getAxis().getZ() + getPose().getTranslationZ());

   /**
    * Creates a new cylinder with length of {@code 1} and radius of {@code 0.5}.
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
    * Creates a new cylinder 3D and initializes its length and radius.
    *
    * @param length the cylinder length along the z-axis.
    * @param radius the radius of the cylinder.
    * @throws IllegalArgumentException if either {@code length} or {@code radius} is negative.
    */
   public Cylinder3D(double length, double radius)
   {
      setSize(length, radius);
   }

   /**
    * Creates a new cylinder 3D and initializes its pose, length, and radius.
    *
    * @param pose the position and orientation of this cylinder. Not modified.
    * @param length the cylinder length along the z-axis.
    * @param radius the radius of the cylinder.
    * @throws IllegalArgumentException if either {@code length} or {@code radius} is negative.
    */
   public Cylinder3D(RigidBodyTransformReadOnly pose, double length, double radius)
   {
      set(pose, length, radius);
   }

   /**
    * Creates a new cylinder 3D and initializes its pose, length, and radius.
    *
    * @param pose the position and orientation of this cylinder. Not modified.
    * @param length the cylinder length along the z-axis.
    * @param radius the radius of the cylinder.
    * @throws IllegalArgumentException if either {@code length} or {@code radius} is negative.
    */
   public Cylinder3D(Pose3DReadOnly pose, double length, double radius)
   {
      set(pose, length, radius);
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
   @Override
   public void setRadius(double radius)
   {
      if (radius < 0.0)
         throw new IllegalArgumentException("The radius of a Cylinder3D cannot be negative: " + radius);
      this.radius = radius;
   }

   /**
    * Sets the length of this cylinder.
    *
    * @param length the cylinder length along the z-axis.
    * @throws IllegalArgumentException if {@code length} is negative.
    */
   @Override
   public void setLength(double length)
   {
      if (length < 0.0)
         throw new IllegalArgumentException("The length of a Cylinder3D cannot be negative: " + length);
      this.length = length;
   }

   @Override
   public Shape3DPose getPose()
   {
      return pose;
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
    * Gets the length of this cylinder.
    *
    * @return the value of the length.
    */
   @Override
   public double getLength()
   {
      return length;
   }

   @Override
   public Point3DReadOnly getTopCenter()
   {
      return topCenter;
   }

   @Override
   public Point3DReadOnly getBottomCenter()
   {
      return bottomCenter;
   }

   @Override
   public IntermediateVariableSupplier getIntermediateVariableSupplier()
   {
      return supplier;
   }

   @Override
   public void setIntermediateVariableSupplier(IntermediateVariableSupplier newSupplier)
   {
      supplier = newSupplier;
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
    * This method accounts for the multiple combinations of radius/length and rotations that generate
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

   @Override
   public String toString()
   {
      return EuclidShapeIOTools.getCylinder3DString(this);
   }
}
