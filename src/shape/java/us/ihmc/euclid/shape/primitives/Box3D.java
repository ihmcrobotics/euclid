package us.ihmc.euclid.shape.primitives;

import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Box3DBasics;
import us.ihmc.euclid.shape.primitives.interfaces.Box3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.IntermediateVariableSupplier;
import us.ihmc.euclid.shape.tools.EuclidShapeIOTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;

/**
 * Implementation of a box 3D.
 * <p>
 * A box 3D is represented by its size, the position of its center, and its orientation.
 * </p>
 *
 * @author Sylvain Bertrand
 */
public class Box3D implements Box3DBasics, GeometryObject<Box3D>
{
   /** Pose of this box. */
   private final Shape3DPose pose = new Shape3DPose();
   /** Current supplier to use for storing intermediate results. */
   private IntermediateVariableSupplier supplier = IntermediateVariableSupplier.defaultIntermediateVariableSupplier();

   /**
    * Represents the sizeX, sizeY, and sizeZ of this box.
    */
   private final Vector3D size = new Vector3D()
   {
      @Override
      public void setX(double x)
      {
         if (x < 0.0)
            throw new IllegalArgumentException("The x-size of a Box3D cannot be negative: " + x);
         super.setX(x);
      }

      @Override
      public void setY(double y)
      {
         if (y < 0.0)
            throw new IllegalArgumentException("The y-size of a Box3D cannot be negative: " + y);
         super.setY(y);
      }

      @Override
      public void setZ(double z)
      {
         if (z < 0.0)
            throw new IllegalArgumentException("The z-size of a Box3D cannot be negative: " + z);
         super.setZ(z);
      }
   };

   /**
    * Creates a 1-by-1-by-1 box 3D.
    */
   public Box3D()
   {
      this(1.0, 1.0, 1.0);
   }

   /**
    * Creates a new box 3D and initializes its size.
    *
    * @param sizeX the size of this box along the x-axis.
    * @param sizeY the size of this box along the y-axis.
    * @param sizeZ the size of this box along the z-axis.
    * @throws IllegalArgumentException if any of {@code sizeX}, {@code sizeY}, or {@code sizeZ} is
    *            negative.
    */
   public Box3D(double sizeX, double sizeY, double sizeZ)
   {
      setSize(sizeX, sizeY, sizeZ);
   }

   /**
    * Creates a new box 3D and initializes its pose and size.
    *
    * @param position the position of this box. Not modified.
    * @param orientation the orientation of this box. Not modified.
    * @param sizeX the size of this box along the x-axis.
    * @param sizeY the size of this box along the y-axis.
    * @param sizeZ the size of this box along the z-axis.
    * @throws IllegalArgumentException if any of {@code sizeX}, {@code sizeY}, or {@code sizeZ} is
    *            negative.
    */
   public Box3D(Point3DReadOnly position, Orientation3DReadOnly orientation, double sizeX, double sizeY, double sizeZ)
   {
      set(position, orientation, sizeX, sizeY, sizeZ);
   }

   /**
    * Creates a new box 3D and initializes its pose and size.
    *
    * @param pose the position and orientation of this box. Not modified.
    * @param sizeX the size of this box along the x-axis.
    * @param sizeY the size of this box along the y-axis.
    * @param sizeZ the size of this box along the z-axis.
    * @throws IllegalArgumentException if any of {@code sizeX}, {@code sizeY}, or {@code sizeZ} is
    *            negative.
    */
   public Box3D(Pose3DReadOnly pose, double sizeX, double sizeY, double sizeZ)
   {
      set(pose, sizeX, sizeY, sizeZ);
   }

   /**
    * Creates a new box 3D and initializes its pose and size.
    *
    * @param pose the position and orientation of this box. Not modified.
    * @param sizeX the size of this box along the x-axis.
    * @param sizeY the size of this box along the y-axis.
    * @param sizeZ the size of this box along the z-axis.
    * @throws IllegalArgumentException if any of {@code sizeX}, {@code sizeY}, or {@code sizeZ} is
    *            negative.
    */
   public Box3D(RigidBodyTransformReadOnly pose, double sizeX, double sizeY, double sizeZ)
   {
      getPose().set(pose);
      setSize(sizeX, sizeY, sizeZ);
   }

   /**
    * Creates a new box 3D identical to {@code other}.
    *
    * @param other the other box to copy. Not modified.
    */
   public Box3D(Box3DReadOnly other)
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
   public Vector3DBasics getSize()
   {
      return size;
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
      this.supplier = newSupplier;
   }

   /**
    * Copies the {@code other} box data into {@code this}.
    *
    * @param other the other box to copy. Not modified.
    */
   @Override
   public void set(Box3D other)
   {
      Box3DBasics.super.set(other);
   }

   /**
    * Tests on a per component basis if {@code other} and {@code this} are equal to an {@code epsilon}.
    *
    * @param other the other box to compare against this. Not modified.
    * @param epsilon tolerance to use when comparing each component.
    * @return {@code true} if the two boxes are equal component-wise, {@code false} otherwise.
    */
   @Override
   public boolean epsilonEquals(Box3D other, double epsilon)
   {
      return Box3DBasics.super.epsilonEquals(other, epsilon);
   }

   /**
    * Compares {@code this} to {@code other} to determine if the two boxes are geometrically similar.
    *
    * @param other the box to compare to. Not modified.
    * @param epsilon the tolerance of the comparison.
    * @return {@code true} if the two boxes represent the same geometry, {@code false} otherwise.
    */
   @Override
   public boolean geometricallyEquals(Box3D other, double epsilon)
   {
      return Box3DBasics.super.geometricallyEquals(other, epsilon);
   }

   /**
    * Tests if the given {@code object}'s class is the same as this, in which case the method returns
    * {@link #equals(Box3DReadOnly)}, it returns {@code false} otherwise.
    *
    * @param object the object to compare against this. Not modified.
    * @return {@code true} if {@code object} and this are exactly equal, {@code false} otherwise.
    */
   @Override
   public boolean equals(Object object)
   {
      if (object instanceof Box3DReadOnly)
         return Box3DBasics.super.equals((Box3DReadOnly) object);
      else
         return false;
   }

   /**
    * Calculates and returns a hash code value from the value of each component of this box 3D.
    *
    * @return the hash code value for this box 3D.
    */
   @Override
   public int hashCode()
   {
      long hash = EuclidHashCodeTools.combineHashCode(pose.hashCode(), size.hashCode());
      return EuclidHashCodeTools.toIntHashCode(hash);
   }

   /**
    * Provides a {@code String} representation of this box 3D as follows:<br>
    * Box 3D: [position: ( 0.540, 0.110, 0.319 ), yaw-pitch-roll: (-2.061, -0.904, -1.136), size: (
    * 0.191, 0.719, 0.479 )]
    * 
    * @return the {@code String} representing this box 3D.
    */
   @Override
   public String toString()
   {
      return EuclidShapeIOTools.getBox3DString(this);
   }
}
