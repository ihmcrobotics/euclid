package us.ihmc.euclid.shape;

import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.orientation.interfaces.Orientation3DReadOnly;
import us.ihmc.euclid.shape.interfaces.Box3DBasics;
import us.ihmc.euclid.shape.tools.EuclidShapeTools;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;

/**
 * {@code Box3D} represents a box with a sizeX, a sizeY, and a sizeZ.
 * <p>
 * Its origin is located at its centroid.
 * </p>
 */
public class Box3D extends Shape3D implements GeometryObject<Box3D>, Box3DBasics
{
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
    * Creates a new box 3D identical to {@code other}.
    *
    * @param other the other box to copy. Not modified.
    */
   public Box3D(Box3D other)
   {
      set(other);
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
      setPose(pose);
      setSize(sizeX, sizeY, sizeZ);
   }

   /**
    * Creates a new box 3D and initializes its pose and size.
    *
    * @param pose the position and orientation of this box. Not modified.
    * @param size the size of this box along in order the x, y, and z axes.
    * @throws IllegalArgumentException if any of {@code size}'s elements is negative.
    */
   public Box3D(RigidBodyTransformReadOnly pose, double[] size)
   {
      set(pose, size);
   }

   /** {@inheritDoc} */
   @Override
   protected double evaluateQuery(Point3DReadOnly query, Point3DBasics closestPointToPack, Vector3DBasics normalToPack)
   {
      return EuclidShapeTools.evaluatePoint3DWithBox3D(query, closestPointToPack, normalToPack, size);
   }

   @Override
   public Vector3DBasics getSize()
   {
      return size;
   }

   /** {@inheritDoc} */
   @Override
   protected boolean isInsideEpsilonShapeFrame(Point3DReadOnly query, double epsilon)
   {
      return EuclidShapeTools.isPoint3DInsideBox3D(query, size, epsilon);
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
    * Tests separately and on a per component basis if the pose and the size of this box and
    * {@code other}'s pose and size are equal to an {@code epsilon}.
    *
    * @param other the other box which pose and size is to be compared against this box pose and size.
    *           Not modified.
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
    * <p>
    * This method accounts for the multiple combinations of sizes and rotations that generate identical
    * boxes. For instance, two boxes that are identical but one is flipped by 180 degrees are
    * considered geometrically equal.
    * </p>
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
    * Provides a {@code String} representation of this box 3D as follows:<br>
    * Box 3D: size = (sizeX, sizeY, sizeZ), pose = <br>
    * m00, m01, m02 | m03 <br>
    * m10, m11, m12 | m13 <br>
    * m20, m21, m22 | m23
    *
    * @return the {@code String} representing this box 3D.
    */
   @Override
   public String toString()
   {
      return "Box 3D: size = " + size + ", pose =\n" + getPoseString();
   }
}
