package us.ihmc.euclid.shape.primitives;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.shape.primitives.interfaces.Capsule3DBasics;
import us.ihmc.euclid.shape.primitives.interfaces.Capsule3DReadOnly;
import us.ihmc.euclid.shape.tools.EuclidShapeIOTools;
import us.ihmc.euclid.tools.EuclidCoreFactories;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.UnitVector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

/**
 * Implementation of a capsule 3D.
 * <p>
 * A capsule 3D is represented by its length, i.e. the distance separating the center of the two
 * half-spheres, its radius, the position of its center, and its axis of revolution.
 * </p>
 *
 * @author Sylvain Bertrand
 */
public class Capsule3D implements Capsule3DBasics, GeometryObject<Capsule3D>
{
   /** Position of this capsule's center. */
   private final Point3D position = new Point3D();
   /** Axis of revolution of this capsule. */
   private final UnitVector3D axis = new UnitVector3D(Axis3D.Z);

   /** This capsule radius. */
   private double radius;
   /** This capsule length. */
   private double length;
   /** This capsule half-length. */
   private double halfLength;

   /** Position of the top half-sphere center linked to this capsule properties. */
   private final Point3DReadOnly topCenter = EuclidCoreFactories.newLinkedPoint3DReadOnly(() -> halfLength * axis.getX() + position.getX(),
                                                                                          () -> halfLength * axis.getY() + position.getY(),
                                                                                          () -> halfLength * axis.getZ() + position.getZ());
   /** Position of the bottom half-sphere center linked to this capsule properties. */
   private final Point3DReadOnly bottomCenter = EuclidCoreFactories.newLinkedPoint3DReadOnly(() -> -halfLength * axis.getX() + position.getX(),
                                                                                             () -> -halfLength * axis.getY() + position.getY(),
                                                                                             () -> -halfLength * axis.getZ() + position.getZ());

   /**
    * Creates a new capsule which axis is along the z-axis, a length of 1, and radius of 0.5.
    */
   public Capsule3D()
   {
      this(1.0, 0.5);
   }

   /**
    * Creates a new capsule which axis is along the z-axis and initializes its size.
    *
    * @param length the length of this capsule.
    * @param radius the radius of this capsule.
    * @throws IllegalArgumentException if {@code length} or {@code radius} is negative.
    */
   public Capsule3D(double length, double radius)
   {
      setSize(length, radius);
   }

   /**
    * Creates a new capsule 3D and initializes its pose and size.
    *
    * @param position the position of the center. Not modified.
    * @param axis     the axis of revolution. Not modified.
    * @param length   the length of this capsule.
    * @param radius   the radius of this capsule.
    * @throws IllegalArgumentException if {@code length} or {@code radius} is negative.
    */
   public Capsule3D(Point3DReadOnly position, Vector3DReadOnly axis, double length, double radius)
   {
      set(position, axis, length, radius);
   }

   /**
    * Creates a new capsule 3D identical to {@code other}.
    *
    * @param other the other capsule to copy. Not modified.
    */
   public Capsule3D(Capsule3DReadOnly other)
   {
      set(other);
   }

   /**
    * Copies the {@code other} capsule data into {@code this}.
    *
    * @param other the other capsule to copy. Not modified.
    */
   @Override
   public void set(Capsule3D other)
   {
      Capsule3DBasics.super.set(other);
   }

   /** {@inheritDoc} */
   @Override
   public void setRadius(double radius)
   {
      if (radius < 0.0)
         throw new IllegalArgumentException("The radius of a Capsule3D cannot be negative: " + radius);
      this.radius = radius;
   }

   /** {@inheritDoc} */
   @Override
   public void setLength(double length)
   {
      if (length < 0.0)
         throw new IllegalArgumentException("The length of a Capsule3D cannot be negative: " + length);
      this.length = length;
      halfLength = 0.5 * length;
   }

   /** {@inheritDoc} */
   @Override
   public double getRadius()
   {
      return radius;
   }

   /** {@inheritDoc} */
   @Override
   public double getLength()
   {
      return length;
   }

   /** {@inheritDoc} */
   @Override
   public double getHalfLength()
   {
      return halfLength;
   }

   /** {@inheritDoc} */
   @Override
   public Point3DBasics getPosition()
   {
      return position;
   }

   /** {@inheritDoc} */
   @Override
   public Vector3DBasics getAxis()
   {
      return axis;
   }

   /** {@inheritDoc} */
   @Override
   public Point3DReadOnly getTopCenter()
   {
      return topCenter;
   }

   /** {@inheritDoc} */
   @Override
   public Point3DReadOnly getBottomCenter()
   {
      return bottomCenter;
   }

   @Override
   public Capsule3D copy()
   {
      return new Capsule3D(this);
   }

   /**
    * Tests on a per component basis if {@code other} and {@code this} are equal to an {@code epsilon}.
    *
    * @param other   the other capsule to compare against this. Not modified.
    * @param epsilon tolerance to use when comparing each component.
    * @return {@code true} if the two capsules are equal component-wise, {@code false} otherwise.
    */
   @Override
   public boolean epsilonEquals(Capsule3D other, double epsilon)
   {
      return Capsule3DBasics.super.epsilonEquals(other, epsilon);
   }

   /**
    * Compares {@code this} and {@code other} to determine if the two capsules are geometrically
    * similar.
    *
    * @param other   the capsule to compare to. Not modified.
    * @param epsilon the tolerance of the comparison.
    * @return {@code true} if the capsules represent the same geometry, {@code false} otherwise.
    */
   @Override
   public boolean geometricallyEquals(Capsule3D other, double epsilon)
   {
      return Capsule3DBasics.super.geometricallyEquals(other, epsilon);
   }

   /**
    * Tests if the given {@code object}'s class is the same as this, in which case the method returns
    * {@link #equals(Capsule3DReadOnly)}, it returns {@code false} otherwise.
    *
    * @param object the object to compare against this. Not modified.
    * @return {@code true} if {@code object} and this are exactly equal, {@code false} otherwise.
    */
   @Override
   public boolean equals(Object object)
   {
      if (object instanceof Capsule3DReadOnly)
         return Capsule3DBasics.super.equals((Capsule3DReadOnly) object);
      else
         return false;
   }

   /**
    * Calculates and returns a hash code value from the value of each component of this capsule 3D.
    *
    * @return the hash code value for this capsule 3D.
    */
   @Override
   public int hashCode()
   {
      long hash = 1L;
      hash = EuclidHashCodeTools.toLongHashCode(length, radius);
      hash = EuclidHashCodeTools.combineHashCode(hash, EuclidHashCodeTools.toLongHashCode(position, axis));
      return EuclidHashCodeTools.toIntHashCode(hash);
   }

   /**
    * Provides a {@code String} representation of this capsule 3D as follows:
    *
    * <pre>
    * Capsule 3D: [position: (-0.362, -0.617,  0.066 ), axis: ( 0.634, -0.551, -0.543 ), length:  0.170, radius:  0.906]
    * </pre>
    *
    * @return the {@code String} representing this capsule 3D.
    */
   @Override
   public String toString()
   {
      return EuclidShapeIOTools.getCapsule3DString(this);
   }
}
