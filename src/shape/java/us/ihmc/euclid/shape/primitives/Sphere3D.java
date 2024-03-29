package us.ihmc.euclid.shape.primitives;

import us.ihmc.euclid.interfaces.EuclidGeometry;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.shape.primitives.interfaces.Sphere3DBasics;
import us.ihmc.euclid.shape.primitives.interfaces.Sphere3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

/**
 * {@code Sphere3D} represents a 3D sphere defined by its radius and with its origin at its center.
 */
public class Sphere3D implements Sphere3DBasics, Settable<Sphere3D>
{
   /** The position of the center of this sphere. */
   private final Point3D position = new Point3D();
   /** The radius of this sphere. */
   private double radius;

   /**
    * Creates a new sphere 3D with a radius of {@code 1}.
    */
   public Sphere3D()
   {
      this(1.0);
   }

   /**
    * Creates a new sphere 3D and initializes its radius.
    *
    * @param radius the radius for this sphere.
    * @throws IllegalArgumentException if {@code radius} is negative.
    */
   public Sphere3D(double radius)
   {
      setRadius(radius);
   }

   /**
    * Creates a new sphere 3D and initializes its position and radius.
    *
    * @param center the coordinates of this sphere. Not modified.
    * @param radius the radius for this sphere.
    * @throws IllegalArgumentException if {@code radius} is negative.
    */
   public Sphere3D(Point3DReadOnly center, double radius)
   {
      set(center, radius);
   }

   /**
    * Creates a new sphere 3D and initializes its position and radius.
    *
    * @param centerX the x-coordinate of this sphere. Not modified.
    * @param centerY the y-coordinate of this sphere. Not modified.
    * @param centerZ the z-coordinate of this sphere. Not modified.
    * @param radius  the radius for this sphere.
    * @throws IllegalArgumentException if {@code radius} is negative.
    */
   public Sphere3D(double centerX, double centerY, double centerZ, double radius)
   {
      set(centerX, centerY, centerZ, radius);
   }

   /**
    * Creates a new sphere 3D identical to {@code other}
    *
    * @param other the other sphere to copy. Not modified.
    */
   public Sphere3D(Sphere3DReadOnly other)
   {
      set(other);
   }

   @Override
   public Point3D getPosition()
   {
      return position;
   }

   /**
    * Gets the radius of this sphere.
    *
    * @return the value of the radius.
    */
   @Override
   public double getRadius()
   {
      return radius;
   }

   /**
    * Sets the radius of this sphere.
    *
    * @param radius the radius for this sphere.
    * @throws IllegalArgumentException if {@code radius} is negative.
    */
   @Override
   public void setRadius(double radius)
   {
      if (radius < 0.0)
         throw new IllegalArgumentException("The radius of a Sphere 3D cannot be negative.");
      this.radius = radius;
   }

   /**
    * Copies the {@code other} sphere data into {@code this}.
    *
    * @param other the other sphere to copy. Not modified.
    */
   @Override
   public void set(Sphere3D other)
   {
      Sphere3DBasics.super.set(other);
   }

   @Override
   public Sphere3D copy()
   {
      return new Sphere3D(this);
   }

   /**
    * Tests if the given {@code object}'s class is the same as this, in which case the method returns
    * {@link #equals(EuclidGeometry)}, it returns {@code false} otherwise.
    *
    * @param object the object to compare against this. Not modified.
    * @return {@code true} if {@code object} and this are exactly equal, {@code false} otherwise.
    */
   @Override
   public boolean equals(Object object)
   {
      if (object instanceof Sphere3DReadOnly)
         return equals((EuclidGeometry) object);
      else
         return false;
   }

   /**
    * Calculates and returns a hash code value from the value of each component of this sphere 3D.
    *
    * @return the hash code value for this sphere 3D.
    */
   @Override
   public int hashCode()
   {
      long hash = EuclidHashCodeTools.addToHashCode(position.hashCode(), radius);
      return EuclidHashCodeTools.toIntHashCode(hash);
   }

   /**
    * Provides a {@code String} representation of this sphere 3D as follows:
    *
    * <pre>
    * Sphere 3D: [position: (-0.362, -0.617,  0.066 ), radius:  0.906]
    * </pre>
    *
    * @return the {@code String} representing this sphere 3D.
    */
   @Override
   public String toString()
   {
      return toString(EuclidCoreIOTools.DEFAULT_FORMAT);
   }
}
