package us.ihmc.euclid.geometry;

import us.ihmc.euclid.geometry.interfaces.Triangle3DBasics;
import us.ihmc.euclid.geometry.interfaces.Triangle3DReadOnly;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

/**
 * Represents a 3D triangle defined by its three vertices A, B, and C.
 *
 * @author Sylvain Bertrand
 */
public class Triangle3D implements Triangle3DBasics, Settable<Triangle3D>
{
   /** The first vertex defining this triangle. */
   private final Point3D a = new Point3D();
   /** The second vertex defining this triangle. */
   private final Point3D b = new Point3D();
   /** The third vertex defining this triangle. */
   private final Point3D c = new Point3D();

   /**
    * Default constructor that initializes all vertices of this triangle to zero.
    */
   public Triangle3D()
   {
   }

   /**
    * Creates a new triangle 3D and initializes it to have the given vertices.
    *
    * @param a the coordinate for the first vertex of this triangle. Not modified.
    * @param b the coordinate for the second vertex of this triangle. Not modified.
    * @param c the coordinate for the third vertex of this triangle. Not modified.
    */
   public Triangle3D(Point3DReadOnly a, Point3DReadOnly b, Point3DReadOnly c)
   {
      set(a, b, c);
   }

   /**
    * Creates a new triangle 3D and initializes it to {@code other}.
    *
    * @param other the other triangle used to initialize this triangle. Not modified.
    */
   public Triangle3D(Triangle3DReadOnly other)
   {
      set(other);
   }

   /**
    * Sets this triangle to be the same as the given triangle.
    *
    * @param other the other triangle to copy. Not modified.
    */
   @Override
   public void set(Triangle3D other)
   {
      Triangle3DBasics.super.set(other);
   }

   /** {@inheritDoc} */
   @Override
   public Point3D getA()
   {
      return a;
   }

   /** {@inheritDoc} */
   @Override
   public Point3D getB()
   {
      return b;
   }

   /** {@inheritDoc} */
   @Override
   public Point3D getC()
   {
      return c;
   }

   @Override
   public boolean equals(Object object)
   {
      if (object instanceof Triangle3DReadOnly)
         return Triangle3DBasics.super.equals((Triangle3DReadOnly) object);
      else
         return false;
   }

   /**
    * Calculates and returns a hash code value from the value of each vertex coordinate of this
    * triangle.
    *
    * @return the hash code value for this triangle 3D.
    */
   @Override
   public int hashCode()
   {
      return EuclidHashCodeTools.toIntHashCode(a, b, c);
   }

   /**
    * Provides a {@code String} representation of this triangle 3D as follows:<br>
    * Triangle 3D: [(Ax, Ay, Az), (Bx, By, Bz), (Cx, Cy, Cz)]
    *
    * @return the {@code String} representing this triangle 3D.
    */
   @Override
   public String toString()
   {
      return toString(EuclidCoreIOTools.DEFAULT_FORMAT);
   }
}