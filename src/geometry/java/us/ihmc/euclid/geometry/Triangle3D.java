package us.ihmc.euclid.geometry;

import us.ihmc.euclid.geometry.interfaces.Triangle3DBasics;
import us.ihmc.euclid.geometry.interfaces.Triangle3DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryIOTools;
import us.ihmc.euclid.interfaces.GeometryObject;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

/**
 * Represents a 3D triangle defined by its three vertices A, B, and C.
 *
 * @author Sylvain Bertrand
 */
public class Triangle3D implements Triangle3DBasics, GeometryObject<Triangle3D>
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

   /**
    * Tests on a per component basis on each vertex if this triangle is equal to {@code other} with the
    * tolerance {@code epsilon}.
    *
    * @param other   the query. Not modified.
    * @param epsilon the tolerance to use.
    * @return {@code true} if the two triangles are equal, {@code false} otherwise.
    */
   @Override
   public boolean epsilonEquals(Triangle3D other, double epsilon)
   {
      return Triangle3DBasics.super.epsilonEquals(other, epsilon);
   }

   /**
    * Compares {@code this} to {@code other} to determine if the two triangle are geometrically
    * similar.
    * <p>
    * Two triangles are geometrically similar 3 pairs geometrically equal vertices and the same
    * ordering and winding, i.e. clockwise or counter-clockwise.
    * </p>
    *
    * @param other   the triangle to compare to. Not modified.
    * @param epsilon the tolerance of the comparison.
    * @return {@code true} if the two triangles represent the same geometry, {@code false} otherwise.
    */
   @Override
   public boolean geometricallyEquals(Triangle3D other, double epsilon)
   {
      return Triangle3DBasics.super.geometricallyEquals(other, epsilon);
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
      long hashCode = EuclidHashCodeTools.combineHashCode(a.hashCode(), b.hashCode());
      hashCode = EuclidHashCodeTools.combineHashCode(hashCode, c.hashCode());
      return EuclidHashCodeTools.toIntHashCode(hashCode);
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
      return EuclidGeometryIOTools.getTriangle3DString(this);
   }
}