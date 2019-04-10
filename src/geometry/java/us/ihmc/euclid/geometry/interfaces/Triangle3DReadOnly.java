package us.ihmc.euclid.geometry.interfaces;

import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tools.EuclidCoreTools;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

/**
 * Read-only interface for a 3D triangle defined by its three vertices A, B, and C.
 * 
 * @author Sylvain Bertrand
 */
public interface Triangle3DReadOnly
{
   /**
    * Gets the read-only reference to the first vertex of this triangle.
    * 
    * @return the reference to the first vertex of this line segment.
    */
   Point3DReadOnly getA();

   /**
    * Gets the read-only reference to the second vertex of this triangle.
    * 
    * @return the reference to the second vertex of this line segment.
    */
   Point3DReadOnly getB();

   /**
    * Gets the read-only reference to the third vertex of this triangle.
    * 
    * @return the reference to the second third of this line segment.
    */
   Point3DReadOnly getC();

   /**
    * Tests whether any of this triangle vertices contains {@link Double#NaN}.
    * 
    * @return {@code true} if at least one vertex contains {@link Double#NaN}.
    */
   default boolean containsNaN()
   {
      return getA().containsNaN() || getB().containsNaN() || getC().containsNaN();
   }

   /**
    * Calculates the length of the first edge.
    * 
    * @return the AB edge's length.
    */
   default double getAB()
   {
      return getA().distance(getB());
   }

   /**
    * Calculates the length of the second edge.
    * 
    * @return the BC edge's length.
    */
   default double getBC()
   {
      return getB().distance(getC());
   }

   /**
    * Calculates the length of the third edge.
    * 
    * @return the CA edge's length.
    */
   default double getCA()
   {
      return getC().distance(getA());
   }

   /**
    * Calculates the length squared of the first edge.
    * 
    * @return the AB edge's length squared.
    */
   default double getABSquared()
   {
      return getA().distanceSquared(getB());
   }

   /**
    * Calculates the length squared of the second edge.
    * 
    * @return the BC edge's length squared.
    */
   default double getBCSquared()
   {
      return getB().distanceSquared(getC());
   }

   /**
    * Calculates the length squared of the third edge.
    * 
    * @return the CA edge's length squared.
    */
   default double getCASquared()
   {
      return getC().distanceSquared(getA());
   }

   /**
    * Calculates this triangle's area.
    * 
    * @return this triangle's area.
    */
   default double getArea()
   {
      return EuclidGeometryTools.triangleArea(getA(), getB(), getC());
   }

   /**
    * Tests whether this triangle is equilateral, i.e. <tt>AB==BC==CA</tt>.
    * 
    * @param epsilon the tolerance to use when comparing edges' lengths.
    * @return {@code true} if this triangle is equilateral, {@code false} otherwise.
    */
   default boolean isEquilateral(double epsilon)
   {
      if (!EuclidCoreTools.epsilonEquals(getABSquared(), getCASquared(), epsilon))
         return false;

      return EuclidCoreTools.epsilonEquals(getCASquared(), getBCSquared(), epsilon);
   }

   /**
    * Tests on a per component basis on each vertex if this triangle is equal to {@code other} with the
    * tolerance {@code epsilon}.
    * 
    * @param other the query. Not modified.
    * @param epsilon the tolerance to use.
    * @return {@code true} if the two triangles are equal, {@code false} otherwise.
    */
   default boolean epsilonEquals(Triangle3DReadOnly other, double epsilon)
   {
      if (!getA().epsilonEquals(other.getA(), epsilon))
         return false;
      if (!getB().epsilonEquals(other.getB(), epsilon))
         return false;
      if (!getC().epsilonEquals(other.getC(), epsilon))
         return false;
      return true;
   }

   /**
    * Tests on a per component basis if this triangle is exactly equal to {@code other}.
    * 
    * @param other the other triangle to compare against this. Not modified.
    * @return {@code true} if the triangle are exactly equal component-wise, {@code false} otherwise.
    */
   default boolean equals(Triangle3DReadOnly other)
   {
      if (other == this)
         return true;
      else if (other == null)
         return false;
      else
         return getA().equals(other.getA()) && getB().equals(other.getB()) && getC().equals(other.getC());
   }

   /**
    * Compares {@code this} to {@code other} to determine if the two triangle are geometrically
    * similar.
    * <p>
    * Two triangles are geometrically similar 3 pairs geometrically equal vertices and the same
    * ordering and winding, i.e. clockwise or counter-clockwise.
    * </p>
    * 
    * @param other the triangle to compare to. Not modified.
    * @param epsilon the tolerance of the comparison.
    * @return {@code true} if the two triangles represent the same geometry, {@code false} otherwise.
    */
   default boolean geometricallyEquals(Triangle3DReadOnly other, double epsilon)
   {
      return geometricallyEquals(other.getA(), other.getB(), other.getC(), epsilon);
   }

   /**
    * Compares {@code this} to the triangle represented by the given vertices to determine if the two
    * triangle are geometrically similar.
    * 
    * @param a the first vertex of the query. Not modified.
    * @param b the second vertex of the query. Not modified.
    * @param c the third vertex of the query. Not modified.
    * @param epsilon the tolerance of the comparison.
    * @return {@code true} if the two triangles represent the same geometry, {@code false} otherwise.
    */
   default boolean geometricallyEquals(Point3DReadOnly a, Point3DReadOnly b, Point3DReadOnly c, double epsilon)
   {
      if (getA().geometricallyEquals(a, epsilon))
      {
         if (getB().geometricallyEquals(b, epsilon))
            return getC().geometricallyEquals(c, epsilon);
         else
            return getB().geometricallyEquals(c, epsilon) && getC().geometricallyEquals(b, epsilon);
      }
      else if (getA().geometricallyEquals(b, epsilon))
      {
         if (getB().geometricallyEquals(a, epsilon))
            return getC().geometricallyEquals(c, epsilon);
         else
            return getB().geometricallyEquals(c, epsilon) && getC().geometricallyEquals(a, epsilon);
      }
      else if (getA().geometricallyEquals(c, epsilon))
      {
         if (getB().geometricallyEquals(a, epsilon))
            return getC().geometricallyEquals(b, epsilon);
         else
            return getB().geometricallyEquals(b, epsilon) && getC().geometricallyEquals(a, epsilon);
      }
      else
      {
         return false;
      }
   }
}
