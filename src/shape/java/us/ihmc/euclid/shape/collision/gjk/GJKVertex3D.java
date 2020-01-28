package us.ihmc.euclid.shape.collision.gjk;

import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.tools.TupleTools;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;

/**
 * Vertex 3D that belongs to a simplex used in the Gilbert-Johnson-Keerthi algorithm.
 * <p>
 * A {@code GJKVertex3D} represents the difference of two supporting vertices of two shapes.
 * </p>
 *
 * @author Sylvain Bertrand
 * @see GilbertJohnsonKeerthiCollisionDetector
 */
public class GJKVertex3D implements Point3DReadOnly
{
   /** The coordinates of this vertex. */
   private final double x, y, z;
   /** The supporting vertex from the first shape. */
   private final Point3DReadOnly vertexOnShapeA;
   /** The supporting vertex from the second shape. */
   private final Point3DReadOnly vertexOnShapeB;

   /**
    * Creates a new vertex and initializes its coordinates as follows:<br>
    * {@code this = vertexOnShapeA - vertexOnShapeB}.
    *
    * @param vertexOnShapeA the supporting vertex from the first shape. Not modified, reference saved.
    * @param vertexOnShapeB the supporting vertex from the second shape. Not modified, reference saved.
    */
   public GJKVertex3D(Point3DReadOnly vertexOnShapeA, Point3DReadOnly vertexOnShapeB)
   {
      this.vertexOnShapeA = vertexOnShapeA;
      this.vertexOnShapeB = vertexOnShapeB;
      x = vertexOnShapeA.getX() - vertexOnShapeB.getX();
      y = vertexOnShapeA.getY() - vertexOnShapeB.getY();
      z = vertexOnShapeA.getZ() - vertexOnShapeB.getZ();
   }

   /**
    * Gets the supporting vertex from the first shape this vertex was constructed with.
    *
    * @return the supporting vertex on the first shape.
    */
   public Point3DReadOnly getVertexOnShapeA()
   {
      return vertexOnShapeA;
   }

   /**
    * Gets the supporting vertex from the second shape this vertex was constructed with.
    *
    * @return the supporting vertex on the second shape.
    */
   public Point3DReadOnly getVertexOnShapeB()
   {
      return vertexOnShapeB;
   }

   /**
    * Calculates the dot product of {@code this} and the given {@code tuple3D}.
    *
    * @param tuple3D the second term in the dot product. Not modified.
    * @return the dot product value.
    */
   public double dot(Tuple3DReadOnly tuple3D)
   {
      return TupleTools.dot(this, tuple3D);
   }

   /** {@inheritDoc} */
   @Override
   public double getX()
   {
      return x;
   }

   /** {@inheritDoc} */
   @Override
   public double getY()
   {
      return y;
   }

   /** {@inheritDoc} */
   @Override
   public double getZ()
   {
      return z;
   }

   /**
    * Tests if the given {@code object}'s class is the same as this, in which case the method returns
    * {@link #equals(Tuple3DReadOnly)}, it returns {@code false} otherwise.
    *
    * @param object the object to compare against this. Not modified.
    * @return {@code true} if {@code object} and this are exactly equal, {@code false} otherwise.
    */
   @Override
   public boolean equals(Object object)
   {
      if (object instanceof GJKVertex3D)
         return equals((Tuple3DReadOnly) object);
      else
         return false;
   }

   /**
    * Calculates and returns a hash code value from the value of each component of this vertex 3D.
    *
    * @return the hash code value for this vertex 3D.
    */
   @Override
   public int hashCode()
   {
      long bits = 1L;
      bits = EuclidHashCodeTools.addToHashCode(bits, x);
      bits = EuclidHashCodeTools.addToHashCode(bits, y);
      bits = EuclidHashCodeTools.addToHashCode(bits, z);
      return EuclidHashCodeTools.toIntHashCode(bits);
   }

   /**
    * Provides a {@code String} representation of this point 3D as follows:<br>
    * GJK Vertex 3D: (x, y, z).
    *
    * @return the {@code String} representing this point 3D.
    */
   @Override
   public String toString()
   {
      return "GJK Vertex 3D: " + EuclidCoreIOTools.getTuple3DString(this);
   }
}