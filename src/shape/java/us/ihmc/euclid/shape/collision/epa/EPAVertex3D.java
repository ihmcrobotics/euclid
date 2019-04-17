package us.ihmc.euclid.shape.collision.epa;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import us.ihmc.euclid.shape.collision.gjk.GJKVertex3D;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Vertex3DReadOnly;
import us.ihmc.euclid.shape.tools.EuclidShapeIOTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

/**
 * Vertex 3D that belongs to a polytope used in the Expanding Polytope algorithm.
 * <p>
 * A {@code EPAVertex3D} represents the difference of two supporting vertices of two shapes.
 * </p>
 * 
 * @author Sylvain Bertrand
 * @see ExpandingPolytopeAlgorithm
 */
public class EPAVertex3D implements Vertex3DReadOnly
{
   /** The coordinates of this vertex. */
   private final double x, y, z;
   /** The supporting vertex from the first shape. */
   private final Point3DReadOnly vertexOnShapeA;
   /** The supporting vertex from the second shape. */
   private final Point3DReadOnly vertexOnShapeB;
   /** List of edges that start at this vertex. */
   private final List<EPAHalfEdge3D> associatedEdges = new ArrayList<>();

   /**
    * Creates a new vertex from a {@code GJKVertex3D} copying its coordinates the supporting vertex
    * from both shapes.
    * 
    * @param gjkVertex3D the GJK vertex to copy. Not modified.
    */
   public EPAVertex3D(GJKVertex3D gjkVertex3D)
   {
      vertexOnShapeA = gjkVertex3D.getVertexOnShapeA();
      vertexOnShapeB = gjkVertex3D.getVertexOnShapeB();
      x = gjkVertex3D.getX();
      y = gjkVertex3D.getY();
      z = gjkVertex3D.getZ();
   }

   /**
    * Creates a new vertex and initializes its coordinates as follows:<br>
    * {@code this = vertexOnShapeA - vertexOnShapeB}.
    * 
    * @param vertexOnShapeA the supporting vertex from the first shape. Not modified, reference saved.
    * @param vertexOnShapeB the supporting vertex from the second shape. Not modified, reference saved.
    */
   public EPAVertex3D(Point3DReadOnly vertexOnShapeA, Point3DReadOnly vertexOnShapeB)
   {
      this.vertexOnShapeA = vertexOnShapeA;
      this.vertexOnShapeB = vertexOnShapeB;
      x = vertexOnShapeA.getX() - vertexOnShapeB.getX();
      y = vertexOnShapeA.getY() - vertexOnShapeB.getY();
      z = vertexOnShapeA.getZ() - vertexOnShapeB.getZ();
   }

   /**
    * Removes an edge previously associated to this vertex.
    * <p>
    * In the case the given edge was never associated, nothing changes.
    * </p>
    *
    * @param indexOfEdgeToRemove the associated edge that is to be removed. Not modified.
    */
   public void removeAssociatedEdge(int indexOfEdgeToRemove)
   {
      int indexOfLastElement = associatedEdges.size() - 1;

      if (indexOfEdgeToRemove != indexOfLastElement)
         Collections.swap(associatedEdges, indexOfEdgeToRemove, indexOfLastElement);

      associatedEdges.remove(indexOfLastElement);
   }

   /**
    * Removes an edge previously associated to this vertex.
    * <p>
    * In the case the given edge was never associated, nothing changes.
    * </p>
    *
    * @param edgeToRemove the associated edge that is to be removed. Not modified.
    * @return {@code true} if the edge was associated to this vertex, {@code false} otherwise.
    */
   public boolean removeAssociatedEdge(EPAHalfEdge3D edgeToRemove)
   {
      return associatedEdges.remove(edgeToRemove);
   }

   /**
    * Remove all edges previously associated to this vertex.
    */
   public void clearAssociatedEdgeList()
   {
      associatedEdges.clear();
   }

   /**
    * Associates an edge to this vertex, the edge has to start from this vertex.
    * <p>
    * In the case the given edge was already associated, nothing changes.
    * </p>
    *
    * @param edgeToAdd the add to associate to this vertex. Not modified, reference saved.
    * @return {@code true} if the edge was not associated to this vertex, {@code false} otherwise.
    * @throws IllegalArgumentException if {@code edgeToAdd.getOrigin() != this}.
    */
   public boolean addAssociatedEdge(EPAHalfEdge3D edgeToAdd)
   {
      if (!isEdgeAssociated(edgeToAdd))
      {
         if (edgeToAdd.getOrigin() != this)
            throw new IllegalArgumentException("A vertex's associated edges should originate from this same vertex.");
         associatedEdges.add(edgeToAdd);
         return true;
      }
      return false;
   }

   /** {@inheritDoc} */
   @Override
   public EPAHalfEdge3D getEdgeTo(Vertex3DReadOnly destination)
   {
      return (EPAHalfEdge3D) Vertex3DReadOnly.super.getEdgeTo(destination);
   }

   /** {@inheritDoc} */
   @Override
   public int getNumberOfAssociatedEdges()
   {
      return associatedEdges.size();
   }

   /** {@inheritDoc} */
   @Override
   public List<EPAHalfEdge3D> getAssociatedEdges()
   {
      return associatedEdges;
   }

   /** {@inheritDoc} */
   @Override
   public EPAHalfEdge3D getAssociatedEdge(int index)
   {
      return associatedEdges.get(index);
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
    * {@link #equals(Vertex3DReadOnly)}, it returns {@code false} otherwise.
    *
    * @param object the object to compare against this. Not modified.
    * @return {@code true} if {@code object} and this are exactly equal, {@code false} otherwise.
    */
   @Override
   public boolean equals(Object object)
   {
      if (object instanceof Vertex3DReadOnly)
         return equals((Vertex3DReadOnly) object);
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
      long hashCode = 1L;
      hashCode = EuclidHashCodeTools.addToHashCode(hashCode, x);
      hashCode = EuclidHashCodeTools.addToHashCode(hashCode, y);
      hashCode = EuclidHashCodeTools.addToHashCode(hashCode, z);
      return EuclidHashCodeTools.toIntHashCode(hashCode);
   }

   /**
    * Provides a {@code String} representation of this vertex 3D as follows:
    * 
    * <pre>
    * EPA Vertex 3D: (-1.004, -3.379, -0.387 ), number of edges: 3
    *         [(-1.004, -3.379, -0.387 ); ( 1.372, -3.150,  0.556 )]
    *         [(-1.004, -3.379, -0.387 ); (-0.937, -3.539, -0.493 )]
    *         [(-1.004, -3.379, -0.387 ); (-1.046, -3.199, -0.303 )]
    * </pre>
    * 
    * @return the {@code String} representing this vertex 3D.
    */
   @Override
   public String toString()
   {
      return "EPA " + EuclidShapeIOTools.getVertex3DString(this);
   }
}