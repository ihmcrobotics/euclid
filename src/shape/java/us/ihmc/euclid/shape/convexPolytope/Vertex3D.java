package us.ihmc.euclid.shape.convexPolytope;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.shape.convexPolytope.interfaces.Vertex3DReadOnly;
import us.ihmc.euclid.shape.tools.EuclidShapeIOTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

/**
 * Implementation of a vertex 3D that belongs to a convex polytope 3D.
 * <p>
 * This is part of a Doubly Connected Edge List data structure
 * <a href="https://en.wikipedia.org/wiki/Doubly_connected_edge_list"> link</a>.
 * </p>
 *
 * @author Apoorv Shrivastava
 * @author Sylvain Bertrand
 */
public class Vertex3D implements Vertex3DReadOnly, Point3DBasics
{
   /** The coordinates of this vertex. */
   private double x, y, z;
   /** List of edges that start at this vertex. */
   private final List<HalfEdge3D> associatedEdges = new ArrayList<>();

   /**
    * Creates a new vertex and initializes its coordinates.
    *
    * @param x the x-coordinate of this vertex.
    * @param y the y-coordinate of this vertex.
    * @param z the z-coordinate of this vertex.
    */
   public Vertex3D(double x, double y, double z)
   {
      set(x, y, z);
   }

   /**
    * Creates a new vertex and initializes its coordinates.
    *
    * @param position the initial position for this vertex. Not modified.
    */
   public Vertex3D(Point3DReadOnly position)
   {
      set(position);
   }

   /** {@inheritDoc} */
   @Override
   public List<HalfEdge3D> getAssociatedEdges()
   {
      return associatedEdges;
   }

   /** {@inheritDoc} */
   @Override
   public HalfEdge3D getAssociatedEdge(int index)
   {
      return associatedEdges.get(index);
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
   public boolean removeAssociatedEdge(HalfEdge3D edgeToRemove)
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
   public boolean addAssociatedEdge(HalfEdge3D edgeToAdd)
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
   public HalfEdge3D getEdgeTo(Vertex3DReadOnly destination)
   {
      return (HalfEdge3D) Vertex3DReadOnly.super.getEdgeTo(destination);
   }

   /** {@inheritDoc} */
   @Override
   public int getNumberOfAssociatedEdges()
   {
      return getAssociatedEdges().size();
   }

   /** {@inheritDoc} */
   @Override
   public void setX(double x)
   {
      this.x = x;
   }

   /** {@inheritDoc} */
   @Override
   public void setY(double y)
   {
      this.y = y;
   }

   /** {@inheritDoc} */
   @Override
   public void setZ(double z)
   {
      this.z = z;
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
    * Vertex 3D: (-1.004, -3.379, -0.387 ), number of edges: 3
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
      return EuclidShapeIOTools.getVertex3DString(this);
   }
}
