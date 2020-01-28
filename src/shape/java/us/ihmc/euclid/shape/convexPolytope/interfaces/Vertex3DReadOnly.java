package us.ihmc.euclid.shape.convexPolytope.interfaces;

import java.util.Collection;

import us.ihmc.euclid.tools.TupleTools;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

/**
 * Read-only interface for a vertex 3D that belongs to a convex polytope 3D.
 * <p>
 * This is part of a Doubly Connected Edge List data structure
 * <a href="https://en.wikipedia.org/wiki/Doubly_connected_edge_list"> link</a>.
 * </p>
 *
 * @author Sylvain Bertrand
 */
public interface Vertex3DReadOnly extends Point3DReadOnly
{
   /**
    * Get the collection of edges that originate at this vertex.
    *
    * @return collection of read only references to the edges that originate at this edge.
    */
   Collection<? extends HalfEdge3DReadOnly> getAssociatedEdges();

   /**
    * Gets the i<sup>th</sup> edge that originates at this vertex.
    *
    * @param index the edge index &in; [0; {@link #getNumberOfAssociatedEdges()}[.
    * @return the read only reference to the edge.
    */
   HalfEdge3DReadOnly getAssociatedEdge(int index);

   /**
    * Tests whether the given edge is already associated to this vertex.
    *
    * @param edgeToCheck the half edge that is to be tested for association. Not modified.
    * @return {@code true} if the given edge is already associated to this vertex, {@code false}
    *         otherwise.
    */
   default boolean isEdgeAssociated(HalfEdge3DReadOnly edgeToCheck)
   {
      for (HalfEdge3DReadOnly associatedEdge : getAssociatedEdges())
      {
         if (edgeToCheck == associatedEdge)
            return true;
      }
      return false;
   }

   /**
    * Returns the number of edges already associated to this vertex.
    *
    * @return the number of edges that originate at this vertex.
    */
   int getNumberOfAssociatedEdges();

   /**
    * Retrieves the half-edge that originates from this vertex and ends at the given
    * {@code destination}.
    *
    * @param destination the vertex to which the desired half-edge ends at.
    * @return the half-edge starting from {@code this} and ending at {@code destination}, or
    *         {@code null} if no such half-edge exists.
    */
   default HalfEdge3DReadOnly getEdgeTo(Vertex3DReadOnly destination)
   {
      for (int edgeIndex = 0; edgeIndex < getNumberOfAssociatedEdges(); edgeIndex++)
      {
         HalfEdge3DReadOnly candidate = getAssociatedEdge(edgeIndex);

         if (candidate.getDestination() == destination)
            return candidate;
      }

      return null;
   }

   /**
    * Calculates the dot product of {@code this} and the given {@code vector}.
    *
    * @param vector the second term in the dot product. Not modified.
    * @return the dot product value.
    */
   default double dot(Vector3DReadOnly vector)
   {
      return TupleTools.dot(this, vector);
   }

   /**
    * Tests on a per component basis, if this vertex is exactly equal to {@code other}.
    *
    * @param other the other vertex to compare against this. Not modified.
    * @return {@code true} if the two vertices are exactly equal component-wise, {@code false}
    *         otherwise.
    */
   default boolean equals(Vertex3DReadOnly other)
   {
      return Point3DReadOnly.super.equals(other);
   }
}