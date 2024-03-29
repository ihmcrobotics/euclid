package us.ihmc.euclid.referenceFrame.polytope.interfaces;

import java.util.Collection;

import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameShapeIOTools;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Vertex3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreIOTools;

/**
 * Read-only interface for a vertex 3D that belongs to a convex polytope 3D expressed in given
 * reference frame.
 * <p>
 * This is part of a Doubly Connected Edge List data structure
 * <a href="https://en.wikipedia.org/wiki/Doubly_connected_edge_list"> link</a>.
 * </p>
 *
 * @author Sylvain Bertrand
 */
public interface FrameVertex3DReadOnly extends Vertex3DReadOnly, FramePoint3DReadOnly
{
   /** {@inheritDoc} */
   @Override
   Collection<? extends FrameHalfEdge3DReadOnly> getAssociatedEdges();

   /** {@inheritDoc} */
   @Override
   FrameHalfEdge3DReadOnly getAssociatedEdge(int index);

   /**
    * Tests whether the given edge is already associated to this vertex.
    *
    * @param edgeToCheck the half edge that is to be tested for association. Not modified.
    * @return {@code true} if the given edge is already associated to this vertex, {@code false}
    *         otherwise.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default boolean isEdgeAssociated(FrameHalfEdge3DReadOnly edgeToCheck)
   {
      checkReferenceFrameMatch(edgeToCheck);
      return Vertex3DReadOnly.super.isEdgeAssociated(edgeToCheck);
   }

   /** {@inheritDoc} */
   @Override
   default FrameHalfEdge3DReadOnly getEdgeTo(Vertex3DReadOnly destination)
   {
      return (FrameHalfEdge3DReadOnly) Vertex3DReadOnly.super.getEdgeTo(destination);
   }

   /**
    * Retrieves the half-edge that originates from this vertex and ends at the given
    * {@code destination}.
    *
    * @param destination the vertex to which the desired half-edge ends at.
    * @return the half-edge starting from {@code this} and ending at {@code destination}, or
    *         {@code null} if no such half-edge exists.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default FrameHalfEdge3DReadOnly getEdgeTo(FrameVertex3DReadOnly destination)
   {
      checkReferenceFrameMatch(destination);
      return (FrameHalfEdge3DReadOnly) Vertex3DReadOnly.super.getEdgeTo(destination);
   }

   /**
    * Calculates the dot product of {@code this} and the given {@code vector}.
    *
    * @param vector the second term in the dot product. Not modified.
    * @return the dot product value.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default double dot(FrameVector3DReadOnly vector)
   {
      return Vertex3DReadOnly.super.dot(vector);
   }

   /**
    * Gets the representative {@code String} of this frame vertex 3D given a specific format to use.
    * <p>
    * Using the default format {@link EuclidCoreIOTools#DEFAULT_FORMAT}, this provides a {@code String}
    * as follows:
    *
    * <pre>
    * Vertex 3D: (-1.004, -3.379, -0.387 ), number of edges: 3
    *         [(-1.004, -3.379, -0.387 ); ( 1.372, -3.150,  0.556 )]
    *         [(-1.004, -3.379, -0.387 ); (-0.937, -3.539, -0.493 )]
    *         [(-1.004, -3.379, -0.387 ); (-1.046, -3.199, -0.303 )]
    *         worldFrame
    * </pre>
    * </p>
    */
   @Override
   default String toString(String format)
   {
      return EuclidFrameShapeIOTools.getFrameVertex3DString(format, this);
   }
}
