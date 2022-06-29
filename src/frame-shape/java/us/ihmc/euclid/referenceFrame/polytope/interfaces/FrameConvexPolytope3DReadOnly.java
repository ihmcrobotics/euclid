package us.ihmc.euclid.referenceFrame.polytope.interfaces;

import java.util.List;

import us.ihmc.euclid.geometry.interfaces.BoundingBox3DBasics;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameShape3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameBoundingBox3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameShape3DPoseReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameShape3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameShapeIOTools;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameShapeTools;
import us.ihmc.euclid.shape.convexPolytope.interfaces.ConvexPolytope3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Vertex3DReadOnly;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

/**
 * Read-only interface for a convex polytope 3D expressed in a given reference frame.
 * <p>
 * This is part of a Doubly Connected Edge List data structure
 * <a href="https://en.wikipedia.org/wiki/Doubly_connected_edge_list"> link</a>.
 * </p>
 *
 * @author Sylvain Bertrand
 */
public interface FrameConvexPolytope3DReadOnly extends ConvexPolytope3DReadOnly, FrameShape3DReadOnly
{
   /**
    * Gets this polytope's faces.
    *
    * @return this polytope's faces.
    */
   @Override
   List<? extends FrameFace3DReadOnly> getFaces();

   /**
    * Gets the i<sup>th</sup> face of this polytope.
    *
    * @param index the face index &in; [0; {@link #getNumberOfFaces()}[.
    * @return the read-only reference to the face.
    */
   @Override
   default FrameFace3DReadOnly getFace(int index)
   {
      return getFaces().get(index);
   }

   /**
    * Gets this polytope's half-edges.
    * <p>
    * Note that the number of half-edges is twice the number of edges in this polytope.
    * </p>
    *
    * @return this polytope's half-edges.
    */
   @Override
   List<? extends FrameHalfEdge3DReadOnly> getHalfEdges();

   /**
    * Gets the i<sup>th</sup> half-edge of this polytope.
    *
    * @param index the half-edge index &in; [0; {@link #getNumberOfHalfEdges()}[.
    * @return the read-only reference to the half-edge.
    */
   @Override
   default FrameHalfEdge3DReadOnly getHalfEdge(int index)
   {
      return getHalfEdges().get(index);
   }

   /**
    * Gets this polytope's vertices.
    *
    * @return this polytope's vertices.
    */
   @Override
   List<? extends FrameVertex3DReadOnly> getVertices();

   /**
    * Gets the i<sup>th</sup> vertex of this polytope.
    *
    * @param index the vertex index &in; [0; {@link #getNumberOfVertices()}[.
    * @return the read-only reference to the vertex.
    */
   @Override
   default FrameVertex3DReadOnly getVertex(int index)
   {
      return getVertices().get(index);
   }

   /** {@inheritDoc} */
   @Override
   FrameBoundingBox3DReadOnly getBoundingBox();

   /**
    * Finds and returns the closest face to the query.
    *
    * @param query the coordinates of the query. Not modified.
    * @return the closest face to the query.
    */
   @Override
   default FrameFace3DReadOnly getClosestFace(Point3DReadOnly query)
   {
      return (FrameFace3DReadOnly) ConvexPolytope3DReadOnly.super.getClosestFace(query);
   }

   /**
    * Finds and returns the closest face to the query.
    *
    * @param query the coordinates of the query. Not modified.
    * @return the closest face to the query.
    */
   default FrameFace3DReadOnly getClosestFace(FramePoint3DReadOnly query)
   {
      checkReferenceFrameMatch(query);
      return (FrameFace3DReadOnly) ConvexPolytope3DReadOnly.super.getClosestFace(query);
   }

   /** {@inheritDoc} */
   @Override
   default FrameVertex3DReadOnly getSupportingVertex(Vector3DReadOnly supportDirection)
   {
      return getSupportingVertex((FrameVertex3DReadOnly) null, supportDirection);
   }

   /** {@inheritDoc} */
   @Override
   default FrameVertex3DReadOnly getSupportingVertex(FrameVector3DReadOnly supportDirection)
   {
      checkReferenceFrameMatch(supportDirection);
      return getSupportingVertex((Vector3DReadOnly) supportDirection);
   }

   /** {@inheritDoc} */
   @Override
   default FrameVertex3DReadOnly getSupportingVertex(Vertex3DReadOnly seed, Vector3DReadOnly supportDirection)
   {
      return (FrameVertex3DReadOnly) ConvexPolytope3DReadOnly.super.getSupportingVertex(seed, supportDirection);
   }

   /**
    * Finds the supporting vertex in the given direction given a starting vertex for the search.
    * <p>
    * The supporting vertex is the point or vertex on this shape that is the farthest along a given
    * direction.
    * </p>
    *
    * @param seed                   the starting point for the search, the closer it is to the
    *                               supporting vertex, the faster the search will converge. The seed
    *                               has to be a vertex that belongs to this polytope. Can be
    *                               {@code null}.
    * @param supportDirection       the direction to search for the farthest point on this shape. Not
    *                               modified.
    * @param supportingVertexToPack point used to store the supporting vertex coordinates. Modified.
    * @return {@code true} when the method succeeded and packed the supporting vertex coordinates,
    *         {@code false} when the method failed in which case {@code supportingVertexToPack} remains
    *         unchanged.
    * @throws ReferenceFrameMismatchException if the frame argument is not expressed in the same
    *                                         reference frame as {@code this}.
    */
   default FrameVertex3DReadOnly getSupportingVertex(Vertex3DReadOnly seed, FrameVector3DReadOnly supportDirection)
   {
      checkReferenceFrameMatch(supportDirection);
      return getSupportingVertex(seed, (Vector3DReadOnly) supportDirection);
   }

   /** {@inheritDoc} */
   @Override
   default void getBoundingBox(BoundingBox3DBasics boundingBoxToPack)
   {
      ConvexPolytope3DReadOnly.super.getBoundingBox(boundingBoxToPack);
   }

   /** {@inheritDoc} */
   @Override
   default void getBoundingBox(ReferenceFrame destinationFrame, BoundingBox3DBasics boundingBoxToPack)
   {
      EuclidFrameShapeTools.boundingBoxConvexPolytope3D(this, destinationFrame, boundingBoxToPack);
   }

   /**
    * Returns {@code null} as this shape is not defined by a pose.
    */
   @Override
   default FrameShape3DPoseReadOnly getPose()
   {
      return null;
   }

   /** {@inheritDoc} */
   @Override
   FixedFrameShape3DBasics copy();

   /**
    * Gets the representative {@code String} of this frame convex polytope 3D given a specific format
    * to use.
    * <p>
    * Using the default format {@link EuclidCoreIOTools#DEFAULT_FORMAT}, this provides a {@code String} as follows:
    *
    * <pre>
    * Convex polytope 3D: number of: [faces: 4, edges: 12, vertices: 4
    * Face list:
    *    centroid: ( 0.582, -0.023,  0.160 ), normal: ( 0.516, -0.673,  0.530 )
    *    centroid: ( 0.420,  0.176,  0.115 ), normal: (-0.038,  0.895, -0.444 )
    *    centroid: ( 0.264, -0.253, -0.276 ), normal: ( 0.506,  0.225, -0.833 )
    *    centroid: ( 0.198, -0.176, -0.115 ), normal: (-0.643, -0.374,  0.668 )
    * Edge list:
    *    [( 0.674,  0.482,  0.712 ); ( 0.870,  0.251,  0.229 )]
    *    [( 0.870,  0.251,  0.229 ); ( 0.204, -0.803, -0.461 )]
    *    [( 0.204, -0.803, -0.461 ); ( 0.674,  0.482,  0.712 )]
    *    [( 0.870,  0.251,  0.229 ); ( 0.674,  0.482,  0.712 )]
    *    [( 0.674,  0.482,  0.712 ); (-0.283, -0.207, -0.595 )]
    *    [(-0.283, -0.207, -0.595 ); ( 0.870,  0.251,  0.229 )]
    *    [( 0.204, -0.803, -0.461 ); ( 0.870,  0.251,  0.229 )]
    *    [( 0.870,  0.251,  0.229 ); (-0.283, -0.207, -0.595 )]
    *    [(-0.283, -0.207, -0.595 ); ( 0.204, -0.803, -0.461 )]
    *    [( 0.674,  0.482,  0.712 ); ( 0.204, -0.803, -0.461 )]
    *    [( 0.204, -0.803, -0.461 ); (-0.283, -0.207, -0.595 )]
    *    [(-0.283, -0.207, -0.595 ); ( 0.674,  0.482,  0.712 )]
    * Vertex list:
    *    ( 0.674,  0.482,  0.712 )
    *    ( 0.870,  0.251,  0.229 )
    *    ( 0.204, -0.803, -0.461 )
    *    (-0.283, -0.207, -0.595 )
    * worldFrame
    * </pre>
    * </p>
    */
   @Override
   default String toString(String format)
   {
      return EuclidFrameShapeIOTools.getFrameConvexPolytope3DString(format, this);
   }
}
