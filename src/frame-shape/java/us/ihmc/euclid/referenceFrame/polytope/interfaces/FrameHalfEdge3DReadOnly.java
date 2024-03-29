package us.ihmc.euclid.referenceFrame.polytope.interfaces;

import us.ihmc.euclid.referenceFrame.exceptions.ReferenceFrameMismatchException;
import us.ihmc.euclid.referenceFrame.interfaces.FrameLineSegment3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameShapeIOTools;
import us.ihmc.euclid.shape.convexPolytope.interfaces.HalfEdge3DReadOnly;

/**
 * Read-only interface for a half-edge 3D that belongs to a convex polytope 3D expressed in a given
 * reference frame.
 * <p>
 * This is part of a Doubly Connected Edge List data structure
 * <a href="https://en.wikipedia.org/wiki/Doubly_connected_edge_list"> link</a>.
 * </p>
 *
 * @author Sylvain Bertrand
 */
public interface FrameHalfEdge3DReadOnly extends HalfEdge3DReadOnly, FrameLineSegment3DReadOnly
{
   /** {@inheritDoc} */
   @Override
   FrameVertex3DReadOnly getOrigin();

   /** {@inheritDoc} */
   @Override
   FrameVertex3DReadOnly getDestination();

   /** {@inheritDoc} */
   @Override
   default FramePoint3DReadOnly getFirstEndpoint()
   {
      return getOrigin();
   }

   /** {@inheritDoc} */
   @Override
   default FramePoint3DReadOnly getSecondEndpoint()
   {
      return getDestination();
   }

   /** {@inheritDoc} */
   @Override
   FrameHalfEdge3DReadOnly getTwin();

   /** {@inheritDoc} */
   @Override
   FrameHalfEdge3DReadOnly getNext();

   /** {@inheritDoc} */
   @Override
   FrameHalfEdge3DReadOnly getPrevious();

   /**
    * Computes the minimum distance between a given point and the infinitely long line supporting this
    * half-edge.
    *
    * @param point the location of the query. Not modified.
    * @return the distance from the query to the support line.
    * @throws ReferenceFrameMismatchException if the argument is not expressed in the same reference
    *                                         frame as {@code this}.
    */
   default double distanceFromSupportLine(FramePoint3DReadOnly point)
   {
      checkReferenceFrameMatch(point);
      return HalfEdge3DReadOnly.super.distanceFromSupportLine(point);
   }

   /**
    * Provides a {@code String} representation of this frame half-edge 3D as follows:
    *
    * <pre>
    * Half-edge 3D: [( 2.350,  4.284,  0.427 ); ( 3.310,  6.118, -3.108 )]
    *    Twin    : [( 3.310,  6.118, -3.108 ); ( 2.350,  4.284,  0.427 )]
    *    Next    : [( 3.310,  6.118, -3.108 ); ( 3.411,  2.581, -3.144 )]
    *    Previous: [( 3.411,  2.581, -3.144 ); ( 2.350,  4.284,  0.427 )]
    *    Face: centroid: ( 3.024,  4.328, -1.941 ), normal: ( 0.961,  0.025,  0.274 )
    *    worldFrame
    * </pre>
    */
   @Override
   default String toString(String format)
   {
      return EuclidFrameShapeIOTools.getFrameHalfEdge3DString(format, this);
   }
}
