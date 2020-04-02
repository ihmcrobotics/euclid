package us.ihmc.euclid.referenceFrame.polytope;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameLineSegment3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.ReferenceFrameHolder;
import us.ihmc.euclid.referenceFrame.polytope.interfaces.FrameHalfEdge3DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameShapeIOTools;
import us.ihmc.euclid.shape.convexPolytope.impl.AbstractHalfEdge3D;

/**
 * Implementation of a half-edge 3D that belongs to a convex polytope 3D expressed in a given
 * reference frame.
 * <p>
 * This is part of a Doubly Connected Edge List data structure
 * <a href="https://en.wikipedia.org/wiki/Doubly_connected_edge_list"> link</a>.
 * </p>
 *
 * @author Sylvain Bertrand
 */
public class FrameHalfEdge3D extends AbstractHalfEdge3D<FrameVertex3D, FrameHalfEdge3D, FrameFace3D>
      implements FrameHalfEdge3DReadOnly, FixedFrameLineSegment3DBasics
{
   /**
    * This object does not manage its reference frame, this field is the owner of this half-edge and
    * manages the current reference frame.
    */
   private ReferenceFrameHolder referenceFrameHolder;

   /**
    * Creates a new half-edge and initializes its origin and destination.
    *
    * @param referenceFrameHolder the owner of this half-edge which manages its reference frame.
    *                             Reference saved.
    * @param origin               the vertex the half-edge starts from. Not modified, reference saved.
    * @param destination          the vertex the half-edge ends at. Not modified, reference saved.
    */
   public FrameHalfEdge3D(ReferenceFrameHolder referenceFrameHolder, FrameVertex3D origin, FrameVertex3D destination)
   {
      super(origin, destination);
      this.referenceFrameHolder = referenceFrameHolder;
   }

   /** {@inheritDoc} */
   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return referenceFrameHolder.getReferenceFrame();
   }

   /**
    * Redirects to {@link #getOrigin()} to comply to {@code LineSegment3DBasics}.
    */
   @Override
   public FixedFramePoint3DBasics getFirstEndpoint()
   {
      return getOrigin();
   }

   /**
    * Redirects to {@link #getDestination()} to comply to {@code LineSegment3DBasics}.
    */
   @Override
   public FixedFramePoint3DBasics getSecondEndpoint()
   {
      return getDestination();
   }

   /**
    * Tests if the given {@code object}'s class is the same as this, in which case the method returns
    * {@link #equals(FrameHalfEdge3DReadOnly)}, it returns {@code false} otherwise.
    * <p>
    * If the two half-edges have different frames, this method returns {@code false}.
    * </p>
    *
    * @param object the object to compare against this. Not modified.
    * @return {@code true} if {@code object} and this are exactly equal and are expressed in the same
    *         reference frame, {@code false} otherwise.
    */
   @Override
   public boolean equals(Object object)
   {
      if (object == this)
         return true;
      else if (object instanceof FrameHalfEdge3DReadOnly)
         return FrameHalfEdge3DReadOnly.super.equals((FrameHalfEdge3DReadOnly) object);
      else
         return false;
   }

   /**
    * Provides a {@code String} representation of this half-edge 3D as follows:
    *
    * <pre>
    * Half-edge 3D: [( 2.350,  4.284,  0.427 ); ( 3.310,  6.118, -3.108 )]
    *    Twin    : [( 3.310,  6.118, -3.108 ); ( 2.350,  4.284,  0.427 )]
    *    Next    : [( 3.310,  6.118, -3.108 ); ( 3.411,  2.581, -3.144 )]
    *    Previous: [( 3.411,  2.581, -3.144 ); ( 2.350,  4.284,  0.427 )]
    *    Face: centroid: ( 3.024,  4.328, -1.941 ), normal: ( 0.961,  0.025,  0.274 )
    *    worldFrame
    * </pre>
    *
    * @return the {@code String} representing this half-edge 3D.
    */
   @Override
   public String toString()
   {
      return EuclidFrameShapeIOTools.getFrameHalfEdge3DString(this);
   }
}
