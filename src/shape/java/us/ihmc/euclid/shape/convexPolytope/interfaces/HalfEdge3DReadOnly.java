package us.ihmc.euclid.shape.convexPolytope.interfaces;

import us.ihmc.euclid.geometry.interfaces.LineSegment3DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

/**
 * Read-only interface for a half-edge 3D that belongs to a convex polytope 3D.
 * <p>
 * This is part of a Doubly Connected Edge List data structure
 * <a href="https://en.wikipedia.org/wiki/Doubly_connected_edge_list"> link</a>.
 * </p>
 * 
 * @author Sylvain Bertrand
 */
public interface HalfEdge3DReadOnly extends LineSegment3DReadOnly
{
   /**
    * Gets the read-only reference to the vertex this half-edge starts from.
    * 
    * @return the origin vertex for the half-edge.
    */
   Vertex3DReadOnly getOrigin();

   /**
    * Gets the read-only reference to the vertex this half-edge ends to.
    * 
    * @return the destination vertex for the half-edge.
    */
   Vertex3DReadOnly getDestination();

   /**
    * Redirects to {@link #getOrigin()} to comply to {@code LineSegment3DReadOnly}.
    */
   @Override
   default Point3DReadOnly getFirstEndpoint()
   {
      return getOrigin();
   }

   /**
    * Redirects to {@link #getDestination()} to comply to {@code LineSegment3DReadOnly}.
    */
   @Override
   default Point3DReadOnly getSecondEndpoint()
   {
      return getDestination();
   }

   /**
    * Gets the read-only reference to this half-edge's twin.
    * <p>
    * The twin half-edge shares the same vertices with {@code this} and its direction is flipped. The
    * faces associated to {@code this} and the twin are neighbors.
    * </p>
    * 
    * @return this twin half-edge.
    */
   HalfEdge3DReadOnly getTwin();

   /**
    * Gets the read-only reference to this half-edge's next.
    * <p>
    * The next half-edge starts from {@code this.getDestination()} and shares the same associated face.
    * </p>
    * 
    * @return this next half-edge.
    */
   HalfEdge3DReadOnly getNext();

   /**
    * Gets the read-only reference to this half-edge's previous.
    * <p>
    * The previous half-edge ends to {@code this.getOrigin()} and shares the same associated face.
    * </p>
    * 
    * @return this previous half-edge.
    */
   HalfEdge3DReadOnly getPrevious();

   /**
    * Gets the read-only reference to the face associated to this half-edge.
    * <p>
    * This half-edge belongs to its associated face.
    * </p>
    * 
    * @return this associated face.
    */
   Face3DReadOnly getFace();

   /**
    * Computes the minimum distance between a given point and the infinitely long line supporting this
    * half-edge.
    * 
    * @param point the location of the query. Not modified.
    * @return the distance from the query to the support line.
    */
   default double distanceFromSupportLine(Point3DReadOnly point)
   {
      return EuclidGeometryTools.distanceFromPoint3DToLine3D(point, getOrigin(), getDestination());
   }

   /**
    * Tests on a per component basis, if this half-edge 3D is exactly equal to {@code other}.
    *
    * @param other the other half-edge 3D to compare against this. Not modified.
    * @return {@code true} if the two half-edges are exactly equal component-wise, {@code false}
    *         otherwise.
    */
   default boolean equals(HalfEdge3DReadOnly other)
   {
      if (other == this)
         return true;
      else if (other == null)
         return false;
      else if ((getOrigin() == null) != (other.getOrigin() == null))
         return false;
      else if ((getDestination() == null) != (other.getDestination() == null))
         return false;
      else
         return LineSegment3DReadOnly.super.equals(other);
   }
}