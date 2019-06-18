package us.ihmc.euclid.shape.collision.epa;

import us.ihmc.euclid.shape.convexPolytope.interfaces.HalfEdge3DReadOnly;
import us.ihmc.euclid.shape.tools.EuclidShapeIOTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;

/**
 * Edge 3D that belongs to a polytope used in the Expanding Polytope algorithm.
 * 
 * @author Sylvain Bertrand
 * @see ExpandingPolytopeAlgorithm
 */
public class EPAHalfEdge3D implements HalfEdge3DReadOnly
{
   /** The vertex this half-edge starts from. */
   private final EPAVertex3D v0;
   /** The vertex this half-edge ends at. */
   private final EPAVertex3D v1;
   /**
    * The half-edge on an adjacent face that starts from {@code destination} and ends at
    * {@code origin}.
    */
   private EPAHalfEdge3D twin;
   /**
    * The half-edge on the same face as this edge that starts from {@code destination}.
    */
   private EPAHalfEdge3D next;
   /**
    * The half-edge on the same face as this edge that ends at {@code origin}.
    */
   private EPAHalfEdge3D previous;
   /** The face that this edge is part of. */
   private final EPAFace3D face;
   /** Whether this face has been discarded and is no longer part of a polytope. */
   private boolean obsolete = false;

   /**
    * Creates a new edge and initializes its endpoints and the face it belongs to.
    * 
    * @param v0   the vertex the half-edge starts from. Not modified, reference saved.
    * @param v1   the vertex the half-edge ends at. Not modified, reference saved.
    * @param face the face the half-edge belongs to. Not modified, reference saved.
    */
   public EPAHalfEdge3D(EPAVertex3D v0, EPAVertex3D v1, EPAFace3D face)
   {
      this.face = face;
      this.v0 = v0;
      this.v1 = v1;
      v0.addAssociatedEdge(this);
   }

   /**
    * Sets the reference to the twin half-edge.
    *
    * @param twin the twin half-edge of this half-edge. Not modified, reference saved.
    * @throws IllegalArgumentException if the given half-edge is not {@code null} and that its origin
    *                                  and destination do not match this half-edge's destination and
    *                                  origin, respectively.
    */
   public void setTwin(EPAHalfEdge3D twin)
   {
      if (v0 != twin.v1 || v1 != twin.v0)
         throw new IllegalArgumentException("Twin does not match: this edge:\n" + this + "\ntwin:\n" + twin);

      this.twin = twin;

      if (twin.twin != this)
         twin.setTwin(this);
   }

   /**
    * Gets the reference to this half-edge's twin.
    * <p>
    * The twin half-edge shares the same vertices with {@code this} and its direction is flipped. The
    * faces associated to {@code this} and the twin are neighbors.
    * </p>
    * 
    * @return this twin half-edge.
    */
   @Override
   public EPAHalfEdge3D getTwin()
   {
      return twin;
   }

   /**
    * Sets the reference to the next half-edge.
    *
    * @param next the next half-edge of this half-edge. Not modified, reference saved.
    */
   public void setNext(EPAHalfEdge3D next)
   {
      this.next = next;
   }

   /**
    * Gets the reference to this half-edge's next.
    * <p>
    * The next half-edge starts from {@code this.getDestination()} and shares the same associated face.
    * </p>
    * 
    * @return this next half-edge.
    */
   @Override
   public EPAHalfEdge3D getNext()
   {
      return next;
   }

   /**
    * Sets the reference to the previous half-edge.
    *
    * @param previous the previous half-edge of this half-edge. Not modified, reference saved.
    */
   public void setPrevious(EPAHalfEdge3D previous)
   {
      this.previous = previous;
   }

   /**
    * Gets the reference to this half-edge's previous.
    * <p>
    * The previous half-edge ends to {@code this.getOrigin()} and shares the same associated face.
    * </p>
    * 
    * @return this previous half-edge.
    */
   @Override
   public EPAHalfEdge3D getPrevious()
   {
      return previous;
   }

   /**
    * Marks this edge as obsolete indicating that it is no longer part of a polytope and that it can be
    * discarded.
    */
   public void markObsolete()
   {
      obsolete = true;
   }

   /**
    * Whether this edge has been discarded and is no longer part of a polytope.
    *
    * @return {@code true} if this edge has been marked as obsolete.
    */
   public boolean isObsolete()
   {
      return obsolete;
   }

   /**
    * Gets the reference to the vertex this half-edge starts from.
    * 
    * @return the origin vertex for the half-edge.
    */
   @Override
   public EPAVertex3D getOrigin()
   {
      return v0;
   }

   /**
    * Gets the read-only reference to the vertex this half-edge ends to.
    * 
    * @return the destination vertex for the half-edge.
    */
   @Override
   public EPAVertex3D getDestination()
   {
      return v1;
   }

   /**
    * Gets the reference to the face associated to this half-edge.
    * <p>
    * This half-edge belongs to its associated face.
    * </p>
    * 
    * @return this associated face.
    */
   @Override
   public EPAFace3D getFace()
   {
      return face;
   }

   /**
    * Tests if the given {@code object}'s class is the same as this, in which case the method returns
    * {@link #equals(HalfEdge3DReadOnly)}, it returns {@code false} otherwise.
    *
    * @param object the object to compare against this. Not modified.
    * @return {@code true} if {@code object} and this are exactly equal, {@code false} otherwise.
    */
   @Override
   public boolean equals(Object object)
   {
      if (object == this)
         return true;
      else if (object instanceof HalfEdge3DReadOnly)
         return equals((HalfEdge3DReadOnly) object);
      else
         return false;
   }

   /**
    * Calculates and returns a hash code value from the value of each component of this half-edge 3D.
    *
    * @return the hash code value for this half-edge 3D.
    */
   @Override
   public int hashCode()
   {
      return EuclidHashCodeTools.toIntHashCode(EuclidHashCodeTools.combineHashCode(getOrigin().hashCode(), getDestination().hashCode()));
   }

   /**
    * Provides a {@code String} representation of this half-edge 3D as follows:
    *
    * <pre>
    * EPA Half-edge 3D: [( 2.350,  4.284,  0.427 ); ( 3.310,  6.118, -3.108 )]
    *    Twin    : [( 3.310,  6.118, -3.108 ); ( 2.350,  4.284,  0.427 )]
    *    Next    : [( 3.310,  6.118, -3.108 ); ( 3.411,  2.581, -3.144 )]
    *    Previous: [( 3.411,  2.581, -3.144 ); ( 2.350,  4.284,  0.427 )]
    *    Face: centroid: ( 3.024,  4.328, -1.941 ), normal: ( 0.961,  0.025,  0.274 )
    * </pre>
    * 
    * @return the {@code String} representing this half-edge 3D.
    */
   @Override
   public String toString()
   {
      return "EPA " + EuclidShapeIOTools.getHalfEdge3DString(this);
   }
}