package us.ihmc.euclid.referenceFrame.polytope;

import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameLineSegment3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.ReferenceFrameHolder;
import us.ihmc.euclid.referenceFrame.polytope.interfaces.FrameHalfEdge3DReadOnly;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameShapeIOTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;

public class FrameHalfEdge3D implements FrameHalfEdge3DReadOnly, FixedFrameLineSegment3DBasics
{
   private ReferenceFrameHolder referenceFrameHolder;
   /** The vertex this half-edge starts from. */
   private FrameVertex3D origin;
   /** The vertex this half-edge ends at. */
   private FrameVertex3D destination;
   /**
    * The half-edge on an adjacent face that starts from {@code destination} and ends at
    * {@code origin}.
    */
   private FrameHalfEdge3D twin;
   /**
    * The half-edge on the same face as this edge that starts from {@code destination}.
    */
   private FrameHalfEdge3D next;
   /**
    * The half-edge on the same face as this edge that ends at {@code origin}.
    */
   private FrameHalfEdge3D previous;
   /** The face that this edge is part of. */
   private FrameFace3D face;

   /**
    * Creates a new half-edge and initializes its origin and destination.
    *
    * @param origin      the vertex the half-edge starts from. Not modified, reference saved.
    * @param destination the vertex the half-edge ends at. Not modified, reference saved.
    */
   public FrameHalfEdge3D(ReferenceFrameHolder referenceFrameHolder, FrameVertex3D origin, FrameVertex3D destination)
   {
      this.referenceFrameHolder = referenceFrameHolder;
      setOrigin(origin);
      setDestination(destination);
   }

   /**
    * Sets the reference of this half-edge's origin.
    * <p>
    * This method also updates the old origin by removing this half-edge of its associated edges, and
    * associating it to the new origin.
    * </p>
    *
    * @param origin the new origin for this half-edge. Associated edges modified, reference saved.
    */
   public void setOrigin(FrameVertex3D origin)
   {
      if (this.origin != null)
         this.origin.removeAssociatedEdge(this);
      this.origin = origin;
      if (this.origin != null)
         this.origin.addAssociatedEdge(this);
   }

   /**
    * Gets the reference to the vertex this half-edge starts from.
    *
    * @return the origin vertex for the half-edge.
    */
   @Override
   public FrameVertex3D getOrigin()
   {
      return origin;
   }

   /**
    * Sets the reference of this half-edge's destination.
    *
    * @param destination the new destination for this half-edge. Not modified, reference saved.
    */
   public void setDestination(FrameVertex3D destination)
   {
      this.destination = destination;
   }

   /**
    * Gets the read-only reference to the vertex this half-edge ends to.
    *
    * @return the destination vertex for the half-edge.
    */
   @Override
   public FrameVertex3D getDestination()
   {
      return destination;
   }

   @Override
   public ReferenceFrame getReferenceFrame()
   {
      return referenceFrameHolder.getReferenceFrame();
   }

   /**
    * Sets the reference to the twin half-edge.
    *
    * @param twin the twin half-edge of this half-edge. Not modified, reference saved.
    * @throws IllegalArgumentException if the given half-edge is not {@code null} and that its origin
    *                                  and destination do not match this half-edge's destination and
    *                                  origin, respectively.
    */
   public void setTwin(FrameHalfEdge3D twin)
   {
      if (twin != null && (twin.getDestination() != origin || twin.getOrigin() != destination))
         throw new IllegalArgumentException("Twin does not match: this edge:\n" + this + "\ntwin:\n" + twin);
      this.twin = twin;
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
   public FrameHalfEdge3D getTwin()
   {
      return twin;
   }

   /**
    * Sets the reference to the next half-edge.
    *
    * @param next the next half-edge of this half-edge. Not modified, reference saved.
    * @throws IllegalArgumentException if the given half-edge is not {@code null} and that either
    *                                  {@code next.getOrigin() != this.destination} or
    *                                  {@code next.getFace() != this.face}.
    */
   public void setNext(FrameHalfEdge3D next)
   {
      if (next == null || next.getOrigin() == getDestination() && next.getFace() == getFace())
         this.next = next;
      else
         throw new IllegalArgumentException("Mismatch between vertices, destination vertex: " + getDestination().toString() + " , next origin: "
               + next.getOrigin().toString());
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
   public FrameHalfEdge3D getNext()
   {
      return next;
   }

   /**
    * Sets the reference to the previous half-edge.
    *
    * @param previous the previous half-edge of this half-edge. Not modified, reference saved.
    * @throws IllegalArgumentException if the given half-edge is not {@code null} and that either
    *                                  {@code previous.getDestination() != this.origin} or
    *                                  {@code previous.getFace() != this.face}.
    */
   public void setPrevious(FrameHalfEdge3D previous)
   {
      if (previous == null || previous.getDestination() == getOrigin() && previous.getFace() == getFace())
         this.previous = previous;
      else
         throw new IllegalArgumentException("Mismatch between vertices, origin vertex: " + getOrigin().toString() + " , previous destination: "
               + previous.getDestination().toString());
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
   public FrameHalfEdge3D getPrevious()
   {
      return previous;
   }

   /**
    * Sets the reference to the associated face.
    *
    * @param face the face this half-edge belongs to. Not modified, reference saved.
    */
   public void setFace(FrameFace3D face)
   {
      this.face = face;
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
   public FrameFace3D getFace()
   {
      return face;
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
    * Sets all the references that are held by this half-edge to null and also updates the previously
    * associated objects
    */
   public void destroy()
   {
      if (twin != null && twin.getTwin() == this)
         twin.setTwin(null);
      if (next != null && next.getPrevious() == this)
         next.setPrevious(null);
      if (previous != null && previous.getNext() == this)
         previous.setNext(null);

      setTwin(null);
      setNext(null);
      setPrevious(null);
      setFace(null);
      setOrigin(null);
      setDestination(null);
   }

   /**
    * Changes the direction of this edge so that it starts at the previous {@code destinationVertex}
    * and ends at the {@code originVertex} The references to the {@code nextHalfEdge} and
    * {@code previousHalfEdge} are also updated as its the twin edge. Reference to the {@code face} is
    * not changed
    */
   public void flip()
   {
      FrameVertex3D newDestination = origin;
      setOrigin(destination);
      setDestination(newDestination);
      FrameHalfEdge3D newNext = previous;
      previous = next;
      next = newNext;
   }

   /**
    * Tests if the given {@code object}'s class is the same as this, in which case the method returns
    * {@link #equals(FrameHalfEdge3DReadOnly)}, it returns {@code false} otherwise.
    *
    * @param object the object to compare against this. Not modified.
    * @return {@code true} if {@code object} and this are exactly equal, {@code false} otherwise.
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
    * Calculates and returns a hash code value from the value of each component of this half-edge 3D.
    *
    * @return the hash code value for this half-edge 3D.
    */
   @Override
   public int hashCode()
   {
      return EuclidHashCodeTools.toIntHashCode(getOrigin(), getDestination());
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
