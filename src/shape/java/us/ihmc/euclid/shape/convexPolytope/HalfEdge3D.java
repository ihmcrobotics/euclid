package us.ihmc.euclid.shape.convexPolytope;

import us.ihmc.euclid.geometry.interfaces.LineSegment3DBasics;
import us.ihmc.euclid.shape.convexPolytope.interfaces.HalfEdge3DReadOnly;
import us.ihmc.euclid.shape.tools.EuclidShapeIOTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;

/**
 * Implementation of a half-edge 3D that belongs to a convex polytope 3D.
 * <p>
 * This is part of a Doubly Connected Edge List data structure
 * <a href="https://en.wikipedia.org/wiki/Doubly_connected_edge_list"> link</a>.
 * </p>
 * 
 * @author Apoorv Shrivastava
 * @author Sylvain Bertrand
 */
public class HalfEdge3D implements HalfEdge3DReadOnly, LineSegment3DBasics
{
   /** The vertex this half-edge starts from. */
   private Vertex3D origin;
   /** The vertex this half-edge ends at. */
   private Vertex3D destination;
   /**
    * The half-edge on an adjacent face that starts from {@code destination} and ends at
    * {@code origin}.
    */
   private HalfEdge3D twin;
   /**
    * The half-edge on the same face as this edge that starts from {@code destination}.
    */
   private HalfEdge3D next;
   /**
    * The half-edge on the same face as this edge that ends at {@code origin}.
    */
   private HalfEdge3D previous;
   /** The face that this edge is part of. */
   private Face3D face;

   /**
    * Creates a new half-edge and initializes its origin and destination.
    *
    * @param origin the vertex the half-edge starts from. Not modified, reference saved.
    * @param destination the vertex the half-edge ends at. Not modified, reference saved.
    */
   public HalfEdge3D(Vertex3D origin, Vertex3D destination)
   {
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
   public void setOrigin(Vertex3D origin)
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
   public Vertex3D getOrigin()
   {
      return origin;
   }

   /**
    * Sets the reference of this half-edge's destination.
    *
    * @param destination the new destination for this half-edge. Not modified, reference saved.
    */
   public void setDestination(Vertex3D destination)
   {
      this.destination = destination;
   }

   /**
    * Gets the read-only reference to the vertex this half-edge ends to.
    * 
    * @return the destination vertex for the half-edge.
    */
   @Override
   public Vertex3D getDestination()
   {
      return destination;
   }

   /**
    * Sets the reference to the twin half-edge.
    *
    * @param twin the twin half-edge of this half-edge. Not modified, reference saved.
    * @throws IllegalArgumentException if the given half-edge is not {@code null} and that its origin
    *            and destination do not match this half-edge's destination and origin, respectively.
    */
   public void setTwin(HalfEdge3D twin)
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
   public HalfEdge3D getTwin()
   {
      return twin;
   }

   /**
    * Sets the reference to the next half-edge.
    *
    * @param next the next half-edge of this half-edge. Not modified, reference saved.
    * @throws IllegalArgumentException if the given half-edge is not {@code null} and that either
    *            {@code next.getOrigin() != this.destination} or {@code next.getFace() != this.face}.
    */
   public void setNext(HalfEdge3D next)
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
   public HalfEdge3D getNext()
   {
      return next;
   }

   /**
    * Sets the reference to the previous half-edge.
    *
    * @param previous the previous half-edge of this half-edge. Not modified, reference saved.
    * @throws IllegalArgumentException if the given half-edge is not {@code null} and that either
    *            {@code previous.getDestination() != this.origin} or
    *            {@code previous.getFace() != this.face}.
    */
   public void setPrevious(HalfEdge3D previous)
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
   public HalfEdge3D getPrevious()
   {
      return previous;
   }

   /**
    * Sets the reference to the associated face.
    *
    * @param face the face this half-edge belongs to. Not modified, reference saved.
    */
   public void setFace(Face3D face)
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
   public Face3D getFace()
   {
      return face;
   }

   @Override
   public Point3DBasics getFirstEndpoint()
   {
      return getOrigin();
   }

   @Override
   public Point3DBasics getSecondEndpoint()
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
      Vertex3D newDestination = origin;
      setOrigin(destination);
      setDestination(newDestination);
      HalfEdge3D newNext = previous;
      this.previous = next;
      this.next = newNext;
   }

   @Override
   public boolean equals(Object object)
   {
      if (object == this)
         return true;
      else if (object instanceof HalfEdge3DReadOnly)
         return HalfEdge3DReadOnly.super.equals((HalfEdge3DReadOnly) object);
      else
         return false;
   }

   @Override
   public int hashCode()
   {
      return EuclidHashCodeTools.toIntHashCode(EuclidHashCodeTools.combineHashCode(getOrigin().hashCode(), getDestination().hashCode()));
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public String toString()
   {
      return EuclidShapeIOTools.getHalfEdge3DString(this);
   }
}
