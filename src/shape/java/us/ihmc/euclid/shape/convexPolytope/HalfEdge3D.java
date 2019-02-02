package us.ihmc.euclid.shape.convexPolytope;

import us.ihmc.euclid.geometry.interfaces.LineSegment3DBasics;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.shape.convexPolytope.interfaces.HalfEdge3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.tools.EuclidPolytopeIOTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

/**
 * A template that defines the basic structure of a DCEL half edge. A half edge is composed of
 * <li>{@code originVertex} starting point reference for this directed edge
 * <li>{@code destinatioVertex} ending point reference for this directed edge
 * <li>{@code twinHalfEdge} reference to the twin half edge on an adjacent face, if defined
 * <li>{@code nextHalfEdge} reference to the half edge on {@code face} that succeeds this edge in a
 * counter clockwise sense
 * <li>{@code previousHalfEdge} reference to the half edge on {@code face} that precedes this edge
 * in a counter clockwise sense
 * <li>{@code face} the face that this half edge is a part of
 *
 * @author Apoorv S
 *
 * @param <V> Data structure representing a point in 3D space
 * @param <E> A class that extends this data structure Represents an directed edge formed by joining
 *           two vertices
 * @param <F> A collection of edges that constitute a face of the polytope
 */
public class HalfEdge3D implements HalfEdge3DReadOnly, LineSegment3DBasics
{
   /**
    * Specifies the spatial location at which the half edge originates
    */
   private Vertex3D origin;
   /**
    * Specifies the spatial location at which the half edge terminates
    */
   private Vertex3D destination;
   /**
    * The half edge on an adjacent face that originates at the {@code destinatioVertex} and terminates
    * at the {@code originVertex}. Represents the opposite spatial direction
    */
   private HalfEdge3D twin;
   /**
    * The half edge on the same face as this edge that originates at the {@code destinationVertex}
    */
   private HalfEdge3D next;
   /**
    * The half edge on the same face as this edge that terminates at the {@code originVertex}
    */
   private HalfEdge3D previous;
   /**
    * The face that this edge is a part of
    */
   private Face3D face;
   /**
    * A vector that represents the direction and lenght of the half edge. Not recomputed on change of
    * values. Only recomputed when called through its getter
    */
   private Vector3D edgeVector = new Vector3D();

   public HalfEdge3D()
   {
      super();
   }

   /**
    * Primary constructor for half edge
    *
    * @param origin
    * @param destination
    */
   public HalfEdge3D(Vertex3D origin, Vertex3D destination)
   {
      setOrigin(origin);
      setDestination(destination);
   }

   public HalfEdge3D(Vertex3D origin, Vertex3D destination, HalfEdge3D previous, HalfEdge3D next, Face3D face)
   {
      this(origin, destination);
      setFace(face);
      setPrevious(previous);
      setNext(next);
   }

   /**
    * Creates a half edge using all specified values
    *
    * @param origin the vertex that the new half edge will start at. Stored as a reference. Can be
    *           {@code null}
    * @param destination the vertex that the new half edge will end at. Stored as a reference. Can be
    *           {@code null}
    * @param twinEdge the half edge that is the DCEL twin of the new edge . Stored as a reference. Can
    *           be {@code null}
    * @param nextEdge the half edge that is originates at the destination vertex and comes after the
    *           current edge when the face is traversed in a counter clockwise manner w.r.t. its face
    *           normal. Can be {@code null}
    * @param previousEdge the half edge that is terminates at the origin vertex and comes before the
    *           current edge when the face is traversed in a counter clockwise manner w.r.t. its face
    *           normal. Can be {@code null}
    * @param face the face that this half edge is a part of. Can be {@code null}
    */
   public HalfEdge3D(Vertex3D origin, Vertex3D destination, HalfEdge3D twin, HalfEdge3D next, HalfEdge3D previous, Face3D face)
   {
      this(origin, destination, previous, next, face);
      setTwin(twin);
   }

   /**
    * Update the reference to the {@code originVertex} field to the specified value. Also updates the
    * associated edges of the previously held and newly specified {@code originVertex} and the
    * {@code twinEdge} of this edge
    *
    * @param origin the new vertex that the half edge originates at. Can be null. Is modified
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
    * Returns a reference to the origin vertex for this half edge
    */
   @Override
   public Vertex3D getOrigin()
   {
      return origin;
   }

   /**
    * Update the reference to the {@code destinationVertex} to the specified value. Also updates the
    * associated twin edge
    *
    * @param destination the new vertex that the half edge originates at. Can be null. Is not modified
    *           in this function
    */
   public void setDestination(Vertex3D destination)
   {
      this.destination = destination;
   }

   /**
    * Returns a reference to the {@code destinationVertex} of this half edge
    */
   @Override
   public Vertex3D getDestination()
   {
      return destination;
   }

   /**
    * Store a reference to the specified half edge as a twin of this half edge.
    *
    * @param twinEdge the half edge to be stored as a twin edge of this half edge.
    */
   public void setTwin(HalfEdge3D twin)
   {
      if (twin != null && (twin.getDestination() != origin || twin.getOrigin() != destination))
         throw new IllegalArgumentException("Twin does not match: this edge:\n" + this + "\ntwin:\n" + twin);
      this.twin = twin;
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public HalfEdge3D getTwin()
   {
      return twin;
   }

   /**
    * Update the reference to the {@code nextHalfEdge}. Checks to ensure that the origin of the
    * specified edge and destination of the this half edge are the same
    *
    * @param nextEdge the new next half edge for the current half edge. Can be null
    * @throws RuntimeException in case the origin of this specified next half edge is not the same as
    *            the destination of the this edge
    */
   public void setNext(HalfEdge3D next)
   {
      if (next == null || next.getOrigin() == getDestination() && next.getFace() == getFace())
         this.next = next;
      else
         throw new RuntimeException("Mismatch between vertices, destination vertex: " + getDestination().toString() + " , next origin: "
               + next.getOrigin().toString());
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public HalfEdge3D getNext()
   {
      return next;
   }

   /**
    * Update the reference to the {@code previous HalfEdge}. Checks to ensure that the destination of
    * the specified edge and origin of the this half edge are the same
    *
    * @param previousEdge the new previous half edge for the current half edge. Can be null
    * @throws RuntimeException in case the destination of this specified next half edge is not the same
    *            as the origin of the this edge
    */
   public void setPrevious(HalfEdge3D previous)
   {
      if (previous == null || previous.getDestination() == getOrigin() && previous.getFace() == getFace())
         this.previous = previous;
      else
         throw new RuntimeException("Mismatch between vertices, origin vertex: " + getOrigin().toString() + " , previous destination: "
               + previous.getDestination().toString());
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public HalfEdge3D getPrevious()
   {
      return previous;
   }

   /**
    * Update the reference to the face that this half edge is a part of
    *
    * @param face the face reference to be stored. Can be null
    */
   public void setFace(Face3D face)
   {
      this.face = face;
   }

   /**
    * {@inheritDoc}
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
    * {@inheritDoc}
    */
   @Override
   public Vector3DReadOnly getEdgeVector()
   {
      edgeVector.sub(destination, origin);
      return edgeVector;
   }

   public double distanceFromSupportLine(Point3DReadOnly point)
   {
      return EuclidGeometryTools.distanceFromPoint3DToLine3D(point, origin, destination);
   }

   /**
    * Sets all the references that are held by this half edge to null and also updates the previously
    * associated objects
    */
   public void detroy()
   {
      if (twin != null)
         twin.setTwin(null);
      if (next != null)
         next.setPrevious(null);
      if (previous != null)
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
      return EuclidPolytopeIOTools.getHalfEdge3DString(this);
   }
}
