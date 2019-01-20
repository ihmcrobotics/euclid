package us.ihmc.euclid.shape.convexPolytope;

import us.ihmc.euclid.geometry.interfaces.LineSegment3DBasics;
import us.ihmc.euclid.shape.convexPolytope.interfaces.HalfEdge3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.tools.EuclidPolytopeIOTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
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
   private HalfEdge3D twinEdge;
   /**
    * The half edge on the same face as this edge that originates at the {@code destinationVertex}
    */
   private HalfEdge3D nextEdge;
   /**
    * The half edge on the same face as this edge that terminates at the {@code originVertex}
    */
   private HalfEdge3D previousEdge;
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

   /**
    * Creates a half edge using the {@code createTwinHalfEdge{} function and stores a reference to the
    * new object in the twin edge field @return a twin edge
    */
   public HalfEdge3D setAndCreateTwinHalfEdge()
   {
      HalfEdge3D twinEdge = createTwinEdge();
      setTwinEdge(twinEdge);
      return twinEdge;
   }

   /**
    * Creates a half edge from a twin edge and the face that the new half edge is to be a part of
    *
    * @param twinEdge the edge that is to be the twin of the new half edgegetShortestDistanceTo
    * @param face the face that the new half edge is to be a part of
    */
   public HalfEdge3D(HalfEdge3D twinEdge, Face3D face)
   {
      setTwinEdge(twinEdge);
      setOrigin(twinEdge.getDestination());
      setDestination(twinEdge.getOrigin());
      setFace(face);
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
   public HalfEdge3D(Vertex3D origin, Vertex3D destination, HalfEdge3D twinEdge, HalfEdge3D nextEdge, HalfEdge3D previousEdge, Face3D face)
   {
      setOrigin(origin);
      setDestination(destination);
      setTwinEdge(twinEdge);
      setNextEdge(nextEdge);
      setPreviousEdge(previousEdge);
      setFace(face);
   }

   /**
    * Takes a edge, clears all its fields and assigns it all the values that a twin edge for this half
    * edge would have i.e. {@code originVertex}, {@code destinationVertex}, {@code twinEdge = this}
    *
    * @param twinEdge
    */
   public void setToTwin(HalfEdge3D twinEdge)
   {
      twinEdge.detroy();
      twinEdge.setOrigin(destination);
      twinEdge.setDestination(origin);
      twinEdge.setTwinEdge(this);
   }

   /**
    *
    * @return a twin edge that can be used to generate a adjacent face. The twin edge generated stores
    *         references to the {@code originVertex} and {@code destinationVertex}. Half edge generated
    *         stores this edge as its twin but this half edge does not store the generated half edge as
    *         its twin
    */
   public HalfEdge3D createTwinEdge()
   {
      HalfEdge3D twinEdge = new HalfEdge3D(getDestination(), getOrigin());
      twinEdge.setTwinEdge(this);
      return twinEdge;
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
      setOriginUnsafe(origin);
      if (this.origin != null)
         this.origin.addAssociatedEdge(this);
      updateTwinDestination();
   }

   /**
    * Update the reference to the {@code originVertex} to the specified value. Associations are not
    * updated
    *
    * @param origin the new vertex that the half edge originates at. Can be null. Is not modified in
    *           this function
    */
   public void setOriginUnsafe(Vertex3D origin)
   {
      this.origin = origin;
   }

   /**
    * Internal function to update the origin of the twin edge only if the twin is not null. Is needed
    * since the public versions that can set the origin would lead to a cyclic non-terminating call.
    * Also updates the requisite references.
    */
   private void updateTwinOrigin()
   {
      if (twinEdge != null)
      {
         if (twinEdge.getOrigin() != null)
            twinEdge.getOrigin().removeAssociatedEdge(twinEdge);
         twinEdge.setOriginUnsafe(destination);
         if (twinEdge.getOrigin() != null)
            twinEdge.getOrigin().addAssociatedEdge(twinEdge);
      }
   }

   /**
    * Internal function to update the destination vertex of the twin edge only if the twin is not null.
    * Is needed since the public versions that can set the destination would lead to a cyclic
    * non-terminating call.
    */
   private void updateTwinDestination()
   {
      if (twinEdge != null)
         twinEdge.setDestinationUnsafe(origin);
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
      updateTwinOrigin();
   }

   /**
    * Update the reference to the {@code destinationVertex} to the specified value. Associations of the
    * specified vertex and of this object are not updated
    *
    * @param destination the new vertex that the half edge originates at. Can be null. Is not modified
    *           in this function
    * @param destination
    */
   public void setDestinationUnsafe(Vertex3D destination)
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
    * Store a reference to the specified half edge as a twin of this half edge. No checks are performed
    * while updating the twin edge.
    *
    * @param twinEdge the half edge to be stored as a twin edge of this half edge.
    */
   public void setTwinEdge(HalfEdge3D twinEdge)
   {
      this.twinEdge = twinEdge;
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public HalfEdge3D getTwinEdge()
   {
      return twinEdge;
   }

   /**
    * Update the reference to the {@code nextHalfEdge}. Checks to ensure that the origin of the
    * specified edge and destination of the this half edge are the same
    *
    * @param nextEdge the new next half edge for the current half edge. Can be null
    * @throws RuntimeException in case the origin of this specified next half edge is not the same as
    *            the destination of the this edge
    */
   public void setNextEdge(HalfEdge3D nextEdge)
   {
      if (nextEdge == null || nextEdge.getOrigin() == getDestination() && nextEdge.getFace() == getFace())
         setNextEdgeUnsafe(nextEdge);
      else
         throw new RuntimeException("Mismatch between vertices, destination vertex: " + getDestination().toString() + " , next edge origin vertex: "
               + nextEdge.getOrigin().toString());
   }

   /**
    * Internal method to update the next half edge without any checks
    *
    * @param nextHalfEdge the half edge whose reference is to be stored in the next half edge field
    */
   private void setNextEdgeUnsafe(HalfEdge3D nextHalfEdge)
   {
      this.nextEdge = nextHalfEdge;
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public HalfEdge3D getNextEdge()
   {
      return nextEdge;
   }

   /**
    * Update the reference to the {@code previous HalfEdge}. Checks to ensure that the destination of
    * the specified edge and origin of the this half edge are the same
    *
    * @param previousEdge the new previous half edge for the current half edge. Can be null
    * @throws RuntimeException in case the destination of this specified next half edge is not the same
    *            as the origin of the this edge
    */
   public void setPreviousEdge(HalfEdge3D previousEdge)
   {
      if (previousEdge == null || previousEdge.getDestination() == getOrigin() && previousEdge.getFace() == getFace())
         setPreviousEdgeUnsafe(previousEdge);
      else
         throw new RuntimeException("Mismatch between vertices, origin vertex: " + getOrigin().toString() + " , previous edge destination vertex: "
               + previousEdge.getDestination().toString());
   }

   /**
    * Internal method to update the next half edge without any checks
    *
    * @param previousEdge the half edge whose reference is to be stored in the previous half edge field
    */
   private void setPreviousEdgeUnsafe(HalfEdge3D previousEdge)
   {
      this.previousEdge = previousEdge;
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public HalfEdge3D getPreviousEdge()
   {
      return previousEdge;
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

   /**
    * Sets all the references that are held by this half edge to null and also updates the previously
    * associated objects
    */
   public void detroy()
   {
      if (twinEdge != null)
         twinEdge.setTwinEdge(null);
      if (nextEdge != null)
         nextEdge.setPreviousEdge(null);
      if (previousEdge != null)
         previousEdge.setNextEdge(null);

      setTwinEdge(null);
      setNextEdge(null);
      setPreviousEdge(null);
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
      Vertex3D newDestinationVertex = origin;
      setOrigin(destination);
      setDestination(newDestinationVertex);
      HalfEdge3D newNextHalfEdge = previousEdge;
      setPreviousEdgeUnsafe(nextEdge);
      setNextEdgeUnsafe(newNextHalfEdge);
   }

   @Override
   public boolean equals(Object object)
   {
      if (object instanceof HalfEdge3DReadOnly)
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
