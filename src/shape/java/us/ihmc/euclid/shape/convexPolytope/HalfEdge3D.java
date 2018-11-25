package us.ihmc.euclid.shape.convexPolytope;

import us.ihmc.euclid.geometry.interfaces.LineSegment3DBasics;
import us.ihmc.euclid.shape.convexPolytope.interfaces.HalfEdge3DReadOnly;
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
   private Vertex3D originVertex;
   /**
    * Specifies the spatial location at which the half edge terminates
    */
   private Vertex3D destinationVertex;
   /**
    * The half edge on an adjacent face that originates at the {@code destinatioVertex} and terminates
    * at the {@code originVertex}. Represents the opposite spatial direction
    */
   private HalfEdge3D twinEdge;
   /**
    * The half edge on the same face as this edge that originates at the {@code destinationVertex}
    */
   private HalfEdge3D nextHalfEdge;
   /**
    * The half edge on the same face as this edge that terminates at the {@code originVertex}
    */
   private HalfEdge3D previousHalfEdge;
   /**
    * The face that this edge is a part of
    */
   private Face3DBasics face;
   /**
    * A vector that represents the direction and lenght of the half edge. Not recomputed on change of
    * values. Only recomputed when called through its getter
    */
   private Vector3D edgeVector = new Vector3D();

   private final PolytopeHalfEdgeBuilder halfEdgeBuilder = new PolytopeHalfEdgeBuilder();

   public HalfEdge3D()
   {
      super();
   }

   /**
    * Primary constructor for half edge
    *
    * @param originVertex
    * @param destinationVertex
    */
   public HalfEdge3D(Vertex3D originVertex, Vertex3D destinationVertex)
   {
      setOriginVertex(originVertex);
      setDestinationVertex(destinationVertex);
   }

   /**
    * Creates a half edge using the {@code createTwinHalfEdge{} function and stores a reference to the
    * new object in the twin edge field @return a twin edge
    */
   public HalfEdge3D setAndCreateTwinHalfEdge()
   {
      HalfEdge3D twinEdge = createTwinHalfEdge();
      setTwinHalfEdge(twinEdge);
      return twinEdge;
   }

   /**
    * Creates a half edge from a twin edge and the face that the new half edge is to be a part of
    *
    * @param twinEdge the edge that is to be the twin of the new half edgegetShortestDistanceTo
    * @param face the face that the new half edge is to be a part of
    */
   public HalfEdge3D(HalfEdge3D twinEdge, Face3DBasics face)
   {
      setTwinHalfEdge(twinEdge);
      setOriginVertex(twinEdge.getDestinationVertex());
      setDestinationVertex(twinEdge.getOriginVertex());
      setFace(face);
   }

   /**
    * Creates a half edge using all specified values
    *
    * @param originVertex the vertex that the new half edge will start at. Stored as a reference. Can
    *           be {@code null}
    * @param destinationVertex the vertex that the new half edge will end at. Stored as a reference.
    *           Can be {@code null}
    * @param twinEdge the half edge that is the DCEL twin of the new edge . Stored as a reference. Can
    *           be {@code null}
    * @param nextHalfEdge the half edge that is originates at the destination vertex and comes after
    *           the current edge when the face is traversed in a counter clockwise manner w.r.t. its
    *           face normal. Can be {@code null}
    * @param previousHalfEdge the half edge that is terminates at the origin vertex and comes before
    *           the current edge when the face is traversed in a counter clockwise manner w.r.t. its
    *           face normal. Can be {@code null}
    * @param face the face that this half edge is a part of. Can be {@code null}
    */
   public HalfEdge3D(Vertex3D originVertex, Vertex3D destinationVertex, HalfEdge3D twinEdge, HalfEdge3D nextHalfEdge,
                           HalfEdge3D previousHalfEdge, Face3DBasics face)
   {
      setOriginVertex(originVertex);
      setDestinationVertex(destinationVertex);
      setTwinHalfEdge(twinEdge);
      setNextHalfEdge(nextHalfEdge);
      setPreviousHalfEdge(previousHalfEdge);
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
      twinEdge.clear();
      twinEdge.setOriginVertex(destinationVertex);
      twinEdge.setDestinationVertex(originVertex);
      twinEdge.setTwinHalfEdge(this);
   }

   /**
    *
    * @return a twin edge that can be used to generate a adjacent face. The twin edge generated stores
    *         references to the {@code originVertex} and {@code destinationVertex}. Half edge generated
    *         stores this edge as its twin but this half edge does not store the generated half edge as
    *         its twin
    */
   public HalfEdge3D createTwinHalfEdge()
   {
      HalfEdge3D twinEdge = halfEdgeBuilder.getHalfEdge(getDestinationVertex(), getOriginVertex());
      twinEdge.setTwinHalfEdge(this);
      return twinEdge;
   }

   /**
    * Update the reference to the {@code originVertex} field to the specified value. Also updates the
    * associated edges of the previously held and newly specified {@code originVertex} and the
    * {@code twinEdge} of this edge
    *
    * @param originVertex the new vertex that the half edge originates at. Can be null. Is modified
    */
   public void setOriginVertex(Vertex3D originVertex)
   {
      if (this.originVertex != null)
         this.originVertex.removeAssociatedEdge(this);
      setOriginVertexUnsafe(originVertex);
      if (this.originVertex != null)
         this.originVertex.addAssociatedEdge(this);
      updateTwinDestination();
   }

   /**
    * Update the reference to the {@code originVertex} to the specified value. Associations are not
    * updated
    *
    * @param originVertex the new vertex that the half edge originates at. Can be null. Is not modified
    *           in this function
    */
   public void setOriginVertexUnsafe(Vertex3D originVertex)
   {
      this.originVertex = originVertex;
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
         if (twinEdge.getOriginVertex() != null)
            twinEdge.getOriginVertex().removeAssociatedEdge(twinEdge);
         twinEdge.setOriginVertexUnsafe(destinationVertex);
         if (twinEdge.getOriginVertex() != null)
            twinEdge.getOriginVertex().addAssociatedEdge(twinEdge);
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
         twinEdge.setDestinationVertexUnsafe(originVertex);
   }

   /**
    * Returns a reference to the origin vertex for this half edge
    */
   @Override
   public Vertex3D getOriginVertex()
   {
      return originVertex;
   }

   /**
    * Update the reference to the {@code destinationVertex} to the specified value. Also updates the
    * associated twin edge
    *
    * @param destinationVertex the new vertex that the half edge originates at. Can be null. Is not
    *           modified in this function
    */
   public void setDestinationVertex(Vertex3D destinationVertex)
   {
      this.destinationVertex = destinationVertex;
      updateTwinOrigin();
   }

   /**
    * Update the reference to the {@code destinationVertex} to the specified value. Associations of the
    * specified vertex and of this object are not updated
    *
    * @param destinationVertex the new vertex that the half edge originates at. Can be null. Is not
    *           modified in this function
    * @param destinationVertex
    */
   public void setDestinationVertexUnsafe(Vertex3D destinationVertex)
   {
      this.destinationVertex = destinationVertex;
   }

   /**
    * Returns a reference to the {@code destinationVertex} of this half edge
    */
   @Override
   public Vertex3D getDestinationVertex()
   {
      return destinationVertex;
   }

   /**
    * Store a reference to the specified half edge as a twin of this half edge. No checks are performed
    * while updating the twin edge.
    *
    * @param twinEdge the half edge to be stored as a twin edge of this half edge.
    */
   public void setTwinHalfEdge(HalfEdge3D twinEdge)
   {
      this.twinEdge = twinEdge;
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public HalfEdge3D getTwinHalfEdge()
   {
      return twinEdge;
   }

   /**
    * Update the reference to the {@code nextHalfEdge}. Checks to ensure that the origin of the
    * specified edge and destination of the this half edge are the same
    *
    * @param nextHalfEdge the new next half edge for the current half edge. Can be null
    * @throws RuntimeException in case the origin of this specified next half edge is not the same as
    *            the destination of the this edge
    */
   public void setNextHalfEdge(HalfEdge3D nextHalfEdge)
   {
      if (nextHalfEdge == null || nextHalfEdge.getOriginVertex() == getDestinationVertex() && nextHalfEdge.getFace() == getFace())
         setNextHalfEdgeUnsafe(nextHalfEdge);
      else
         throw new RuntimeException("Mismatch between vertices, destination vertex: " + getDestinationVertex().toString() + " , next edge origin vertex: "
               + nextHalfEdge.getOriginVertex().toString());
   }

   /**
    * Internal method to update the next half edge without any checks
    *
    * @param nextHalfEdge the half edge whose reference is to be stored in the next half edge field
    */
   private void setNextHalfEdgeUnsafe(HalfEdge3D nextHalfEdge)
   {
      this.nextHalfEdge = nextHalfEdge;
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public HalfEdge3D getNextHalfEdge()
   {
      return nextHalfEdge;
   }

   /**
    * Update the reference to the {@code previous HalfEdge}. Checks to ensure that the destination of
    * the specified edge and origin of the this half edge are the same
    *
    * @param previousHalfEdge the new previous half edge for the current half edge. Can be null
    * @throws RuntimeException in case the destination of this specified next half edge is not the same
    *            as the origin of the this edge
    */
   public void setPreviousHalfEdge(HalfEdge3D previousHalfEdge)
   {
      if (previousHalfEdge == null || previousHalfEdge.getDestinationVertex() == getOriginVertex() && previousHalfEdge.getFace() == getFace())
         setPreviousHalfEdgeUnsafe(previousHalfEdge);
      else
         throw new RuntimeException("Mismatch between vertices, origin vertex: " + getOriginVertex().toString() + " , previous edge destination vertex: "
               + previousHalfEdge.getDestinationVertex().toString());
   }

   /**
    * Internal method to update the next half edge without any checks
    *
    * @param previousHalfEdge the half edge whose reference is to be stored in the previous half edge
    *           field
    */
   private void setPreviousHalfEdgeUnsafe(HalfEdge3D previousHalfEdge)
   {
      this.previousHalfEdge = previousHalfEdge;
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public HalfEdge3D getPreviousHalfEdge()
   {
      return previousHalfEdge;
   }

   /**
    * Update the reference to the face that this half edge is a part of
    *
    * @param face the face reference to be stored. Can be null
    */
   public void setFace(Face3DBasics face)
   {
      this.face = face;
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public Face3DBasics getFace()
   {
      return face;
   }

   @Override
   public Point3DBasics getFirstEndpoint()
   {
      return getOriginVertex();
   }

   @Override
   public Point3DBasics getSecondEndpoint()
   {
      return getDestinationVertex();
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public Vector3DReadOnly getEdgeVector()
   {
      edgeVector.sub(destinationVertex, originVertex);
      return edgeVector;
   }

   /**
    * Sets all the references that are held by this half edge to null and also updates the previously
    * associated objects
    */
   public void clear()
   {
      setTwinHalfEdge(null);
      setOriginVertex(null);
      setDestinationVertex(null);
      setNextHalfEdge(null);
      setPreviousHalfEdge(null);
      setFace(null);
   }

   /**
    * Changes the direction of this edge so that it starts at the previous {@code destinationVertex}
    * and ends at the {@code originVertex} The references to the {@code nextHalfEdge} and
    * {@code previousHalfEdge} are also updated as its the twin edge. Reference to the {@code face} is
    * not changed
    */
   public void reverseEdge()
   {
      Vertex3D newDestinationVertex = originVertex;
      setOriginVertex(destinationVertex);
      setDestinationVertex(newDestinationVertex);
      HalfEdge3D newNextHalfEdge = previousHalfEdge;
      setPreviousHalfEdgeUnsafe(nextHalfEdge);
      setNextHalfEdgeUnsafe(newNextHalfEdge);
   }

   /**
    * {@inheritDoc}
    */
   @Override
   public String toString()
   {
      return "From: " + (originVertex == null ? "null" : originVertex.toString()) + ", To: "
            + (destinationVertex == null ? "null" : destinationVertex.toString());
   }
}
