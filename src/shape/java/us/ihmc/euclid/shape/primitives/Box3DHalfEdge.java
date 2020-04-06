package us.ihmc.euclid.shape.primitives;

import us.ihmc.euclid.shape.convexPolytope.interfaces.Face3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.HalfEdge3DReadOnly;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Vertex3DReadOnly;

class Box3DHalfEdge implements HalfEdge3DReadOnly
{
   private final Box3DVertex origin;
   private final Box3DVertex destination;

   private Box3DFace face;
   private Box3DHalfEdge previous;
   private Box3DHalfEdge next;
   private Box3DHalfEdge twin;

   Box3DHalfEdge(Box3DVertex origin, Box3DVertex destination)
   {
      this.origin = origin;
      this.destination = destination;
      origin.addAssociatedEdge(this);
   }

   void setFace(Box3DFace face)
   {
      this.face = face;
   }

   void setTwin(Box3DHalfEdge twin)
   {
      this.twin = twin;
   }

   void setNext(Box3DHalfEdge next)
   {
      this.next = next;
   }

   void setPrevious(Box3DHalfEdge previous)
   {
      this.previous = previous;
   }

   void findAndSetTwin()
   {
      twin = destination.getEdgeTo(origin);
      twin.twin = this;
   }

   @Override
   public Vertex3DReadOnly getOrigin()
   {
      return origin;
   }

   @Override
   public Vertex3DReadOnly getDestination()
   {
      return destination;
   }

   @Override
   public HalfEdge3DReadOnly getTwin()
   {
      return twin;
   }

   @Override
   public HalfEdge3DReadOnly getNext()
   {
      return next;
   }

   @Override
   public HalfEdge3DReadOnly getPrevious()
   {
      return previous;
   }

   @Override
   public Face3DReadOnly getFace()
   {
      return face;
   }
}
