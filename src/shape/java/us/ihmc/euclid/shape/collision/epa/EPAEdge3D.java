package us.ihmc.euclid.shape.collision.epa;

import us.ihmc.euclid.shape.convexPolytope.interfaces.HalfEdge3DReadOnly;
import us.ihmc.euclid.shape.tools.EuclidShapeIOTools;

public class EPAEdge3D implements HalfEdge3DReadOnly
{
   private final EPAFace3D face;
   private final EPAVertex3D v0;
   private final EPAVertex3D v1;
   private EPAEdge3D twin;
   private EPAEdge3D next;
   private EPAEdge3D prev;

   public EPAEdge3D(EPAFace3D face, EPAVertex3D v0, EPAVertex3D v1)
   {
      this.face = face;
      this.v0 = v0;
      this.v1 = v1;
      v0.addAssociatedEdge(this);
   }

   public void setTwin(EPAEdge3D twin)
   {
      if (v0 != twin.v1 || v1 != twin.v0)
         throw new IllegalArgumentException("Twin does not match: this edge:\n" + this + "\ntwin:\n" + twin);

      this.twin = twin;

      if (twin.twin != this)
         twin.setTwin(this);
   }

   public void setNext(EPAEdge3D next)
   {
      this.next = next;
   }

   public void setPrevious(EPAEdge3D prev)
   {
      this.prev = prev;
   }

   public void destroy()
   {
      v0.removeAssociatedEdge(this);
   }

   @Override
   public EPAVertex3D getOrigin()
   {
      return v0;
   }

   @Override
   public EPAVertex3D getDestination()
   {
      return v1;
   }

   @Override
   public EPAEdge3D getNext()
   {
      return next;
   }

   @Override
   public EPAEdge3D getPrevious()
   {
      return prev;
   }

   @Override
   public EPAEdge3D getTwin()
   {
      return twin;
   }

   @Override
   public EPAFace3D getFace()
   {
      return face;
   }

   @Override
   public String toString()
   {
      return EuclidShapeIOTools.getHalfEdge3DString(this);
   }
}