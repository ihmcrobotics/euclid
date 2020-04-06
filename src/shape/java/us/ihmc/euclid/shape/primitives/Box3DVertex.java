package us.ihmc.euclid.shape.primitives;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.function.DoubleSupplier;

import us.ihmc.euclid.shape.convexPolytope.interfaces.Vertex3DReadOnly;
import us.ihmc.euclid.shape.primitives.interfaces.Shape3DChangeListener;
import us.ihmc.euclid.tools.EuclidCoreFactories;
import us.ihmc.euclid.transform.interfaces.Transform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

class Box3DVertex implements Vertex3DReadOnly, Shape3DChangeListener
{
   private final List<Box3DHalfEdge> associatedEdges = new ArrayList<Box3DHalfEdge>();

   private final Point3D position = new Point3D();
   private final Point3DReadOnly positionLocal;
   private final Transform pose;

   private boolean dirty = true;

   Box3DVertex(Transform pose, DoubleSupplier xLocal, DoubleSupplier yLocal, DoubleSupplier zLocal)
   {
      this.pose = pose;
      positionLocal = EuclidCoreFactories.newLinkedPoint3DReadOnly(xLocal, yLocal, zLocal);
   }

   @Override
   public void changed()
   {
      dirty = true;
   }

   private void update()
   {
      if (dirty)
      {
         pose.inverseTransform(positionLocal, position);
         dirty = false;
      }
   }

   void addAssociatedEdge(Box3DHalfEdge associatedEdge)
   {
      associatedEdges.add(associatedEdge);
   }

   @Override
   public Box3DHalfEdge getEdgeTo(Vertex3DReadOnly destination)
   {
      return (Box3DHalfEdge) Vertex3DReadOnly.super.getEdgeTo(destination);
   }

   @Override
   public double getX()
   {
      update();
      return position.getX();
   }

   @Override
   public double getY()
   {
      update();
      return position.getY();
   }

   @Override
   public double getZ()
   {
      update();
      return position.getZ();
   }

   @Override
   public Collection<Box3DHalfEdge> getAssociatedEdges()
   {
      return associatedEdges;
   }

   @Override
   public Box3DHalfEdge getAssociatedEdge(int index)
   {
      return associatedEdges.get(index);
   }

   @Override
   public int getNumberOfAssociatedEdges()
   {
      return 3;
   }
}
