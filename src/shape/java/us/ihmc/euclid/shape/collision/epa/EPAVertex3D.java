package us.ihmc.euclid.shape.collision.epa;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.shape.collision.gjk.GJKVertex3D;
import us.ihmc.euclid.shape.convexPolytope.interfaces.Vertex3DReadOnly;
import us.ihmc.euclid.shape.tools.EuclidShapeIOTools;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

public class EPAVertex3D implements Vertex3DReadOnly
{
   private final double x, y, z;
   private final Point3DReadOnly vertexOnShapeA;
   private final Point3DReadOnly vertexOnShapeB;
   private final List<EPAEdge3D> associatedEdges = new ArrayList<>();

   public EPAVertex3D(GJKVertex3D gjkVertex3D)
   {
      vertexOnShapeA = gjkVertex3D.getVertexOnShapeA();
      vertexOnShapeB = gjkVertex3D.getVertexOnShapeB();
      x = gjkVertex3D.getX();
      y = gjkVertex3D.getY();
      z = gjkVertex3D.getZ();
   }

   public EPAVertex3D(Point3DReadOnly vertexOnShapeA, Point3DReadOnly vertexOnShapeB)
   {
      this.vertexOnShapeA = vertexOnShapeA;
      this.vertexOnShapeB = vertexOnShapeB;
      x = vertexOnShapeA.getX() - vertexOnShapeB.getX();
      y = vertexOnShapeA.getY() - vertexOnShapeB.getY();
      z = vertexOnShapeA.getZ() - vertexOnShapeB.getZ();
   }

   public void addAssociatedEdge(EPAEdge3D edge)
   {
      if (!isEdgeAssociated(edge))
      {
         if (edge.getOrigin() != this)
            throw new IllegalArgumentException("A vertex's associated edges should originate from this same vertex.");
         associatedEdges.add(edge);
      }
   }

   public void removeAssociatedEdge(EPAEdge3D edgeToAdd)
   {
      associatedEdges.remove(edgeToAdd);
   }

   @Override
   public List<EPAEdge3D> getAssociatedEdges()
   {
      return associatedEdges;
   }

   @Override
   public EPAEdge3D getAssociatedEdge(int index)
   {
      return associatedEdges.get(index);
   }

   @Override
   public int getNumberOfAssociatedEdges()
   {
      return associatedEdges.size();
   }

   @Override
   public EPAEdge3D getEdgeTo(Vertex3DReadOnly destination)
   {
      return (EPAEdge3D) Vertex3DReadOnly.super.getEdgeTo(destination);
   }

   public Point3DReadOnly getVertexOnShapeA()
   {
      return vertexOnShapeA;
   }

   public Point3DReadOnly getVertexOnShapeB()
   {
      return vertexOnShapeB;
   }

   @Override
   public double getX()
   {
      return x;
   }

   @Override
   public double getY()
   {
      return y;
   }

   @Override
   public double getZ()
   {
      return z;
   }

   @Override
   public boolean equals(Object object)
   {
      if (object == this)
         return true;
      else if (object instanceof Point3DReadOnly)
         return equals((Point3DReadOnly) object);
      else
         return false;
   }

   @Override
   public String toString()
   {
      return EuclidShapeIOTools.getVertex3DString(this);
   }
}