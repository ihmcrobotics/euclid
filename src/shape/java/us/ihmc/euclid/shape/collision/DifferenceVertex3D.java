package us.ihmc.euclid.shape.collision;

import us.ihmc.euclid.shape.convexPolytope.Vertex3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;

public class DifferenceVertex3D extends Vertex3D
{
   private Point3DReadOnly shapeAVertexReference;
   private Point3DReadOnly shapeBVertexReference;

   public DifferenceVertex3D()
   {
      super();
   }

   public DifferenceVertex3D(Point3DReadOnly vertexOnShapeA, Point3DReadOnly vertexOnShapeB)
   {
      set(vertexOnShapeA, vertexOnShapeB);
   }

   public DifferenceVertex3D(DifferenceVertex3D other)
   {
      this.shapeAVertexReference = other.shapeAVertexReference;
      this.shapeBVertexReference = other.shapeBVertexReference;
      set(other);
   }

   public void set(Point3DReadOnly vertexOnShapeA, Point3DReadOnly vertexOnShapeB)
   {
      this.shapeAVertexReference = vertexOnShapeA;
      this.shapeBVertexReference = vertexOnShapeB;
      sub(vertexOnShapeA, vertexOnShapeB);
   }

   public Point3DReadOnly getVertexOnShapeA()
   {
      return shapeAVertexReference;
   }

   public Point3DReadOnly getVertexOnShapeB()
   {
      return shapeBVertexReference;
   }

   @Override
   public boolean equals(Object object)
   {
      if (object == this)
      {
         return true;
      }
      else if (object instanceof Tuple3DReadOnly)
      {
         return equals((Tuple3DReadOnly) object);
      }
      else
      {
         return false;
      }
   }
}