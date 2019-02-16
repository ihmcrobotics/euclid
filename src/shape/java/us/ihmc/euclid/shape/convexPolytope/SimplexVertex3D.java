package us.ihmc.euclid.shape.convexPolytope;

import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

public class SimplexVertex3D extends Vertex3D
{
   private Point3DReadOnly shapeAVertexReference;
   private Point3DReadOnly shapeBVertexReference;

   public SimplexVertex3D()
   {
      super();
   }

   public SimplexVertex3D(Point3DReadOnly vertexOnShapeA, Point3DReadOnly vertexOnShapeB)
   {
      set(vertexOnShapeA, vertexOnShapeB);
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
}
