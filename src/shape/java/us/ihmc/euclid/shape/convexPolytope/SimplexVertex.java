package us.ihmc.euclid.shape.convexPolytope;

import us.ihmc.euclid.shape.convexPolytope.interfaces.Vertex3DReadOnly;

public class SimplexVertex extends Vertex3D
{
   Vertex3DReadOnly polytopeAVertexReference;
   Vertex3DReadOnly polytopeBVertexReference;

   public SimplexVertex()
   {
      super();
   }

   public SimplexVertex(Vertex3D vertexOnPolytopeA, Vertex3D vertexOnPolytopeB)
   {
      set(vertexOnPolytopeA, vertexOnPolytopeB);
   }

   public void set(Vertex3DReadOnly vertexOnPolytopeA, Vertex3DReadOnly vertexOnPolytopeB)
   {
      this.polytopeAVertexReference = vertexOnPolytopeA;
      this.polytopeBVertexReference = vertexOnPolytopeB;
      sub(vertexOnPolytopeA, vertexOnPolytopeB);
   }

   public Vertex3DReadOnly getVertexOnPolytopeA()
   {
      return polytopeAVertexReference;
   }

   public Vertex3DReadOnly getVertexOnPolytopeB()
   {
      return polytopeBVertexReference;
   }
}
