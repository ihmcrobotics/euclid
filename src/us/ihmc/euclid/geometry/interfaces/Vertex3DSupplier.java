package us.ihmc.euclid.geometry.interfaces;

import java.util.List;

import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

public interface Vertex3DSupplier
{
   Point3DReadOnly getVertex(int index);

   public int getNumberOfVertices();

   public static Vertex3DSupplier asVertex3DSupplier(Point3DReadOnly... vertices)
   {
      return asVertex3DSupplier(vertices, vertices.length);
   }

   public static Vertex3DSupplier asVertex3DSupplier(Point3DReadOnly[] vertices, int numberOfVertices)
   {
      return asVertex3DSupplier(vertices, 0, numberOfVertices);
   }

   public static Vertex3DSupplier asVertex3DSupplier(Point3DReadOnly[] vertices, int startIndex, int numberOfVertices)
   {
      if (startIndex >= numberOfVertices)
         throw new IllegalArgumentException("The starting index cannot be greater or equal than the number of vertices.");
      if (startIndex + numberOfVertices > vertices.length)
         throw new IllegalArgumentException("The array is too small. Array length = " + vertices.length + ", expected minimum length = "
               + (startIndex + numberOfVertices));

      return new Vertex3DSupplier()
      {
         @Override
         public Point3DReadOnly getVertex(int index)
         {
            return vertices[index + startIndex];
         }

         @Override
         public int getNumberOfVertices()
         {
            return numberOfVertices;
         }
      };
   }

   public static Vertex3DSupplier asVertex3DSupplier(List<? extends Point3DReadOnly> vertices)
   {
      return asVertex3DSupplier(vertices, vertices.size());
   }

   public static Vertex3DSupplier asVertex3DSupplier(List<? extends Point3DReadOnly> vertices, int numberOfVertices)
   {
      return asVertex3DSupplier(vertices, 0, numberOfVertices);
   }

   public static Vertex3DSupplier asVertex3DSupplier(List<? extends Point3DReadOnly> vertices, int startIndex, int numberOfVertices)
   {
      if (startIndex >= numberOfVertices)
         throw new IllegalArgumentException("The starting index cannot be greater or equal than the number of vertices.");
      if (startIndex + numberOfVertices > vertices.size())
         throw new IllegalArgumentException("The list is too small. List size = " + vertices.size() + ", expected minimum size = "
               + (startIndex + numberOfVertices));

      return new Vertex3DSupplier()
      {
         @Override
         public Point3DReadOnly getVertex(int index)
         {
            return vertices.get(index + startIndex);
         }

         @Override
         public int getNumberOfVertices()
         {
            return numberOfVertices;
         }
      };
   }
}
