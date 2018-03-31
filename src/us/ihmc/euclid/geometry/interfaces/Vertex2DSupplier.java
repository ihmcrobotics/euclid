package us.ihmc.euclid.geometry.interfaces;

import java.util.List;

import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;

public interface Vertex2DSupplier
{
   Point2DReadOnly getVertex(int index);

   public int getNumberOfVertices();

   public static Vertex2DSupplier asVertex2DSupplier(Point2DReadOnly... vertices)
   {
      return asVertex2DSupplier(vertices, vertices.length);
   }

   public static Vertex2DSupplier asVertex2DSupplier(Point2DReadOnly[] vertices, int numberOfVertices)
   {
      return asVertex2DSupplier(vertices, 0, numberOfVertices);
   }

   public static Vertex2DSupplier asVertex2DSupplier(Point2DReadOnly[] vertices, int startIndex, int numberOfVertices)
   {
      if (startIndex >= numberOfVertices)
         throw new IllegalArgumentException("The starting index cannot be greater or equal than the number of vertices.");
      if (startIndex + numberOfVertices > vertices.length)
         throw new IllegalArgumentException("The array is too small. Array length = " + vertices.length + ", expected minimum length = "
               + (startIndex + numberOfVertices));

      return new Vertex2DSupplier()
      {
         @Override
         public Point2DReadOnly getVertex(int index)
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

   public static Vertex2DSupplier asVertex2DSupplier(List<? extends Point2DReadOnly> vertices)
   {
      return asVertex2DSupplier(vertices, vertices.size());
   }

   public static Vertex2DSupplier asVertex2DSupplier(List<? extends Point2DReadOnly> vertices, int numberOfVertices)
   {
      return asVertex2DSupplier(vertices, 0, numberOfVertices);
   }

   public static Vertex2DSupplier asVertex2DSupplier(List<? extends Point2DReadOnly> vertices, int startIndex, int numberOfVertices)
   {
      if (startIndex >= numberOfVertices)
         throw new IllegalArgumentException("The starting index cannot be greater or equal than the number of vertices.");
      if (startIndex + numberOfVertices > vertices.size())
         throw new IllegalArgumentException("The list is too small. List size = " + vertices.size() + ", expected minimum size = "
               + (startIndex + numberOfVertices));

      return new Vertex2DSupplier()
      {
         @Override
         public Point2DReadOnly getVertex(int index)
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

   public static Vertex2DSupplier asVertex2DSupplier(double[][] vertices)
   {
      return asVertex2DSupplier(vertices, vertices.length);
   }

   public static Vertex2DSupplier asVertex2DSupplier(double[][] vertices, int numberOfVertices)
   {
      return asVertex2DSupplier(vertices, 0, numberOfVertices);
   }

   public static Vertex2DSupplier asVertex2DSupplier(double[][] vertices, int startIndex, int numberOfVertices)
   {
      if (startIndex >= numberOfVertices)
         throw new IllegalArgumentException("The starting index cannot be greater or equal than the number of vertices.");
      if (startIndex + numberOfVertices > vertices.length)
         throw new IllegalArgumentException("The array is too small. Array length = " + vertices.length + ", expected minimum length = "
               + (startIndex + numberOfVertices));

      return new Vertex2DSupplier()
      {
         @Override
         public Point2DReadOnly getVertex(int index)
         {
            return new Point2DReadOnly()
            {
               @Override
               public double getX()
               {
                  return vertices[index][0];
               }

               @Override
               public double getY()
               {
                  return vertices[index][1];
               }
            };
         }

         @Override
         public int getNumberOfVertices()
         {
            return numberOfVertices;
         }
      };
   }
}
