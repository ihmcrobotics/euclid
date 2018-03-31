package us.ihmc.euclid.referenceFrame.interfaces;

import java.util.List;

import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;

public interface FrameVertex2DSupplier extends Vertex2DSupplier
{
   @Override
   FramePoint2DReadOnly getVertex(int index);

   public static FrameVertex2DSupplier asVertex2DSupplier(FramePoint2DReadOnly... vertices)
   {
      return asVertex2DSupplier(vertices, vertices.length);
   }

   public static FrameVertex2DSupplier asVertex2DSupplier(FramePoint2DReadOnly[] vertices, int numberOfVertices)
   {
      return asVertex2DSupplier(vertices, 0, numberOfVertices);
   }

   public static FrameVertex2DSupplier asVertex2DSupplier(FramePoint2DReadOnly[] vertices, int startIndex, int numberOfVertices)
   {
      if (startIndex >= numberOfVertices)
         throw new IllegalArgumentException("The starting index cannot be greater or equal than the number of vertices.");
      if (startIndex + numberOfVertices > vertices.length)
         throw new IllegalArgumentException("The array is too small. Array length = " + vertices.length + ", expected minimum length = "
               + (startIndex + numberOfVertices));

      return new FrameVertex2DSupplier()
      {
         @Override
         public FramePoint2DReadOnly getVertex(int index)
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

   public static FrameVertex2DSupplier asFrameVertex2DSupplier(List<? extends FramePoint2DReadOnly> vertices)
   {
      return asFrameVertex2DSupplier(vertices, vertices.size());
   }

   public static FrameVertex2DSupplier asFrameVertex2DSupplier(List<? extends FramePoint2DReadOnly> vertices, int numberOfVertices)
   {
      return asFrameVertex2DSupplier(vertices, 0, numberOfVertices);
   }

   public static FrameVertex2DSupplier asFrameVertex2DSupplier(List<? extends FramePoint2DReadOnly> vertices, int startIndex, int numberOfVertices)
   {
      if (startIndex >= numberOfVertices)
         throw new IllegalArgumentException("The starting index cannot be greater or equal than the number of vertices.");
      if (startIndex + numberOfVertices > vertices.size())
         throw new IllegalArgumentException("The list is too small. List size = " + vertices.size() + ", expected minimum size = "
               + (startIndex + numberOfVertices));

      return new FrameVertex2DSupplier()
      {
         @Override
         public FramePoint2DReadOnly getVertex(int index)
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
