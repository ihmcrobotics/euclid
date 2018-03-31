package us.ihmc.euclid.referenceFrame.interfaces;

import java.util.List;

import us.ihmc.euclid.geometry.interfaces.Vertex3DSupplier;

public interface FrameVertex3DSupplier extends Vertex3DSupplier
{
   @Override
   FramePoint3DReadOnly getVertex(int index);

   default boolean equals(FrameVertex3DSupplier other)
   {
      if (other == null)
         return false;
      if (getNumberOfVertices() != other.getNumberOfVertices())
         return false;
      for (int i = 0; i < getNumberOfVertices(); i++)
      {
         if (!getVertex(i).equals(other.getVertex(i)))
            return false;
      }
      return true;
   }

   default boolean epsilonEquals(FrameVertex3DSupplier other, double epsilon)
   {
      if (getNumberOfVertices() != other.getNumberOfVertices())
         return false;
      for (int i = 0; i < getNumberOfVertices(); i++)
      {
         if (!getVertex(i).epsilonEquals(other.getVertex(i), epsilon))
            return false;
      }
      return true;
   }

   public static FrameVertex3DSupplier asVertex3DSupplier(FramePoint3DReadOnly... vertices)
   {
      return asVertex3DSupplier(vertices, vertices.length);
   }

   public static FrameVertex3DSupplier asVertex3DSupplier(FramePoint3DReadOnly[] vertices, int numberOfVertices)
   {
      return asVertex3DSupplier(vertices, 0, numberOfVertices);
   }

   public static FrameVertex3DSupplier asVertex3DSupplier(FramePoint3DReadOnly[] vertices, int startIndex, int numberOfVertices)
   {
      if (startIndex >= numberOfVertices)
         throw new IllegalArgumentException("The starting index cannot be greater or equal than the number of vertices.");
      if (startIndex + numberOfVertices > vertices.length)
         throw new IllegalArgumentException("The array is too small. Array length = " + vertices.length + ", expected minimum length = "
               + (startIndex + numberOfVertices));

      return new FrameVertex3DSupplier()
      {
         @Override
         public FramePoint3DReadOnly getVertex(int index)
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

   public static FrameVertex3DSupplier asFrameVertex3DSupplier(List<? extends FramePoint3DReadOnly> vertices)
   {
      return asFrameVertex3DSupplier(vertices, vertices.size());
   }

   public static FrameVertex3DSupplier asFrameVertex3DSupplier(List<? extends FramePoint3DReadOnly> vertices, int numberOfVertices)
   {
      return asFrameVertex3DSupplier(vertices, 0, numberOfVertices);
   }

   public static FrameVertex3DSupplier asFrameVertex3DSupplier(List<? extends FramePoint3DReadOnly> vertices, int startIndex, int numberOfVertices)
   {
      if (startIndex >= numberOfVertices)
         throw new IllegalArgumentException("The starting index cannot be greater or equal than the number of vertices.");
      if (startIndex + numberOfVertices > vertices.size())
         throw new IllegalArgumentException("The list is too small. List size = " + vertices.size() + ", expected minimum size = "
               + (startIndex + numberOfVertices));

      return new FrameVertex3DSupplier()
      {
         @Override
         public FramePoint3DReadOnly getVertex(int index)
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
