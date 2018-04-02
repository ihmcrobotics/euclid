package us.ihmc.euclid.referenceFrame.interfaces;

import java.util.Arrays;
import java.util.List;

import us.ihmc.euclid.geometry.interfaces.Vertex3DSupplier;

/**
 * Implement this interface to create a custom supplier of 3D frame vertices or use the static
 * methods to create default suppliers.
 * 
 * @author Sylvain Bertrand
 */
public interface FrameVertex3DSupplier extends Vertex3DSupplier
{
   /**
    * A supplier with no vertices.
    */
   static final FrameVertex3DSupplier EMPTY_SUPPLIER = new FrameVertex3DSupplier()
   {
      @Override
      public FramePoint3DReadOnly getVertex(int index)
      {
         return null;
      }

      @Override
      public int getNumberOfVertices()
      {
         return 0;
      }
   };

   /** {@inheritDoc} */
   @Override
   FramePoint3DReadOnly getVertex(int index);

   /**
    * Tests on a per-vertex basis if this supplier and {@code other} are equal.
    * 
    * @param other the other supplier to compare against this.
    * @return {@code true} if the two suppliers are equal.
    */
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

   /**
    * Tests on a per-vertex basis if this supplier and {@code other} are equal to an {@code epsilon}.
    * 
    * @param other the other supplier to compare against this.
    * @param epsilon the tolerance to use.
    * @return {@code true} if the two suppliers are equal.
    */
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

   /**
    * Gets an empty supplier.
    * 
    * @return the supplier.
    */
   public static FrameVertex3DSupplier emptyFrameVertex3DSupplier()
   {
      return EMPTY_SUPPLIER;
   }

   /**
    * Returns a fixed-size supplier backed by the given array.
    * 
    * @param vertices the array by which the supplier will be backed.
    * @return the supplier.
    */
   public static FrameVertex3DSupplier asFrameVertex3DSupplier(FramePoint3DReadOnly... vertices)
   {
      return asFrameVertex3DSupplier(Arrays.asList(vertices));
   }

   /**
    * Returns a fixed-size supplier backed by a portion of the given array starting with the first
    * element and specified by its length {@code numberOfVertices}.
    * 
    * @param vertices the array by which the supplier will be backed.
    * @param numberOfVertices the portion's length.
    * @return the supplier.
    */
   public static FrameVertex3DSupplier asFrameVertex3DSupplier(FramePoint3DReadOnly[] vertices, int numberOfVertices)
   {
      return asFrameVertex3DSupplier(Arrays.asList(vertices), numberOfVertices);
   }

   /**
    * Returns a fixed-size supplier backed by a portion of the given array specified by the first index
    * {@code startIndex} and the portion length {@code numberOfVertices}.
    * 
    * @param vertices the array by which the supplier will be backed.
    * @param startIndex the first vertex index.
    * @param numberOfVertices the portion's length.
    * @return the supplier.
    */
   public static FrameVertex3DSupplier asFrameVertex3DSupplier(FramePoint3DReadOnly[] vertices, int startIndex, int numberOfVertices)
   {
      if (numberOfVertices == 0)
         return emptyFrameVertex3DSupplier();
      if (startIndex + numberOfVertices > vertices.length)
         throw new IllegalArgumentException("The array is too small. Array length = " + vertices.length + ", expected minimum length = "
               + (startIndex + numberOfVertices));

      return asFrameVertex3DSupplier(Arrays.asList(vertices), startIndex, numberOfVertices);
   }

   /**
    * Returns a fixed-size supplier backed by the given list.
    * 
    * @param vertices the list by which the supplier will be backed.
    * @return the supplier.
    */
   public static FrameVertex3DSupplier asFrameVertex3DSupplier(List<? extends FramePoint3DReadOnly> vertices)
   {
      return asFrameVertex3DSupplier(vertices, vertices.size());
   }

   /**
    * Returns a fixed-size supplier backed by a portion of the given list starting with the first
    * element and specified by its length {@code numberOfVertices}.
    * 
    * @param vertices the list by which the supplier will be backed.
    * @param numberOfVertices the portion's length.
    * @return the supplier.
    */
   public static FrameVertex3DSupplier asFrameVertex3DSupplier(List<? extends FramePoint3DReadOnly> vertices, int numberOfVertices)
   {
      return asFrameVertex3DSupplier(vertices, 0, numberOfVertices);
   }

   /**
    * Returns a fixed-size supplier backed by a portion of the given list specified by the first index
    * {@code startIndex} and the portion length {@code numberOfVertices}.
    * 
    * @param vertices the list by which the supplier will be backed.
    * @param startIndex the first vertex index.
    * @param numberOfVertices the portion's length.
    * @return the supplier.
    */
   public static FrameVertex3DSupplier asFrameVertex3DSupplier(List<? extends FramePoint3DReadOnly> vertices, int startIndex, int numberOfVertices)
   {
      if (numberOfVertices == 0)
         return emptyFrameVertex3DSupplier();
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

         @Override
         public String toString()
         {
            return "Vertex 3D Supplier: " + vertices.subList(startIndex, startIndex + numberOfVertices).toString();
         }
      };
   }
}
