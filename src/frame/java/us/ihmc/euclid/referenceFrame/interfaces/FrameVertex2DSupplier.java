package us.ihmc.euclid.referenceFrame.interfaces;

import java.util.Arrays;
import java.util.List;

import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreIOTools;
import us.ihmc.euclid.tools.EuclidHashCodeTools;

/**
 * Implement this interface to create a custom supplier of 2D frame vertices or use the static
 * methods to create default suppliers.
 *
 * @author Sylvain Bertrand
 */
public interface FrameVertex2DSupplier extends Vertex2DSupplier, EuclidFrameGeometry
{
   /**
    * A supplier with no vertices.
    */
   static final FrameVertex2DSupplier EMPTY_SUPPLIER = new FrameVertex2DSupplier()
   {
      @Override
      public FramePoint2DReadOnly getVertex(int index)
      {
         return null;
      }

      @Override
      public int getNumberOfVertices()
      {
         return 0;
      }

      @Override
      public int hashCode()
      {
         return 1;
      }

      @Override
      public boolean equals(Object object)
      {
         if (object instanceof FrameVertex2DSupplier)
            return equals((FrameVertex2DSupplier) object);
         else
            return false;
      }

      @Override
      public String toString()
      {
         return toString(EuclidCoreIOTools.DEFAULT_FORMAT);
      }
   };

   /** {@inheritDoc} */
   @Override
   FramePoint2DReadOnly getVertex(int index);

   /**
    * Gets the reference frame of the first vertex or returns {@code null} if this supplier is empty.
    */
   @Override
   default ReferenceFrame getReferenceFrame()
   {
      return isEmpty() ? null : getVertex(0).getReferenceFrame();
   }

   /** {@inheritDoc} */
   @Override
   default String toString(String format)
   {
      StringBuilder sb = new StringBuilder("Frame vertex 2D Supplier: [");

      for (int i = 0; i < getNumberOfVertices(); i++)
      {
         if (i > 0)
            sb.append(", ");
         sb.append(getVertex(i).toString(format));
      }

      sb.append(']');

      return sb.toString();
   }

   /**
    * Gets an empty supplier.
    *
    * @return the supplier.
    */
   public static FrameVertex2DSupplier emptyFrameVertex2DSupplier()
   {
      return EMPTY_SUPPLIER;
   }

   /**
    * Returns a fixed-size supplier backed by the given array.
    *
    * @param vertices the array by which the supplier will be backed.
    * @return the supplier.
    */
   public static FrameVertex2DSupplier asFrameVertex2DSupplier(FramePoint2DReadOnly... vertices)
   {
      return asFrameVertex2DSupplier(Arrays.asList(vertices));
   }

   /**
    * Returns a fixed-size supplier backed by a portion of the given array starting with the first
    * element and specified by its length {@code numberOfVertices}.
    *
    * @param vertices         the array by which the supplier will be backed.
    * @param numberOfVertices the portion's length.
    * @return the supplier.
    */
   public static FrameVertex2DSupplier asFrameVertex2DSupplier(FramePoint2DReadOnly[] vertices, int numberOfVertices)
   {
      return asFrameVertex2DSupplier(Arrays.asList(vertices), numberOfVertices);
   }

   /**
    * Returns a fixed-size supplier backed by a portion of the given array specified by the first index
    * {@code startIndex} and the portion length {@code numberOfVertices}.
    *
    * @param vertices         the array by which the supplier will be backed.
    * @param startIndex       the first vertex index.
    * @param numberOfVertices the portion's length.
    * @return the supplier.
    */
   public static FrameVertex2DSupplier asFrameVertex2DSupplier(FramePoint2DReadOnly[] vertices, int startIndex, int numberOfVertices)
   {
      if (numberOfVertices == 0)
         return emptyFrameVertex2DSupplier();
      if (startIndex + numberOfVertices > vertices.length)
         throw new IllegalArgumentException("The array is too small. Array length = " + vertices.length + ", expected minimum length = "
               + (startIndex + numberOfVertices));

      return asFrameVertex2DSupplier(Arrays.asList(vertices), startIndex, numberOfVertices);
   }

   /**
    * Returns a fixed-size supplier backed by the given list.
    *
    * @param vertices the list by which the supplier will be backed.
    * @return the supplier.
    */
   public static FrameVertex2DSupplier asFrameVertex2DSupplier(List<? extends FramePoint2DReadOnly> vertices)
   {
      return asFrameVertex2DSupplier(vertices, vertices.size());
   }

   /**
    * Returns a fixed-size supplier backed by a portion of the given list starting with the first
    * element and specified by its length {@code numberOfVertices}.
    *
    * @param vertices         the list by which the supplier will be backed.
    * @param numberOfVertices the portion's length.
    * @return the supplier.
    */
   public static FrameVertex2DSupplier asFrameVertex2DSupplier(List<? extends FramePoint2DReadOnly> vertices, int numberOfVertices)
   {
      return asFrameVertex2DSupplier(vertices, 0, numberOfVertices);
   }

   /**
    * Returns a fixed-size supplier backed by a portion of the given list specified by the first index
    * {@code startIndex} and the portion length {@code numberOfVertices}.
    *
    * @param vertices         the list by which the supplier will be backed.
    * @param startIndex       the first vertex index.
    * @param numberOfVertices the portion's length.
    * @return the supplier.
    */
   public static FrameVertex2DSupplier asFrameVertex2DSupplier(List<? extends FramePoint2DReadOnly> vertices, int startIndex, int numberOfVertices)
   {
      if (numberOfVertices == 0)
         return emptyFrameVertex2DSupplier();
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

         @Override
         public int hashCode()
         {
            long bits = 1;
            for (int i = 0; i < getNumberOfVertices(); i++)
            {
               bits = EuclidHashCodeTools.addToHashCode(bits, getVertex(i));
            }
            return EuclidHashCodeTools.toIntHashCode(bits);
         }

         @Override
         public boolean equals(Object object)
         {
            if (object instanceof FrameVertex2DSupplier)
               return equals((FrameVertex2DSupplier) object);
            else
               return false;
         }

         @Override
         public String toString()
         {
            return toString(EuclidCoreIOTools.DEFAULT_FORMAT);
         }
      };
   }
}
